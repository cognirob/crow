from concurrent.futures import thread
from numpy.core.fromnumeric import size
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.exceptions import ParameterNotDeclaredException
from ros2param.api import call_get_parameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image as msg_image
from crow_msgs.msg import FilteredPose, NlpStatus, ObjectPointcloud, StampedString, SentenceProgram
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge
import message_filters
from crow_ontology.crowracle_client import CrowtologyClient
from crow_vision_ros2.utils import ftl_pcl2numpy
from crow_nlp.nlp_crow.templates.actions.RemoveCommandX import RemoveCommandX

import cv2
import numpy as np
import time
import open3d as o3d
from PIL import Image, ImageFont, ImageDraw
from unicodedata import normalize
import wx
import wx.grid
from datetime import datetime
import yaml
import os
from importlib.util import find_spec
import pynput
from crow_control.utils import QueueClient, UniversalParamClient
from concurrent import futures
from threading import Lock
import functools
from collections import OrderedDict
import json
import sys
import argparse


thread_pool_executor = futures.ThreadPoolExecutor(max_workers=5)
print("WX Version (should be 4+): {}".format(wx.__version__))


def wx_call_after(target):

    @functools.wraps(target)
    def wrapper(self, *args, **kwargs):
        args = (self,) + args
        wx.CallAfter(target, *args, **kwargs)

    return wrapper


def submit_to_pool_executor(executor):
    '''Decorates a method to be sumbitted to the passed in executor'''
    def decorator(target):

        @functools.wraps(target)
        def wrapper(*args, **kwargs):
            result = executor.submit(target, *args, **kwargs)
            result.add_done_callback(executor_done_call_back)
            return result
        return wrapper

    return decorator


def executor_done_call_back(future):
    exception = future.exception()
    if exception:
        raise exception


class ImageViewPanel(wx.Panel):
    """ class ImageViewPanel creates a panel with an image on it, inherits wx.Panel """
    def __init__(self, parent):
        self.width = int(848 / 1.5)
        self.height = int(480 / 1.5)
        # self.width = 848
        # self.height = 480
        super(ImageViewPanel, self).__init__(parent, id=-1, size=wx.Size(self.width, self.height))
        self.bitmaps = {}
        self.Bind(wx.EVT_PAINT, self.onPaint)
        self.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
        self.current_camera = None
        self._cvb = CvBridge()

    def updateImage(self, img_msg, camera):
        image = self._cvb.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        stamp = datetime.fromtimestamp(img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9).strftime("%H:%M:%S.%f")
        cv2.putText(image, f"{stamp}", (1, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 164, 32), lineType=cv2.LINE_AA)

        if image.shape[2] == 4:
            alpha = image[:, :, 3].astype(np.uint8)
            image = image[:, :, :3].astype(np.uint8)
            image = wx.Image(image.shape[1], image.shape[0], image.tostring(), alpha.tostring())
        else:
            image = wx.Image(image.shape[1], image.shape[0], image.astype(np.uint8).ravel().tostring())
        self.imgWidth, self.imgHeight = image.GetSize()
        if self.imgWidth > self.width or self.imgHeight > self.height:
            self.ratio = float(self.height) / self.imgHeight
            bitmap = wx.Bitmap(image.Scale(self.imgWidth * self.ratio, self.height))
        else:
            bitmap = wx.Bitmap(image)

        self.bitmaps[camera] = bitmap
        self.Refresh()

    def setCurrentCamera(self, camera):
        self.current_camera = camera

    def onPaint(self, event):
        if self.current_camera is None or self.current_camera not in self.bitmaps:
            return
        bitmap = self.bitmaps[self.current_camera]
        margin = 0
        self.SetSize(wx.Size(self.imgWidth + margin * 2, self.imgHeight + margin * 2))
        dc = wx.AutoBufferedPaintDC(self)
        dc.Clear()
        dc.DrawBitmap(bitmap, margin, margin, useMask=True)

class Colors():

    @classmethod
    def initializeColors(cls):
        colorDB = wx.ColourDatabase()
        cls.OBJ_NORMAL = wx.BLACK
        cls.OBJ_NEW = wx.GREEN
        cls.OBJ_ERROR = colorDB.Find("ORANGE")
        cls.OBJ_WEIRD = colorDB.Find("MAGENTA")
        cls.OBJ_DEAD = wx.RED

        cls.CMD_CURRENT = colorDB.Find("MEDIUM SPRING GREEN")


class Visualizator(wx.Frame):
    TIMER_FREQ = 10 # milliseconds
    UPDATE_FREQ = 200 # milliseconds
    LANGUAGE = 'CZ' #language of the visualization
    COLOR_GRAY = (128, 128, 128)
    CONFIG_PATH = "../config/gui.yaml"

    DEBUG = False

    WIDTH = 1200
    HEIGHT = 800

    IMAGE_TOPIC = "/detections/image_annot"

    def __init__(self):
        super().__init__(None, title="Visualizator", size=(self.WIDTH, self.HEIGHT))
        self.maxed = False

        # >>> Threading craziness
        self.ros_spin_lock = Lock()
        self.gui_update_lock = Lock()

        # >>> Key monitor
        self.keyDaemon = pynput.keyboard.Listener(on_press=self.onKeyDown)
        self.keyDaemon.daemon = True
        self.keyDaemon.start()

        # >>> ROS initialization
        self.node = rclpy.create_node("visualizator")
        self.executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        self.logger = self.node.get_logger()

        # >>> load config
        spec = find_spec("crow_control")
        with open(os.path.join(spec.submodule_search_locations[0], self.CONFIG_PATH), "r") as f:
            self.config = yaml.safe_load(f)
        lang_idx = self.config["languages"].index(self.LANGUAGE)
        self.translator = {f: {k: v[lang_idx] for k, v in t.items()} for f, t in self.config.items() if f in ["field", "option", "info", "tab", "input", "nlp_mode", "action", "button"]}

        # >>> spinning
        self.spinTimer = wx.Timer()
        self.spinTimer.Bind(wx.EVT_TIMER, self.HandleROS)
        self.spinTimer.Start(self.TIMER_FREQ)
        self.updateTimer = wx.Timer()
        self.updateTimer.Bind(wx.EVT_TIMER, self.updateParamAObjects)
        self.updateTimer.Start(self.UPDATE_FREQ)

        # >>> Crowracle client
        self.crowracle = CrowtologyClient(node=self.node)
        self.object_properties = self.crowracle.get_filter_object_properties()
        self.INVERSE_OBJ_MAP = {v["name"]: i for i, v in enumerate(self.object_properties.values())}

        if not self.DEBUG:
            try:
                # >>> Cameras
                calib_client = self.node.create_client(GetParameters, '/calibrator/get_parameters')
                self.logger.info("Waiting for calibrator to setup cameras")
                calib_client.wait_for_service()
                self.image_topics, self.cameras, self.camera_instrinsics, self.camera_frames = [p.string_array_value for p in call_get_parameters(node=self.node, node_name="/calibrator", parameter_names=["image_topics", "camera_namespaces", "camera_intrinsics", "camera_frames"]).values]
                while len(self.cameras) == 0: #wait for cams to come online
                    self.logger.warn("No cams detected, waiting 2s.")
                    time.sleep(2)
                    self.image_topics, self.cameras, self.camera_instrinsics, self.camera_frames = [p.string_array_value for p in call_get_parameters(node=self.node, node_name="/calibrator", parameter_names=["image_topics", "camera_namespaces", "camera_intrinsics", "camera_frames"]).values]

                self.mask_topics = [cam + self.IMAGE_TOPIC for cam in self.cameras] #input masks from 2D rgb (from our detector.py)

                # create listeners
                qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                for i, (cam, maskTopic) in enumerate(zip(self.cameras, self.mask_topics)):
                    self.node.create_subscription(msg_type=msg_image,
                                                  topic=maskTopic,
                                                  callback=lambda img_msg, cam=cam: wx.CallAfter(self.imageView.updateImage, img_msg, cam),
                                                  qos_profile=qos)
                    self.logger.info('Input listener created on topic: "%s"' % maskTopic)
            except BaseException:
                self.cameras = []
        else:
            self.cameras = []

        # >>> Publisher for remove command
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.rem_cmd_publisher = self.node.create_publisher(StampedString, "/nlp/command", qos)
        listener = self.node.create_subscription(msg_type=SentenceProgram,
                                          topic='nl_input',
                                          callback=self.sentence_callback, qos_profile=qos) #the listener QoS has to be =1, "keep last only".

        # >>> Helpers
        self.old_objects = {}
        self.notebook_tab = self.translator["tab"]["camera"]
        self.cmd_notebook_tab = self.translator["tab"]["detection"]

        # >>> Initialize GUI frame
        self.Bind(wx.EVT_CLOSE, self.destroy)

        self.mainVBox = wx.BoxSizer(wx.VERTICAL)

        # Toolbar
        self.toolbar = wx.ToolBar(self, -1)
        self.toolbar.SetToolSeparation(20)
        button = wx.Button(self.toolbar, -1, self.translator["input"]["nlp_suspend"], name="buttHaltNLP")
        self.toolbar.AddControl(button)
        self.toolbar.AddSeparator()
        self.nlp_mode_label = wx.StaticText(self.toolbar, size=wx.Size(200, 20), style=wx.ALIGN_LEFT)
        self.toolbar.AddControl(self.nlp_mode_label)
        self.nlp_mode_slider = wx.Slider(self.toolbar, value=1, minValue=1, maxValue=3, name="nlp_mode_slider")
        # self.nlp_mode_slider.Disable()
        self.toolbar.AddControl(self.nlp_mode_slider)
        self.toolbar.AddSeparator()

        self.toolbar.Realize()
        self.SetToolBar(self.toolbar)
        self.mainVBox.Add(self.toolbar, flag=wx.EXPAND)

        # NL TEXT
        self.text_recognized = wx.StaticText(self, style=wx.ALIGN_CENTER, size=wx.Size(self.WIDTH, 20))
        f = wx.Font()
        f.SetPointSize(14)
        self.text_recognized.SetFont(f)
        self.mainVBox.Add(self.text_recognized, flag=wx.EXPAND)

        # NOTEBOOK
        self.notebook = wx.Notebook(self)
        self.notebook.Bind(wx.EVT_NOTEBOOK_PAGE_CHANGED, self.onNBPageChange)

        # NOTEBOOK - CAMERAS
        nb_page_cam = wx.Panel(self.notebook)

        cameraViewBox = wx.BoxSizer(wx.VERTICAL)
        imageAndControlBox = wx.BoxSizer(wx.HORIZONTAL)
        self.imageView = ImageViewPanel(nb_page_cam)  # Image view
        if self.cameras:
            self.imageView.setCurrentCamera(self.cameras[0])
        imageAndControlBox.Add(self.imageView, flag=wx.EXPAND)

        controlBox = wx.BoxSizer(wx.VERTICAL)
        controlBox.Add(wx.StaticText(nb_page_cam, -1, f'{self.translator["input"]["select_camera"]}:'), flag=wx.EXPAND)
        cameraSelector = wx.Choice(nb_page_cam, choices=self.cameras)
        cameraSelector.Bind(wx.EVT_CHOICE, lambda event: self.imageView.setCurrentCamera(event.GetString()))
        cameraSelector.Select(0)
        controlBox.Add(cameraSelector, flag=wx.EXPAND)
        infoText = wx.StaticText(nb_page_cam, -1, self.translator["info"]["infoText"])
        controlBox.Add(infoText, flag=wx.EXPAND)
        self.button_command_rm = wx.Button(nb_page_cam, label=self.translator["button"]["command_rm"])
        self.button_command_rm.Disable()
        self.button_command_rm.Bind(wx.EVT_BUTTON, self.onCMD_RM_button)
        controlBox.Add(self.button_command_rm, flag=wx.EXPAND)

        imageAndControlBox.Add(controlBox, flag=wx.EXPAND)
        cameraViewBox.Add(imageAndControlBox, flag=wx.EXPAND)

        # COMMAND QUEUE & PARAMS
        self.command_notebook = wx.Notebook(nb_page_cam)
        self.command_notebook.Bind(wx.EVT_NOTEBOOK_PAGE_CHANGED, self.onCMD_NBPageChange)
        # detection grid
        self.cmd_detection_param_grid = wx.grid.Grid(self.command_notebook)
        self.cmd_detection_param_grid.CreateGrid(5, 2)
        self.cmd_detection_param_grid.SetColSize(0, int(self.WIDTH * 0.3))
        self.cmd_detection_param_grid.SetColSize(1, int(self.WIDTH * 0.7))
        self.cmd_detection_param_grid.SetColLabelSize(0)
        self.cmd_detection_param_grid.EnableEditing(False)
        self.cmd_detection_param_grid.EnableDragGridSize(False)
        self.cmd_detection_param_grid.EnableGridLines(False)
        self.cmd_detection_param_grid.EnableHidingColumns(False)
        self.cmd_detection_param_grid.SetRowLabelSize(0)
        f = wx.Font()
        f.SetWeight(wx.FONTWEIGHT_SEMIBOLD)
        self.table_blendin_attr = wx.grid.GridCellAttr(wx.BLACK, wx.SystemSettings.GetColour(wx.SYS_COLOUR_WINDOWFRAME), f, 0, 0)
        self.cmd_detection_param_grid.SetCellValue(0, 0, self.translator["field"]["detected_cmd"])
        self.cmd_detection_param_grid.SetRowAttr(0, self.table_blendin_attr)
        self.cmd_detection_param_grid.SetCellValue(1, 0, self.translator["field"]["detected_object"])
        self.cmd_detection_param_grid.SetRowAttr(1, self.table_blendin_attr)
        self.cmd_detection_param_grid.SetCellValue(2, 0, self.translator["field"]["detected_name"])
        self.cmd_detection_param_grid.SetRowAttr(2, self.table_blendin_attr)
        self.cmd_detection_param_grid.SetCellValue(3, 0, self.translator["field"]["detected_is_present"])
        self.cmd_detection_param_grid.SetRowAttr(3, self.table_blendin_attr)
        self.cmd_detection_param_grid.SetCellValue(4, 0, self.translator["field"]["state"])
        self.cmd_detection_param_grid.SetRowAttr(4, self.table_blendin_attr)
        self.cmd_detection_param_grid.ClearSelection()
        self.command_notebook.AddPage(self.cmd_detection_param_grid, self.translator["tab"]["detection"])
        # queue grid
        self.cmd_queue_grid = wx.grid.Grid(self.command_notebook, size=wx.Size(self.WIDTH, 100))
        self.cmd_queue_grid.SetDefaultRowSize(40)
        self.cmd_queue_grid.Bind(wx.grid.EVT_GRID_CELL_LEFT_CLICK, self.onCmdQueueClick)
        n_cmd_rows = 10
        self.cmd_queue_grid.CreateGrid(n_cmd_rows, 3)
        self.cmd_queue_grid.SetRowLabelSize(0)
        self.cmd_queue_grid.SetColSize(0, int(self.WIDTH * 0.3))
        self.cmd_queue_grid.SetColLabelValue(0, self.translator["field"]["command"])
        self.cmd_queue_grid.SetColSize(1, int(self.WIDTH * 0.55))
        self.cmd_queue_grid.SetColLabelValue(1, self.translator["field"]["info"])
        self.cmd_queue_grid.SetColSize(2, int(self.WIDTH * 0.15))
        self.cmd_queue_grid.SetColLabelValue(2, self.translator["field"]["command_name"])
        self.cmd_queue_grid.EnableEditing(False)
        self.cmd_queue_grid.EnableDragGridSize(False)
        self.cmd_queue_grid.EnableHidingColumns(False)
        self.cmd_queue_grid.ClearSelection()
        f_cmd = wx.Font()
        f_cmd.SetPointSize(22)
        f_cmd_small = wx.Font()
        f_cmd_small.SetPointSize(10)
        self.table_cmd_normal_attr = wx.grid.GridCellAttr(Colors.OBJ_NORMAL, wx.WHITE, f_cmd, wx.ALIGN_CENTRE, wx.ALIGN_CENTRE)
        self.table_cmd_current_attr = wx.grid.GridCellAttr(Colors.OBJ_NORMAL, Colors.CMD_CURRENT, f_cmd, wx.ALIGN_CENTRE, wx.ALIGN_CENTRE)
        self.cmd_queue_grid.SetRowAttr(0, self.table_cmd_current_attr)
        self.cmd_queue_grid.SetCellAlignment(0, 1, wx.ALIGN_LEFT, 0)
        self.cmd_queue_grid.SetCellFont(0, 1, f_cmd_small)
        for i in range(1, n_cmd_rows + 1):
            self.cmd_queue_grid.SetRowAttr(i, self.table_cmd_normal_attr)
            self.cmd_queue_grid.SetCellAlignment(i, 1, wx.ALIGN_LEFT, 0)
            self.cmd_queue_grid.SetCellFont(i, 1, f_cmd_small)

        self.command_notebook.AddPage(self.cmd_queue_grid, self.translator["tab"]["cmd_queue"])

        cameraViewBox.Add(self.command_notebook, flag=wx.EXPAND)

        nb_page_cam.SetSizerAndFit(cameraViewBox)
        self.notebook.AddPage(nb_page_cam, self.translator["tab"]["camera"])

        # NOTEBOOK - OBJECTS
        self.nb_page_obj = wx.grid.Grid(self.notebook)
        self.nb_page_obj.CreateGrid(30, 3)
        self.nb_page_obj.EnableEditing(False)
        self.nb_page_obj.EnableDragGridSize(False)
        self.nb_page_obj.EnableHidingColumns(False)
        self.nb_page_obj.SetColSize(0, int(self.WIDTH * 0.45))
        self.nb_page_obj.SetColLabelValue(0, self.translator["field"]["object"])
        self.nb_page_obj.SetColSize(1, int(self.WIDTH * 0.3))
        self.nb_page_obj.SetColLabelValue(1, self.translator["field"]["location"])
        self.nb_page_obj.SetColSize(2, int(self.WIDTH * 0.25))
        self.nb_page_obj.SetColLabelValue(2, self.translator["field"]["id"])
        self.nb_page_obj.ClearSelection()
        self.table_attr = wx.grid.GridCellAttr(Colors.OBJ_NORMAL, wx.WHITE, wx.Font(), 0, 0)
        self.notebook.AddPage(self.nb_page_obj, self.translator["tab"]["object"])

        # NOTEBOOK - PARAMS
        self.nb_page_param = wx.grid.Grid(self.notebook)
        self.nb_page_param.CreateGrid(30, 2)
        self.nb_page_param.EnableEditing(False)
        self.nb_page_param.EnableDragGridSize(False)
        self.nb_page_param.EnableHidingColumns(False)
        self.nb_page_param.SetColSize(0, int(self.WIDTH * 0.2))
        self.nb_page_param.SetColLabelValue(0, "parameter")
        self.nb_page_param.SetColSize(1, int(self.WIDTH * 0.8))
        self.nb_page_param.SetColLabelValue(1, "value")
        self.nb_page_param.ClearSelection()
        self.notebook.AddPage(self.nb_page_param, self.translator["tab"]["parameter"])
        self.current_parameters = OrderedDict()

        # Adding NOTEBOOK
        self.mainVBox.Add(self.notebook, flag=wx.EXPAND)

        # STATUSBAR
        # self.statbar = self.CreateStatusBar(number=2, style=wx.STB_SHOW_TIPS | wx.STB_ELLIPSIZE_END | wx.FULL_REPAINT_ON_RESIZE)
        self.statbar = wx.StatusBar()
        # self.statbar = wx.StatusBar(style=wx.STB_SHOW_TIPS | wx.STB_ELLIPSIZE_END | wx.FULL_REPAINT_ON_RESIZE)
        self.statbar.Create(self, id=1, name="status bar")
        stat_n_fields = 2
        self.statbar.SetFieldsCount(number=stat_n_fields, widths=[int(self.WIDTH / stat_n_fields) for i in range(stat_n_fields)])
        self.SetStatusBar(self.statbar)
        self.statbar.SetStatusText(f'{self.translator["field"]["silent_mode"]}: -', 0)
        self.statbar.SetStatusText(f'{self.translator["field"]["nlp_suspended"]}: -', 1)
        # self.mainVBox.Add(self.statbar, flag=wx.EXPAND)

        # Finalizing
        self.SetSizerAndFit(self.mainVBox)
        self.ShowWithEffect(wx.SHOW_EFFECT_BLEND)
#        self.vbox.ComputeFittingWindowSize()

        self.Bind(wx.EVT_BUTTON, self.onButton)
        # self.Bind(wx.EVT_TOGGLEBUTTON, self.onToggle)

        # >>> PARAMS
        # from time import sleep
        # sleep(10)
        self.pclient = UniversalParamClient()
        self.pclient.set_callback(self.update_params)
        self.pclient.attach("det_obj", default_value="-")
        self.pclient.attach("det_command", default_value="-")
        self.pclient.attach("det_obj_name", default_value="-")
        self.pclient.attach("det_obj_in_ws", default_value="-")
        self.pclient.attach("status", default_value="-")
        self.pclient.attach("silent_mode", default_value=1, force=True)
        self.pclient.attach("halt_nlp", default_value=False, force=True)
        self.qclient = QueueClient(queue_name="main")
        self.qclient.hook_on_update(self.update_cmd_queue)
        self.qclient.hook_on_pop(self.update_cmd_pop)

        # >>> SET STATUS
        # self.statbar.SetStatusText(f'{self.translator["field"]["silent_mode"]}: {self.translator["option"][str(self.pclient.silent_mode)]}', 0)
        self.statbar.SetStatusText(f'{self.translator["field"]["nlp_suspended"]}: {self.translator["option"][str(self.pclient.halt_nlp)]}', 1)
        wx.CallAfter(self._display_silent_mode)
        # wx.CallAfter(self.update_cmd_queue)
        # wx.CallAfter(self.update_cmd_pop)

    def _display_silent_mode(self):
        wx.CallAfter(self.nlp_mode_label.SetLabel, f'{self.translator["field"]["silent_mode"]}: {self.translator["nlp_mode"][str(self.pclient.silent_mode)]}')
        wx.CallAfter(self.nlp_mode_slider.SetValue, True and self.pclient.silent_mode or 0)
        wx.CallAfter(self.toolbar.Refresh)
        wx.CallAfter(self.nlp_mode_slider.Refresh)
        wx.CallAfter(self.nlp_mode_slider.Update)

    def onButton(self, event):
        obj = event.GetEventObject()
        if type(obj.GetParent()) is wx.ToolBar:
            if obj.Name == 'buttHaltNLP':
                self.toggle_nlp_halting()

    def onCMD_RM_button(self, evt):
        srows = self.cmd_queue_grid.GetSelectedRows()
        if len(srows) != 1:
            self.button_command_rm.Disable()
            return
        row = srows[0]
        cmd_disp_name = self.cmd_queue_grid.GetCellValue(row, 2)
        if row > 0 and cmd_disp_name:
            self.remove_command(cmd_disp_name)
        self.button_command_rm.Disable()
        self.cmd_queue_grid.ClearSelection()

    def update_params(self, param, msg):
        # print(self.pclient.get_all())
        if param in self.current_parameters:
            idx = list(self.current_parameters).index(param)
        else:
            idx = len(self.current_parameters)
            self.nb_page_param.SetCellValue(idx, 0, param)
            self.current_parameters[param] = msg
        self.nb_page_param.SetCellValue(idx, 1, str(msg))

        if "robot_done" in param and msg:
            # print(msg)
            if self.cmd_queue_grid.GetCellValue(0, 0):
                self.cmd_queue_grid.SetCellValue(0, 0, "")
                self.cmd_queue_grid.SetCellValue(0, 1, "")
                self.cmd_queue_grid.SetCellValue(0, 2, "")
        # elif "halt_nlp" in param:
        #     # self.statbar
        #     try:
        #         # self.statbar.GetStatusText(1)
        #         self.statbar.PopStatusText(1)
        #     except BaseException:
        #         pass
        #     try:
        #         self.statbar.PushStatusText(f'{self.translator["field"]["nlp_suspended"]}: {self.translator["option"][str(msg)]}', 1)
        #     except BaseException:
        #         pass
        elif "silent_mode" in param:
            self._display_silent_mode()
            # self.statbar.PopStatusText(0)
            # self.statbar.PushStatusText(f'{self.translator["field"]["silent_mode"]}: {self.translator["option"][str(msg)]}', 0)

    def onCmdQueueClick(self, evt):
        row = evt.GetRow()
        cmd_disp_name = self.cmd_queue_grid.GetCellValue(row, 2)
        # print(cmd_disp_name)
        if cmd_disp_name and row > 0:
            self.cmd_queue_grid.ClearSelection()
            self.cmd_queue_grid.SelectRow(row)
            self.button_command_rm.Enable()

    def onNBPageChange(self, evt):
        self.notebook_tab = self.notebook.GetPageText(evt.Selection)

    def onCMD_NBPageChange(self, evt):
        self.cmd_notebook_tab = self.command_notebook.GetPageText(evt.Selection)

    def onKeyDown(self, key):
        # Pynput is great but captures events when window is not focused. The following condition prevents that.
        if not self.IsActive():
            return

        if type(key) is pynput.keyboard.KeyCode:
            if key.char == "f":
                print(self.GetWindowStyleFlag())
                if self.maxed:
                    # self.SetWindowStyleFlag(wx.BORDER_SIMPLE)
                    self.Hide()
                    self.Show(True)
                else:
                    # self.SetWindowStyleFlag(wx.MAXIMIZE | wx.BORDER_NONE)
                    self.ShowFullScreen(True, style=wx.FULLSCREEN_NOCAPTION | wx.FULLSCREEN_NOBORDER)
                self.maxed = not self.maxed
                self.Refresh()
            elif key.char == "q":
                self.Close()

    @submit_to_pool_executor(thread_pool_executor)
    def HandleROS(self, _=None):
        self.ros_spin_lock.acquire()
        rclpy.spin_once(self.node, executor=self.executor)
        self.ros_spin_lock.release()

    # @submit_to_pool_executor(thread_pool_executor)
    @wx_call_after
    def updateParamAObjects(self, _=None):
        self.gui_update_lock.acquire()
        if self.notebook_tab == self.translator["tab"]["object"]:
            self.refresh_objects()

        if self.cmd_notebook_tab == self.translator["tab"]["detection"]:
            self.refresh_detections()
        # elif self.cmd_notebook_tab == self.translator["tab"]["cmd_queue"]:
        #     self.refresh_commands()
        self.gui_update_lock.release()

    def sentence_callback(self, msg):
        wx.CallAfter(self.text_recognized.SetLabel, str(msg.data))

    def refresh_detections(self):
        self.cmd_detection_param_grid.SetCellValue(0, 1, str(self.pclient.det_command))
        self.cmd_detection_param_grid.SetCellValue(1, 1, str(self.pclient.det_obj))
        self.cmd_detection_param_grid.SetCellValue(2, 1, str(self.pclient.det_obj_name))
        if self.pclient.det_obj_in_ws == "-":
            self.cmd_detection_param_grid.SetCellValue(3, 1, str(self.pclient.det_obj_in_ws))
        else:
            self.cmd_detection_param_grid.SetCellValue(3, 1, self.translator["option"][str(self.pclient.det_obj_in_ws)])
        self.cmd_detection_param_grid.SetCellValue(4, 1, str(self.pclient.status))

    def display_command_in_grid(self, row, data):
        # print("data ", data)
        if len(data) == 3:
            action, action_type, disp_name, kwargs = *data, {}
        else:
            action, action_type, disp_name, kwargs = data
        # print("kwargs ", kwargs)
        if action in self.translator["action"]:
            self.cmd_queue_grid.SetCellValue(row, 0, self.translator["action"][action])
        else:
            self.cmd_queue_grid.SetCellValue(row, 0, str(action))

        infoText = ''
        if "target" in kwargs:
            infoText += f'{self.translator["field"]["target"]} = {str(kwargs["target"])}'
        else:
            infoText += str(action_type)
        if "location" in kwargs:
            sep = "\n" if "target" in kwargs else ""
            infoText += f'{sep}{self.translator["field"]["location"]} = {str(kwargs["location"])}'
        self.cmd_queue_grid.SetCellValue(row, 1, infoText)

        self.cmd_queue_grid.SetCellValue(row, 2, disp_name)

    def update_cmd_queue(self, _=None):
        noUpdates = wx.grid.GridUpdateLocker(self.cmd_queue_grid)  # pauses grid update until this scope is exited
        vcache = self.qclient.last_value_cache
        # print(vcache)
        self.cmd_queue_grid.ClearSelection()
        self.button_command_rm.Disable()
        if self.cmd_queue_grid.GetCellValue(1, 0):
            bkp = []
            if self.cmd_queue_grid.GetCellValue(0, 0):  # backup the current command
                bkp = [self.cmd_queue_grid.GetCellValue(0, i) for i in range(3)]
            self.cmd_queue_grid.ClearGrid()
            if bkp:
                for i, f in enumerate(bkp):
                    self.cmd_queue_grid.SetCellValue(0, i, f)
        if vcache is not None:
            for i, cmd in enumerate(vcache):
                self.display_command_in_grid(i + 1, cmd)

    def update_cmd_pop(self, _=None):
        noUpdates = wx.grid.GridUpdateLocker(self.cmd_queue_grid)  # pauses grid update until this scope is exited
        popped = self.qclient.last_pop_cache
        # print(popped)
        if popped is not None:
            self.display_command_in_grid(0, popped)

    def refresh_objects(self):
        combined_objects = {}
        try:
            new_objects = {}
            for s in self.crowracle.getTangibleObjects_nocls():
                uri = s
                loc, id = "N/A", "N/A"
                try:
                    loc = self.crowracle.get_location_of_obj(s)
                    id = self.crowracle.get_id_of_obj(s)
                except Exception as e:
                    loc = e
                    id = "error"
                finally:
                    new_objects[uri] = (loc, id)

            combined_objects = {**self.old_objects, **new_objects}

        except BaseException as e:
            print(e)
            return

        try:
            noUpdates = wx.grid.GridUpdateLocker(self.nb_page_obj)  # pauses grid update until this scope is exited
            if (self.nb_page_obj.GetCellValue(0, 0) + self.nb_page_obj.GetCellValue(0, 1) + self.nb_page_obj.GetCellValue(0, 2)):
                self.nb_page_obj.ClearGrid()
            if len(combined_objects) == 0:  # quit early if there are no objects
                return

            for i, (uri, (loc, id)) in enumerate(combined_objects.items()):
                attr = self.table_attr.Clone()
                dead = False
                if uri in self.old_objects:
                    if uri not in new_objects:
                        dead = True
                        attr.SetTextColour(Colors.OBJ_DEAD)
                    # else:
                        attr.SetTextColour(Colors.OBJ_NORMAL)
                elif uri in new_objects:
                    attr.SetTextColour(Colors.OBJ_NEW)
                else:
                    attr.SetTextColour(Colors.OBJ_WEIRD)
                if not dead and (id is None or "error" in id):
                    attr.SetTextColour(Colors.OBJ_ERROR)

                self.nb_page_obj.SetRowAttr(i, attr)
                self.nb_page_obj.SetCellValue(i, 0, f"{uri}")
                if not dead:
                    self.nb_page_obj.SetCellValue(i, 1, f"{loc}")
                    self.nb_page_obj.SetCellValue(i, 2, f"{id}")
        except BaseException as e:
            print(e)
        finally:
            self.old_objects = new_objects

    def toggle_nlp_halting(self):
        # print("aaaaa")
        # print(self.pclient.halt_nlp)
        # self.pclient.halt_nlp = True
        self.pclient.halt_nlp = not self.pclient.halt_nlp

    def remove_command(self, disp_name):
        # click on command in GUI, identify its disp_name, call this function remove_command(disp_name)
        data = []
        template = RemoveCommandX()
        template.command_name = disp_name #string
        dict1 = template.get_inputs()
        data.append(dict1)
        actions = json.dumps(data)
        msg = StampedString()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.data = actions

        print(f'GUI will publish {msg.data}')
        self.rem_cmd_publisher.publish(msg)

    def destroy(self, something=None):
        self.spinTimer.Stop()
        self.updateTimer.Stop()
        self.pclient.destroy()
        self.qclient.destroy()
        self.node.destroy_node()
        super().Destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", "-d", action="store_true")
    args, _ = parser.parse_known_args(sys.argv[1:])
    if args.debug:
        Visualizator.IMAGE_TOPIC = "/color/image_raw"
    rclpy.init()
    app = wx.App(False)
    Colors.initializeColors()
    visualizator = Visualizator()
    app.MainLoop()
    try:
        visualizator.destroy()
    except BaseException:
        print("Could not destroy node or frame, probably already destroyed.")
    thread_pool_executor.shutdown()

if __name__ == "__main__":
    main()
