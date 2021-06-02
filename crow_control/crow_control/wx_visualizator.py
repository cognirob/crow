import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.exceptions import ParameterNotDeclaredException
from ros2param.api import call_get_parameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image as msg_image
from crow_msgs.msg import FilteredPose, NlpStatus, ObjectPointcloud
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge
import message_filters
from crow_ontology.crowracle_client import CrowtologyClient
from crow_vision_ros2.utils import ftl_pcl2numpy

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
from crow_control.utils import ParamClient, QueueClient, UniversalParamClient
from concurrent import futures
import functools
from collections import OrderedDict


thread_pool_executor = futures.ThreadPoolExecutor(max_workers=1)
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
        self.width = 848
        self.height = 480
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


class Visualizator(wx.Frame):
    TIMER_FREQ = 10 # milliseconds
    LANGUAGE = 'CZ' #language of the visualization
    COLOR_GRAY = (128, 128, 128)
    CONFIG_PATH = "../config/gui.yaml"

    DEBUG = False

    WIDTH = 1200
    HEIGHT = 800

    def __init__(self):
        super().__init__(None, title="Visualizator", size=(self.WIDTH, self.HEIGHT))
        self.maxed = False
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
        # self.translator = [t for f, t in self.config.items()]
        # self.translator = {f: {k: v[lang_idx] for k, v in t} for f, t in self.config.items()}
        # print(self.translator)

        # >>> spinning
        self.spinTimer = wx.Timer()
        self.spinTimer.Bind(wx.EVT_TIMER, self.HandleROS)
        self.spinTimer.Start(self.TIMER_FREQ)

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

                self.mask_topics = [cam + "/color/image_raw" for cam in self.cameras] #input masks from 2D rgb (from our detector.py)
                # self.mask_topics = [cam + "/detections/image_annot" for cam in self.cameras] #input masks from 2D rgb (from our detector.py)

                self.nlp_topics = ["/nlp/status"] #nlp status (from our sentence_processor.py)
                # response = str(subprocess.check_output("ros2 param get /sentence_processor halt_nlp".split()))
                self.NLP_HALTED = False  #"False" in response

                # create listeners
                qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                for i, (cam, maskTopic) in enumerate(zip(self.cameras, self.mask_topics)):
                    listener = self.node.create_subscription(msg_type=msg_image,
                                                topic=maskTopic,
                                                callback=lambda img_msg, cam=cam: wx.CallAfter(self.imageView.updateImage, img_msg, cam),
                                                qos_profile=qos) #the listener QoS has to be =1, "keep last only".

                    self.logger.info('Input listener created on topic: "%s"' % maskTopic)

                self.node.create_subscription(msg_type=NlpStatus,
                                                    topic=self.nlp_topics[0],
                                                    # we're using the lambda here to pass additional(topic) arg to the listner. Which then calls a different Publisher for relevant topic.
                                                    callback=lambda status_array_msg: self.input_nlp_callback(status_array_msg),
                                                    qos_profile=qos) #the listener QoS has to be =1, "keep last only".

                self.logger.info('Input listener created on topic: "%s"' % self.nlp_topics[0])
            except BaseException:
                self.cameras = []
        else:
            self.cameras = []

        # >>> Helpers
        self.old_objects = {}
        self.notebook_tab = "cameras"

        # >>> Initialize GUI frame
        self.Bind(wx.EVT_CLOSE, self.destroy)

        self.mainVBox = wx.BoxSizer(wx.VERTICAL)

        # Toolbar
        toolbar = wx.ToolBar(self, -1)
        toolbar.SetToolSeparation(20)
        button = wx.Button(toolbar, -1, "Pozastav NLP", name="buttonMaximize")
        # button = wx.Button(toolbar, -1, self.translator["fields"]["nlp_suspend"], name="buttonMaximize")
        toolbar.AddControl(button)
        # toolbar.AddSeparator()
        # button = wx.Button(toolbar, -1, 'Save setup', name="buttonSaveSetup")
        # toolbar.AddControl(button)
        # toolbar.AddSeparator()
        # button = wx.Button(toolbar, -1, 'Load setup', name="buttonLoad")
        # toolbar.AddControl(button)
        # toolbar.AddSeparator()
        # button = wx.Button(toolbar, -1, 'Load last setup', name="buttonLoadLast")
        # toolbar.AddControl(button)
        # toolbar.AddStretchableSpace()
        # button = wx.Button(toolbar, -1, 'Tracking calibration', name="buttonTrackingCalib")
        # toolbar.AddControl(button)
        toolbar.Realize()
        self.SetToolBar(toolbar)

        self.mainVBox.Add(toolbar, flag=wx.EXPAND)

        # NOTEBOOK
        self.notebook = wx.Notebook(self)
        self.notebook.Bind(wx.EVT_NOTEBOOK_PAGE_CHANGED, self.onNBPageChange)

        # NOTEBOOK - CAMERAS
        nb_page1 = wx.Panel(self.notebook)
        imageAndControlBox = wx.BoxSizer(wx.HORIZONTAL)
        self.imageView = ImageViewPanel(nb_page1)  # Image view
        if self.cameras:
            self.imageView.setCurrentCamera(self.cameras[0])
        imageAndControlBox.Add(self.imageView, flag=wx.EXPAND)

        controlBox = wx.BoxSizer(wx.VERTICAL)
        cameraSelector = wx.Choice(nb_page1, choices=self.cameras)
        cameraSelector.Bind(wx.EVT_CHOICE, lambda event: self.imageView.setCurrentCamera(event.GetString()))
        cameraSelector.Select(0)
        controlBox.Add(cameraSelector, flag=wx.EXPAND)
        imageAndControlBox.Add(controlBox, flag=wx.EXPAND)

        nb_page1.SetSizerAndFit(imageAndControlBox)
        self.notebook.AddPage(nb_page1, "cameras")

        # NOTEBOOK - OBJECTS
        self.nb_page_obj = wx.grid.Grid(self.notebook)
        self.nb_page_obj.CreateGrid(30, 3)
        self.nb_page_obj.EnableEditing(False)
        self.nb_page_obj.SetColSize(0, int(self.WIDTH * 0.5))
        self.nb_page_obj.SetColLabelValue(0, "object")
        self.nb_page_obj.SetColSize(1, int(self.WIDTH * 0.2))
        self.nb_page_obj.SetColLabelValue(1, "location")
        self.nb_page_obj.SetColSize(2, int(self.WIDTH * 0.3))
        self.nb_page_obj.SetColLabelValue(2, "id")
        self.table_attr = wx.grid.GridCellAttr(wx.BLACK, wx.WHITE, wx.Font(), 0, 0)
        self.notebook.AddPage(self.nb_page_obj, "objects")

        # NOTEBOOK - PARAMS
        self.nb_page_param = wx.grid.Grid(self.notebook)
        self.nb_page_param.CreateGrid(30, 2)
        self.nb_page_param.EnableEditing(False)
        self.nb_page_param.SetColSize(0, int(self.WIDTH * 0.2))
        self.nb_page_param.SetColLabelValue(0, "parameter")
        self.nb_page_param.SetColSize(1, int(self.WIDTH * 0.8))
        self.nb_page_param.SetColLabelValue(1, "value")
        self.notebook.AddPage(self.nb_page_param, "parameters")
        self.current_parameters = OrderedDict()

        # Adding NOTEBOOK
        self.mainVBox.Add(self.notebook, flag=wx.EXPAND)

        self.SetSizerAndFit(self.mainVBox)
        self.ShowWithEffect(wx.SHOW_EFFECT_BLEND)
#        self.vbox.ComputeFittingWindowSize()

        # self.Bind(wx.EVT_BUTTON, self.onButton)
        # self.Bind(wx.EVT_TOGGLEBUTTON, self.onToggle)

        # >>> PARAMS
        self.pclient = UniversalParamClient()
        self.pclient.set_callback(self.update_params)
        self.pclient.declare("det_obj", default_value="-")
        self.pclient.declare("det_command", default_value="-")
        self.pclient.declare("det_obj_name", default_value="-")
        self.pclient.declare("det_obj_in_ws", default_value="-")
        self.pclient.declare("status", default_value="-")
        self.pclient.declare("halt_nlp", default_value=False)
        # self.params = {'det_obj': '-', 'det_command': '-', 'det_obj_name': '-', 'det_obj_in_ws': '-', 'status': '-'}
        self.qclient = QueueClient(queue_name="commands")

    def update_params(self, param, msg):
        if param in self.current_parameters:
            idx = list(self.current_parameters).index(param)
        else:
            idx = len(self.current_parameters)
            self.nb_page_param.SetCellValue(idx, 0, param)
            self.current_parameters[param] = msg

        self.nb_page_param.SetCellValue(idx, 1, str(msg))

    def onNBPageChange(self, evt):
        self.notebook_tab = self.notebook.GetPageText(evt.Selection)

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
        rclpy.spin_once(self.node, executor=self.executor)
        # print("aaaaa")
        # print(self.pclient.det_obj)
        # print(self.pclient.det_command)
        # print(self.pclient.det_obj_name)
        # print(self.pclient.det_obj_in_ws)
        # print(self.pclient.status)
        # print("bbbb")
        if self.notebook_tab == "objects":
            wx.CallAfter(self.refresh_objects)

        # wx.CallAfter(rclpy.spin_once, self.node, executor=self.executor)
        # pass

    def refresh_objects(self):
        new_objects = {}
        for s in self.crowracle.getTangibleObjects_nocls():
            uri = s
            try:
                loc = self.crowracle.get_location_of_obj(s)
                id = self.crowracle.get_id_of_obj(s)
            except Exception as e:
                loc = e
                id = "error"
            finally:
                new_objects[uri] = (loc, id)

        combined_objects = {**self.old_objects, **new_objects}

        self.nb_page_obj.ClearGrid()
        if len(combined_objects) == 0:
            return

        for i, (uri, (loc, id)) in enumerate(combined_objects.items()):
            attr = self.table_attr.Clone()
            # grid.SetCellTextColour(3, 3, wx.GREEN)
            # grid.SetCellBackgroundColour(3, 3, wx.LIGHT_GREY)
            dead = False
            if uri in self.old_objects:
                if uri not in new_objects:
                    dead = True
                    attr.SetTextColour(wx.RED)
                # else:
                #     attr.SetTextColour(wx.BLACK)
            elif uri in new_objects:
                attr.SetTextColour(wx.GREEN)
            else:
                attr.SetTextColour(wx.MAGENTA)
            if not dead and (id is None or "error" in id):
                attr.SetBackgroundColour(wx.ORANGE)
                # attr.SetTextColour(wx.ORANGE)

            self.nb_page_obj.SetRowAttr(i, attr)
            self.nb_page_obj.SetCellValue(i, 0, f"{uri}")
            if not dead:
                self.nb_page_obj.SetCellValue(i, 1, f"loc: {loc}")
                self.nb_page_obj.SetCellValue(i, 2, f"ID: {id}")


        self.old_objects = new_objects

    def toggle_nlp_halting(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pclient.halt_nlp = not self.pclient.halt_nlp

    # def input_nlp_callback(self, status_array_msg):
    #     if not status_array_msg.det_obj:
    #         self.get_logger().info("No nlp detections. Quitting early.")
    #         return  # no nlp detections received (for some reason)

    #     obj_str = status_array_msg.det_obj
    #     obj_uri = self.crowracle.get_uri_from_str(obj_str)
    #     nlp_name = self.crowracle.get_nlp_from_uri(obj_uri)
    #     if len(nlp_name) > 0:
    #         nlp_name = nlp_name[0]
    #     else:
    #         nlp_name = '-'

    #     if status_array_msg.found_in_ws:
    #         obj_in_ws = 'ano'
    #     else:
    #         obj_in_ws = 'ne'

    #     self.params['det_obj'] = status_array_msg.det_obj
    #     self.params['det_command'] = status_array_msg.det_command
    #     self.params['det_obj_name'] = nlp_name
    #     self.params['det_obj_in_ws'] = obj_in_ws
    #     self.params['status'] = status_array_msg.status
    #     wx.CallAfter(self.update_detection)

    # def _get_obj_color(self, obj_name):
    #     return self.object_properties[self.INVERSE_OBJ_MAP[obj_name]]["color"]

    # def update_detection(self):
    #     if self.LANGUAGE == 'CZ':
    #         scoreScreen = self.__putText(scoreScreen, "Detekovany prikaz: {}".format(self.params['det_command']), (xp, yp*1), color=(255, 255, 255), size=0.5, thickness=1)
    #         scoreScreen = self.__putText(scoreScreen, "Detekovany objekt: {}".format(self.params['det_obj']), (xp, yp*2), color=self.COLOR_GRAY, size=0.5, thickness=1)
    #         scoreScreen = self.__putText(scoreScreen, "Detekovany objekt (jmeno): {}".format(self.params['det_obj_name']), (xp, yp*3), color=(255, 255, 255), size=0.5, thickness=1)
    #         scoreScreen = self.__putText(scoreScreen, "Objekt je na pracovisti: {}".format(self.params['det_obj_in_ws']), (xp, yp*4), color=(255, 255, 255), size=0.5, thickness=1)
    #         scoreScreen = self.__putText(scoreScreen, "Stav: {}".format(self.params['status']), (xp, yp*5 + 10), color=(255, 224, 200), size=0.7, thickness=2)
    #         if self.NLP_HALTED:
    #             scoreScreen = self.__putText(scoreScreen, "STOP", (im_shape[1] - 70, yp), color=(0, 0, 255), size=0.7, thickness=2)

    #         for cam, img in self.cv_image.items():
    #             up_image = np.hstack((self.infoPanel, np.vstack((img, scoreScreen))))
    #             cv2.imshow('Detekce{}'.format(cam), up_image)
    #             key = cv2.waitKey(10) & 0xFF
    #             if key == ord("f"):
    #                 print(cv2.getWindowProperty('Detekce{}'.format(cam), cv2.WND_PROP_FULLSCREEN))
    #                 print(cv2.WINDOW_FULLSCREEN)
    #                 print(cv2.getWindowProperty('Detekce{}'.format(cam), cv2.WND_PROP_FULLSCREEN) == cv2.WINDOW_FULLSCREEN)
    #                 if cv2.getWindowProperty('Detekce{}'.format(cam), cv2.WND_PROP_FULLSCREEN) == cv2.WINDOW_FULLSCREEN:
    #                     cv2.setWindowProperty('Detekce{}'.format(cam), cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_AUTOSIZE)
    #                 else:
    #                     cv2.setWindowProperty('Detekce{}'.format(cam), cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    #             if key == ord(" "):
    #                 self.window_click(cv2.EVENT_LBUTTONDOWN, None, None, None, None)
    #             if key == ord("q"):
    #                 rclpy.utilities.try_shutdown()

    #     else:
    #         scoreScreen = self.__putText(scoreScreen, "Detected this command: {}".format(self.params['det_command']), (xp, yp*1), color=(255, 255, 255), size=0.5, thickness=1)
    #         scoreScreen = self.__putText(scoreScreen, "Detected this object: {}".format(self.params['det_obj']), (xp, yp*2), color=(255, 255, 255), size=0.5, thickness=1)
    #         scoreScreen = self.__putText(scoreScreen, "Detected object name: {}".format(self.params['det_obj_name']), (xp, yp*3), color=(255, 255, 255), size=0.5, thickness=1)
    #         scoreScreen = self.__putText(scoreScreen, "Object in the workspace: {}".format(self.params['det_obj_in_ws']), (xp, yp*4), color=(255, 255, 255), size=0.5, thickness=1)
    #         scoreScreen = self.__putText(scoreScreen, "Status: {}".format(self.params['status']), (xp, yp*5), color=(255, 255, 255), size=0.5, thickness=1)

    #         for cam, img in self.cv_image.items():
    #             up_image = np.hstack((self.infoPanel, np.vstack((img, scoreScreen))))
    #             cv2.imshow('Detections{}'.format(cam), up_image)
    #             cv2.waitKey(10)

    def destroy(self, something=None):
        self.spinTimer.Stop()
        try:
            super().Destroy()
            self.node.destroy_node()
        except BaseException:
            self.logger.warn("Could not destroy node or frame, probably already destroyed.")


def main():
    rclpy.init()
    app = wx.App(False)
    visualizator = Visualizator()
    app.MainLoop()
    visualizator.destroy()

if __name__ == "__main__":
    main()