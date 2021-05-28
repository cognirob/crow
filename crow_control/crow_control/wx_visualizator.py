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
from pyquaternion import Quaternion
from unicodedata import normalize
import subprocess
import wx
from datetime import datetime
import yaml
import os
from importlib.util import find_spec
import pynput
from crow_control.utils import ParamClient, QueueClient


print("WX Version (should be 4+): {}".format(wx.__version__))


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


class VizGUI(wx.Frame):

    def __init__(self):
        super().__init__(None, title="Visualizator", size=(1024, 768))
        self.maxed = False
        self.keyDaemon = pynput.keyboard.Listener(on_press=self.onKeyDown)
        self.keyDaemon.daemon = True
        self.keyDaemon.start()

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


class Visualizator(wx.App):
    TIMER_FREQ = 10 # milliseconds
    LANGUAGE = 'CZ' #language of the visualization
    COLOR_GRAY = (128, 128, 128)
    CONFIG_PATH = "config/gui.yaml"

    DEBUG = True

    def OnInit(self):
        super().OnInit()

        # >>> ROS initialization
        self.node = rclpy.create_node("visualizator")
        self.executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        self.logger = self.node.get_logger()

        # >>> load config
        spec = find_spec("crow_control")
        self.config = yaml.safe_load(os.path.join(spec.submodule_search_locations[0], "config", self.CONFIG_PATH))

        # >>> spinning
        self.spinTimer = wx.Timer()
        self.spinTimer.Bind(wx.EVT_TIMER, self.HandleROS)
        self.spinTimer.Start(self.TIMER_FREQ)

        # >>> Crowracle client
        self.crowracle = CrowtologyClient(node=self.node)
        self.object_properties = self.crowracle.get_filter_object_properties()
        self.INVERSE_OBJ_MAP = {v["name"]: i for i, v in enumerate(self.object_properties.values())}

        # >>> params
        self.pclient = ParamClient()
        self.pclient.declare("det_obj", default_value="-")
        self.pclient.declare("det_command", default_value="-")
        self.pclient.declare("det_obj_name", default_value="-")
        self.pclient.declare("det_obj_in_ws", default_value="-")
        self.pclient.declare("status", default_value="-")
        self.pclient.declare("halt_nlp", default_value=False)
        # self.params = {'det_obj': '-', 'det_command': '-', 'det_obj_name': '-', 'det_obj_in_ws': '-', 'status': '-'}
        self.qclient = QueueClient(queue_name="commands")

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

        # >>> Initialize GUI frame
        self.frame = VizGUI()
        self.frame.Bind(wx.EVT_CLOSE, self.destroy)

        self.mainVBox = wx.BoxSizer(wx.VERTICAL)

        # Toolbar
        toolbar = wx.ToolBar(self.frame, -1)
        toolbar.SetToolSeparation(20)
        button = wx.Button(toolbar, -1, 'Maximize', name="buttonMaximize")
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
        self.frame.SetToolBar(toolbar)
        self.mainVBox.Add(toolbar, flag=wx.EXPAND)

        # self.box = wx.GridBagSizer(vgap=5, hgap=10)
        # self.mainVBox.Add(self.box, flag=wx.EXPAND)
        imageAndControlBox = wx.BoxSizer(wx.HORIZONTAL)
        self.mainVBox.Add(imageAndControlBox, flag=wx.EXPAND)

        # Image view
        self.imageView = ImageViewPanel(self.frame)
        if self.cameras:
            self.imageView.setCurrentCamera(self.cameras[0])
        imageAndControlBox.Add(self.imageView, flag=wx.EXPAND)

        controlBox = wx.BoxSizer(wx.VERTICAL)
        cameraSelector = wx.Choice(self.frame, choices=self.cameras)
        cameraSelector.Bind(wx.EVT_CHOICE, lambda event: self.imageView.setCurrentCamera(event.GetString()))
        cameraSelector.Select(0)

        controlBox.Add(cameraSelector, flag=wx.EXPAND)
        imageAndControlBox.Add(controlBox, flag=wx.EXPAND)

        self.frame.SetSizerAndFit(self.mainVBox)
        self.frame.ShowWithEffect(wx.SHOW_EFFECT_BLEND)
#        self.vbox.ComputeFittingWindowSize()

        # self.frame.Bind(wx.EVT_BUTTON, self.onButton)
        # self.frame.Bind(wx.EVT_TOGGLEBUTTON, self.onToggle)

        return True

    def HandleROS(self, _=None):
        wx.CallAfter(rclpy.spin_once, self.node, executor=self.executor)

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
        self.ExitMainLoop()
        try:
            self.frame.Destroy()
            self.node.destroy_node()
        except BaseException:
            self.logger.warn("Could not destroy node or frame, probably already destroyed.")


def main():
    rclpy.init()
    visualizator = Visualizator()
    visualizator.MainLoop()
    visualizator.destroy()

if __name__ == "__main__":
    main()