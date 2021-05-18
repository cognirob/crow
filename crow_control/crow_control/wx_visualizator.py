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


print("WX Version (should be 4+): {}".format(wx.__version__))


class ImageViewPanel(wx.Panel):
    """ class ImageViewPanel creates a panel with an image on it, inherits wx.Panel """
    def __init__(self, parent, sampleCallback=None):
        self.width = 1024
        self.height = 768
        super(ImageViewPanel, self).__init__(parent, id=-1, size=wx.Size(self.width, self.height))
        self.bitmap = None
        self.Bind(wx.EVT_PAINT, self.onPaint)
        self.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)

        self.Bind(wx.EVT_LEFT_DCLICK, self.onClick)
        self.Bind(wx.EVT_LEFT_DOWN, self.onMouseDown)
        self.Bind(wx.EVT_LEFT_UP, self.onMouseUp)
        self.Bind(wx.EVT_MOTION, self.onMouseMove)

        self.mouseIsDown = False
        self.trackingStart = None
        self.trackingStop = None
        self.selection = None
        self.sampleCallback = sampleCallback
        self.brush = wx.Brush(wx.Colour(200, 200, 255, 128), wx.BRUSHSTYLE_FDIAGONAL_HATCH)

    def onClick(self, event):
        if not self.sampleCallback:
            return
        pos = event.GetPosition()
        if self.bitmap:
            rect = wx.Rect(pos.x, pos.y, 10, 10)
            sample = np.asarray(self.bitmap.GetSubBitmap(rect).ConvertToImage().GetDataBuffer(), dtype=np.uint8).reshape((-1, 1, 3))
            wx.CallAfter(self.sendSample, sample, rect)

    def onMouseDown(self, event):
        if not self.sampleCallback:
            return
        self.mouseIsDown = True
        self.trackingStart = event.GetPosition()

    def onMouseUp(self, event):
        if not (self.sampleCallback and self.trackingStart and self.trackingStop):
            return
        self.mouseIsDown = False
        a, b = (self.trackingStart, self.trackingStop) if self.trackingStart < self.trackingStop else (self.trackingStop, self.trackingStart)
        rect = wx.Rect(a, b)
        size = rect.GetSize()
        # Send the samples only if the selection area is bigger than 64 pixels
        if size[0] * size[1] < 16:
            self.selection = None
        else:
            self.selection = rect
            self.Refresh()

    def onMouseMove(self, event):
        if not (self.sampleCallback and self.mouseIsDown):
            return
        self.trackingStop = event.GetPosition()
        a, b = (self.trackingStart, self.trackingStop) if self.trackingStart < self.trackingStop else (self.trackingStop, self.trackingStart)
        self.selection = wx.Rect(a, b)
        self.Refresh()

    def update(self, image):
        if image.shape[2] == 4:
            alpha = image[:, :, 3].astype(np.uint8)
            image = image[:, :, :3].astype(np.uint8)
            image = wx.Image(image.shape[1], image.shape[0], image.tostring(), alpha.tostring())
        else:
            # print(image.astype(np.uint8).shape)
            # print(dir(wx.Image))
            # print(dirwx.Image.__module__)
            image = wx.Image(image.shape[1], image.shape[0], image.astype(np.uint8).ravel().tostring())
            # image = wx.Image(name="RGB imae", width=image.shape[1], height=image.shape[0], data=image.astype(np.uint8).tostring())
            # image = wx.Image(image.shape[1], image.shape[0], image.astype(np.uint8).tostring())
        self.imgWidth, self.imgHeight = image.GetSize()
        if self.imgWidth > self.width or self.imgHeight > self.height:
            self.ratio = float(self.height) / self.imgHeight
            self.bitmap = wx.Bitmap(image.Scale(self.imgWidth * self.ratio, self.height))
        else:
            self.bitmap = wx.Bitmap(image)
        self.Refresh()

    def backmap(self, x, y):
        return x * self.ratio, y * self.ratio

    def sendSample(self, sample, rect):
        invRatio = 1 / self.ratio
        ogrid = np.ogrid[int(np.round(rect.Top * invRatio)):int(np.round(rect.Bottom * invRatio)), int(np.round(rect.Left * invRatio)):int(np.round(rect.Right * invRatio)), 0:3]
        self.sampleCallback(sample, ogrid)

    def onPaint(self, event):
        if self.bitmap:
            # img_w, img_h = self.bitmap.GetSize()
            margin = 0
            self.SetSize(wx.Size(self.imgWidth + margin * 2, self.imgHeight + margin * 2))
            dc = wx.AutoBufferedPaintDC(self)
            dc.Clear()
            dc.DrawBitmap(self.bitmap, margin, margin, useMask=True)
            if self.sampleCallback and self.selection:
                dc.SetBrush(self.brush)
                dc.DrawRectangle(self.selection)
                sample = np.asarray(self.bitmap.GetSubBitmap(self.selection).ConvertToImage().GetDataBuffer(),
                                    dtype=np.uint8).reshape((-1, 1, 3))
                # If mouse is up and something is still selected, send the sample
                if not self.mouseIsDown:
                    # opcvsamp = np.asarray(self.bitmap.GetSubBitmap(self.selection).ConvertToImage().GetDataBuffer()).reshape((self.selection.Height, self.selection.Width, 3))
                    # print(self.selection.GetSize())
                    # print(opcvsamp.shape)
                    # wx.CallAfter(cv2.imshow, 'sample', opcvsamp)
                    wx.CallAfter(self.sendSample, sample, self.selection)
                    self.selection = None


class VizGUI(wx.Frame):
    pass




class Visualizator(wx.App):
    TIMER_FREQ = .5 # seconds
    LANGUAGE = 'CZ' #language of the visualization
    COLOR_GRAY = (128, 128, 128)

    def OnInit(self):
        super().OnInit()

        self.frame = wx.Frame(None, title="Visualizator", size=(1024, 768))
        self.frame.Show()

        self.node = rclpy.create_node("visualizator")
        self.logger = self.node.get_logger()

        self.processor_state_srv = self.node.create_client(GetParameters, '/sentence_processor/get_parameters')
        self.crowracle = CrowtologyClient(node=self.node)
        self.object_properties = self.crowracle.get_filter_object_properties()
        self.INVERSE_OBJ_MAP = {v["name"]: i for i, v in enumerate(self.object_properties.values())}

        calib_client = self.node.create_client(GetParameters, '/calibrator/get_parameters')
        self.logger.info("Waiting for calibrator to setup cameras")
        calib_client.wait_for_service()
        self.image_topics, self.cameras, self.camera_instrinsics, self.camera_frames = [p.string_array_value for p in call_get_parameters(node=self.node, node_name="/calibrator", parameter_names=["image_topics", "camera_namespaces", "camera_intrinsics", "camera_frames"]).values]
        while len(self.cameras) == 0: #wait for cams to come online
            self.logger.warn("No cams detected, waiting 2s.")
            time.sleep(2)
            self.image_topics, self.cameras, self.camera_instrinsics, self.camera_frames = [p.string_array_value for p in call_get_parameters(node=self.node, node_name="/calibrator", parameter_names=["image_topics", "camera_namespaces", "camera_intrinsics", "camera_frames"]).values]
        self.mask_topics = [cam + "/detections/image_annot" for cam in self.cameras] #input masks from 2D rgb (from our detector.py)
        self.filter_topics = ["filtered_poses"] #input masks from 2D rgb (from our detector.py)
        self.nlp_topics = ["/nlp/status"] #nlp status (from our sentence_processor.py)
        self.cvb_ = CvBridge()
        self.cv_image = {} # initialize dict of images, each one for one camera
        # response = str(subprocess.check_output("ros2 param get /sentence_processor halt_nlp".split()))
        self.NLP_HALTED = False  #"False" in response

        self.infoPanel = None
        #create listeners
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        for i, (cam, maskTopic) in enumerate(zip(self.cameras, self.mask_topics)):
            listener = self.node.create_subscription(msg_type=msg_image,
                                          topic=maskTopic,
                                          # we're using the lambda here to pass additional(topic) arg to the listner. Which then calls a different Publisher for relevant topic.
                                          callback=lambda img_array_msg, cam=cam: self.input_detector_callback(img_array_msg, cam),
                                          qos_profile=qos) #the listener QoS has to be =1, "keep last only".

            self.logger.info('Input listener created on topic: "%s"' % maskTopic)

        self.node.create_subscription(msg_type=NlpStatus,
                                            topic=self.nlp_topics[0],
                                            # we're using the lambda here to pass additional(topic) arg to the listner. Which then calls a different Publisher for relevant topic.
                                            callback=lambda status_array_msg: self.input_nlp_callback(status_array_msg),
                                            qos_profile=qos) #the listener QoS has to be =1, "keep last only".

        self.logger.info('Input listener created on topic: "%s"' % self.nlp_topics[0])

        # Initialize nlp params for info bellow image
        self.params = {'det_obj': '-', 'det_command': '-', 'det_obj_name': '-', 'det_obj_in_ws': '-', 'status': '-'}
        return True

    def MainLoop(self):
        super().MainLoop()
        rclpy.spin_once(self.node)

    def window_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            response = str(subprocess.check_output("ros2 param get /sentence_processor halt_nlp".split()))
            if "False" in response:
                subprocess.run("ros2 param set /sentence_processor halt_nlp True".split())
                self.NLP_HALTED = True
            else:
                subprocess.run("ros2 param set /sentence_processor halt_nlp False".split())
                self.NLP_HALTED = False

    def input_detector_callback(self, img_array_msg, cam):
        if not img_array_msg.data:
            self.get_logger().info("No image. Quitting early.")
            return  # no annotated image received (for some reason)
        self.cv_image['{}'.format(cam)] = self.cvb_.imgmsg_to_cv2(img_array_msg, desired_encoding='bgr8')
        self.update_annot_image()

    def input_nlp_callback(self, status_array_msg):
        if not status_array_msg.det_obj:
            self.get_logger().info("No nlp detections. Quitting early.")
            return  # no nlp detections received (for some reason)

        obj_str = status_array_msg.det_obj
        obj_uri = self.crowracle.get_uri_from_str(obj_str)
        nlp_name = self.crowracle.get_nlp_from_uri(obj_uri)
        if len(nlp_name) > 0:
            nlp_name = nlp_name[0]
        else:
            nlp_name = '-'

        if status_array_msg.found_in_ws:
            obj_in_ws = 'ano'
        else:
            obj_in_ws = 'ne'

        self.params['det_obj'] = status_array_msg.det_obj
        self.params['det_command'] = status_array_msg.det_command
        self.params['det_obj_name'] = nlp_name
        self.params['det_obj_in_ws'] = obj_in_ws
        self.params['status'] = status_array_msg.status
        self.update_annot_image()

    def _get_obj_color(self, obj_name):
        return self.object_properties[self.INVERSE_OBJ_MAP[obj_name]]["color"]

    def update_annot_image(self):
        xp = 5
        yp = 20
        scHeight = 128
        im_shape = next(iter(self.cv_image.values())).shape
        scoreScreen = np.zeros((scHeight, im_shape[1], 3), dtype=np.uint8)
        if self.infoPanel is None:
            self.infoPanel = np.zeros((im_shape[0] + scHeight, 256, 3), dtype=np.uint8)
            self.infoPanel = self.__putText(self.infoPanel, "Prikazy:", (xp, yp*1), color=self.COLOR_GRAY, size=0.5, thickness=1)
            self.infoPanel = self.__putText(self.infoPanel, "Ukaz na <OBJEKT>", (xp, yp*2), color=self.COLOR_GRAY, size=0.5, thickness=1)
            self.infoPanel = self.__putText(self.infoPanel, "Ukaz na <BARVA> <OBJEKT>", (xp, yp*3), color=self.COLOR_GRAY, size=0.5, thickness=1)
            self.infoPanel = self.__putText(self.infoPanel, "Seber <OBJEKT>", (xp, yp*4), color=self.COLOR_GRAY, size=0.5, thickness=1)

        if self.LANGUAGE == 'CZ':
            scoreScreen = self.__putText(scoreScreen, "Detekovany prikaz: {}".format(self.params['det_command']), (xp, yp*1), color=(255, 255, 255), size=0.5, thickness=1)
            scoreScreen = self.__putText(scoreScreen, "Detekovany objekt: {}".format(self.params['det_obj']), (xp, yp*2), color=self.COLOR_GRAY, size=0.5, thickness=1)
            scoreScreen = self.__putText(scoreScreen, "Detekovany objekt (jmeno): {}".format(self.params['det_obj_name']), (xp, yp*3), color=(255, 255, 255), size=0.5, thickness=1)
            scoreScreen = self.__putText(scoreScreen, "Objekt je na pracovisti: {}".format(self.params['det_obj_in_ws']), (xp, yp*4), color=(255, 255, 255), size=0.5, thickness=1)
            scoreScreen = self.__putText(scoreScreen, "Stav: {}".format(self.params['status']), (xp, yp*5 + 10), color=(255, 224, 200), size=0.7, thickness=2)
            if self.NLP_HALTED:
                scoreScreen = self.__putText(scoreScreen, "STOP", (im_shape[1] - 70, yp), color=(0, 0, 255), size=0.7, thickness=2)

            for cam, img in self.cv_image.items():
                up_image = np.hstack((self.infoPanel, np.vstack((img, scoreScreen))))
                cv2.imshow('Detekce{}'.format(cam), up_image)
                key = cv2.waitKey(10) & 0xFF
                if key == ord("f"):
                    print(cv2.getWindowProperty('Detekce{}'.format(cam), cv2.WND_PROP_FULLSCREEN))
                    print(cv2.WINDOW_FULLSCREEN)
                    print(cv2.getWindowProperty('Detekce{}'.format(cam), cv2.WND_PROP_FULLSCREEN) == cv2.WINDOW_FULLSCREEN)
                    if cv2.getWindowProperty('Detekce{}'.format(cam), cv2.WND_PROP_FULLSCREEN) == cv2.WINDOW_FULLSCREEN:
                        cv2.setWindowProperty('Detekce{}'.format(cam), cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_AUTOSIZE)
                    else:
                        cv2.setWindowProperty('Detekce{}'.format(cam), cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                if key == ord(" "):
                    self.window_click(cv2.EVENT_LBUTTONDOWN, None, None, None, None)
                if key == ord("q"):
                    rclpy.utilities.try_shutdown()

        else:
            scoreScreen = self.__putText(scoreScreen, "Detected this command: {}".format(self.params['det_command']), (xp, yp*1), color=(255, 255, 255), size=0.5, thickness=1)
            scoreScreen = self.__putText(scoreScreen, "Detected this object: {}".format(self.params['det_obj']), (xp, yp*2), color=(255, 255, 255), size=0.5, thickness=1)
            scoreScreen = self.__putText(scoreScreen, "Detected object name: {}".format(self.params['det_obj_name']), (xp, yp*3), color=(255, 255, 255), size=0.5, thickness=1)
            scoreScreen = self.__putText(scoreScreen, "Object in the workspace: {}".format(self.params['det_obj_in_ws']), (xp, yp*4), color=(255, 255, 255), size=0.5, thickness=1)
            scoreScreen = self.__putText(scoreScreen, "Status: {}".format(self.params['status']), (xp, yp*5), color=(255, 255, 255), size=0.5, thickness=1)

            for cam, img in self.cv_image.items():
                up_image = np.hstack((self.infoPanel, np.vstack((img, scoreScreen))))
                cv2.imshow('Detections{}'.format(cam), up_image)
                cv2.waitKey(10)

    def destroy(self):
        self.node.destroy_node()


def main():
    rclpy.init()
    #time.sleep(5)
    visualizator = Visualizator()
    # rclpy.spin(visualizator)
    visualizator.MainLoop()
    visualizator.destroy()

if __name__ == "__main__":
    main()