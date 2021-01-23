# x saves the background image of the frame(the 10th frame from the beginning), whish is used to for image substraction with the current image to find the newly introdues objects in the field of view

#img_raw_color is the color image stream form the ros realsense node

#img_raw is the grayscale of the img_raw_color for faster processing

#img2 is the int16 concersion of the img_raw

#th is the threshold matrix with same size as of the img_raw used for comparison with the np.where() to find out the parts of the images with pixels crossing the set thresold values providing a smoother image with just black(background) and relevant parts(new object) of the image so that findcountours runs easier an faster.


import rclpy #add to package.xml deps
from rclpy.node import Node
import sensor_msgs
import std_msgs
from cv_bridge import CvBridge
import cv2
import numpy as np
import imutils
from imutils import perspective
from imutils import contours
from scipy.spatial import distance as dist
import pandas as pd
#import matplotlib.pyplot as plt

from numba import jit

import time
import cProfile
import io
import pstats

def profile(func):
    def wrapper(*args, **kwargs):
        pr = cProfile.Profile()
        pr.enable()
        retval = func(*args, **kwargs)
        pr.disable()
        s = io.StringIO()
        sortby = 'cumulative'
        ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
        ps.print_stats()
        print(s.getvalue())
        return retval

    return wrapper


#import commentjson as json
import pkg_resources
import argparse

from std_msgs.msg import String
from sensor_msgs.msg import Image
from custom_batch.msg import Batch


#z = np.uint8(np.full((1080,1920), 255))
#white = np.full((1080,1920), 255)
#white = white.astype('uint8')



def nothing(x):
    pass

#Resized resolution:
size = (480, 640)
#BG image variable:
x = np.int16(np.zeros((size[0],size[1],3)))
y = 0
t1=t2=0
n = 0




#@profile
class CrowVision(Node):

  def __init__(self):

    super().__init__('crow_detector')
    self.ros = {}
    ## handle multiple inputs (cameras).
    # store the ROS Listeners,Publishers in a dict{}, keys by topic.

    prefix = 'camera1'

    # create Input listener with raw images
    topic = 'color/image_raw'
    camera_topic = prefix + "/" + topic
    listener = self.create_subscription(msg_type=sensor_msgs.msg.Image,
                                          topic=camera_topic,
                                          # we're using the lambda here to pass additional(topic) arg to the listner. Which then calls a different Publisher for relevant topic.
                                          callback=lambda msg, topic=camera_topic: self.input_callback(msg, topic),
                                          qos_profile=1) #the listener QoS has to be =1, "keep last only".

    self.get_logger().info('Input listener created on topic: "%s"' % camera_topic)
    self.ros[camera_topic] = {} # camera_topic is used as an ID for this input, all I/O listeners,publishers will be based under that id.
    self.ros[camera_topic]["listener"] = listener

    self.cvb_ = CvBridge()

      #TODO others publishers

    #self.publisher_ = self.create_publisher(Image, 'Cropped', 10)
    self.publisher_ = self.create_publisher(Batch, 'Cropped', 10) #### change here
    timer_period = 0.5  # seconds
    #self.timer = self.create_timer(timer_period, self.timer_callback)
    #self.i = 0


  #@profile
  def timer_callbackX(self):
      msg = string()
      msg.data = 'Lets ROS: %d' % self.i
      self.publisher_.publish(msg)
      self.get_logger().info('Publishing: "%s"' % msg.data)       
      self.i += 1


  #@profile
  def cropp(img, a, b, w, h, r, gp):
          if h>w:
            xx = int((((h + 2*gp) / r) - w )/ 2)
  
            try:
              cropped = img[b : b+h+2*gp , a: a+w+2*xx]

            except :
              cropped = img[b-gp : b+h+gp , a-xx : a+w+xx]
  
          else :
            yy = int((((w + 2*gp) * r) - h )/ 2)
  
            try :
              cropped = img[b : b+h+2*yy , a : a+w+2*gp]

            except :
              cropped = img[b-yy : b+h+yy , a-gp : a+w+gp]

          return cropped

  #@staticmethod
  #@jit(nopython=True, parallel=False)
  def numm(img_raw_Color):
    size = (480, 640)
    img_raw = cv2.resize(img_raw_Color, (size[1],size[0]), interpolation = cv2.INTER_AREA)
    return img_raw

  @profile
  #@staticmethod
  #@jit(nopython=True, parallel=False)
  def mainfunn(img_raw_Color):

    #Global variables
    global n, x, y, t1, t2 , size

    lists = []

    n = n+1

    #cropped = np.uint8(np.zeros((1,1,3)))

    # 30 is the threshold for logic gates
    th = np.full((size[0], size[1], 3), 30)

    img_raw = cv2.resize(img_raw_Color, (size[1],size[0]), interpolation = cv2.INTER_AREA)
    #print('resized: ', img_raw.dtype, img_raw)
    #img_raw = np.uint8(np.resize(img_raw_Color, (480, 640,3)))
    #print('resized: ', img_raw.dtype, img_raw)
    #cv2.imshow('img_raw', img_raw)
    #img_raw = CrowVision.numm(img_raw_Color)


    ## img2 is x2 is defined and set to int16 format to be able to be utilized for subtraction in the integer number line    global x, y, t1, t2 , n
    img2 = np.int16(img_raw)

    ## Substraction for initial background substraction and increasing contrast of the substracted img
    img_sub = cv2.absdiff(img2, x)

    ## Masking out the object
    #bg_sub1= np.where(img_sub>th, img_raw, 0)
    bg_sub1 = img_sub>th#[..., 1:]
    bg_sub1 = np.logical_or(np.logical_or(bg_sub1[..., 0], bg_sub1[..., 1]), bg_sub1[..., 2])
    bg_sub1 = np.dstack((bg_sub1, bg_sub1, bg_sub1))
    #bg_sub1 = bg_sub1.astype(np.uint8) * white
    bg_sub1 = bg_sub1.astype(np.uint8) * img_raw
    bg_sub1 = cv2.cvtColor(bg_sub1, cv2.COLOR_BGR2GRAY)

    bg_sub1 = cv2.medianBlur(bg_sub1, 3)
    cv2.imshow('Background sub mask', bg_sub1)

    #saving background image in X
    if y < 11:
      x = img2
      y=y+1
      return 0
    #else:
      #xy = (img_sub<th)
      #xy = np.logical_or(np.logical_or(xy[..., 0], xy[..., 1]), xy[..., 2])
      #xy = np.dstack((xy, xy, xy))
      #yy = ((np.add(img2,x))* 0.5).astype(np.int16)
      #x= np.where(xy, yy, x)
      #x = xy * img2
      #y=y+1  
    #cv2.imshow('X', np.uint8(x))


    #summ = numm(bg_sub1)
    summ = np.sum(np.sum(bg_sub1,axis = 0))
    print('summ', summ)
 
    if summ < 1000:
      t2=time.time()
      print('Breaking loop due to no significat detection, dT=',t2-t1)
      key = cv2.waitKey(1)
      #Press esc or 'q' to close the image window
      if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
      return 0

    # Extracting contours out from the background substracted image
    contours = cv2.findContours(bg_sub1, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    # Extracting contours out of the Edged image
    # contours = cv2.findContours(edged1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    i = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        #print('area', area)

        if area>800:
          print('detection: ', i+1 , 'area:', area)
          a,b,w,h = cv2.boundingRect(contour)
          # x coord=a, y coord=b
          
          print(img_raw_Color.shape,x.shape)
          ww= (img_raw_Color.shape[1]/x.shape[1])
          hh= (img_raw_Color.shape[0]/x.shape[0])

          #xcood= int(a*ww)
          #ycood= int(b*hh)
          #width= int(w*ww)
          #height= int(h*hh)

          try:
            coord= np.append(coord, [[int(a*ww),int(b*hh),int(w*ww),int(h*hh)]], axis=0)
          except:
            coord= np.array([[int(a*ww),int(b*hh),int(w*ww),int(h*hh)]])


          # fixed ratio: H/W = r
          # r = 1 # we can change as per requirements; r=H/W
          # gap=10
          cropped= CrowVision.cropp(img_raw_Color, int(a*ww), int(b*hh), int(w*ww), int(h*hh), 1, 10)

          #### custom message
          my_msg = Batch()
          my_msg.data[i] = self.bridge.cv2_to_imgmsg(np.array(cropped))


          #print("showing cropped image", i)
          cv2.imshow('Cropped image', cropped)
          i = i+1

    try:
      self.publisher_.publish(my_msg) ## custom message
      self.get_logger().info('Publishing a batch of images')
    except:
      print('Could not publish data')

    print("Total objects found", i)

    if i>0:
      print("coordinate: ",coord.dtype, coord.shape, coord)


    try:
      return cropped
    except:
      return 0


  #@profile
  def input_callback(self, msg, topic):
    """
    @param msg - ROS msg (Image data) to be processed. From camera
    @param topic - str, from camera/input on given topic.
    @return nothing, but send new message(s) via output Publishers.
    """
    global t1, t2 , n, x
    #t1 is the initial runtime for finding Execution time of this section
    t1=time.time()

    self.get_logger().info("I heard: {} for topic {}".format(str(msg.height), topic))
    assert topic in self.ros, "We don't have registered listener for the topic {} !".format(topic)
    

    #Image data from the realsense camera is translated into Numpy array format using CvBridge()
    img_raw_Color = self.cvb_.imgmsg_to_cv2(msg)


    crop = np.array(CrowVision.mainfunn(img_raw_Color))
    #print('contour lenght:', len(contours))
    #l=len(contours)

    #Publishing
    #pub_msg = crop


    # Publishing:
    #self.publisher_.publish(pub_msg)
    #self.publisher_.publish(self.cvb_.cv2_to_imgmsg(np.array(pub_msg)))
    #self.get_logger().info('Publishing:')#"%s"' % pub_msg.data)       

    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
    	cv2.destroyAllWindows()

    #t2 is the final runtime for finding Execution time of this section
    t2=time.time()
    print('dT=',t2-t1)



def main(args=None):
  rclpy.init(args=args)
  
  crow_detector=CrowVision()
  #rclpy.spin_once(crow_detector)
  rclpy.spin(crow_detector)

  cv2.destroyAllWindows()
  rclpy.shutdown()


if __name__ == '__main__':
    main()

