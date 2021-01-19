# Distingstion to detector1try2-2.py
#:: PREPROCESSING in GRAYSCALE to speedup, implementing masking and np.where() to substract background and then shadow_out for shadow(lighter ones and not the strong ones) removal
#:: Also Publishes the cropped image on a topic : 'Cropped'
#RESULTS: 3x faster, dt~(20ms-30ms)


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



#z = np.uint8(np.full((1080,1920), 255))
#white = np.full((1080,1920), 255)
#white = white.astype('uint8')



def nothing(x):
    pass

x = np.int16(np.zeros((1080,1920)))
y = 0
t1=t2=0
n = 0

#@staticmethod
@jit#(nopython=True, parallel=False)
def numm(img_raw_Color):
    th = np.full((1080,1920), 38)
    img_raw = cv2.cvtColor(img_raw_Color, cv2.COLOR_BGR2GRAY)

    #cv2.imwrite('img_raw.jpg', img_raw)

    #Initial blurring for smoothening
    img_raw = cv2.medianBlur(img_raw, 3)

    #img2 is x2 is defined and set to int16 format to be able to be utilized for subtraction in the integer number line    global x, y, t1, t2 , n
    img2 = np.int16(img_raw)

    #Substraction for initial background substraction
    #img_raw_sub = abs(img_raw2 - x2)
    #print(x, x.shape, x.dtype)
    #print(img2, img2.shape, img2.dtype)
    img_sub = cv2.absdiff(img2, x)
    #cv2.imwrite('img_sub.jpg', img_sub)


    #Use Z for scaling blacks to whites
    #img_raw_sub = z - img_raw_sub

    #Masking out the object
    bg_sub1 = np.where(img_sub>th, img_raw, 0)
    #bg_sub1 = cv2.medianBlur(bg_sub1, 3)

    summ = np.sum(np.sum(bg_sub1,axis = 0))
    #print('summ', summ)
    return bg_sub1, summ, img2


#not using
def croppx(img, a, b, w, h, r, gp):
        if h>w:
          xx = int((((h + 2*gp) / r) - w )/ 2)

          if xx>a:
            cropped = img[b : b+h+2*gp , a: a+w+2*xx]
            #cv2.imwrite('Bordered_img.jpg', Bordered_img)

          #if xx<a and (((b+h+2*gp)<1080) or ((a+w+2*xx)<1920))
            #cropped = Bordered_img[b : b+h+2*gp , a: a+w+2*xx]
          else:
            cropped = img[b-gp : b+h+gp , a-xx : a+w+xx]

        else:
          yy = int((((w + 2*gp) * r) - h )/ 2)

          if yy>b:
            cropped = img[b : b+h+2*yy , a : a+w+2*gp]
            #cv2.imwrite('Bordered_img.jpg', Bordered_img)
          else:
            cropped = img[b-yy : b+h+yy , a-gp : a+w+gp]
        return cropped


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

    self.publisher_ = self.create_publisher(Image, 'Cropped', 10)
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
  @staticmethod
  def edgedetect(img, low, high, i):
    edged = cv2.Canny(img, low, high)
    edged = cv2.dilate(edged, None, iterations= i)
    edged = cv2.erode(edged, None, iterations= i)
    #edged = cv2.bitwise_not(edged);
    return edged

  #not used anymore to reduce runtime
  #@profile
  @staticmethod
  def shadow_out(img, dil, bg_blur):
    dilated_img = cv2.dilate(img, np.ones((dil,dil), np.uint8))
    bg_img = cv2.medianBlur(dilated_img, bg_blur)
    diff_img = 255 - cv2.absdiff(img, bg_img)
    #norm_img = cv2.normalize(diff_img,None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    return diff_img



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

 
  @staticmethod
  #@jit#(nopython=True, parallel=False)
  def numm(img_raw_Color):
    th = np.full((1080,1920), 38)
    img_raw = cv2.cvtColor(img_raw_Color, cv2.COLOR_BGR2GRAY)

    #cv2.imwrite('img_raw.jpg', img_raw)

    #Initial blurring for smoothening
    img_raw = cv2.medianBlur(img_raw, 3)

    #img2 is x2 is defined and set to int16 format to be able to be utilized for subtraction in the integer number line    global x, y, t1, t2 , n
    img2 = np.int16(img_raw)

    #Substraction for initial background substraction
    #img_raw_sub = abs(img_raw2 - x2)
    #print(x, x.shape, x.dtype)
    #print(img2, img2.shape, img2.dtype)
    img_sub = cv2.absdiff(img2, x)
    #cv2.imwrite('img_sub.jpg', img_sub)


    #Use Z for scaling blacks to whites
    #img_raw_sub = z - img_raw_sub

    #Masking out the object
    bg_sub1 = np.where(img_sub>th, img_raw, 0)
    #bg_sub1 = cv2.medianBlur(bg_sub1, 3)

    summ = np.sum(np.sum(bg_sub1,axis = 0))
    #print('summ', summ)
    return bg_sub1, summ, img2


  @profile
  #@staticmethod
  #@jit(nopython=True, parallel=False)
  def mainfunn(img_raw_Color):

    #Global variables
    global x, y, t1, t2 , n
    #x = np.int16(np.zeros((1080,1920)))
    #y = 0
    n = n+1
    #white = np.uint8(np.full((1080,1920), 255)

    #img_raw_sub = np.uint8(np.zeros((1080,1920)))
    cropped = np.uint8(np.zeros((1,1)))
    
    
    #Assigning values to the variables based on the Slie bar's position
    #BLUR = 3
    #BLUR2 = 3
    mThres = 38

    #e_low = 76
    #e_high = 255
    #e_iter = 8
    #cntrst = (0.1)*9

    #dil = 11
    #bg_blur = 3
    th = np.full((1080,1920), mThres)


    img_raw = cv2.cvtColor(img_raw_Color, cv2.COLOR_BGR2GRAY)


    ## Initial blurring for smoothening
    img_raw = cv2.medianBlur(img_raw, 3)

    ## img2 is x2 is defined and set to int16 format to be able to be utilized for subtraction in the integer number line    global x, y, t1, t2 , n
    img2 = np.int16(img_raw)


    ## Substraction for initial background substraction and increasing contrast of the substracted img
    img_sub = cv2.absdiff(img2, x)
    #img_sub = cntrst*cv2.absdiff(img2, x)
    #img_sub = np.uint8((cntrst*cv2.absdiff(img2, x))
    ##cv2.imwrite('img_sub.jpg', img_sub)

    ## numm function trail:
    #bg_sub1, summ, img2 = CrowVision.numm(img_raw_Color)
    #bg_sub1, summ, img2 = numm(img_raw_Color)
    #bg_sub1 = img_sub

    ## Masking out the object
    bg_sub1= np.where(img_sub>th, img_raw, 0)
    #bg_sub1 = cv2.medianBlur(bg_sub1, BLUR)
    cv2.imshow('Background sub mask', bg_sub1)
    ##cv2.imwrite('bg_sub1_NP.jpg', bg_sub1)

    # not using white masking anymore
    #mask = np.where(img_sub>th, white, 0) 
    #mask = cv2.medianBlur(mask, BLUR2)

    #saving background image in X
    if y <= 10:
      x = img2
      y=y+1  
      #cv2.imwrite('BG.jpg', x)

    #summ = numm(bg_sub1)
    summ = np.sum(np.sum(bg_sub1,axis = 0))
    print('summ', summ)
 
    if summ < 30000:
      t2=time.time()
      print('Breaking loop due to no significat detection, dT=',t2-t1)
      key = cv2.waitKey(10)
      #Press esc or 'q' to close the image window
      if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
      return cropped

    ####Shadow removal and smoothening
    #bg_sub1 = shadow_out(bg_sub1, dil, bg_blur)
    #bg_sub1 = cv2.medianBlur(bg_sub1, BLUR2)

    #cv2.imshow('Background sub mask-shadowOut', bg_sub1)
    #cv2.imwrite('bg_sub1_ShwOut.jpg', bg_sub1)

    #edged1 = CrowVision.edgedetect(bg_sub1, e_low, e_high,e_iter)
    #edged2 = edgedetect(bg_subn1, e_low, e_high,e_iter)
    #cv2.imshow('edged1', edged1)
    #cv2.imwrite('edged1.jpg', edged1)

    ##Extracting contours out from the background substracted image
    contours = cv2.findContours(bg_sub1, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #Extracting contours out of the Edged image
    #contours = cv2.findContours(edged1, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    print('contour lenght:', len(contours))
    l=len(contours)
    i = 1
    j = 0

    for contour in contours:
      area = cv2.contourArea(contour)
      print('area', area)

      if area>800:
        j = j+1
        a,b,w,h = cv2.boundingRect(contour)
        # x=a, y=b

        #let fixed ratio: H/W = r
        #r = 1 # we can change as per requirements; r=H/W
        #gp=10
        cropped= CrowVision.cropp(img_raw_Color, a, b, w, h, 1, 10)

        i = i+1


    return cropped




  def input_callback(self, msg, topic):
    """
    @param msg - ROS msg (Image data) to be processed. From camera
    @param topic - str, from camera/input on given topic.
    @return nothing, but send new message(s) via output Publishers.
    """
    global t1, t2 , n
    #t1 is the initial runtime for finding Execution time of this section
    t1=time.time()

    self.get_logger().info("I heard: {} for topic {}".format(str(msg.height), topic))
    assert topic in self.ros, "We don't have registered listener for the topic {} !".format(topic)
    
    #trackbar consists of the variables which have a possibility to be tuned up over external factors
    #cv2.namedWindow('Trackbar')
    #if y == 0:
      #cv2.createTrackbar('BLUR','Trackbar',1,20,nothing)
      #cv2.createTrackbar('BLUR2','Trackbar',1,20,nothing)
      #cv2.createTrackbar('maskingThres','Trackbar',38,60,nothing)

      #cv2.createTrackbar('e_low','Trackbar',76,255,nothing)
      #cv2.createTrackbar('e_high','Trackbar',255,255,nothing)
      #cv2.createTrackbar('e_iter','Trackbar',8,20,nothing)

      #cv2.createTrackbar('cntrst','Trackbar',9,100,nothing) 

      #cv2.createTrackbar('S_dil_Krnl','Trackbar',5,20,nothing)
      #cv2.createTrackbar('S_bg_blur','Trackbar',1,20,nothing)


    #Image data from the realsense camera is translated into Numpy array format using CvBridge()
    img_raw_Color = self.cvb_.imgmsg_to_cv2(msg)

    crop = CrowVision.mainfunn(img_raw_Color)
    #print(crop, type(crop))
    if len(crop) != 1:
       print("showing cropped image", n)
       cv2.imshow('Cropped image', crop)

    #try :
       #cv2.imshow('Cropped image', crop)
       #print("showing cropped image", n)
    #except:
       #print("Nothing cropped!!!")


    #Publishing cropped image
    pub_msg = crop
    #pub_msg.data = 'Cropped image: %d' % i
    self.publisher_.publish(self.cvb_.cv2_to_imgmsg(np.array(pub_msg)))
    self.get_logger().info('Publishing:')#"%s"' % pub_msg.data)       


    #t2 is the final runtime for finding Execution time of this section


    key = cv2.waitKey(10)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
    	cv2.destroyAllWindows()

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

