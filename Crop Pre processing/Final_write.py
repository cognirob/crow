# x saves the background image of the frame(the 10th frame from the beginning), whish is used to for image substraction with the current image to find the newly introdues objects in the field of view

#img_raw_color is the color image stream form the ros realsense node

#img_raw is the grayscale of the img_raw_color for faster processing

#img2 is the int16 concersion of the img_raw

#th is the threshold matrix with same size as of the img_raw used for comparison with the np.where() to find out the parts of the images with pixels crossing the set thresold values providing a smoother image with just black(background) and relevant parts(new object) of the image so that findcountours runs easier an faster.

# sudo -s for switching to root user

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

import keyboard
import os

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
        try:
            dirr = "Dataset" + str(n)
            os.chdir(os.path.join(home, dirr, path)) 
            path = "Capture" + str(k) 
            os.chdir(path)
        except:
            nothing(0)
        #write Profiling output
        pr.dump_stats('profile_dump')
        with open('profile_dump.txt', 'w+') as f:
          f.write(s.getvalue())
        return retval

    return wrapper


#import commentjson as json
import pkg_resources
import argparse

from std_msgs.msg import String
from sensor_msgs.msg import Image
#from custom_batch.msg import Batch


#z = np.uint8(np.full((1080,1920), 255))
#white = np.full((1080,1920), 255)
#white = white.astype('uint8')



def nothing(x):
    pass

#Resized resolution:
size = (480, 640)
min_size = (120,160)
#BG image variable:
x = np.int16(np.zeros((size[0],size[1],3)))
y = 0
t1=t2=0
n = 1
k = 1

pp=0
#os.mkdir(pp)
#os.chdir(pp)

zz=0
while zz==0:
  try: 
    os.mkdir(str(pp))
    os.chdir(str(pp))
    home = os.getcwd()
    zz=1
  except:
    pp=pp+1



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

    timer_period = 0.5  # seconds



  #@profile
  def timer_callbackX(self):
      msg = string()
      msg.data = 'Lets ROS: %d' % self.i
      self.publisher_.publish(msg)
      self.get_logger().info('Publishing: "%s"' % msg.data)       
      self.i += 1


  def cropp(img, a, b, w, h, xx, yy, gp, r):
      #r= 3/4
      #xx = abs(int((1/2)*(((h+2*gp)/r)-(-w-2*gp))))
      #yy = abs(int((1/2)*((r*(w+2*gp)-h-2*gp))))
      m = yy+gp
      n = xx+gp
      try:
        h<w
        if h>=w: 
          print('h>=w')
          raise Exception('next loop')
        try:
          #cv2.rectangle(img, (a-gp, b-yy-gp), (a+w+gp, b+h+gp+yy), (255, 0, 0), 20) 
          cropped = img[b-yy-gp : b+h+gp+yy , a-gp: a+w+gp]
          ar = 1.0 * (cropped.shape[1]/cropped.shape[0]) 
          abs(ar-1.33)<0.03
          if abs(ar-1.33)>0.03: 
            raise Exception('next loop')
          #cv2.imshow('Cropped image 1a', cropped)

        except:
          ww = np.full(((2*m +1080), (2*m+1920),3), 0, dtype = np.uint8)
          ww[m:-m,m:-m] = img
          #cv2.rectangle(ww, (a-gp+m, b-yy-gp+m), (a+w+gp+m+m, b+h+gp+yy+m+m), (0, 255, 0), 20)
          cropped = ww[b-yy-gp+m : b+h+gp+yy+m , a-gp+m: a+w+gp+m]
          ar = 1.0 * (cropped.shape[1]/cropped.shape[0]) 
          abs(ar-1.33)<0.03
          if abs(ar-1.33)>0.03: 
            raise Exception('next loop')
          #cv2.imshow('Cropped image 1b', cropped)


      except:

        try:
          #cv2.rectangle(img, (a-xx-gp, b-gp), (a+w+xx+gp, b+h+gp), (0, 0, 255), 20)
          cropped = img[b-gp : b+h+gp , a-xx-gp: a+w+xx+gp]
          ar = 1.0 * (cropped.shape[1]/cropped.shape[0]) 
          abs(ar-1.33)<0.03
          if abs(ar-1.33)>0.08:
            raise Exception('next loop')
          #cv2.imshow('Cropped image 2a', cropped)
        except:
          ww = np.full(((2*n +1080), (2*n +1920),3), 0, dtype = np.uint8)
          ww[n:-n,n:-n] = img
          #cv2.rectangle(ww, (a-xx-gp+n, b-gp+n), (a+w+xx+gp+n+n, b+h+gp+n), (255,255, 0), 10)
          cropped = ww[b-gp+n : b+h+gp+n , a-xx-gp+n: a+w+xx+gp+n]
          ar = 1.0 * (cropped.shape[1]/cropped.shape[0]) 
          #cv2.imshow('Cropped image 2b', cropped)

      if cropped.shape[1]<160 or cropped.shape[0]<120:
        #new frame size= 135X180 so that boundary cases are covered likr((119,161) or (121,159)
        frame = np.full(((135), (180),3), 0, dtype = np.uint8)
        frame[0:cropped.shape[0],0:cropped.shape[1]] = cropped
        return frame

      else:
        return cropped





  @profile
  #@staticmethod
  #@jit(nopython=True, parallel=False)
  def mainfunn(img_raw_Color):

    #Global variablesis affected
    global n, x, y, t1, t2 , size, k, home

    #cropped = np.uint8(np.zeros((1,1,3)))

    # 30 is the threshold for logic gates
    th = np.full((size[0], size[1], 3), 30)

    img_raw = cv2.resize(img_raw_Color, (size[1],size[0]), interpolation = cv2.INTER_AREA)


    ## img2 is x2 is defined and set to int16 format to be able to be utilized for subtraction in the integer number line    global x, y, t1, t2 , n
    img2 = np.int16(img_raw)

    ## Substraction for initial background substraction and increasing contrast of the substracted img
    img_sub = cv2.absdiff(img2, x)

    ## Masking out the object
    #bg_sub1= np.where(img_sub>th, img_raw, 0)
    bg_sub1 = img_sub>th#[..., 1:]
    bg_sub1 = np.logical_or(np.logical_or(bg_sub1[..., 0], bg_sub1[..., 1]), bg_sub1[..., 2])
    bg_sub1 = np.dstack((bg_sub1, bg_sub1, bg_sub1))
    bg_sub1 = bg_sub1.astype(np.uint8) * img_raw
    bg_sub1 = cv2.cvtColor(bg_sub1, cv2.COLOR_RGB2GRAY)

    bg_sub1 = cv2.medianBlur(bg_sub1, 3)
    #cv2.imshow('Background sub mask', bg_sub1)

    #saving background image in X
    if y < 5:
      x = img2
      y=y+1
      return 9


    #summ = numm(bg_sub1)
    summ = np.sum(np.sum(bg_sub1,axis = 0))
    print('summ', summ)
 
    if summ < 2000:
      t2=time.time()
      print('Breaking loop due to no significat detection, dT=',t2-t1)
      key = cv2.waitKey(1)
      #Press esc or 'q' to close the image window
      if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
      return 9

    # Extracting contours out from the background substracted image
    contours = cv2.findContours(bg_sub1, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    i = 1

    dirr = "Dataset" + str(n)
    if i ==1:
        try:
         os.mkdir(os.path.join(home, dirr))
         os.chdir(os.path.join(home, dirr))
        except:
         print("error file directory")
        
    path = "Capture" + str(k) 
    os.mkdir(os.path.join(home,dirr, path))
    os.chdir(os.path.join(home,dirr, path))


    name1 = str(n) + "HDinput.jpg"
    cv2.imwrite(name1, img_raw_Color)

    name2 = str(n) + "Resized_toVGA.jpg"
    cv2.imwrite(name2, img_raw)

    Coordinate = str(n) + "coordinates.txt" 

    for contour in contours:
        area = cv2.contourArea(contour)
        print('area', area)

        if area>85: 


          print('detection: ', i , 'area:', area)
          a,b,w,h = cv2.boundingRect(contour)
          # x coord=a, y coord=b
          
          print(img_raw_Color.shape,x.shape)
          wo= (img_raw_Color.shape[1]/x.shape[1])
          ho= (img_raw_Color.shape[0]/x.shape[0])

          xcood= (a*wo)
          ycood= (b*ho)
          width= (w*wo)
          height= (h*ho)

          # fixed ratio: H/W = r
          # gap=10
          r= 3/4
          gp = 40
          xx = abs(int((1/2)*(((height+2*gp)/r)-(width+2*gp))))
          yy = abs(int((1/2)*((r*(width+2*gp)-(height+2*gp)))))
          cropped= CrowVision.cropp(img_raw_Color, int(xcood), int(ycood), int(width), int(height), xx, yy, gp, r)

          coord= np.array([[int(a*wo),int(b*ho),int(w*wo),int(h*ho)]])
 
          IMGname = str(n) + "shot" + str(k)+ "cropImg" + str(i) + ".jpg"
          cv2.imwrite(IMGname, cropped)

          print ("Successfully created the directory %s")

          #print("showing cropped image", i)
          #cv2.imshow('Cropped image', cropped)

          # n counts the increament in the arrangements 

          write_coord= str(i) + '. ' + str(coord) + '\n'
          txtfile = open(Coordinate,"a+") 
          txtfile.write(write_coord)

          i = i+1


    print("Total objects found", i)

    
    if i>1:
      #print("coordinate: ",coord.dtype, coord.shape, coord)
      return 0
    else:
      return 9


  #@profile
  def input_callback(self, msg, topic):
    """
    @param msg - ROS msg (Image data) to be processed. From camera
    @param topic - str, from camera/input on given topic.
    @return nothing, but send new message(s) via output Publishers.
    """
    global t1, t2 , n, x, k, home
    #t1 is the initial runtime for finding Execution time of this section
    #t1=time.time()
    nn=9
    self.get_logger().info("I heard: {} for topic {}".format(str(msg.height), topic))
    assert topic in self.ros, "We don't have registered listener for the topic {} !".format(topic)
    
    #Image data from the realsense camera is translated into Numpy array format using CvBridge()

    img_raw_Color = cv2.cvtColor(self.cvb_.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)
    #home = os.getcwd()

    if keyboard.is_pressed('l'): 
      k=1

    if k==1:
      print('Ready!!! - Press p to write data')
    if k==6:
      print('Stop!!!- - Press l to reset')
    if k!=1 and k!=6:
      print('busy')

    if keyboard.is_pressed('p'):
      while k<6:

        t1=time.time()
        nn = CrowVision.mainfunn(img_raw_Color)

        if n>1: 
          try:
            dirr = "Dataset" + str(n)
            os.chdir(os.path.join(home, dirr)) 
            path = "Capture" + str(k) 
            os.chdir(path)
          except:
            nothing(0)
        #t2 is the final runtime for finding Execution time of this section
        t2=time.time()
        dT= 'Processing time' + str(k) +'= ' + str(t2-t1) + '\n'
        txtfile = open('dT',"a+") 
        txtfile.write(dT)
        print('dT', k , '= ',t2-t1)
        k=k+1
        
    if k==6 and nn==0:
        n = n+1

    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
      cv2.destroyAllWindows()



def main(args=None):
  rclpy.init(args=args)
  
  crow_detector=CrowVision()
  rclpy.spin(crow_detector)

  cv2.destroyAllWindows()
  rclpy.shutdown()


if __name__ == '__main__':
    main()
