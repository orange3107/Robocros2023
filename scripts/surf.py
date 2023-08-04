#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import serial

 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):

    self.beforeFrame = np.zeros((720,1280,3), np.uint8)
    self.orb = cv2.ORB_create()
    self.kpBefore, self.desBefore = self.orb.detectAndCompute(self.beforeFrame, None)
    self.before_frame_gray =  cv2.cvtColor(self.beforeFrame, cv2.COLOR_BGR2GRAY)
    self.mask = np.zeros((480,640,3), np.uint8)
    
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image1
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/camera/image', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):

    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    #current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
    current_frame_gray =  cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

    
    # Find key points 
    lk_params = dict(winSize  = (15, 15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    
    
    orb = cv2.ORB_create()
    kp, des = orb.detectAndCompute(current_frame, None)

    flow = cv2.calcOpticalFlowFarneback(self.before_frame_gray, current_frame_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    
    step = 100
    h, w = current_frame.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2, -1).astype(int)
    fx, fy = flow[y, x].T

    print(len(fx))

    min_distance = 30
    angles = [0]
    distVect = []
    vecDeltX = []
    vecDeltY = []
    color = np.random.randint(0,255,(500,3))

    kps = 0
    vectors = []
    vectorSr = []
    angleX = 0.0


    for i in range(len(fx)):
            a,b = x[i], y[i]
            c,d = x[i] - fx[i], y[i] - fy[i]


            if dist((a, b), (c, d)) >= 5 and dist((a, b), (c, d)) <= 30:

              vecDeltX.append(c-a)
              vecDeltY.append(d-b)
                        
              vec = [c-a,d-b]  # дельта вектора
              vectors.append(vec)

              distVect.append(dist((a, b), (c, d)))

    meanVectX = np.mean(vecDeltX)
    meanVectY = np.mean(vecDeltY)

    angleX = calc_mean_Vect(meanVectX, meanVectY)

    font = cv2.FONT_HERSHEY_SIMPLEX
    
    self.mask = np.zeros((480,640,3), np.uint8)

    meanDistVect = np.mean(distVect)
    if angleX >0:

      cv2.arrowedLine(self.mask, (300, 300), (int(300 + meanDistVect*np.cos(angleX)), int(300 + meanDistVect*np.sin(angleX))), (0, 0, 255), 2, cv2.LINE_AA, 0, 0.3)
    cv2.putText(self.mask, str(meanDistVect) ,(100, 100), font, 1,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(self.mask, str(angleX * 180/np.pi) ,(100, 150), font, 1,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(self.mask, str(meanVectX) ,(100, 200), font, 1,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(self.mask, str(meanVectY) ,(100, 250), font, 1,(0,0,255),2,cv2.LINE_AA)

    #current_frame = cv2.drawKeypoints(current_frame, kp, None)
    bf = cv2.BFMatcher(cv2.NORM_L1,crossCheck=False)
    matches = bf.match(self.desBefore, des)
    matches = sorted(matches, key = lambda x:x.distance)

    finFrame = cv2.drawMatches(self.beforeFrame, self.kpBefore, current_frame, kp, matches[:20], None)

    #current_frame = cv2.drawKeypoints(current_frame, self.kpBefore, None)
    #fin_frame = cv2.drawMatches(current_frame, kp, self.beforeFrame,self.kpBefore, matches[:20], None)
    

    # Display image
    #print(np.mean(vectorSr))
    cv2.imshow("camera", current_frame)
    cv2.imshow("camera1", self.mask)
    cv2.waitKey(1)


    # Copy Frame
    self.kpBefore, self.desBefore = orb.detectAndCompute(current_frame, None)
    self.before_frame_gray =  cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    self.beforeFrame = current_frame

def calc_mean_Vect(meanVectX, meanVectY):
    
    if meanVectX >= 0 and meanVectY >= 0:
      return(np.arccos(abs(meanVectX)/np.sqrt((abs(meanVectX))**2 + (abs(meanVectY))**2)))

    elif meanVectX < 0 and meanVectY > 0:
      return(np.arccos(meanVectX/np.sqrt((abs(meanVectX))**2 + (abs(meanVectY))**2)))

    elif meanVectX < 0 and meanVectY < 0:
      return(np.arccos(-meanVectX/np.sqrt((abs(meanVectX))**2 + (abs(meanVectY))**2)) + np.pi)

    elif meanVectX > 0 and meanVectY < 0:
       return(np.arccos((-meanVectX)/np.sqrt((abs(meanVectX))**2 + (abs(meanVectY))**2)) + np.pi)
    
    return 0
    
def draw_flow(frame, flow, step = 16):
   
   h, w = frame.shape[:2]
   y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2, -1).astype(int)
   fx, fy = flow[y, x].T

   lines = np.vstack([x, y, x-fx, y-fy]).T.reshape(-1, 2, 2)
   cv2.polylines(frame, lines, 0)

def dist(a, b):
    return np.sqrt(np.power(b[0] - a[0], 2) + np.power(b[1] - a[1], 2))

def get_angle(v1, v2):
    dx = v2[0] - v1[0]
    dy = v2[1] - v2[1]
    return np.arctan2(dy, dx) * 360 / np.pi

def norm_dist(v1, v2, sig=15):
    theta = get_angle(v1, v2)

    x = v1[0] + sig * np.cos(theta)
    y = v1[1] + sig * np.sin(theta)

    #print 'check', dist((x, y), v1)
    return v1, (int(x), int(y))
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()