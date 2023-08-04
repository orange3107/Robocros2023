#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import serial
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import PointCloud2, PointField
import ros2_numpy
import open3d as o3d
import struct
import ctypes
import sensor_msgs_py.point_cloud2 
import math
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Int8MultiArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from random import randint

class Points:
    def __init__(self, X, Y):
        self.x = X
        self.y = Y
        self.children = np.empty((1,1))
        self.parent = None


class RrtStar:
    def __init__(self, start, goal, iter, grid, gridArr, stepSize):
        self.start = Points(start[0], start[1])
        self.goal = Points(goal[0], goal[1])
        self.mearestNode = None
        self.iterations = min(iter, 10000)
        self.grid = grid
        self.gridArr = np.flip(gridArr,0)
        self.stp = stepSize
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints = []
        self.count = -1

    def planning(self):
        self.Waypoints = np.append(self.Waypoints, self.start)
        for k in range(self.iterations):
          node = self.rndPont(self.grid.shape[:2])
          min = 9999.0
          minPoint = -1

          for i in range(len(self.Waypoints)):

             if(self.lenPoints(self.Waypoints[i], node) < min):
                min = self.lenPoints(self.Waypoints[i], node)
                minPoint = i

          point = self.steerPoint(self.Waypoints[minPoint], node)
          #print(self.gridArr)
          
          if(len(self.gridArr) > 2):
            if(np.any(self.grid[point[1]][point[0]]) == 0):
              point = Points(point[0], point[1])
              point.parent = minPoint
              self.Waypoints = np.append(self.Waypoints, point)

              if(self.IsPointInCircle(point.x, point.y, self.goal.x, self.goal.y, 10)):
                self.count = len(self.Waypoints)-1
                print(self.count)
                break

              #self.grid = cv2.line(self.grid, (point.x, point.y), (self.Waypoints[minPoint].x, self.Waypoints[minPoint].y), (50,50,50), 2) 
        while(True):
          print(self.count)
          if(self.count > 0):
            x = self.Waypoints[self.count].x
            y = self.Waypoints[self.count].y
            self.grid = cv2.circle(self.grid, (x, y), 2, (50,50,50), -1)
            self.count = self.Waypoints[self.count].parent
          else:
              #print("blea")
              break
                    





          
    

    def rndPont(self, hw):
       x = randint(0, hw[0])
       y = randint(0, hw[1])
       return (x, y)
    
    def drawPath(self):
       pass
  
    def lenPoints(self, locStart, locEnd):
      lenght = math.sqrt((locEnd[0] - locStart.x)**2 + (locEnd[1] - locStart.y)**2)
      return lenght

    def steerPoint(self, locStart, locEnd):
       vec = np.array([locEnd[0] - locStart.x, locEnd[1] - locStart.y])
       if(vec[0]+vec[1] == 0):
          offset = (0,0)
       else:
        offset = self.stp*self.UnVector(vec)
       point = np.array([int(offset[0] + locStart.x), int(offset[1] + locStart.y)])
       h, w = self.grid.shape[:2]
       if(point[0] < 0):
          point[0] = 0
          
       if(point[0] > h-1):
          point[0] = h-1

       if(point[1] < 0):
          point[1] = 0

       if(point[1] > w-1):
          point[1] = w-1
       return point

    def UnVector(self, vec):
       u_hat = vec/np.linalg.norm(vec)
       return u_hat
    
    def IsPointInCircle(self, x, y, xc, yc, r):
      return ((x-xc)**2+(y-yc)**2) ** 0.5 <= r
    
       
    
class CreateLocalRrt(Node):



  def __init__(self):
    
    super().__init__('local_map')

    print("hello")

    #timer_period = 0.5  # seconds
    #self.timer = self.create_timer(timer_period, self.timer_callback)

    self.br = CvBridge()
    width = 1
    height = 1
    self.image_map = np.zeros((width,height,3), np.uint8)
    self.goalX = 0
    self.goalY = 0
    self.carX = 0
    self.carY = 0

    self.posX = 0
    self.posY = 0
    self.angleAutoPoint = 0
    self.current_map = 0
    self.height = 0
    self.width = 0
    self.mapArr = np.empty((1,1))

    self.subscription = self.create_subscription(
      Marker, 
      '/goalPoint', 
      self.goal_point_callback, 
      10)
    
    self.subscription = self.create_subscription(
      Float32, 
      '/angle_auto_point', 
      self.angle_auto_point_callback, 
      10)
    
    self.subscription = self.create_subscription(
      Image, 
      '/map_image', 
      self.local_map_image_collback, 
      10)

    self.subscription = self.create_subscription(
      Marker, 
      '/poseAuto', 
      self.poseAuto_collback, 
      10)

    self.subscription = self.create_subscription(
      OccupancyGrid,
      '/local_cost_map', 
      self.local_map_collback, 
      1)
    self.subscription # prevent unused variable warning

    #print(self.current_map)





  def angle_auto_point_callback(self, data):
    self.angleAutoPoint = data.data
   
  def poseAuto_collback(self, data):
      self.posX = data.pose.position.x
      self.posY = data.pose.position.y

  def local_map_image_collback(self, data):
     
     self.current_map = self.br.imgmsg_to_cv2(data)
     width, height = self.current_map.shape[:2]
     #self.current_map = cv2.rotate(self.current_map, cv2.ROTATE_180)
     self.current_map = cv2.flip(self.current_map, 0)
     posCarX = -int(20*self.carX)
     posCarY = width+int(20*self.carY)
     self.current_map = cv2.circle(self.current_map, (posCarX, posCarY), 5, (50,50,50), -1)
     #print(height, width)
     
     len = 20*math.sqrt((self.goalX - self.posX)**2 + (self.goalY - self.posY)**2)
     posGaolX = 0
     posGaolY = int(len)
     
     posGaolX = -int(len*math.sin(self.angleAutoPoint)) + posCarX
     posGaolY = -int(len*math.cos(self.angleAutoPoint)) + posCarY
     
     self.current_map = cv2.circle(self.current_map, (posGaolX, posGaolY), 5, (127,127,127), -1)

     rrtStar = RrtStar((posCarX, posCarY), (posGaolX, posGaolY), 2000, self.current_map,self.mapArr, 10)
     rrtStar.planning()

     #print(posGaolX, posGaolY)
     cv2.imshow("map", self.current_map)
     cv2.waitKey(1)

  
  def goal_point_callback(self, data):
    self.goalX = data.pose.position.x
    self.goalY = data.pose.position.y

  def local_map_collback(self, data):
    self.height = data.info.height
    self.width = data.info.width
    self.mapArr = np.reshape(data.data, (self.height, self.width))
    #print(self.mapArr)
    self.carX = data.info.origin.position.x
    self.carY = data.info.origin.position.y
    
  

def checkPoint(radius, x, y):
 
    polarradius = math.sqrt(x * x + y * y)
    # Check whether polarradius is less
    # then radius of circle or not and
    # Angle is between startAngle and
    # endAngle or not
    if (polarradius < radius):
        return True
    else:
        return False
 
# Driver code
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  local_rrt = CreateLocalRrt()
  
  # Spin the node so the callback function is called.
  rclpy.spin(local_rrt)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  local_rrt.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
