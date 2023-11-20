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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv

class Points:
    def __init__(self, X, Y):
        self.x = X
        self.y = Y
        self.children = np.empty((1,1))
        self.parent = 0
        self.lenghtPath = 0


class RrtStar:
    def __init__(self, start, goal, iter, gridArr, stepSize, neighborhood, eulerAuto, posX, posY, waypoint_publisher):
        self.start = Points(start[0], start[1])
        self.goal = Points(goal[0], goal[1])
        self.mearestNode = None
        self.iterations = min(iter, 10000)
        self.eulerAuto = eulerAuto
        self.gridArray = gridArr
        self.stp = stepSize
        self.nhood = neighborhood
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints = []
        self.count = -1
        self.posX = posX
        self.posY = posY
        self.waypoint_publisher = waypoint_publisher


    def planning(self):
        
        width = len(self.gridArray)
        #print(width)
        self.Waypoints = np.append(self.Waypoints, self.start)
        for k in range(self.iterations):
          node = self.rndPont((width, width))
          min = 9999.0
          minPoint = -1

          for i in range(len(self.Waypoints)):
             #print(self.Waypoints[i].x)
             if(self.lenPoints(self.Waypoints[i], node) < min):
                
                min = self.lenPoints(self.Waypoints[i], node)
                minPoint = i

          point = self.steerPoint(self.Waypoints[minPoint], node, width, width, self.stp)
          #print(self.gridArr)
          
          
          if(len(self.gridArray) > 2):
            if(np.any(self.gridArray[point[1]][point[0]]) == 0):
              point = Points(point[0], point[1])
              point.parent = minPoint
              
              ##point.lenghtPath = self.distance(point.x, point.y, self.Waypoints[point.parent].x, self.Waypoints[point.parent].y)
              closelyPointArray = np.array([], 'int32')
              min = 9999.0
              for i in range(len(self.Waypoints)): # ищем самый малый по длине путь
                 if(self.IsPointInCircle(self.Waypoints[i].x, self.Waypoints[i].y, point.x, point.y, self.nhood)):
                    closelyPointArray = np.append(closelyPointArray, i)
                    if self.Waypoints[i].lenghtPath < min:
                      min = self.Waypoints[i].lenghtPath
                      point.parent = i
                #self.Waypoints = np.append(self.Waypoints, point)
              point.lenghtPath = self.distance(point.x, point.y, self.Waypoints[point.parent].x, self.Waypoints[point.parent].y) + self.Waypoints[point.parent].lenghtPath
              #print(point.lenghtPath) 
              self.Waypoints = np.append(self.Waypoints, point)

              for i in range(len(closelyPointArray)):
                 newLength = point.lenghtPath + self.distance(point.x, point.y, self.Waypoints[closelyPointArray[i]].x, self.Waypoints[closelyPointArray[i]].y)
                 if self.Waypoints[closelyPointArray[i]].lenghtPath > newLength:
                    self.Waypoints[closelyPointArray[i]].parent = len(self.Waypoints) - 1
                    self.Waypoints[closelyPointArray[i]].lenghtPath = newLength
              #print(point.x, point.y, self.goal.x, self.goal.y)
              if(self.IsPointInCircle(point.x, point.y, self.goal.x, self.goal.y, 10)):
                self.count = len(self.Waypoints)-1
                #print(self.count)
                #break

              #self.grid = cv2.line(self.grid, (point.x, point.y), (self.Waypoints[minPoint].x, self.Waypoints[minPoint].y), (50,50,50), 2)


        msg = Path()
        msg.header.frame_id = "map"
        #msg.header.stamp = rclpy.time.Time()
        localPath = np.empty((0,2))
        while(True):
          #print(self.count)
          if(self.count > 0):
            x = self.Waypoints[self.count].x
            y = self.Waypoints[self.count].y
            
            x1 = ((x - width/2)*math.cos(self.eulerAuto) - (y - width/2)*math.sin(self.eulerAuto)) / 20.0 + self.posX
            y1 = ((y - width/2)*math.cos(self.eulerAuto) + (x - width/2)*math.sin(self.eulerAuto)) / 20.0 + self.posY

            pose = PoseStamped()
            pose.pose.position.x = x1
            pose.pose.position.y = y1
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            msg.poses.append(pose)
            localPath = np.append(localPath, [[x1, y1]], axis=0)

            x1 = self.Waypoints[self.Waypoints[self.count].parent].x
            y1 = self.Waypoints[self.Waypoints[self.count].parent].y

            #self.grid = cv2.line(self.grid, (x, y), (x1, y1), (50,50,50), 2)
            #self.grid = cv2.circle(self.grid, (x, y), 2, (127,127,127), -1)
            self.count = self.Waypoints[self.count].parent
          else:
              break
          
          self.waypoint_publisher.publish(msg)
        return localPath

        #self.grid = cv2.flip(self.grid, 0)
        #cv2.imshow("path", self.grid)
        #cv2.waitKey(1)
             
                    



    def distance(self, x1, y1, x2, y2):
      c = math.sqrt((x2-x1)**2 + (y2-y1)**2)
      return c

          
    

    def rndPont(self, hw):
       x = randint(0, hw[0])
       y = randint(0, hw[1])
       return (x, y)
    
    def drawPath(self):
       pass
  
    def lenPoints(self, locStart, locEnd):
      #print(locEnd[0], locStart.x, locEnd[1], locStart.y)
      lenght = math.sqrt((locEnd[0] - locStart.x)**2 + (locEnd[1] - locStart.y)**2)
      return lenght

    def steerPoint(self, locStart, locEnd, h, w, stp):
       vec = np.array([locEnd[0] - locStart.x, locEnd[1] - locStart.y])
       if(vec[0]+vec[1] == 0):
          offset = (0,0)
       else:
        offset = stp*self.UnVector(vec)
       point = np.array([int(offset[0] + locStart.x), int(offset[1] + locStart.y)])
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

    #print("hello")

    #timer_period = 0.5  # seconds
    #self.timer = self.create_timer(timer_period, self.timer_callback)

    self.br = CvBridge()
    width = 1
    height = 1
    self.image_map = np.zeros((width,height,3), np.uint8)
    self.goalX = 0
    self.goalY = 0
    self.c = 0
    self.carX = 0
    self.carY = 0
    self.eulerAuto = 0

    self.corrStartI = None
    self.corrEndI = None

    self.posX = 0
    self.posY = 0
    self.angleAutoPoint = 0
    self.current_map = 0
    self.height = 0
    self.width = 0
    self.mapArr = np.empty((1,1))

    self.timer = self.create_timer(1.0, self.timer_path_callback)
    self.timer1 = self.create_timer(0.1, self.timer_path_out)

    self.subscription = self.create_subscription(
      Marker, 
      '/goalPoint', 
      self.goal_point_callback, 
      10)
    
    self.subscription = self.create_subscription(
      Path, 
      '/global_path', 
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

    #print(self.current_map)

    self.waypoint_publisher = self.create_publisher(
            Path, '/pathWay', 1)
    
    self.global_path_publisher = self.create_publisher(Path, '/global_path', 10)
    
    self.pathArr = np.empty((0,2))
    with open('/home/ilya22/ros2_humble/src/robocross2023/paths/path1.csv', 'r') as file:
     reader = csv.reader(file)
     msg = Path()
     msg.header.frame_id = "map"
     for index, line in enumerate(reader):
       if(index > 0):
        x = float(line[0])
        y = float(line[1])
        self.pathArr = np.append(self.pathArr, [[x, y]], axis=0)

  def angle_between_lines(self, p1, p2, p3):
    v1 = (p1[0] - p2[0], p1[1] - p2[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])
    mult = np.dot(v1, v2)
    vl1 = math.sqrt(v1[0]**2 + v1[1]**2)
    vl2 = math.sqrt(v2[0]**2 + v2[1]**2)
    angle = math.acos(mult/(vl1 * vl2))
    #angle = math.degrees(angle)
    #angle = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
    #angle = abs(180-math.degrees(angle))
    # если нужен острый угол
    #return min(180 - angle, angle)
    return angle


  def timer_path_out(self):
     
     lenArr = len(self.pathArr)
     #print(lenArr)
     
     for i in range(lenArr):
        if(i < lenArr - 2):
           angle = self.angle_between_lines(self.pathArr[i], self.pathArr[i+1], self.pathArr[i+2])
           lenAangle = (3/math.tan(angle/2))
           dist1 = self.dist(self.pathArr[i][0], self.pathArr[i][1], self.pathArr[i+1][0], self.pathArr[i+1][1])
           dist2 = self.dist(self.pathArr[i+1][0], self.pathArr[i+1][1], self.pathArr[i+2][0], self.pathArr[i+2][1])
           print(lenAangle, dist1, dist2)
           if(lenAangle > dist1 or lenAangle > dist2):
              
              self.pathArr = np.delete(self.pathArr, i + 1, axis = 0)
           #print(self.pathArr[i], self.pathArr[i+1], self.pathArr[i+2])
           lenArr = len(self.pathArr)
           print(angle)
           
     print("END") 
     msg = Path()
     msg.header.frame_id = "map"
        #msg.header.stamp = rclpy.time.Time()
     for i in range(len(self.pathArr)):

            x = self.pathArr[i][0]
            y = self.pathArr[i][1]

            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            msg.poses.append(pose)

     self.global_path_publisher.publish(msg)
    
  def dist(self, x1, y1, x2, y2):
     dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
     return dist

     
  def timer_path_callback(self):
      imageMapArr = self.mapArr
      width = len(self.mapArr)
      c = 0
      posCarX = -int(20*self.carX)
      posCarY = width+int(20*self.carY)  
      #print(posCarX, posCarY)
      includePoints = []    

      for i in range(len(self.pathArr)):
         
         
         length = 20*math.sqrt((self.pathArr[i][0] - self.posX)**2 + (self.pathArr[i][1] - self.posY)**2)
         
         VecGoalX = self.pathArr[i][0] - self.posX
         VecGoalY = self.pathArr[i][1] - self.posY

         VecAutoX = math.cos(self.eulerAuto + math.pi/2) #вектор авто
         VecAutoY = math.sin(self.eulerAuto + math.pi/2) 

         engleGoaltoAuto = math.atan2(VecAutoX*VecGoalY - VecAutoY*VecGoalX, VecAutoX*VecGoalX + VecAutoY*VecGoalY)

         x = -int(length*math.sin(engleGoaltoAuto)) + posCarX
         y = (int(length*math.cos(engleGoaltoAuto)) + posCarY)
         #print(x, y)
         if x <= width and x >= 0 and y <= width and y >= 0:
            #print('x = ', x)
            #print('y = ', y)
            #print('i = ', i)
            #print('c = ', c)
            if i - c < 2 or c == 0:
              #print("add")
              includePoints = np.append(includePoints, i)
              c = i
            else:                            
               break
           
      #print(includePoints)
      includePoints = np.append(includePoints, includePoints[len(includePoints) - 1] + 1)  
             
   
      startPoint = (0,0)
      endPoint = (0,0)
      start = False
      end = False
      backPoint = None
      for i in range(len(includePoints)-1):
         
      
         p1 = self.global_point_in_local_map(self.posX,self.posY, posCarX, posCarY, self.pathArr[int(includePoints[i])])
         p2 = self.global_point_in_local_map(self.posX,self.posY, posCarX, posCarY, self.pathArr[int(includePoints[i+1])])
         #print(i)
         
         #print(p1, " ", p2)
         
         segment = 8
         while(self.distance(p1, p2) > segment):
          pr = self.steerPoint(p1, p2, segment)
          #print(pr)
          if pr[0] > width-1 or pr[1] > width-1:
            start = False
            end = False   
            break
          
          segment += 8
          #print(imageMapArr[pr[1], pr[0]])
          if(np.any(imageMapArr[pr[1], pr[0]]) != 0):
             #print(backPoint) 
             #print("препядствие: ", pr)
             #print(imageMapArr[pr[1], pr[0]])
             if start == False:
              self.corrStartI = includePoints[i]             
              startPoint = backPoint
              #print("нашли старт: ", backPoint)
              #print('i = ', i)
              start = True
          else:
             #print(end, start)
             if end == False and start == True:
                endPoint = pr
                self.corrEndI = includePoints[i+1]
                end = True
             elif end == False and start == False:
                backPoint = pr
                #print(backPoint) 
                 
          #print(end, start, includePoints)
          if(end == True and start == True):
             start = False
             end = False
             #print("planning")
             #print(startPoint)
             if np.any(startPoint) != None:
              rrtStar = RrtStar((startPoint), (endPoint), 1000, imageMapArr, 10, 35, self.eulerAuto, self.posX, self.posY, self.waypoint_publisher)
              path = rrtStar.planning()
              x = startPoint[0]
              y = startPoint[1]
              #xStart = ((x - width/2)*math.cos(self.eulerAuto) - (y - width/2)*math.sin(self.eulerAuto)) / 20.0 + self.posX
              #yStart = ((y - width/2)*math.cos(self.eulerAuto) + (x - width/2)*math.sin(self.eulerAuto)) / 20.0 + self.posY
             #path = np.append(path, [[xStart, yStart]], axis = 0)
              path = np.flip(path, axis = 0)
             #print(path)
             
             #print(self.corrStartI, self.corrEndI)
             #print(self.pathArr)
              if len(path) > 0:
               #print(path[0])
               for i in range(int(self.corrEndI - self.corrStartI) - 1):
                self.pathArr = np.delete(self.pathArr, int(self.corrStartI) + 1, axis = 0)
               self.pathArr = np.insert(self.pathArr, int(self.corrStartI) + 1, path, axis = 0)
              break
             
         

      length = 20*math.sqrt((self.goalX - self.posX)**2 + (self.goalY - self.posY)**2)
      posGaolX = -int(length*math.sin(self.angleAutoPoint)) + posCarX
      posGaolY = int(length*math.cos(self.angleAutoPoint)) + posCarY
      
      ##rrtStar = RrtStar((posCarX, posCarY), (posGaolX, posGaolY), 1000, imageMapArr, 10, 35, self.eulerAuto, self.posX, self.posY, self.waypoint_publisher)
      ##rrtStar.planning()

  def global_point_in_local_map(self, posX, posY, carXLocal, carYLocal, oldPose):
        
     length = 20*math.sqrt((oldPose[0] - self.posX)**2 + (oldPose[1] - self.posY)**2)
     VecGoalX = oldPose[0] - posX
     VecGoalY = oldPose[1] - posY

     VecAutoX = math.cos(self.eulerAuto + math.pi/2) #вектор авто
     VecAutoY = math.sin(self.eulerAuto + math.pi/2) 

     engleGoaltoAuto = math.atan2(VecAutoX*VecGoalY - VecAutoY*VecGoalX, VecAutoX*VecGoalX + VecAutoY*VecGoalY)

     x = -int(length*math.sin(engleGoaltoAuto)) + carXLocal
     y = (int(length*math.cos(engleGoaltoAuto)) + carYLocal)

     return (x, y)
     

  def distance(self, p1, p2):
      c = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
      return c

  def steerPoint(self, locStart, locEnd, stp):
       vec = np.array([locEnd[0] - locStart[0], locEnd[1] - locStart[1]])
       if(vec[0]+vec[1] == 0):
          offset = (0,0)
       else:
        offset = stp*self.UnVector(vec)
       point = np.array([int(offset[0] + locStart[0]), int(offset[1] + locStart[1])])
       return point

  def UnVector(self, vec):
       u_hat = vec/np.linalg.norm(vec)
       return u_hat


  def angle_auto_point_callback(self, data):
    self.angleAutoPoint = data.data
   
  def poseAuto_collback(self, data):
      euler = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
      self.eulerAuto = euler[2]
      self.posX = data.pose.position.x
      self.posY = data.pose.position.y

  def local_map_image_collback(self, data):
     
     self.current_map = self.br.imgmsg_to_cv2(data)
     width, height = self.current_map.shape[:2]
     #self.current_map = cv2.rotate(self.current_map, cv2.ROTATE_180)
     #####self.current_map = cv2.flip(self.current_map, 0)
     posCarX = -int(20*self.carX)
     posCarY = width+int(20*self.carY)
     self.current_map = cv2.circle(self.current_map, (posCarX, posCarY), 5, (50,50,50), -1)
     #print(height, width)
     
     len = 20*math.sqrt((self.goalX - self.posX)**2 + (self.goalY - self.posY)**2)
     posGaolX = 0
     posGaolY = int(len)
     
     posGaolX = -int(len*math.sin(self.angleAutoPoint)) + posCarX
     posGaolY = int(len*math.cos(self.angleAutoPoint)) + posCarY
     
     self.current_map = cv2.circle(self.current_map, (posGaolX, posGaolY), 5, (127,127,127), -1)

     #rrtStar = RrtStar((posCarX, posCarY), (posGaolX, posGaolY), 1000, self.current_map,self.mapArr, 10, 35)
     #rrtStar.planning()

     #print(posGaolX, posGaolY)
     #self.current_map = cv2.flip(self.current_map, 0)
     #cv2.imshow("map", self.current_map)
     #cv2.waitKey(1)

  
  def goal_point_callback(self, data):
    
    self.goalX =0 #data.pose.position.x
    self.goalY =0 #data.pose.position.y

  def local_map_collback(self, data):
    self.height = data.info.height
    self.width = data.info.width
    self.mapArr = np.reshape(data.data, (self.height, self.width))
    #print(self.mapArr)
    self.carX = data.info.origin.position.x
    self.carY = data.info.origin.position.y
    
  
def euler_from_quaternion(x, y, z, w):
  euler = np.empty((3, ))
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)

  euler[0] = roll_x
  euler[1] = pitch_y
  euler[2] = yaw_z

  return euler

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