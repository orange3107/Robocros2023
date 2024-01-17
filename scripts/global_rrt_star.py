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
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from random import randint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv

class Points:
    def __init__(self, X, Y, A):
        self.x = X
        self.y = Y
        self.children = np.empty((1,1))
        self.parent = 0
        self.lenghtPath = 0
        self.a = A


class RrtStar:
    def __init__(self, start, goals, iter, gridArr, stepSize, neighborhood, eulerAuto, posX, posY, waypoint_publisher, pubposemarker):
        self.start = Points(start[0]*20, start[1]*20, start[2])
        self.start.parent = -1
        self.goals = goals
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
        self.pubposemarker = pubposemarker


    def planning(self):
      stSize = 15
      allLenghtPath = 0
      #print(self.start.x, self.start.y)
      marker = Marker()
      marker.header.frame_id = "/map"
      marker.type = marker.SPHERE
      marker.action = marker.ADD
      marker.scale.x = 0.2
      marker.scale.y = 0.2
      marker.scale.z = 0.2
      marker.color.a = 1.0
      marker.color.r = 1.0
      marker.color.g = 1.0
      marker.color.b = 0.0
      marker.pose.position.x = float(self.posX)
      marker.pose.position.y = float(self.posY)
      marker.pose.position.z = 0.0

      marker.pose.orientation.x = 0.0
      marker.pose.orientation.y = 0.0
      marker.pose.orientation.z = 0.0
      marker.pose.orientation.w = 0.0

      self.pubposemarker.publish(marker)
      correctGoal = None     
      height= len(self.gridArray)
      width= len(self.gridArray[0])
      img = np.zeros((width,height,3), np.uint8)
        #print(width, height)
      self.Waypoints = np.append(self.Waypoints, self.start)
        #for k in range(self.iterations):
      markerArray = MarkerArray()
      """
      for i in range(height):
         for f in range(width):
            if self.gridArray[i][f] != -1:
               print(i, f, self.gridArray[i][f])
               img = cv2.circle(img,(i, f), 3, (100,100,100), -1)
      """

      for i in range(len(self.goals)):
         x = self.goals[i][0]
         y = self.goals[i][1]
         img = cv2.circle(img,(int(x), int(y)), 10, (255,255,255), -1)
      k = 0
      while k < self.iterations:
          
          node = self.rndPont((height, width), 0)
          #print(node)
          min = 9999.0
          minPoint = -1

          for i in range(len(self.Waypoints)):
             infoPoint = None
             #print(i,":",self.Waypoints[i].a)
             #print(self.Waypoints[i].a)
             #print(self.Waypoints[i].x)
             if(self.lenPoints(self.Waypoints[i], node) < min):
                                      
               pointBuff = self.steerPoint(self.Waypoints[i], node, height, width, self.stp)
               check = self.checkR(40, [self.Waypoints[i].x, self.Waypoints[i].y, self.Waypoints[i].a], pointBuff)

               if check[0]:
                  min = self.lenPoints(self.Waypoints[i], node)
                  minPoint = i
                

          point = self.steerPoint(self.Waypoints[minPoint], node, height, width, self.stp)
          #print(self.stp)
          #print(self.distance(point[0], point[1], self.Waypoints[minPoint].x, self.Waypoints[minPoint].y), point[0], point[1])


          #img = cv2.circle(img,(point[0], point[1]), 3, (0,255,0), -1)
          #print(self.gridArr)
          
          
          if(len(self.gridArray) > 2):
            #print(1)
            #print(self.gridArray[point[0]][point[1]])
            if(self.gridArray[point[0]][point[1]] == -1):
               k += 1 
               #print(2)
               point = Points(point[0], point[1], 0)
               point.parent = minPoint
              
               ##point.lenghtPath = self.distance(point.x, point.y, self.Waypoints[point.parent].x, self.Waypoints[point.parent].y)
               closelyPointArray = np.array([], 'int32')
               min = 9999.0
               #начало RRT
               check = None
               for i in range(len(self.Waypoints)): # ищем самый малый по длине путь
                  if(self.IsPointInCircle(self.Waypoints[i].x, self.Waypoints[i].y, point.x, point.y, self.nhood) and self.Waypoints[i].a != -999):
                    closelyPointArray = np.append(closelyPointArray, i) #находим самый кородкий путь по точки по длине

                    check = self.checkR(40, [self.Waypoints[i].x, self.Waypoints[i].y, self.Waypoints[i].a], (point.x, point.y))

                    if self.Waypoints[i].lenghtPath + check[5] < min:
                      if check[0]:
                        min = self.Waypoints[i].lenghtPath + check[5]
                        point.parent = i
                        point.lenghtPath = min
                        point.a = check[4] + self.Waypoints[point.parent].a
               #print(" ")
               #print(self.Waypoints[point.parent].x, "," ,self.Waypoints[point.parent].y)#, self.Waypoints[point.parent].a, point.x,",",point.y)
               #print(point.x,",",point.y)
               #print(math.degrees(self.Waypoints[point.parent].a))
               #print(point.parent)
               img = cv2.circle(img,(point.x,point.y), 3, (0,0,255), -1)
               #img = cv2.circle(img,(int(self.start.x),int(self.start.y)), 3, (255,0,0), -1)
               #cv2.line(img,(int(self.Waypoints[point.parent].x), int(self.Waypoints[point.parent].y)),((point.x,point.y)),(255,0,0),5)

               if(point.parent != -1):
                  self.Waypoints = np.append(self.Waypoints, point)
                  
                  #print(point.parent)
                  #infoPoint = self.checkR(20, [self.Waypoints[point.parent].x, self.Waypoints[point.parent].y, self.Waypoints[point.parent].a], (point.x, point.y))
                  #print(infoPoint)
                  #localArc = self.getArc(infoPoint[2], infoPoint[1],3, infoPoint[3], self.Waypoints[point.parent],img,point.parent)

               
               #point.lenghtPath = self.distance(point.x, point.y, self.Waypoints[point.parent].x, self.Waypoints[point.parent].y) + self.Waypoints[point.parent].lenghtPath
               #print(point.lenghtPath) 

                  #self.Waypoints = np.append(self.Waypoints, localArc)
                  #self.Waypoints = np.append(self.Waypoints, localArc)
                  
   
               #начало STAR
               for i in range(len(closelyPointArray)):

                  check = self.checkR(40, (point.x, point.y, point.a) ,[self.Waypoints[closelyPointArray[i]].x, self.Waypoints[closelyPointArray[i]].y, self.Waypoints[closelyPointArray[i]].a])
                  newLength = point.lenghtPath + check[5]

                  if self.Waypoints[closelyPointArray[i]].lenghtPath > newLength and check[0] and check[4] == self.Waypoints[closelyPointArray[i]].a:
                    self.Waypoints[closelyPointArray[i]].parent = len(self.Waypoints) - 1
                    self.Waypoints[closelyPointArray[i]].lenghtPath = newLength
               #print(point.x, point.y, self.goal.x, self.goal.y)

               for i in range(len(self.goals)):
                  #print(self.goals[i])
                  #img = cv2.circle(img,(int(self.goals[i][0]), int(self.goals[i][1])), 3, (0,255,0), -1)
                  goal = Points(self.goals[i][0], self.goals[i][1], self.goals[i][2])
                  if(self.IsPointInCircle(point.x, point.y, goal.x, goal.y,200)):

                     check = self.checkR(40, (point.x, point.y, point.a) ,(goal.x, goal.y, goal.a))

                     rotatedX = 17*math.cos(goal.a + math.pi/2) + goal.x
                     rotatedY = 17*math.sin(goal.a + math.pi/2) + goal.y

                     #img = cv2.line(img,(int(goal.x),int(goal.y)),(int(rotatedX),int(rotatedY)),(255,0,0),1)
                  
                     if check[0] and (abs(check[4] + point.a - self.goals[i][2]) < 0.2):
                        self.count = len(self.Waypoints)-1
                        correctGoal = self.goals[i]
                        #print("END", correctGoal)
                        

          if(self.count > 0):
             break


              #self.grid = cv2.line(self.grid, (point.x, point.y), (self.Waypoints[minPoint].x, self.Waypoints[minPoint].y), (50,50,50), 2)
              
              
        #print("ggg ")


      msg = Path()
      msg.header.frame_id = "map"
      #msg.header.stamp = rclpy.time.Time()
      localPath = np.empty((0,4))
      while(True):
          #print(self.count)
         if(self.count > -1):
            #print(";;;")
            x = self.Waypoints[self.count].x
            y = self.Waypoints[self.count].y
            a = self.Waypoints[self.count].a
            r = -1

            allLenghtPath += self.Waypoints[self.count].lenghtPath
            
            localPath = np.append(localPath, [[x, y, a, r]], axis=0)

            self.count = self.Waypoints[self.count].parent
         else:
            break

        
        
      localPath = np.flip(localPath, axis = 0)
      lenPath = len(localPath)
      i = 0

      while i < lenPath - 1:
         p = Points(localPath[i][0], localPath[i][1], localPath[i][2])
         infoPoint = self.checkR(40, [p.x, p.y, p.a], [localPath[i+1][0], localPath[i+1][1], localPath[i+1][2]])
         localArc = self.getArc(infoPoint[2], infoPoint[1], stSize, infoPoint[3], p, img, infoPoint[1])
         print(localArc)
         localPath = np.insert(localPath, i+1, localArc, axis = 0)
         lenPath = len(localPath)

         i += len(localArc) + 1

      lenPath = len(localPath)
      if lenPath > 0:
         p = Points(localPath[lenPath - 1][0], localPath[lenPath - 1][1], localPath[lenPath - 1][2])
         infoPoint = self.checkR(40, [p.x, p.y, p.a], [correctGoal[0], correctGoal[1], correctGoal[2]])
         allLenghtPath += infoPoint[5]
         #print(infoPoint[0])
         localArc = self.getArc(infoPoint[2], infoPoint[1], stSize, infoPoint[3], p, img, infoPoint[1])
         localPath = np.append(localPath, localArc, axis=0)
         lenPath = len(localPath)
         
         #print(localPath)
         for i in range(lenPath):

            localPath[i][0] = localPath[i][0]/20
            localPath[i][1] = localPath[i][1]/20

            x = localPath[i][0]
            y = localPath[i][1]
            a = localPath[i][2]

            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            msg.poses.append(pose)
            #print("add")

         self.waypoint_publisher.publish(msg)

        #print(localPath)
         
        #localPath = np.delete(localPath, len(localPath)-1, axis=0)

      scale_percent = 60 # percent of original size
      width1 = int(height * scale_percent / 100)
      height1 = int(width * scale_percent / 100)
      dim = (width1, height1)
      img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
      img = cv2.flip(img, 0)
      cv2.imshow("Image", img)
      cv2.waitKey(1)
        
      return localPath, allLenghtPath

        #self.grid = cv2.flip(self.grid, 0)
        #cv2.imshow("path", self.grid)
        #cv2.waitKey(1)
             


    def getArc(self, mode, maxc, step_size, angleAcr, sP, img, r):
      font = cv2.FONT_HERSHEY_SIMPLEX 
      color = (255, 255, 255) 
      fontScale = 0.5
      thickness = 2
      img = cv2.putText(img, str(mode), (int(sP.x),int(sP.y)), font,  fontScale, color, thickness, cv2.LINE_AA)
      #print("a", sP.x, sP.y, sP.a, mode)

      #rotatedX = 17*math.cos(sP.a + math.pi/2) + sP.x
      #rotatedY = 17*math.sin(sP.a + math.pi/2) + sP.y
      
      #img = cv2.line(img,(int(sP.x),int(sP.y)),(int(rotatedX),int(rotatedY)),(255,255,255),1)

      L = abs(math.pi*maxc*math.degrees(angleAcr))/180
      point_num = int(L / step_size) + 1 # колличество точек для отрисовки локального пути
      if(point_num < 2):
         return [sP.x, sP.y, sP.a, -1]
      #print("arc",mode, angleAcr, sP.a)
      #print(point_num)

      localArc = np.empty((0,4))

      for i in range(point_num):
         localArc = np.append(localArc, [[0,0,0,-1]], axis=0)
      print(localArc)
      
      localArc[0][2] = sP.a

      #print("old", localArc[0][2], mode)

      #print(localArc)
      #len_step = L/(point_num) 
      an = 0
      if mode == 'R':
        an = sP.a
        #print("a", sP.a)
        localArc[0][0] -= maxc
        r = -r
        #localArc[point_num].a = math.pi/2 + angleAcr

      if mode == 'L':
        an = 0
        #localArc[point_num].a = angleAcr
        localArc[0][0] += maxc
        
      
      directions = [0 for _ in range(point_num)] # создание массивов для точек

      angleStep = angleAcr/point_num

      angleStep1 = angleAcr/(point_num-1)
      #print("angleStep:", angleStep)

      if L > 0.0:
        directions[0] = 1
      else:
        directions[0] = -1

      if L > 0.0:
        d = step_size
      else:
        d = -step_size

      ll = angleStep
      #len_buff = len_step + sP.lenghtPath
      #print(mode)

      x = localArc[0][0]
      y = localArc[0][1]
      #img = cv2.circle(img,(int(x+sP.x),int(y + sP.y)), int(maxc), (0,255,0), 2)
      parent = len(self.Waypoints)
      a = 0
      
      for i in range(1, point_num):
        
         if mode == 'R':
            ll -= angleStep
            localArc[i][0] = x*math.cos(ll)+y*math.sin(ll)+ maxc
            localArc[i][1] = y*math.cos(ll)-x*math.sin(ll)
            a = an
            an -= abs(angleStep1)
            
             

         elif mode == 'L':
            ll -= angleStep
            localArc[i][0] = x*math.cos(ll)+y*math.sin(ll)- maxc
            localArc[i][1] = y*math.cos(ll)-x*math.sin(ll)
            a = an + sP.a
            an += abs(angleStep1) 

         #localArc[i].parent = parent
         parent += 1
         #len_buff += len_step
         #len_arr[i] = len_buff
         #print(a)
         localArc[i][2] = a
         
         #print(a)

      localArc[point_num-1][2] = angleAcr + sP.a
      #print("new", localArc[point_num-1][2])

      angle = sP.a 
      for i in range(1, point_num):
         x = localArc[i][0]
         y = localArc[i][1]

         localArc[i][0] = x*math.cos(angle)-y*math.sin(angle) + sP.x
         localArc[i][1] = y*math.cos(angle)+x*math.sin(angle) + sP.y
         localArc[i][3] = r

         rotatedX = 17*math.cos(localArc[i][2] + math.pi/2) + localArc[i][0]
         rotatedY = 17*math.sin(localArc[i][2] + math.pi/2) + localArc[i][1]
         img = cv2.circle(img,(int(localArc[i][0]),int(localArc[i][1])), 2, (0,255,0), -1)
         img = cv2.line(img,(int(localArc[i][0]),int(localArc[i][1])),(int(rotatedX),int(rotatedY)),(255,0,0),1)
         
         img = cv2.circle(img,(int(localArc[i][0]),int(localArc[i][1])), 3, (0,0,255), -1)
         #img = cv2.circle(img,(100,100), 3, (0,255,0), -1)
        
         #print(math.degrees(localArc[i].a)) 
      localArc[0][0] = sP.x
      localArc[0][1] = sP.y
      #localArc[0].parent = pointParent
      #img = cv2.line(img,(int(sP.x),int(sP.y)),(int(rotatedX),int(rotatedY)),(255,255,255),1)

      #print("a1", localArc[point_num].x, localArc[point_num].y, localArc[point_num].a, localArc[point_num].parent)

      #rotatedX = 17*math.cos(localArc[point_num][2] + math.pi/2) + localArc[point_num][0]
      #rotatedY = 17*math.sin(localArc[point_num][2] + math.pi/2) + localArc[point_num][1]

      #img = cv2.line(img,(int(localArc[point_num].x),int(localArc[point_num].y)),(int(rotatedX),int(rotatedY)),(255,0,0),1)
      #localArc[0] = Points(sP.)

      #print(L)
      #print(len_arr)
      #print(pyaw)
      return localArc    





    def checkR(self, R,pointP, pointC):
      mode = ''
      boolCheck = False
      #print(math.degrees(pointP[2]))
      vec1X = math.cos(pointP[2] + math.pi/2)
      vec1Y = math.sin(pointP[2] + math.pi/2)

      #rotatedX = 17*math.sin(sP.a) + sP.x
      #rotatedY = 17*math.cos(sP.a) + sP.y

      vec2X = pointC[0]- pointP[0]
      vec2Y = pointC[1]- pointP[1]

      anglePC = self.angleV1V2(vec1X, vec1Y, vec2X, vec2Y)
      
      if anglePC <= 0:
         mode = 'R'
      else:
         mode = 'L'

      dist = self.distance(pointP[0], pointP[1], pointC[0], pointC[1])
      r = dist/(2*math.cos((math.pi/2)-abs(anglePC)))
      if(r >= R and abs(anglePC) < math.pi/2):
         boolCheck = True
      angleAcr = 2*anglePC
      angleNew = angleAcr

      L = abs(math.pi*r*math.degrees(angleAcr))/180
      info = [boolCheck, r, mode, angleAcr, angleNew, L]

      return info
       

    def distance(self, x1, y1, x2, y2):
      c = math.sqrt((x2-x1)**2 + (y2-y1)**2)
      return c

    def angleV1V2(self, vec1X, vec1Y, vec2X, vec2Y):
      c = math.atan2(vec1X*vec2Y - vec1Y*vec2X, vec1X*vec2X + vec1Y*vec2Y)
      return c      
    

    def rndPont(self, hw, inc):
       x = randint(0, hw[0])
       y = randint(0, hw[1]) + inc
       return (int(x), int(y))
    
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
          
       if(point[0] > h):
          point[0] = h-1

       if(point[1] < 0):
          point[1] = 0

       if(point[1] > w):
          point[1] = w-1
       return point

    def UnVector(self, vec):
       u_hat = vec/np.linalg.norm(vec)
       return u_hat
    
    def IsPointInCircle(self, x, y, xc, yc, r):
      return ((x-xc)**2+(y-yc)**2) ** 0.5 <= r
    
       
    
class CreateGlobalRrt(Node):

   def __init__(self):
    
      super().__init__('global_path')

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
      self.ch = 0

      self.corrStartI = None
      self.corrEndI = None

      self.posX = 0
      self.posY = 0
      self.angleAutoPoint = 0
      self.current_map = 0
      self.height = 3751 
      self.width = 2749
      self.mapArr = np.empty((1,1))

      #self.timer = self.create_timer(1.5, self.timer_path_callback)
    #self.timer1 = self.create_timer(0.1, self.timer_path_out)

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
      '/map', 
      self.global_map_collback, 
      1)

    #print(self.current_map)

      self.waypoint_publisher = self.create_publisher(
            Path, '/pathWay', 1)
    
      self.pubposemarker = self.create_publisher(Marker, '/points', 1)
    
      self.global_path_publisher = self.create_publisher(Path, '/global_path', 10)
    
      self.pathArr = np.empty((0,3))
      with open('/home/ilya22/ros2_humble/src/robocross2023/paths/path2.csv', 'r') as file:
         reader = csv.reader(file)
         #print(reader)
         msg = Path()
         msg.header.frame_id = "map"
         for index, line in enumerate(reader):
            if(index > 0):
               x = float(line[0])
               y = float(line[1])
               a = float(line[2])
               self.pathArr = np.append(self.pathArr, [[x, y, a]], axis=0)
      
        #print(x, y, a)

   def angle_between_lines(self, p1, p2, p3):
    #print(p1, p2, p3)
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
     
   
           
     #print("END") 
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
      print(1)
      imageMapArr = np.rot90(self.mapArr)
      imageMapArr = np.flip(imageMapArr, axis = 1)
      width = len(self.mapArr)
      c = 0
      posCarX = -int(20*self.carX)
      posCarY = width+int(20*self.carY)  
      #print(posCarX, posCarY)
      includePoints = np.empty((0,4))
      #print(self.pathArr)
      for i in range(len(self.pathArr)):
         x = self.pathArr[i][0]*20
         y = self.pathArr[i][1]*20
         a = self.pathArr[i][2]

         includePoints = np.append(includePoints, [[x, y, a, int(i)]], axis=0)
            
           
      #print(includePoints)
      #includePoints = np.append(includePoints, includePoints[len(includePoints) - 1] + 1)  
             
      #print(includePoints)
      startPoint = (self.posX, self.posY, self.eulerAuto)
      start = True
      end = True
      backPoint = None
      seg = 1
      print(2)   
      if(end == True and start == True):
         print(3)
         print(len(imageMapArr))   
         if len(imageMapArr) > 1 and startPoint[0] + startPoint[1] > 0:
            print(4)
            minLen = math.inf
            minPath = None
            rrtStar = RrtStar((startPoint), includePoints, 1000, imageMapArr, 30, 100, self.eulerAuto, self.posX, self.posY, self.waypoint_publisher, self.pubposemarker)
            #print("GO")
            for i in range(5):
               print(5)
               path, lenghtPath = rrtStar.planning()
               #print(lenghtPath)
               if(lenghtPath < minLen):
                  minLen = lenghtPath
                  minPath = path

            print(minLen, minPath)

            with open('/home/ilya22/ros2_humble/src/robocross2023/paths/path1.csv', 'w', newline='') as file:
               writer = csv.writer(file)
               writer.writerow(["X", "Y", "A", "R"])

            for i in range(len(minPath)):
               with open('/home/ilya22/ros2_humble/src/robocross2023/paths/path1.csv', 'a', newline='') as file:
                  writer = csv.writer(file)
                  writer.writerow([minPath[i][0], minPath[i][1], minPath[i][2], minPath[i][3]])       
                  


             
      
      ##rrtStar = RrtStar((posCarX, posCarY), (posGaolX, posGaolY), 1000, imageMapArr, 10, 35, self.eulerAuto, self.posX, self.posY, self.waypoint_publisher)
      ##rrtStar.planning()

   def global_point_in_local_map(self, posX, posY, carXLocal, carYLocal, oldPose):
      #print("auto", math.degrees(self.eulerAuto))  
      length = 20*math.sqrt((oldPose[0] - self.posX)**2 + (oldPose[1] - self.posY)**2)
      VecGoalX = oldPose[0] - posX
      VecGoalY = oldPose[1] - posY

      VecAutoX = math.cos(self.eulerAuto + math.pi/2) #вектор авто
      VecAutoY = math.sin(self.eulerAuto + math.pi/2)


      engleGoaltoAuto = math.atan2(VecAutoX*VecGoalY - VecAutoY*VecGoalX, VecAutoX*VecGoalX + VecAutoY*VecGoalY)

      x = -int(length*math.sin(engleGoaltoAuto)) + carXLocal
      y = (int(length*math.cos(engleGoaltoAuto)) + carYLocal)


      VecPointX = math.cos(oldPose[2] + math.pi/2)
      VecPointY = math.sin(oldPose[2] + math.pi/2)

      engleGoalLocal = math.atan2(VecAutoX*VecPointY - VecAutoY*VecPointX, VecAutoX*VecPointX + VecAutoY*VecPointY)


      VecPointX = math.cos(engleGoalLocal + self.eulerAuto)
      VecPointY = math.sin(engleGoalLocal + self.eulerAuto)

      return [x, y, engleGoalLocal]
     

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
    #print("a",self.angleAutoPoint)
   
   def poseAuto_collback(self, data):
      euler = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
      self.eulerAuto = euler[2]
      #print(self.eulerAuto)
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

   def global_map_collback(self, data):
    self.height = data.info.height
    self.width = data.info.width
    #print("h:", self.height, "w", self.width)
    self.mapArr = np.reshape(data.data, (self.height, self.width))
    if self.ch == 0:
      self.timer_path_callback()
      self.ch = 1
    #print(self.mapArr)
    #self.carX = data.info.origin.position.x
    #self.carY = data.info.origin.position.y
    
  
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
  global_rrt = CreateGlobalRrt()
  
  # Spin the node so the callback function is called.
  rclpy.spin(global_rrt)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  global_rrt.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()