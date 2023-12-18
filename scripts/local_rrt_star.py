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
from time import sleep
from std_msgs.msg import UInt32MultiArray, Float32, Bool, String

class Points:
    def __init__(self, X, Y, A):
        self.x = X
        self.y = Y
        self.children = 0
        self.parent = -1
        self.lenghtPath = 0
        self.a = A


class RrtStar:
   def __init__(self, start, goals, iter, gridArr, stepSize, neighborhood, eulerAuto, posX, posY, waypoint_publisher, RCarF, RCarB):
        self.start = Points(start[0], start[1], start[2])
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
        self.RCarF = RCarF
        self.RCarB = RCarB
        self.waypoint_publisher = waypoint_publisher
        self.height = 0
        self.width = 0


   def planning(self):
      
      height= len(self.gridArray)
      width= len(self.gridArray[0])
      self.height = height
      self.width = width
      img = np.zeros((width,height,3), np.uint8)
      minPanh = math.inf
      stSize = 2
      #print(p)
      #print(int(p[1]))
      vec1X = math.cos(math.pi/2)
      vec1Y = math.sin(math.pi/2)
      #rotatedX = 17*math.sin(sP.a) + sP.x
      correctGoal = None
      img = cv2.rectangle(img, (0,height), (width, 0), (255,255,255), 4)
      #print(width)
      self.Waypoints = np.append(self.Waypoints, self.start)
      
      for i in range(height):
         for f in range(width):
            if self.gridArray[i][f] != 0:
               #print(i, f, self.gridArray[i][f])
               img = cv2.circle(img,(i, f), 1, (100,100,100), -1)


      
      img = cv2.rectangle(img, (0,20), (10,0), (255,255,255), 2)
      k = 0
      #img = cv2.rectangle(img, (int(car[0][0]),int(car[0][1])), (int(car[2][0]),int(car[2][1])), (255,255,255), 2)


      #img = cv2.circle(img,(self.start.x, self.start.y), 10, (0,0,255), -1)

      p = Points(250, 300, -math.pi/2)
      p1 = [135,250]

      
      

      while k < self.iterations:

         scale_percent = 150 # percent of original size
         width1 = int(height * scale_percent / 100)
         height1 = int(width * scale_percent / 100)
         dim = (width1, height1)
         #img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
         #img = cv2.flip(img, 0)
         cv2.imshow("Image", img)
         cv2.waitKey(1)

         
         #node = self.rndPont((width*3, int(height*3)), -(height*3/2 - height/2), width*3/2 - width/2)
         node = self.rndPont((width, height), height/6, 0)

         #img = cv2.circle(img,(node[0], node[1]), 3, (0,255,0), -1)
         min = 9999.0
         minPoint = -1
         point = Points(None, None, None)
         mode = ''

         for i in range(len(self.Waypoints)):
            if self.Waypoints[i].children < 3:
               dist = self.distance(node[0], node[1], self.Waypoints[i].x, self.Waypoints[i].y)
               if dist < min:  
                  pointBuff = self.steerPoint(self.Waypoints[i], node, width, height, self.stp)
                  ch, check = self.checkMain(self.Waypoints[i], pointBuff, img)

                  if ch == False and check[0] == True:
                     arc = self.getArc(check[2], check[1], 10, check[3], self.Waypoints[i], img, check[1])

                     if len(np.shape(arc)) > 1:
                        ch = False
                        #arc = self.getArc(check[2], check[1], 2, check[3], Waypoints, img, check[1])
                        #[boolCheck, r, mode, angleAcr, angleNew, L]
                        for j in range(len(arc)):
                           car = self.getLocalCar((arc[j][0], arc[j][1]), arc[j][2], width, height, img)
                           #img = cv2.circle(img,(int(arc[j][0]),  int(arc[j][1])), 1, (255,255,255), -1)
                           #sleep(0.1)
                           cv2.imshow("Image", img)
                           cv2.waitKey(1)    
                           chCar = self.checkTouch(car, width, height)

                           if chCar:
                              if j > 1:
                                 ch = True
                                 pointBuff[0] = arc[j][0]
                                 pointBuff[1] = arc[j][1]
                                 check[4] = arc[j][2]                                    
                           else:
                              break

                  if ch:                     
                     min = dist
                     point.x = pointBuff[0]
                     point.y = pointBuff[1]
                     point.a = check[4]
                     #img = cv2.circle(img,(point.x,  point.y), 3, (0,255,0), -1)
                     rotatedX = 17*math.cos(point.a + math.pi/2) + point.x
                     rotatedY = 17*math.sin(point.a + math.pi/2) + point.y
                     #img = cv2.line(img,(int(point.x),int(point.y)),(int(rotatedX),int(rotatedY)),(255,0,0),1)

                     point.parent = i
                     self.Waypoints[i].children += 1
                     mode = check[2]



            '''
            
            if check[0]:
               car = self.getLocalCar(pointBuff, self.Waypoints[i].a + check[4], width, height, img)
               ch = self.checkTouch(car, width, height)
               if ch:                  
                  if self.Waypoints[i].lenghtPath + check[5] < min:
                     min = self.Waypoints[i].lenghtPath + check[5]
                     point.a = check[4] + self.Waypoints[point.parent].a
                     point.lenghtPath = min
                     point.x = pointBuff[0]
                     point.y = pointBuff[1]
                     point.parent = i

               else:
                  #print(check)
                  if check[2] == 'L' or check[2] == 'R':

                     #print(check)
                     rotatedX = 30*math.cos(self.Waypoints[i].a) + self.Waypoints[i].x
                     rotatedY = 30*math.sin(self.Waypoints[i].a) + self.Waypoints[i].y

                     #img = cv2.line(img,(int(self.Waypoints[i].x),int(self.Waypoints[i].y)),(int(rotatedX),int(rotatedY)),(255,0,0),3)


                     p = Points(rotatedX, rotatedY, self.Waypoints[i].a)

                     arc = self.getArc('L', 30, 4, math.pi, p, img, 30)
                  
                     for j in range(len(arc)):
                        #img = cv2.circle(img,(int(arc[j][0]),int(arc[j][1])), 3, (255,0,0), -1)
                        check = self.checkR(self.RCar, [self.Waypoints[i].x, self.Waypoints[i].y, self.Waypoints[i].a], (arc[j][0], arc[j][1]))
                        if check[0]:
                           car = self.getLocalCar((arc[j][0], arc[j][1]), self.Waypoints[i].a + check[4], width, height, img)
                           ch = self.checkTouch(car, width, height)
                           if ch:
                  
                              if self.Waypoints[i].lenghtPath + check[5] < min:
                                 min = self.Waypoints[i].lenghtPath + check[5]
                                 point.a = check[4] + self.Waypoints[point.parent].a
                                 point.lenghtPath = min
                                 point.x = (arc[j][0], arc[j][1])[0]
                                 point.y = (arc[j][0], arc[j][1])[1]
                                 point.parent = i

                  elif check[2] == '-L' or check[2] == '-R':

                     #print(check)
                     rotatedX = 30*math.cos(self.Waypoints[i].a + math.pi) + self.Waypoints[i].x
                     rotatedY = 30*math.sin(self.Waypoints[i].a + math.pi) + self.Waypoints[i].y

                     #img = cv2.line(img,(int(self.Waypoints[i].x),int(self.Waypoints[i].y)),(int(rotatedX),int(rotatedY)),(255,0,0),3)


                     p = Points(rotatedX, rotatedY, self.Waypoints[i].a)

                     arc = self.getArc('R', 30, 4, math.pi, p, img, 30)
                  
                     for j in range(len(arc)):
                        #img = cv2.circle(img,(int(arc[j][0]),int(arc[j][1])), 3, (255,0,0), -1)
                        check = self.checkR(self.RCar, [self.Waypoints[i].x, self.Waypoints[i].y, self.Waypoints[i].a], (arc[j][0], arc[j][1]))
                        if check[0]:
                           car = self.getLocalCar((arc[j][0], arc[j][1]), self.Waypoints[i].a + check[4], width, height, img)
                           ch = self.checkTouch(car, width, height)
                           if ch:
                  
                              if self.Waypoints[i].lenghtPath + check[5] < min:
                                 min = self.Waypoints[i].lenghtPath + check[5]
                                 point.a = check[4] + self.Waypoints[point.parent].a
                                 point.lenghtPath = min
                                 point.x = (arc[j][0], arc[j][1])[0]
                                 point.y = (arc[j][0], arc[j][1])[1]
                                 point.parent = i




         #point = self.steerPoint(self.Waypoints[minPoint], node, width, width, self.stp)
          #print(point[0], point[1])
         #img = cv2.circle(img,(point.x, point.y), 3, (0,255,0), -1)

          #print(self.gridArr)
         ''' 
         '''
         if(len(self.gridArray) > 2):
            if(np.any(self.gridArray[point[0]][point[1]]) == 0 or point[0] > width or point[1] > height):
               point = Points(point[0], point[1], 0)
               point.parent = minPoint
              
               ##point.lenghtPath = self.distance(point.x, point.y, self.Waypoints[point.parent].x, self.Waypoints[point.parent].y)
               closelyPointArray = np.array([], 'int32')
               min = 9999.0
               #начало RRT
               check = None
               for i in range(len(self.Waypoints)): # ищем самый малый по длине путь
                  if(self.IsPointInCircle(self.Waypoints[i].x, self.Waypoints[i].y, point.x, point.y, self.nhood)):
                    closelyPointArray = np.append(closelyPointArray, i) #находим самый кородкий путь по точки по длине

                    check = self.checkR(self.RCar, [self.Waypoints[i].x, self.Waypoints[i].y, self.Waypoints[i].a], (point.x, point.y))

                    if self.Waypoints[i].lenghtPath + check[5] < min:
                      if check[0]:
                        car = self.getLocalCar(pointBuff, self.Waypoints[i].a + check[4], width, height, img)
                        ch = self.checkTouch(car, width, height)
                        if ch:
                           min = self.Waypoints[i].lenghtPath + check[5]
                           point.parent = i
                           point.lenghtPath = min
                           point.a = check[4] + self.Waypoints[point.parent].a
               #print(" ")
               #print(self.Waypoints[point.parent].x, "," ,self.Waypoints[point.parent].y)#, self.Waypoints[point.parent].a, point.x,",",point.y)
               #print(point.x,",",point.y)
               #print(math.degrees(self.Waypoints[point.parent].a))
               #print(point.parent)
               #img = cv2.circle(img,(point.x,point.y), 3, (0,0,255), -1)
               #img = cv2.circle(img,(int(self.start.x),int(self.start.y)), 3, (255,0,0), -1)
               #cv2.line(img,(int(self.Waypoints[point.parent].x), int(self.Waypoints[point.parent].y)),((point.x,point.y)),(255,0,0),5)
         '''

         if(point.parent != -1):
                  
                  #print(point.x, point.y, k)
                  self.Waypoints = np.append(self.Waypoints, point)
                  #img = cv2.circle(img,(int(point.x),int(point.y)), 3, (0,0,255), -1)
                  car = self.getLocalCar((point.x, point.y), point.a, width, height, img)

                  for j in range(len(car)): 
                     None        
                     img = cv2.circle(img,(int(car[j][0]), int(car[j][1])), 2, (0,0,255), -1)                        
                  #k += 1

                  
                  #print(point.parent)
                  #infoPoint = self.checkR(20, [self.Waypoints[point.parent].x, self.Waypoints[point.parent].y, self.Waypoints[point.parent].a], (point.x, point.y))
                  #print(infoPoint)
                  #localArc = self.getArc(infoPoint[2], infoPoint[1],3, infoPoint[3], self.Waypoints[point.parent],img,point.parent)

               
               #point.lenghtPath = self.distance(point.x, point.y, self.Waypoints[point.parent].x, self.Waypoints[point.parent].y) + self.Waypoints[point.parent].lenghtPath
               #print(point.lenghtPath) 

                  #self.Waypoints = np.append(self.Waypoints, localArc)
                  #self.Waypoints = np.append(self.Waypoints, localArc)
                  

               #начало STAR
                  closelyPointArray = np.array([], 'int32')
                  for i in range(len(self.Waypoints)):
                     if(self.IsPointInCircle(self.Waypoints[i].x, self.Waypoints[i].y, point.x, point.y, self.nhood)):
                        closelyPointArray = np.append(closelyPointArray, i)


                  for i in range(len(closelyPointArray)):

                     check = self.checkR((point.x, point.y, point.a) ,[self.Waypoints[closelyPointArray[i]].x, self.Waypoints[closelyPointArray[i]].y, self.Waypoints[closelyPointArray[i]].a])
                     newLength = point.lenghtPath + check[5]

                     #print(check[0], check[4] + point.a, self.Waypoints[closelyPointArray[i]].a)

                     if self.Waypoints[closelyPointArray[i]].lenghtPath > newLength and check[0] and abs(check[4] + point.a - self.Waypoints[closelyPointArray[i]].a) < 0.05:
                        #print("Перестройка")     
                        self.Waypoints[closelyPointArray[i]].parent = len(self.Waypoints) - 1
                        self.Waypoints[closelyPointArray[i]].lenghtPath = newLength

                  
                  for i in range(len(self.goals)):
                     
                     
                     #img = cv2.circle(img,(int(self.goals[i][0]), int(self.goals[i][1])), 3, (0,255,0), -1)
                     goal = Points(self.goals[i][0], self.goals[i][1], self.goals[i][2])
                     #print(self.goals[i][0], self.goals[i][1], self.goals[i][2])

                  

                     check = self.checkR((point.x, point.y, point.a), (goal.x, goal.y, goal.a))
                     
                     if check[0]:
                        c = True
                        
                        #img = cv2.circle(img,(int(goal.x),int(goal.y)), 3, (255,255,255), -1)

                        rotatedX = 17*math.cos(goal.a + math.pi/2) + goal.x
                        rotatedY = 17*math.sin(goal.a + math.pi/2) + goal.y

                        #img = cv2.line(img,(int(goal.x),int(goal.y)),(int(rotatedX),int(rotatedY)),(255,0,0),1)
                        
                        #print(abs((check[4] + point.a) - (self.goals[i][2])), check[5] , np.any(correctGoal), point.a)
                        #print(abs((check[4] + point.a) - (self.goals[i][2])) < 0.3, point.a != 0)
                        if abs((check[4] + point.a) - (self.goals[i][2])) < 0.15 and point.a != 0 and check[5] < 600: 
                           #[boolCheck, r, mode, angleAcr, angleNew, L]
                           arc = self.getArc(check[2], check[1], 10, check[3], point, img, check[1])
                           ch = self.checkArc(arc, check, point, img, width, height)
                           #ch = True

                           if ch :
                              #img = cv2.circle(img,(int(point.x),int(point.y)), 4, (0,255,255), -1)
                              #img = cv2.circle(img,(int(goal.x),int(goal.y)), 10, (0,255,255), -1)

                              rotatedX = 17*math.cos(point.a + math.pi/2) + point.x
                              rotatedY = 17*math.sin(point.a + math.pi/2) + point.y

                              #img = cv2.line(img,(int(point.x),int(point.y)),(int(rotatedX),int(rotatedY)),(255,0,0),1)
                        
                              self.count = len(self.Waypoints)-1
                              #print(len(self.goals))
                              correctGoal = self.goals[i]
                              print("1")
                              break
                              

                              #print(point.a, goal.a, check[4]+point.a)
                  #print("dsfgvmuhygtgyhuijikytvosyudfibv")
                  if np.any(correctGoal) != None :
                     print(self.count) 
                     #print("end", k)
                     break
                                        
               #else:
                  #k -= 1           
         else:
            None    #self.grid = cv2.line(self.grid, (point.x, point.y), (self.Waypoints[minPoint].x, self.Waypoints[minPoint].y), (50,50,50), 2)
              
               
        #print("ggg ")

      print(2)
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

            font = cv2.FONT_HERSHEY_SIMPLEX 
            color = (255, 255, 255) 
            fontScale = 1
            thickness = 2

            #img = cv2.putText(img, str(self.Waypoints[self.count].parent), (int(self.Waypoints[self.Waypoints[self.count].parent].x),int(self.Waypoints[self.Waypoints[self.count].parent].y)), font,  fontScale, color, thickness, cv2.LINE_AA) 
            #img = cv2.line(img,(int(x),int(y)),(int(self.Waypoints[self.Waypoints[self.count].parent].x),int(self.Waypoints[self.Waypoints[self.count].parent].y)),(255,255,255),1)
            cv2.imshow("Image", img)
            cv2.waitKey(1)

            if(a != -999):
               a1 = self.eulerAuto + a
            else:
               a1 = a
   
            x1 = ((x - width/2)*math.cos(self.eulerAuto) - (y - width/2)*math.sin(self.eulerAuto)) / 20.0 + self.posX
            y1 = ((y - width/2)*math.cos(self.eulerAuto) + (x - width/2)*math.sin(self.eulerAuto)) / 20.0 + self.posY

            localPath = np.append(localPath, [[x, y, a, r]], axis=0)

            self.count = self.Waypoints[self.count].parent
            print("p", self.count)
         else:
            print(3)
            break
          

      localPath = np.flip(localPath, axis = 0)
      lenPath = len(localPath)
     
      i = 0

      while i < lenPath - 1:

         p = Points(localPath[i][0], localPath[i][1], localPath[i][2])

         car = self.getLocalCar((p.x,p.y), p.a, width, height, img)
         for j in range(len(car)):
            img = cv2.circle(img,(int(car[j][0]), int(car[j][1])), 2, (0,0,255), -1)

        #("i", i)

         #print(p.x, p.y, p.a)
         infoPoint = self.checkR([p.x, p.y, p.a], [localPath[i+1][0], localPath[i+1][1], localPath[i+1][2]])
         #print(infoPoint[0])
         localArc = self.getArc(infoPoint[2], infoPoint[1], stSize, infoPoint[3], p, img, infoPoint[1])
         #print(localArc)
         #print(localArc)
         #print(localPath)
         #print("old", lenPath)
         localPath = np.insert(localPath, i+1, localArc, axis = 0)
         lenPath = len(localPath)
         #print("localArc", len(localArc))
         #print(" ")
         i += len(localArc) + 1

      lenPath = len(localPath)
      
      if lenPath > 0:
         p = Points(localPath[lenPath - 1][0], localPath[lenPath - 1][1], localPath[lenPath - 1][2])
         infoPoint = self.checkR([p.x, p.y, p.a], [correctGoal[0], correctGoal[1], correctGoal[2]])
         #print(infoPoint[0])
         localArc = self.getArc(infoPoint[2], infoPoint[1], stSize, infoPoint[3], p, img, infoPoint[1])
         localPath = np.append(localPath, localArc, axis=0)
         lenPath = len(localPath)
         
         #print(localPath)
         
         for i in range(lenPath):

            x = localPath[i][0]
            y = localPath[i][1]
            a = localPath[i][2]
            r = localPath[i][3]

            #print(x, y)

            #print(x, y)

            x1 = ((x - width/2)*math.cos(self.eulerAuto) - (y - width/2)*math.sin(self.eulerAuto)) / 20.0 + self.posX
            y1 = ((y - width/2)*math.cos(self.eulerAuto) + (x - width/2)*math.sin(self.eulerAuto)) / 20.0 + self.posY
            a1 = self.eulerAuto + a

            localPath[i][0] = x1
            localPath[i][1] = y1
            localPath[i][2] = a1
            localPath[i][3] = r
            #print(x, y, a)
            pose = PoseStamped()
            #print(x1, y1)
            pose.pose.position.x = x1
            pose.pose.position.y = y1
            pose.pose.position.z = 0.0

            

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            msg.poses.append(pose)

         self.waypoint_publisher.publish(msg)

        #print(localPath)
         
        #localPath = np.delete(localPath, len(localPath)-1, axis=0)






      return localPath, correctGoal

        #self.grid = cv2.flip(self.grid, 0)
        #cv2.imshow("path", self.grid)
        #cv2.waitKey(1)
   def getInterCar(self, car, seg):
      interCar = np.empty((0,2))
      i = 0
      
      while i < len(car):

         if i < len(car)-1:
            dist = self.distance(car[i][0], car[i][1], car[i+1][0], car[i+1][1])
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint(Points(car[i][0], car[i][1], 0), car[i+1], math.inf, math.inf, segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg
         else:
            dist = self.distance(car[i][0], car[i][1], car[0][0], car[0][1])
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint(Points(car[i][0], car[i][1], 0), car[0], math.inf, math.inf, segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg

            dist = self.distance(car[0][0], car[0][1], car[2][0], car[2][1])
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint(Points(car[0][0], car[0][1], 0), car[2], math.inf, math.inf, segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg

            dist = self.distance(car[1][0], car[1][1], car[3][0], car[3][1])
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint(Points(car[1][0], car[1][1], 0), car[3], math.inf, math.inf, segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg
         i += 1
      return interCar
   

   def checkMain(self, p, p1, img):
      ch = False
      
      car = self.getLocalCar((p.x, p.y), p.a, self.width, self.height, img)
      #img = cv2.circle(img,(int(p.x), int(p.y)), 2, (255,0,0), -1)
      ch0 = self.checkTouch(car, self.width, self.height)
      #print("ПРОВЕРКА препядствий", ch0)

      for j in range(len(car)): 
         None        
         #img = cv2.circle(img,(int(car[j][0]), int(car[j][1])), 2, (0,0,255), -1)

      check = self.checkR([p.x, p.y, p.a], p1)
      #print(check)

      #info = [boolCheck, r, mode, angleAcr, angleNew, L]
      check[4] += p.a
      check[5] += p.lenghtPath

      car1 = self.getLocalCar(p1, check[4], self.width, self.height, img)
      ch1 = self.checkTouch(car1, self.width, self.height)

      if check[0] and ch1:
         ch = True
        #img = cv2.circle(img,(int(p1[0]), int(p1[1])), 2, (0,0,255), -1)
         #print("ПРОВЕРКА препядствий 1", ch1)

         for j in range(len(car1)): 
            None        
            #img = cv2.circle(img,(int(car1[j][0]), int(car1[j][1])), 2, (0,0,255), -1)

      return [ch, check]


   def getLocalCar(self, p, angle, width, height, img):
      inc = 0
      whCar = (40,80)
      car = [[p[0] - whCar[0]/2 + abs(width/2 - self.start.x), p[1] + whCar[1]/2 + abs(height/2 - self.start.y) + inc],
             [p[0] + whCar[0]/2 + abs(width/2 - self.start.x), p[1] + whCar[1]/2 + abs(height/2 - self.start.y) + inc],
             [p[0] + whCar[0]/2 + abs(width/2 - self.start.x), p[1] - whCar[1]/2 + abs(height/2 - self.start.y) + inc], 
             [p[0] - whCar[0]/2 + abs(width/2 - self.start.x), p[1] - whCar[1]/2 + abs(height/2 - self.start.y) + inc]]
      
      seg = 8
      car = self.getInterCar(car, seg)

      for i in range(len(car)):
         car[i][0] -= p[0]
         car[i][1] -= p[1]
         x = car[i][0]
         y = car[i][1]
         car[i][0] = x*math.cos(angle) - y*math.sin(angle) + p[0]
         car[i][1] = x*math.sin(angle) + y*math.cos(angle) + p[1]
      
      for j in range(len(car)): 
         None        
         #img = cv2.circle(img,(int(car[j][0]), int(car[j][1])), 2, (0,0,255), -1)

      
      return car
   




   
   def checkArc(self, arc, check, Waypoints, img, width, height):
      ch = False
      if len(np.shape(arc)) > 1:


         ch = True
         #arc = self.getArc(check[2], check[1], 2, check[3], Waypoints, img, check[1])
         for j in range(len(arc)):
            car = self.getLocalCar((arc[j][0], arc[j][1]), arc[j][2], width, height, img)

            for i in range(len(car)):
               None     
               #img = cv2.circle(img,(int(car[i][0]), int(car[i][1])), 2, (0,0,255), -1)

            chCar = self.checkTouch(car, width, height)          
            if chCar == False:
               ch = False
               break
      #print(print(int(arc[j][0]), int(arc[j][1])), ch)      
      return ch
   

   def checkTouch(self, car, width, height):
      ch = True
      for j in range(len(car)):

         #print(int(car[j][0]),int(car[j][1]))          
         if int(car[j][0]) < width and int(car[j][1]) < height and int(car[j][0]) > 0 and int(car[j][1]) > 0 and np.any(self.gridArray[int(car[j][0])][int(car[j][1])]) != 0 :
            ch = False
            break   
      return ch


   def getArc(self, mode, maxc, step_size, angleAcr, sP, img, r):
      #print("angleAcr", angleAcr)

      d = True
      if mode == '-R' or mode == '-L':
         d = False
      #print("d", d)
      
      font = cv2.FONT_HERSHEY_SIMPLEX 
      color = (255, 255, 255) 
      fontScale = 0.5
      thickness = 2

      L = abs(math.pi*maxc*(math.degrees(angleAcr)))/180
      #print("L", L)
      point_num = int(L / step_size)+1 # колличество точек для отрисовки локального пути
      if(point_num < 2):
         return [sP.x, sP.y, sP.a, -1]
      #print("arc",mode, angleAcr, sP.a)
      #print(point_num)

      localArc = np.empty((0,4))

      for i in range(point_num):
         localArc = np.append(localArc, [[0,0,0,-1]], axis=0)
      
      localArc[0][2] = sP.a
      an = 0
      if mode == 'R' or mode == '-R':
        an = sP.a
        #print("a", sP.a)
        localArc[0][0] -= maxc
        r = -r
        #localArc[point_num].a = math.pi/2 + angleAcr

      if mode == 'L' or mode == '-L':
        an = 0
        localArc[0][0] += maxc
        


      angleStep = angleAcr/point_num
      angleStep1 = angleAcr/(point_num)
      
      if d:
         ll = angleStep
      else:
         ll = angleStep

         
         
      #len_buff = len_step + sP.lenghtPath
      #print(mode)

      x = localArc[0][0]
      y = localArc[0][1]
      #img = cv2.circle(img,(int(x+sP.x),int(y + sP.y)), int(maxc), (0,255,0), 2)
      parent = len(self.Waypoints)
      a = 0
      
      for i in range(1, point_num):
        
         if mode == 'R' or mode == '-R':
            ll -= angleStep
            localArc[i][0] = x*math.cos(ll)+y*math.sin(ll)+ maxc
            localArc[i][1] = y*math.cos(ll)-x*math.sin(ll)
            
            a = an
            if d:   
               an -= abs(angleStep1)
            else:
               an += abs(angleStep1)

            
             

         elif mode == 'L' or mode == '-L':
            #print("an", an)
            ll -= angleStep
            localArc[i][0] = x*math.cos(ll)+y*math.sin(ll)- maxc
            localArc[i][1] = y*math.cos(ll)-x*math.sin(ll)
            


            a = an + sP.a 
            if d:   
               an += abs(angleStep1)
            else:
               an -= abs(angleStep1)
            #print("a", a)

         #localArc[i].parent = parent
         parent += 1
         #len_buff += len_step
         #len_arr[i] = len_buff
         #print(a)
         localArc[i][2] = a
         
         #print(a)

      localArc[point_num-1][2] = angleAcr + sP.a
      localArc[0][1] = sP.y
      localArc[0][0] = sP.x
      #print("new", localArc[point_num-1][2])

      angle = sP.a 
      for i in range(1, point_num):
         x = localArc[i][0]
         y = localArc[i][1]

         localArc[i][0] = x*math.cos(angle)-y*math.sin(angle) + sP.x
         localArc[i][1] = y*math.cos(angle)+x*math.sin(angle) + sP.y
         localArc[i][3] = r

         #print("pa", localArc[i][0], localArc[i][1])

         #rotatedX = 50*math.cos(localArc[i][2] + math.pi/2) + localArc[i][0]
         #rotatedY = 50*math.sin(localArc[i][2] + math.pi/2) + localArc[i][1]
         #img = cv2.circle(img,(int(localArc[i][0]),int(localArc[i][1])), 2, (0,255,0), -1)
         
         
         #img = cv2.circle(img,(int(localArc[i][0]),int(localArc[i][1])), 3, (0,0,255), -1)
         #img = cv2.line(img,(int(localArc[i][0]),int(localArc[i][1])),(int(rotatedX),int(rotatedY)),(255,0,0),1)
         #img = cv2.circle(img,(100,100), 3, (0,255,0), -1)
        
         #print(math.degrees(localArc[i].a)) 
      #localArc[0].parent = pointParent
      #img = cv2.line(img,(int(sP.x),int(sP.y)),(int(rotatedX),int(rotatedY)),(255,255,255),1)

      #print("a1", localArc[point_num].x, localArc[point_num].y, localArc[point_num].a, localArc[point_num].parent)

      #rotatedX = 17*math.cos(localArc[point_num][2] + math.pi/2) + localArc[point_num][0]
      #rotatedY = 17*math.sin(localArc[point_num][2] + math.pi/2) + localArc[point_num][1]

      #img = cv2.line(img,(int(localArc[point_num].x),int(localArc[point_num].y)),(int(rotatedX),int(rotatedY)),(255,0,0),1)
      #localArc[0] = Points(sP.)

      #print(L)
      #print(len_arr)

     
      return localArc       





   def checkR(self,pointP, pointC):
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
      #print(anglePC)
      
      if anglePC <= 0:
         mode = 'R'
      else:
         mode = 'L'

      dist = self.distance(pointP[0], pointP[1], pointC[0], pointC[1])
      r = dist/(2*math.cos((math.pi/2)-abs(anglePC)))  

      if abs(anglePC) > math.pi/2:
         mode = '-' + mode   
         angleAcr = 2*anglePC
         if(angleAcr > 0):
            angleAcr = -(2*math.pi - angleAcr)

         else:       
            angleAcr = -(-2*math.pi - angleAcr)
         if(r >= self.RCarB):
               boolCheck = True
            
   
      else:
         angleAcr = 2*anglePC
         if(r >= self.RCarF):
               boolCheck = True
         

      angleNew = angleAcr
      L = abs(math.pi*r*math.degrees(angleAcr))/180
      info = [boolCheck, r, mode, angleAcr, angleNew, L]
      #print(boolCheck, pointC)
      return info
       

   def distance(self, x1, y1, x2, y2):
      c = math.sqrt((x2-x1)**2 + (y2-y1)**2)
      return c

   def angleV1V2(self, vec1X, vec1Y, vec2X, vec2Y):
      c = math.atan2(vec1X*vec2Y - vec1Y*vec2X, vec1X*vec2X + vec1Y*vec2Y)
      return c      
    

   def rndPont(self, hw, inc, corr):
       x = randint(0, hw[0]-1) - corr
       y = randint(0, hw[1]-1) + inc
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

    self.localMapArr = np.empty((1,1))
    self.cPoint = 0

    self.odomX = 0
    self.odomY = 0
    self.eulerOdom = 0

    self.timer = self.create_timer(0.5, self.timer_path_callback)
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
      Marker, 
      '/odomAuto', 
      self.odomAuto_collback, 
      10)

    self.subscription = self.create_subscription(
      OccupancyGrid,
      '/local_cost_map', 
      self.local_map_collback, 
      1)
    self.nav_status_pub = self.create_publisher(String, '/nav_status', 10)
    #print(self.current_map)

    self.start = np.empty((0,3))
    self.waypoint_publisher = self.create_publisher(
            Path, '/pathWay', 1)
    
    self.global_path_publisher = self.create_publisher(Path, '/global_path', 10)
    
    self.pathArr = np.empty((0,4))
    with open('/home/ilya22/ros2_humble/src/robocross2023/paths/path1.csv', 'r') as file:
     reader = csv.reader(file)
     #print(reader)
     msg = Path()
     msg.header.frame_id = "map"
     for index, line in enumerate(reader):
       if(index > 0):
        x = float(line[0])
        y = float(line[1])
        a = float(line[2])
        r = float(line[3])
        self.pathArr = np.append(self.pathArr, [[x, y, a, r]], axis=0)
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
      

      for i in range(int(len(self.pathArr)/4)):
         #print(self.carX, self.carY, self.pathArr[i][0], self.pathArr[i][1])
         if self.IsPointInCircle(self.odomX, self.odomY, self.pathArr[i][0], self.pathArr[i][1], 0.05) and i < len(self.pathArr)-1 and i - self.cPoint < 2:
            #print("delete")
            self.pathArr = np.delete(self.pathArr, np.s_[0:i+1], axis = 0)
            self.cPoint = i
            break
      
      
      lenArr = len(self.pathArr)
      msg = Path()
      msg.header.frame_id = "map"
      for i in range(len(self.pathArr)):

         x = self.pathArr[i][0]
         y = self.pathArr[i][1]
         r = self.pathArr[i][3]
            #print(r)

         pose = PoseStamped()
         pose.pose.position.x = x
         pose.pose.position.y = y
         pose.pose.position.z = 0.0

         pose.pose.orientation.x = r
         pose.pose.orientation.y = 0.0
         pose.pose.orientation.z = 0.0
         pose.pose.orientation.w = 0.0
         msg.poses.append(pose)

      #print("out")
      self.global_path_publisher.publish(msg)
    
   def dist(self, x1, y1, x2, y2):
     dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
     return dist

     
   def timer_path_callback(self):
      
      imageMapArr = self.mapArr
      imageMapArr = np.rot90(imageMapArr)
      imageMapArr = np.flip(imageMapArr, axis = 0)

      self.localMapArr = imageMapArr


      #self.mapArr = imageMapArr
      width = len(self.mapArr)
      height = len(self.mapArr[0])
      c = 0
      posCarX = -int(20*self.carX)
      posCarY = width+int(20*self.carY)  
      self.cPoint = 0
      print(self.posX,self.posY, posCarX, posCarY)
      img = np.zeros((width*2,height*2,3), np.uint8)



      

      includePoints = np.empty((0,4))

      for i in range(len(self.pathArr)-1):
         px = self.pathArr[i][0]
         py = self.pathArr[i][1]
         pa = self.pathArr[i][2]
         #print(px, py, pa)
         p = self.global_point_in_local_map(self.posX,self.posY, posCarX, posCarY, [px, py, pa])
         
         #print("p", p)
         x,y,a = p[0],p[1],p[2]
         
         
         #print(x, y)

         if x <= width and x >= 0 and y <= width and y >= 0:
            c += 1
            #print("add")
            #print(px,py,pa)
            #print('x = ', x)
            #print('y = ', y)
            #print('i = ', i)
            #print('c = ', c)
            if i - c >= 2:
               #print("all")
               break
              #print("add")

         else:
            c = i
         includePoints = np.append(includePoints, [[x, y, a, int(i)]], axis=0)
            
           
      #print(includePoints)
      #includePoints = np.append(includePoints, includePoints[len(includePoints) - 1] + 1)  
             
      #print(includePoints)
      startPoint = (0,0,0)
      p = self.global_point_in_local_map(self.posX,self.posY, posCarX, posCarY, [self.odomX, self.odomY, self.eulerOdom])
      startPoint = [p[0], p[1], p[2]]
      self.start = startPoint
      endPoints = []
      start = False
      end = False
      backPoint = None
      seg = 2
      segi = 0
      
      for i in range(len(includePoints)-1):
         
         p1 = includePoints[i]
         p2 = includePoints[i + 1]
         #print(i)
         
         #print(p1, " ", p2)
         segment = seg
         #print(p1, p2)
         while(self.distance(p1, p2) > segment):
          pr = self.steerPoint(p1, p2, segment)
          #print(pr)
          if pr[0] > width-1 or pr[1] > width-1: 
            break
          
          segment += seg
          
          #print(imageMapArr[pr[1], pr[0]])

          car = self.getLocalCar((pr[0], pr[1]), p1[2],width, height, img)
          ch = self.checkTouch(car, width, height)
          if(ch == False):
             #print("in", start, end)
             if start and end:
                start = False
                end = False
             #print(backPoint) 
             #print("препядствие: ", pr)
             #print(imageMapArr[pr[1], pr[0]])
             if start == False:
               self.corrStartI = includePoints[i][3]
               #print("нашли старт: ", backPoint)
               #print('i = ', i)
               start = True
             if start == True:
                segi += 1
                      
          else:
             #print("out", end, start)
             if end == False and start == True:
                #endPoint = pr
                self.corrEndI = includePoints[i][3]
                #print(int(self.corrEndI), int(len(includePoints)-1))
                endPoints = includePoints[int(self.corrEndI)+1: int(len(includePoints))]
                #print(endPoints)
                end = True
             elif end == False and start == False:
                backPoint = pr
                #print(backPoint) 
          #print(start, end, segi)       
          #print(end, start, includePoints)
          #print(segi)
      #print(start, end)   
      if(end == True and start == True):
             
             img = cv2.flip(img, 0)
             cv2.imshow("Image1", img)
             cv2.waitKey(1)
             
             start = False
             end = False
             #print("planning")
             #print(startPoint)
             if np.any(startPoint) != None:

              msg = String()
              msg.data = "Stop"
              self.nav_status_pub.publish(msg)
                            
              rrtStar = RrtStar((startPoint), endPoints, 20, imageMapArr, 50, 100, self.eulerAuto, self.posX, self.posY, self.waypoint_publisher, 55, 70)
              print("GO")
              
              path, correctGoal = rrtStar.planning()

              print("goal", correctGoal)
              x = startPoint[0]
              y = startPoint[1]
              #xStart = ((x - width/2)*math.cos(self.eulerAuto) - (y - width/2)*math.sin(self.eulerAuto)) / 20.0 + self.posX
              #yStart = ((y - width/2)*math.cos(self.eulerAuto) + (x - width/2)*math.sin(self.eulerAuto)) / 20.0 + self.posY
             #path = np.append(path, [[xStart, yStart]], axis = 0)
              #path = np.flip(path, axis = 0)
             #print(path)
             
             #print(self.corrStartI, self.corrEndI)
             #print(self.pathArr)
              print(np.any(correctGoal))
              if np.any(correctGoal) != None:
               
               self.pathArr = np.delete(self.pathArr, np.s_[0:int(correctGoal[3]+2)], axis = 0)
               self.pathArr = np.insert(self.pathArr, 0, path, axis = 0)
               msg = String()
               msg.data = "Go"
               self.nav_status_pub.publish(msg)
              
             
         

      length = 20*math.sqrt((self.goalX - self.posX)**2 + (self.goalY - self.posY)**2)
      posGaolX = -int(length*math.sin(self.angleAutoPoint)) + posCarX
      posGaolY = int(length*math.cos(self.angleAutoPoint)) + posCarY
      
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
     

   def getLocalCar(self, p, angle, width, height, img):

      whCar = (20,60)
      car = [[p[0] - whCar[0]/2 + abs(width/2 - self.start[0]), p[1] + whCar[1]/2 + abs(height/2 - self.start[1])],
             [p[0] + whCar[0]/2 + abs(width/2 - self.start[0]), p[1] + whCar[1]/2 + abs(height/2 - self.start[1])],
             [p[0] + whCar[0]/2 + abs(width/2 - self.start[0]), p[1] - whCar[1]/2 + abs(height/2 - self.start[1])], 
             [p[0] - whCar[0]/2 + abs(width/2 - self.start[0]), p[1] - whCar[1]/2 + abs(height/2 - self.start[1])]]
      
      seg = 5
      car = self.getInterCar(car, seg)

      for i in range(len(car)):
         car[i][0] -= p[0]
         car[i][1] -= p[1]
         x = car[i][0]
         y = car[i][1]
         car[i][0] = x*math.cos(angle) - y*math.sin(angle) + p[0]
         car[i][1] = x*math.sin(angle) + y*math.cos(angle) + p[1]
      
      for j in range(len(car)):
         None
         img = cv2.circle(img,(int(car[j][0]), int(car[j][1])), 2, (0,0,255), -1)

      
      return car
   

   def getInterCar(self, car, seg):
      interCar = np.empty((0,2))
      i = 0
      
      while i < len(car):

         if i < len(car)-1:
            dist = self.distance((car[i][0], car[i][1]), (car[i+1][0], car[i+1][1]))
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint((car[i][0], car[i][1], 0), car[i+1], segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg
         else:
            dist = self.distance((car[i][0], car[i][1]), (car[0][0], car[0][1]))
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint((car[i][0], car[i][1], 0), car[0], segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg

            dist = self.distance((car[0][0], car[0][1]), (car[2][0], car[2][1]))
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint((car[0][0], car[0][1], 0), car[2], segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg

            dist = self.distance((car[1][0], car[1][1]), (car[3][0], car[3][1]))
            count =  int(dist/seg)
            segment = seg
            for j in range(count):
               pr = self.steerPoint((car[1][0], car[1][1], 0), car[3], segment)
               interCar = np.append(interCar, [[pr[0],pr[1]]], axis = 0) 
               segment += seg
         i += 1
      return interCar


   
   def checkArc(self, arc, check, Waypoints, img, width, height):
      ch = False
      if len(np.shape(arc)) > 1:
         ch = True
         #arc = self.getArc(check[2], check[1], 2, check[3], Waypoints, img, check[1])
         for j in range(len(arc)):
            car = self.getLocalCar((arc[j][0], arc[j][1]), arc[j][2], width, height, img)
            chCar = self.checkTouch(car, width, height)          
            if chCar == False:
               ch = False
               break
      #print(print(int(arc[j][0]), int(arc[j][1])), ch)      
      return ch
   

   def checkTouch(self, car, width, height):
      ch = True
      for j in range(len(car)):          
         if int(car[j][0]) < width and int(car[j][1]) < height and int(car[j][0]) > 0 and int(car[j][1]) > 0 and np.any(self.localMapArr[int(car[j][0])-1][int(car[j][1])-1]) != 0 :
            ch = False
            break   
      return ch

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
      #print("pose", self.posX, self.posY)

   def odomAuto_collback(self, data):
      euler = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)      
      self.eulerOdom = euler[2]
      #print(self.eulerAuto)
      self.odomX = data.pose.position.x
      self.odomY = data.pose.position.y
      #print("pose", self.posX, self.posY)

   def local_map_image_collback(self, data):
     
      self.current_map = self.br.imgmsg_to_cv2(data)
      width, height = self.current_map.shape[:2]

      posCarX = -int(20*self.carX)
      posCarY = width+int(20*self.carY)
      self.current_map = cv2.circle(self.current_map, (posCarX, posCarY), 5, (50,50,50), -1)

      len = 20*math.sqrt((self.goalX - self.posX)**2 + (self.goalY - self.posY)**2)
      posGaolX = 0
      posGaolY = int(len)
     
      posGaolX = -int(len*math.sin(self.angleAutoPoint)) + posCarX
      posGaolY = int(len*math.cos(self.angleAutoPoint)) + posCarY
     
      self.current_map = cv2.circle(self.current_map, (posGaolX, posGaolY), 5, (127,127,127), -1)


  
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
      print(self.carX, self.carY)

   def IsPointInCircle(self, x, y, xc, yc, r):
      return ((x-xc)**2+(y-yc)**2) ** 0.5 <= r
    
  
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