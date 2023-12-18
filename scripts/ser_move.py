#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from serial import Serial
from time import sleep, time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion, Pose, Vector3
from std_msgs.msg import UInt32MultiArray, Float32, Bool, String
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from math import cos, sin, pi
import serial.tools.list_ports
from rclpy.clock import Clock
import os
import sys
import math
import numpy as np
import re
import threading

class SerialControl(Node):
	
	def __init__(self):
		
		
		super().__init__('serial_control')
		connected = False
		while connected == False:
			try:
				self.serMove = Serial('/dev/ttyUSB1', 115200 ,timeout = 0.05)
				connected = True
				sleep(3)
			except Exception as e:
				print(e)
				sleep(0.5)

		
		self.timer = self.create_timer(0.1, self.read)
		self.tiks = [0,0]
		self.sp = [0,0]
		self.time = self.get_clock().now()
		self.x = 0.0
		self.y = 0.0
		self.a = 0.0
		
		self.previous_cmd_time = Clock().now()
		self.sub_cmd_2 = self.create_subscription(Twist, '/autocar/cmd_vel', self.cmd_cb, 10)
		self.buttons_status_pub = self.create_publisher(String, '/buttons_status', 10)
		self.broadcaster = tf2_ros.TransformBroadcaster(self)
		self.previous_cmd_time = time()
		self.previous_odom_time = time()

		self.theta = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.wz = 0.0
		self.nav_state = 'not' #stay, move, finished
		self.start_msg = False
		self.buttons = []
		self.states = ["Start", "Pause", "Stop"]
		self.status = "None"



	def map_cb(self, data):
		if self.start_msg == False:
			self.nav_state = "empty"
			self.start_msg = True
			
	def read(self):
			stat = "None"
			self.bs = self.serMove.readline()
			self.bs = self.bs.decode()
			self.bs = self.bs.split(":")
			print(self.bs)
			bt = []

			if len(self.bs) > 1:	
				for i in range(4):
					bt = np.append(bt, int(self.bs[len(self.bs) - i -1]))


				print(bt)

				for i in range(len(bt)):
					if i != 0:
						if bt[i] == 1:
							stat = self.states[i-1]
							print(stat)

			msg = String()
			msg.data = stat
			self.buttons_status_pub.publish(msg)
			
			if float(time() - self.previous_cmd_time) > 2.0:
				self.vx = 0.0
				self.vy = 0.0
				self.wz = 0.0
			string_ = "tv:" + str(round(self.vx,2)) + "," + str(round(self.wz,2))+"\n"
			#print(string_)
			bstr = bytes(string_, 'utf-8')
			self.serMove.write(bstr)
			
			

	
	def quaternion_from_euler(self, ai, aj, ak):

		ai /= 2.0
		aj /= 2.0
		ak /= 2.0
		ci = math.cos(ai)
		si = math.sin(ai)
		cj = math.cos(aj)
		sj = math.sin(aj)
		ck = math.cos(ak)
		sk = math.sin(ak)
		cc = ci*ck
		cs = ci*sk
		sc = si*ck
		ss = si*sk
		q = np.empty((4, ))
		q[0] = cj*sc - sj*cs
		q[1] = cj*ss + sj*cc
		q[2] = cj*cs - sj*sc
		q[3] = cj*cc + sj*ss
		
		return q
		
    	
  
	def send_cmd(self, data):
		if data.linear.x > 0.5:
			data.linear.x = 0.5
		elif data.linear.x < -0.5:
			data.linear.x = -0.5
		if data.angular.z > 0.8:
			data.angular.z = 0.8
		elif data.angular.z < -0.8:
			data.angular.z = -0.8
		delta = float(time() - self.previous_cmd_time)
		self.vx = data.linear.x
		self.vy = data.linear.y
		self.wz = data.angular.z 
		self.previous_cmd_time = time()
		
	def cmd_cb(self, data):
		#print(data)
		self.send_cmd(data)

	def cmd_auto_cb(self, data):
		if data.linear.x != 0.0 or data.angular.z != 0.0:
			self.send_cmd(data)

def main(args=None):

    rclpy.init(args=args)
    controller = SerialControl()
    rclpy.spin(controller)
    controller.destroy_node()	
    rclpy.shutdown()
    
    
if __name__ == '__main__':
  main()