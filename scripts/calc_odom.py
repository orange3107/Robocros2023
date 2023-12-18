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
		self.tiks = [0,0]
		self.sp = [0,0]
		self.x = 0.0
		self.y = 0.0
		self.a = 0.0
		self.bs = ''
		
		self.previous_cmd_time = Clock().now()
		self.odom_pub = self.create_publisher(Odometry, '/odom', 1)
		self.broadcaster = tf2_ros.TransformBroadcaster(self)
		self.previous_cmd_time = time()
		self.previous_odom_time = time()

		self.theta = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.wz = 0.0
		self.nav_state = 'not' #stay, move, finished
		self.start_msg = False

		connected = False
		while connected == False:
			try:
				self.serOdom = Serial('/dev/ttyUSB0', 115200 ,timeout = 0.05)
				connected = True
				sleep(3)
			except Exception as e:
				print(e)
				sleep(0.5)
		self.time = self.get_clock().now()	
		while True: self.read()

	def read(self):
		try:
			#timePer = self.get_clock().now() - self.time
			#self.time = self.get_clock().now()

			self.bs = self.serOdom.readline()
			self.bs = self.bs.decode()
			
			if self.bs != '':
				self.bs = self.bs.split(";")
				timePer = self.get_clock().now() - self.time
				self.time = self.get_clock().now()
				
				
				for i in range(len(self.bs)):
					tiks = int(self.bs[i])
					#print(tiks)
					rp = (tiks*10)/80
				
					self.sp[i] = rp/(2 * math.pi * 0.2159)
				
				Vx_real = (self.sp[0]+self.sp[1])/2
				Wz_real = (self.sp[1] - self.sp[0])/1.565
				self.bs = ''
				print([Vx_real, Wz_real], timePer)
				self.calc_odom([Vx_real, Wz_real])

		except Exception as e:
			None
			#print(e)
			

	def calc_odom(self,velocity):
		

		delta = time() - self.previous_odom_time
		self.previous_odom_time = time()
		delta_s = velocity[0] * delta
		delta_a = velocity[1] * delta

		self.x += delta_s*cos(self.a)
		self.y += delta_s*sin(self.a)

		self.a += delta_a

		#print(self.x, self.y)

		
		now =  self.get_clock().now()
		#delta = time() - self.previous_odom_time
		#self.previous_odom_time = time()
		self.theta -= delta*velocity[1]

		#self.x += delta*(cos(self.theta)*velocity[0] - sin(self.theta)*velocity[2])
		#self.y += delta*(sin(self.theta)*velocity[0] - cos(self.theta)*velocity[2])
		#print(self.theta)
		odom_quat = self.quaternion_from_euler(0,0,self.a - math.pi/2)
		quaternion = Quaternion()
		quaternion.x = odom_quat[0]
		quaternion.y = odom_quat[1]
		quaternion.z = odom_quat[2]
		quaternion.w = odom_quat[3]
		odom = Odometry()
		#odom.header.stamp = Clock().now()
		odom.header.frame_id = "odom"
		#odom.pose.pose = Pose(Point(self.x,self.y,0.0), Quaternion(*odom_quat))
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation = quaternion
		odom.child_frame_id = "odometry"

		odom.twist.twist.linear.x = velocity[0]
		odom.twist.twist.linear.y = 0.0
		odom.twist.twist.linear.z = velocity[1]

		transform_stamped_msg = TransformStamped()
		transform_stamped_msg.header.stamp = now.to_msg()
		transform_stamped_msg.header.frame_id = 'odom'
		transform_stamped_msg.child_frame_id = 'odometry'
		transform_stamped_msg.transform.translation.x = self.x
		transform_stamped_msg.transform.translation.y = self.y
		transform_stamped_msg.transform.translation.z = 0.0
		transform_stamped_msg.transform.rotation.x = quaternion.x
		transform_stamped_msg.transform.rotation.y = quaternion.y
		transform_stamped_msg.transform.rotation.z = quaternion.z
		transform_stamped_msg.transform.rotation.w = quaternion.w


		#odom.twist.twist = Twist(Vector3(velocity[0],velocity[1],0.0),Vector3(0,0,velocity[2]))
		odom.twist.covariance[0] = 0.05
		odom.twist.covariance[7] = 0.05
		odom.twist.covariance[35] = 0.01
		self.broadcaster.sendTransform(transform_stamped_msg)
		
		self.odom_pub.publish(odom)


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
			
			

def main(args=None):

    rclpy.init(args=args)
    controller = SerialControl()
    rclpy.spin(controller)
    controller.destroy_node()	
    rclpy.shutdown()
    
    
if __name__ == '__main__':
  main()