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

class SerialControl(Node):
	
	def __init__(self):
		super().__init__('serial_control')
		connected = False
		while connected == False:
			try:
				self.ser = Serial('/dev/ttyACM0', 9600 ,timeout = 0.05)
				connected = True
			except Exception as e:
				print(e)
				sleep(0.5)

		self.timer = self.create_timer(0.1, self.read)
		self.previous_cmd_time = Clock().now()
		self.sub_cmd_1 = self.create_subscription(Twist, '/auto_cmd_vel', self.cmd_auto_cb, 10)
		self.sub_cmd_2 = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
		self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
		self.odom_pub = self.create_publisher(Odometry, '/odom', 1)
		self.serial_status_pub = self.create_publisher(Bool, 'serial_status', 1)
		self.nav_status_pub = self.create_publisher(String, '/nav_status', 10)
		self.broadcaster = tf2_ros.TransformBroadcaster(self)
		self.previous_cmd_time = time()
		self.previous_odom_time = time()
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.wz = 0.0
		self.nav_state = 'not' #stay, move, finished
		self.start_msg = False
		self.a = 0.0
        

	def map_cb(self, data):
		if self.start_msg == False:
			self.nav_state = "empty"
			self.start_msg = True
			
	def read(self):
		try:
			b = str(self.ser.readline())
			#print(b)
			b = b.split(';')
			for i in range(0, len(b)):
				msg = b[i].split(',')
				#print(msg)
				if msg[0] == 'cv':
					print([float(msg[1]), float(msg[2]), float(msg[3])])
					self.calc_odom([float(msg[1]), float(msg[2]), -float(msg[3])])
					msg = Bool()
					msg.data = True
					self.serial_status_pub.publish(msg)
			if float(time() - self.previous_cmd_time) > 2.0:
				self.vx = 0.0
				self.vy = 0.0
				self.wz = 0.0
			#print(self.nav_state)
			string_ = "tv:" + str(round(self.vx,2)) + "," + str(round(self.vy,2)) + "," + str(round(self.wz,2))+";s:"+self.nav_state
			#string_ = "tv:" + str(round(self.vx,2)) + "," + str(round(self.vy,2)) + "," + str(round(self.wz,2))
			#print(string_)
			#print("send")
			bstr = bytes(string_, 'utf-8')
			self.ser.write(bstr)
		except Exception as e:
			print(e)
			myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
			for port in myports:
				print(port[0])
				if port[0] == '/dev/ttyACM0':
					self.ser.close()
					self.ser = Serial('/dev/ttyACM0', 9600 ,timeout = 0.05)
				elif port[0] == '/dev/ttyACM1':
					self.ser.close()
					self.ser = Serial('/dev/ttyACM1', 9600 ,timeout = 0.05)
			

	def calc_odom(self,velocity):

		delta = time() - self.previous_odom_time
		self.previous_odom_time = time()
		delta_s = velocity[0] * delta
		delta_a = velocity[2] * delta

		self.x += delta_s*cos(self.a)
		self.y += delta_s*sin(self.a)

		self.a += delta_a

		print(self.x, self.y)

		
		now =  self.get_clock().now()
		#delta = time() - self.previous_odom_time
		#self.previous_odom_time = time()
		self.theta -= delta*velocity[2]
		#self.x += delta*(cos(self.theta)*velocity[0] - sin(self.theta)*velocity[2])
		#self.y += delta*(sin(self.theta)*velocity[0] - cos(self.theta)*velocity[2])
		#print(self.theta)
		odom_quat = self.quaternion_from_euler(0,0,self.a)
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
		odom.child_frame_id = "base_link"

		odom.twist.twist.linear.x = velocity[0]
		odom.twist.twist.linear.y = velocity[1]
		odom.twist.twist.linear.z = velocity[2]

		transform_stamped_msg = TransformStamped()
		transform_stamped_msg.header.stamp = now.to_msg()
		transform_stamped_msg.header.frame_id = 'odom'
		transform_stamped_msg.child_frame_id = 'base_link'
		transform_stamped_msg.transform.translation.x = self.x
		transform_stamped_msg.transform.translation.y = self.y
		transform_stamped_msg.transform.translation.z = 0.0
		transform_stamped_msg.transform.rotation.x = quaternion.x
		transform_stamped_msg.transform.rotation.y = quaternion.y
		transform_stamped_msg.transform.rotation.z = quaternion.z
		transform_stamped_msg.transform.rotation.w = quaternion.w

		lidar_quat = self.quaternion_from_euler(0,0,0)
		quaternion = Quaternion()
		quaternion.x = lidar_quat[0]
		quaternion.y = lidar_quat[1]
		quaternion.z = lidar_quat[2]
		quaternion.w = lidar_quat[3]
		lidar_transform_stamped_msg = TransformStamped()
		lidar_transform_stamped_msg.header.stamp = now.to_msg()
		lidar_transform_stamped_msg.header.frame_id = 'base_link'
		lidar_transform_stamped_msg.child_frame_id = 'base_scan'
		lidar_transform_stamped_msg.transform.translation.x = 0.0
		lidar_transform_stamped_msg.transform.translation.y = 0.0
		lidar_transform_stamped_msg.transform.translation.z = 0.15 
		lidar_transform_stamped_msg.transform.rotation.x = quaternion.x
		lidar_transform_stamped_msg.transform.rotation.y = quaternion.y
		lidar_transform_stamped_msg.transform.rotation.z = quaternion.z
		lidar_transform_stamped_msg.transform.rotation.w = quaternion.w


		#odom.twist.twist = Twist(Vector3(velocity[0],velocity[1],0.0),Vector3(0,0,velocity[2]))
		odom.twist.covariance[0] = 0.05
		odom.twist.covariance[7] = 0.05
		odom.twist.covariance[35] = 0.01
		self.broadcaster.sendTransform(transform_stamped_msg)
		self.broadcaster.sendTransform(lidar_transform_stamped_msg)
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
		
    	
  
	def send_cmd(self, data):
		if data.linear.x > 0.12:
			data.linear.x = 0.12
		elif data.linear.x < -0.12:
			data.linear.x = -0.12
		if data.angular.z > 0.57:
			data.angular.z = 0.57
		elif data.angular.z < -0.57:
			data.angular.z = -0.57
		delta = float(time() - self.previous_cmd_time)
		#if(delta > 0.2):
		#print(delta)
		self.vx = data.linear.x
		self.vy = data.linear.y
		self.wz = data.angular.z 
		#string = "tv:" + str(round(data.linear.x,2)) + "," + str(round(data.linear.y,2)) + "," + str(round(data.angular.z,2))+"\n"
		#self.ser.write(string)
		self.previous_cmd_time = time()
	def cmd_cb(self, data):
		print(data)
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