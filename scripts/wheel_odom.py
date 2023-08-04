from time import sleep, time
from math import pi, sin, cos, tan, sqrt, radians
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
import json

class CarOdom(Node):
    def __init__(self):
        super().__init__('car_odom')
        self.odom_pub = self.create_publisher(Odometry, "odom", 100)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.odom_broadcaster_laser = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            JointState, 'joint_states', self.js_cb, 100)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.xold = 0
        self.yold = 0
        self.timeold = self.get_clock().now().nanoseconds
        self.angle_whell = []

    def odom_calculate(self):
        self.scale = 1
        self.pi = pi
        self.pi = pi
        self.len_base_car = 0.17   # lenght auto_car
        self.len_mass_c = 0.08 
        self.radius_whell = 0.0325 
        self.len_laser = 0.091  #delite
        n1 = 10
        n2 = 24
        #w1 = 2 * pi *  data[0] * 0.229 / 60
        w2 = w1 * n1 / n2
        get_vel = self.radius_whell * w2   # m/sec
        get_angl = -1 * data[1]
        angular_velocity = get_vel
    
    def js_cb(self, data):
        now =  self.get_clock().now()
        time = now.nanoseconds
        delta = time - self.timeold
        if delta/(10**9) >= 0.1:
            current_time = time/(10**9)
            self.timeold = time
            Vx, Vy, Vtheta = self.calculate_odom(delta/(10**9), [data.velocity[0],  -1 * data.velocity[1]])
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.theta / 2)
            quaternion.w = cos(self.theta / 2)
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = 'odom'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.covariance[0] = 0.1
            odom.pose.covariance[7] = 0.1
            odom.pose.covariance[35] = 0.1
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = 'base_link'
            odom.twist.twist.linear.x = 0.0#V0 
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = 0.0#Vtheta
            self.odom_pub.publish(odom)
                
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.theta / 2)
            quaternion.w = cos(self.theta / 2)
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
            
            q = self.quaternion_from_euler(0.0, 0.0, pi)
            quaternion.x = q[1]
            quaternion.y = q[2]
            quaternion.z = q[3]
            quaternion.w = q[0]
            lidar_transform_stamped_msg = TransformStamped()
            lidar_transform_stamped_msg.header.stamp = now.to_msg()
            lidar_transform_stamped_msg.header.frame_id = 'base_link'
            lidar_transform_stamped_msg.child_frame_id = 'scan'
            lidar_transform_stamped_msg.transform.translation.x = 0.05
            lidar_transform_stamped_msg.transform.translation.y = 0.0
            lidar_transform_stamped_msg.transform.translation.z = 0.091 
            lidar_transform_stamped_msg.transform.rotation.x = quaternion.x
            lidar_transform_stamped_msg.transform.rotation.y = quaternion.y
            lidar_transform_stamped_msg.transform.rotation.z = quaternion.z
            lidar_transform_stamped_msg.transform.rotation.w = quaternion.w
            
            self.odom_broadcaster.sendTransform(transform_stamped_msg)
            self.odom_broadcaster_laser.sendTransform(lidar_transform_stamped_msg)

    def quaternion_from_euler(self, roll, pitch, yaw):    
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr
        return q

    def calculate_odom(self, delta, data ):
        self.scale = 1
        self.pi = pi
        self.pi = pi
        self.len_base_car = 0.104  # lenght auto_car
        self.len_mass_c = 0.1
        self.radius_whell = 0.0325 
        self.len_laser = 0.091  #delite
        n1 = 10
        n2 = 24
        w1 = data[0]
        w2 = w1 * n1 / n2
        get_vel = self.radius_whell * w2   # m/sec
        delta_L = get_vel * delta
        get_angl =  data[1]
        tetta = 0
        if get_angl != 0:
            get_angl +=30
        #print('alfa', alfa)
            tetta = (self.angle_whell[int(get_angl)][0] - self.angle_whell[int(get_angl)][1]) / 2 
            tetta = radians(tetta)
            if tetta > 0:
                tetta +=0.02
            else:
                tetta -=0.02
        self.get_logger().info('tette ' + str(tetta) + '  get_angle '+ str(get_angl))
        
       
        if tetta !=0:
            R = sqrt(self.len_mass_c**2 + (1 / tan(tetta))**2 * self.len_base_car**2)
            delta_theta = delta_L / R
            if tetta > 0:
                delta_theta *= -1
        else:
            delta_theta = 0

        angular_velocity = get_vel * tan(tetta) / self.len_base_car 
        
        x_delta = delta_L * cos(self.theta)
        y_delta = delta_L * sin(self.theta)
        self.theta += delta_theta
        self.x += x_delta 
        self.y += y_delta 
        Vy = 0
        #print(len(self.angle_whell))
        return get_vel, Vy, angular_velocity

    def angle_rul(self): #solution angle whell 
        try:  # loading data on angle values ​​occurs from a file
            path = '/home/nik/dev_ws/src/chicar/chicar/test.json'
            with open(path, 'r', encoding='utf-8') as f:
                data_j = json.load(f)
                change1 = []
                change2 = []
                i = 0
                for i in range(360):
                    if i < 30:
                        print(data_j[i]['%d' % i])
                        change2.append(data_j[i]['%d' % i])  
                    if i >= 330 and i < 360:
                        change1.append(data_j[i]['%d' % i])     
                    
                    
                self.angle_whell = change1 + change2
                
        except: 
            print('it is not possible to open the file')


def main():
    rclpy.init()
    odom = CarOdom()
    odom.angle_rul()
    rclpy.spin(odom)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
