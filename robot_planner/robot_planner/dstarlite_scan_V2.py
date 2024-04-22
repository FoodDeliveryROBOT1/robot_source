#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import threading ,time
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# Ros2 messages
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
# Python libraries
import numpy as np
import math
import matplotlib.pyplot as plt
from dstarlite.d_star_lite import *
from dstarlite.cubic_spline_planner import *


expansion_size = 3

def euler_from_quaternion(x,y,z,w):
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
    return yaw_z


def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data

def get_distance(scan, angle_min, angel_max, angel_increment, yaw):
        dist = len(scan)
        obx, oby = [], []
        for i in range(dist):
            if 0 < scan[i] < 2:
                angle = angle_min + i*angel_increment
                x1 = scan[i] * math.cos(angle)
                y1 = scan[i] * math.sin(angle)
                x11 = math.cos(yaw) * x1 - math.sin(yaw) * y1
                y11 = math.sin(yaw) * x1 + math.cos(yaw) * y1
                obx.append(x11), oby.append(y11)
        return obx, oby

def obstacle_scale(obx, oby):
    obx_s, oby_s = [], []
    expan_size =1
    for k in range(len(obx)):
        for i in range(-expan_size, expan_size + 1):
            for j in range(-expan_size, expan_size + 1):
                if i ==0 and j == 0:
                    continue
                x = obx[k] + i
                y = oby[k] + j
                obx_s.append(x), oby_s.append(y)
    return obx_s, oby_s

class Obstacle(Node):
    def __init__(self):
        super().__init__('obstacle_node')
        qos_profile = QoSProfile(depth=100)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        self.map_sub = self.create_subscription(OccupancyGrid, "map", self.map_callback, 100)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 100)
        #self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 100)
        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.odom_callback, 100)
        self.path_pub = self.create_publisher(Path, 'path1', 100)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 100)
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.path_timer = self.create_timer(0.001, self.path_callback)
        self.move_timer = self.create_timer(0.001, self.move_callback)
        self.publisher_ = self.create_publisher(Marker, 'points', 10)
        self.flag_sub = self.create_subscription(Int16, 'flag', self.flag_callback, 100)
        # self.timer_ = self.create_timer(1.0, self.publish_points)
        self.stage = 0
        self.resolution = 0.0
        self.path_x, self.path_y = [], []
        self.lox = [0]
        self.loy = [1]
        self.ok = 0
        self.i = 0
        self.v, self.w = 0.0, 0.0
        self.sx, self.sy, self.yaw = 0.0, 0.0, 0.0 
        self.mpc_flag = 0
        

    def odom_callback(self, msg):
        self.sx = msg.pose.pose.position.x
        self.sy = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
       
        print(self.sx, self.sy)

    def goal_callback(self, msg):
        self.gx = msg.pose.position.x
        self.gy = msg.pose.position.y
        self.gw = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y,
                                        msg.pose.orientation.z, msg.pose.orientation.w)
        self.ok = 1

    def flag_callback(self, msg):
        self.mpc_flag = msg.data

    def map_callback(self,msg):
        self.ox, self.oy = [], []
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.data = costmap(msg.data, msg.info.width, msg.info.height, self.resolution)
        self.map_data = [item for i in self.data for item in i]

        print(self.data)
        self.stage = 1
        print(self.stage)
        
    def laser_callback(self, laser_msg):
        if self.stage == 1:
            self.range = laser_msg.ranges
            self.angle = laser_msg.angle_increment
            self.angle_min = laser_msg.angle_min
            self.angle_max = laser_msg.angle_max
            self.angle_increment = laser_msg.angle_increment
            self.dx, self.dy = get_distance(self.range, laser_msg.angle_min, laser_msg.angle_max, laser_msg.angle_increment, self.yaw)
            self.lox, self.loy = [], []
            for i in range(len(self.dx)):
                x = int(self.dx[i]/self.resolution)+ int((self.sx - self.origin_x)/self.resolution)
                self.lox.append(x)
            for i in range(len(self.dy)):
                y = int(self.dy[i]/self.resolution)+ int((self.sy - self.origin_y)/self.resolution)
                self.loy.append(y)
                #print(self.lox, "================",self.loy)
            self.lox, self.loy = obstacle_scale(self.lox, self.loy)
            self.stage = 2
            print(self.stage)
        
            
 
    def move_callback(self):
        if  self.stage == 2 and self.ok == 1:
            if(self.sx != self.gx and self.sy != self.gy):
                sx = int((self.sx - self.origin_x)/self.resolution)
                sy = int((self.sy - self.origin_y)/self.resolution)
                # sx, sy = self.sx, self.sy
                gx = int((self.gx - self.origin_x)/self.resolution)
                gy = int((self.gy - self.origin_y)/self.resolution)
                # print(sx, sy)
                # print(gx, gy)
                for i in range(self.width):
                    for j in range(self.height):
                        if(self.map_data[j*self.width+ i] >= 100*self.resolution):
                            x = i 
                            y = j
                            self.ox.append(int(x)), self.oy.append(int(y))
                print(self.ok, '\t', self.stage)
                #print(dx, dy)
            
                spoofed_ox = [[], [], [],self.lox]
                spoofed_oy = [[], [], [],self.loy]
                self.dstarlite = DStarLite(self.ox, self.oy)
                _,x2, y2,x3, y3 = self.dstarlite.main(Node_(x=sx, y=sy), Node_(x=gx, y=gy),
                                spoofed_ox=spoofed_ox, spoofed_oy=spoofed_oy)
                ds = 0.1
                sp = CubicSpline2D(x2, y2)
                s = np.arange(0, sp.s[-1], ds)
                rx, ry, ryaw, rk = [], [], [], []
                for i_s in s:
                    ix, iy = sp.calc_position(i_s)
                    rx.append(ix)
                    ry.append(iy)
                    ryaw.append(sp.calc_yaw(i_s))
                    rk.append(sp.calc_curvature(i_s))
                self.x2, self.y2 = x2, y2
                self.rx, self.ry = rx, ry
                self.path_x, self.path_y = [], []
                for i in range(len(rx)):
                    px = rx[i]*self.resolution + self.origin_x
                    py = ry[i]*self.resolution + self.origin_y
                    self.path_x.append(px), self.path_y.append(py)
                print(self.path_x, self.path_y)
                
                self.sx = int((self.gx - self.origin_x)/self.resolution)
                self.sy = int((self.gy - self.origin_y)/self.resolution)
                # print(self.stage)
               
                self.stage = 3

    def path_callback(self):
        twist_msg = Twist()

        if self.stage == 0:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.twist_pub.publish(twist_msg)
        elif self.stage == 3:
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for i in range(len(self.path_x)):
                pose = PoseStamped()
                pose.pose.position.x = self.path_x[i]
                pose.pose.position.y = self.path_y[i]
                path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)
           
            
            
            if(abs(self.sx - self.path_x[len(self.path_x) - 1])) < 0.05 and abs(self.sy - self.path_y[len(self.path_y) - 1])< 0.05:
                print("STOP")
                print(self.mpc_flag)
                self.flag = 0
                self.stage = 1
                self.ok = 0
            
      
         
                
            

    def timer_callback(self):
        if self.stage == 4:
            plt.cla()
            plt.plot(self.ox, self.oy, 's')
            plt.plot(self.x2, self.y2)
            plt.grid(True)
            plt.pause(0.001)
            plt.show()
            
            self.stage = 0




def main(args = None):

    rclpy.init(args=args)
    obstacle_node = Obstacle()
    try:
        rclpy.spin(obstacle_node)
    except KeyboardInterrupt:
        pass
    obstacle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

    



