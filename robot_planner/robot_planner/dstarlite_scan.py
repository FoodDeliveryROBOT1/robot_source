#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading ,time
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# Ros2 messages
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
# Python libraries
import numpy as np
import math


from dstarlite.d_star_lite import *
from dstarlite.cubic_spline_planner import *

expansion_size = 4
lookahead_distance = 0.25

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

def obstacle_scale(obx, oby):
    obx_s, oby_s = [], []
    expan_size = 4
    for k in range(len(obx)):
        for i in range(-expan_size, expan_size + 1):
            for j in range(-expan_size, expan_size + 1):
                if i ==0 and j == 0:
                    continue
                x = obx[k] + i
                y = oby[k] + j
                obx_s.append(x), oby_s.append(y)
    return obx_s, oby_s

def get_distance(scan, angle_min, angel_max, angel_increment, yaw):
        dist = len(scan)
        obx, oby = [], []
        for i in range(dist):
            if 0 < scan[i] < 1.2:
                angle = angle_min + i*angel_increment
                x1 = scan[i] * math.cos(angle)
                y1 = scan[i] * math.sin(angle)
                x11 = math.cos(yaw) * x1 - math.sin(yaw) * y1
                y11 = math.sin(yaw) * x1 + math.cos(yaw) * y1
                obx.append(x11), oby.append(y11)
        return obx, oby

def pure_pursuit1(current_x, current_y, current_heading,gx,gy, path_x, path_y, index):
    v = 0.1
    closest_point = None
    for i in range(index, len(path_x)):
        x = path_x[i]
        y = path_y[i]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path_y[-1] - current_y, path_x[-1] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path_x)-1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/4
        v = 0.0

    return v, desired_steering_angle/4, index


class Obstacle(Node):
    def __init__(self):
        super().__init__('obstacle_node')
        qos_profile = QoSProfile(depth=100)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        self.path_pub = self.create_publisher(Path, 'path1', 100)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 100)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 100)
        #self.odom_sub = self.create_subscription(Odometry, 'odom_ekf', self.odom_callback, 100)
        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.odom_callback, 100)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 100)
        self.path_timer = self.create_timer(0.001, self.path_callback)
        self.move_timer = self.create_timer(0.001, self.move_callback)
        self.stage = 0
        self.resolution = 0.0
        self.path_x, self.path_y = [], []
        self.lox = [0]
        self.loy = [1]
        self.ok = 0
        self.i = 0
        self.sx, self.sy, self.yaw = 0.0, 0.0, 0.0 
        self.v, self.w = 0.0, 0.0
        

    def odom_callback(self, msg):
        self.sx = msg.pose.pose.position.x
        self.sy = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        print(self.sx, self.sy)

    def goal_callback(self, msg):
        self.gx = msg.pose.position.x
        self.gy = msg.pose.position.y
        print(self.gx, self.gy)
        self.ok = 1

    def map_callback(self,msg):
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
        if self.stage == 1 :
            self.range = laser_msg.ranges
            self.angle = laser_msg.angle_increment
            self.dx, self.dy = get_distance(self.range, laser_msg.angle_min, laser_msg.angle_max, laser_msg.angle_increment, self.yaw)
            #print(dx, dy)
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
                self.ox, self.oy = [], []
                sx = int((self.sx - self.origin_x)/self.resolution)
                sy = int((self.sy - self.origin_y)/self.resolution)
                # sx, sy = self.sx, self.sy
                gx = int((self.gx - self.origin_x)/self.resolution)
                gy = int((self.gy - self.origin_y)/self.resolution)
                print(sx, sy)
                print(gx, gy)
                
                for i in range(self.width):
                    for j in range(self.height):
                        if(self.map_data[j*self.width+ i] >= 100*self.resolution):
                            x = i 
                            y = j
                            self.ox.append(int(x)), self.oy.append(int(y))
                spoofed_ox = [[], [], [],self.lox]
                spoofed_oy = [[], [], [],self.loy]
                # spoofed_ox = [[], [], [],[1]]
                # spoofed_oy = [[], [], [],[1]]
                dstarlite = DStarLite(self.ox, self.oy)
                _,x2, y2,x3, y3 = dstarlite.main(Node_(x=sx, y=sy), Node_(x=gx, y=gy),
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
                # print(self.path_x, self.path_y)
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
            
            v, angle, self.i = pure_pursuit1(self.sx, self.sy, self.yaw, self.gx, self.gy,
                                                  self.path_x, self.path_y, self.i)
            twist_msg.linear.x = v
            twist_msg.angular.z = angle
            print(v, angle)
            self.twist_pub.publish(twist_msg)
            print(abs(self.sx - self.path_x[len(self.path_x) - 1]) , '\t', abs(self.sy - self.path_y[len(self.path_y) - 1]))
            if(abs(self.sx - self.path_x[len(self.path_x) - 1])) < 0.05 and abs(self.sy - self.path_y[len(self.path_y) - 1])< 0.05:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                print("STOP")
                self.twist_pub.publish(twist_msg)
                self.stage = 1
                self.ok = 0
                



def main(args = None):

    rclpy.init(args=args)
    obstacle_node = Obstacle()
    rclpy.spin(obstacle_node)
    obstacle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    


