import rclpy
from rclpy.node import Node

# Ros2 messages
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16

# Python libraries
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from mpc.NMPC import *

import scipy.interpolate as si


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

def correct_state(states, tracjectories):
    error = tracjectories - states
    error[:,2] = error[:,2] - np.floor((error[:,2] + np.pi)/(2*np.pi))*2*np.pi
    tracjectories = states + error
    return tracjectories

def update_next_states(current_state, next_states):
    error = next_states[0] - current_state
    for i in range(len(next_states)):
        next_states[i] = next_states[i] - error
    return next_states


class Controller(Node):
    def __init__(self):
        super().__init__('control_node')

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 100)
        self.path_sub = self.create_subscription(Path, '/path1', self.path_callback, 100)
        
        self.dt = 0.01
        self.path_pub = self.create_publisher(Path, 'predicted_path', 100)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 100)
        self.flag_pub = self.create_publisher(Int16, "flag", 100)

        self.control_timer = self.create_timer(self.dt, self.control_callback)
        

        self.flag = 0
        
        self.Q = np.array([[5.0, 0.0, 0.0],[0.0, 5.0, 0.0],[0.0, 0.0, 5.0]])

        self.R = np.array([[2.0, 0.0], [0.0, 4.0]])
        self.N = 10
        self.T = 0.2
        self.mpc = mpc_controller(self.T,self.N, 0.3,0.1,np.pi/12,self.Q,self.R)
        
        self.t0 = 0
        self.x_c = [] # contains for the history of the state
        self.u_c = []
        self.t_c = [self.t0] # for the time
        self.xx = []
        self.next_controls = np.zeros((self.N, 2))
        self.next_states = np.zeros((self.N+1, 3))
        self.u0 = np.zeros((self.N, 2))
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.initial_state = np.array([0.0, 0.0, 0.0])
        self.current_state = self.initial_state.copy()
        
        self.next_trajectories = np.tile(self.initial_state, self.N+1).reshape(self.N+1, -1)
        

    def odom_callback(self, odom_msg):
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )
  
        self.current_state = np.array([self.x, self.y, self.yaw])
       
      
    
    def path_callback(self, msg):
        self.path_x, self.path_y = [], []
        x = [pose.pose.position.x for pose in msg.poses]
        y = [pose.pose.position.y for pose in msg.poses]
        self.path_x.extend(x)
        self.path_y.extend(y)
        self.flag = 1
    
    def control_callback(self):
        cmd_msg = Twist()
        if self.flag == 0:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.twist_pub.publish(cmd_msg)
        elif self.flag == 1:
            path_msg = Path()
            flag_msg = Int16()

            self.next_states = update_next_states(self.current_state, self.next_states)
            u_res, x_m , f_np = self.mpc.solver(self.next_trajectories, self.next_controls)
            self.u_c.append(u_res[0, :])
            self.t_c.append(self.t0)
            self.x_c.append(x_m)
            self.t0, self.current_state, self.u0, self.next_states = shift_movement(self.T, self.t0, self.current_state, u_res, x_m, f_np)
            self.xx.append(self.current_state)
            self.next_trajectories, self.next_controls = desired_command_and_trajectory(self.t0, self.T, self.current_state, self.path_x, self.path_y, self.N)
            self.next_states[0,:] = self.current_state
            self.next_trajectories = correct_state(self.next_states, self.next_trajectories)

            cmd_msg.linear.x = self.u0[0,0]
            cmd_msg.angular.z = self.u0[0,1]
            # print(self.current_state, "\t", self.u0[0,0], self.u0[0,1])
            self.u_c.append(u_res[0, :])
            self.t_c.append(self.t0)
            self.twist_pub.publish(cmd_msg)

            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for s in self.next_states:
                pose = PoseStamped()
                pose.pose.position.x = s[0]
                pose.pose.position.y = s[1]
                path_msg.poses.append(pose)
            

            self.path_pub.publish(path_msg)
            print(self.u0[0,0],self.u0[0,1])
            # print(abs(self.x - self.path_x[len(self.path_x)-1]), abs(self.y - self.path_y[len(self.path_y)-1]))
            #print(abs(self.next_states[0,0]  - self.path_x[len(self.path_x)-1]), abs(self.next_states[0,1]- self.path_y[len(self.path_y)-1]))
            if (abs(self.x  - self.path_x[len(self.path_x)-1]) <= 0.05 and abs(self.y- self.path_y[len(self.path_y)-1]) <= 0.05 ):
                flag_msg.data = 1
                self.flag = 0
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.twist_pub.publish(cmd_msg)
                print("Stop")
                self.flag_pub.publish(flag_msg)
    
            
        


def main(args = None):
    rclpy.init(args=args)
    control_node = Controller()
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        pass
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()