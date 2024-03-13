import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu
from rclpy.time import Time
from time import time,sleep

import math
import numpy as np
import casadi as ca
from casadi import sin, cos, pi, arctan2

## complimentary filter 
delta = 0.90

wheel_radius = 0.169 # in meters
d = 0.35 # in meters

x_init = 0.0
y_init = 0.0
theta_init = 0.0

def euler_from_quaternion(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = 2.0 * (w * z +x * y)
    t4 = 1.0 - 2.0*(y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return yaw

def quaternion_from_euler(ai, aj, ak):
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

def forward_kinematic(vl, vr):
    r = 0.169 # m
    d = 0.32  # m 
    v = (vr + vl)/2
    w = (r/(2*d))*(vr - vl)
    return v, w

def map(Input, min_input, max_input, min_output, max_output):
    value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
    return value

class odometry(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.feedback_sub = self.create_subscription(UInt16MultiArray, 'feedback', self.feedback_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "hfi_imu", self.imu_callback,10)
        self.odometry_pub = self.create_publisher(Odometry, '/odom', 10)
        self.odometry_timer = self.create_timer(0.01, self.odometry_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.feedback_data = [0, 0]
        self.old_tick = ca.DM([0, 0])
        self.new_tick = ca.DM([0, 0])
        self.diff = ca.DM([0, 0])
        self.q = [0.0, 0.0, 0.0, 0.0]
        self.ppr = 4096 # tick per revolution
        ## time compare
        self.init_param()
        self.new_time = time()
        self.old_time = time()
        
    

    def init_param(self):
        ## state symbolic variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(
            x,
            y,
            theta
        )
        ## control symbolic variables
        Vr = ca.SX.sym('Vr')
        Vl = ca.SX.sym('Vl')
        controls = ca.vertcat(
            Vr,
            Vl
        )
        rot_3d_z = ca.vertcat(
            ca.horzcat(cos(theta), -sin(theta), 0),
            ca.horzcat(sin(theta),  cos(theta), 0),
            ca.horzcat(         0,           0, 1)
        )

        J = (wheel_radius) * ca.DM([          ## inverse kinematic
            [1/2,      1/2],
            [0.0,      0.0],
            [1/(2*d), -1/(2*d)]
        ])
        self.first_init = True
        self.new_state = ca.DM([x_init, y_init, theta_init])        # initial state
        self.old_state = ca.DM([x_init, y_init, theta_init])        # initial state
        self.speed_init = ca.DM([0.0, 0.0, 0.0])                    # initial state
        RHS = rot_3d_z @ J @ controls
        self.f = ca.Function('f', [states, controls], [RHS])
        self.u = ca.DM([0.0, 0.0])

    def imu_callback(self,imu_msg):
        self.feedback_yaw =  euler_from_quaternion(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)

    def odometry_callback(self):
        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base_footprint"
        # speed 
        state_speed = self.f(ca.DM([0.0, 0.0, 0.0]), self.u)
        odometry_msg.twist.twist.linear.x = float(state_speed[0])
        odometry_msg.twist.twist.linear.y = float(state_speed[1])
        odometry_msg.twist.twist.angular.z = float(state_speed[2])
        # Position 
        odometry_msg.pose.pose.position.x = float(self.new_state[0])
        odometry_msg.pose.pose.position.y = float(self.new_state[1])
        odometry_msg.pose.pose.position.z = 0.0
        self.q = quaternion_from_euler(0, 0, self.new_state[2])

        # p = np.array([self.pos[0], self.pos[1], yaw])
        # print(p)
        odometry_msg.pose.pose.orientation.x = self.q[0]
        odometry_msg.pose.pose.orientation.y = self.q[1]
        odometry_msg.pose.pose.orientation.z = self.q[2]
        odometry_msg.pose.pose.orientation.w = self.q[3]
        self.odometry_pub.publish(odometry_msg)

        ########################
        
    
    def feedback_callback(self, tick_msg):
        if (self.first_init):
            self.first_init = False 
            self.new_tick = ca.DM([(tick_msg.data[0]),(tick_msg.data[1])])   ## first init
            self.old_tick = self.new_tick
        else :
            self.new_time = time()
            print('Total time: ', (self.new_time - self.old_time)*1000)
            DT = self.new_time - self.old_time


            self.new_tick = ca.DM([tick_msg.data[0],tick_msg.data[1]])
            self.diff = self.new_tick - self.old_tick
            for i in range(2):
                if (self.diff[i] > 32768):
                    self.diff[i] = self.diff[i] - 65535
                elif (self.diff[i] < -32768):
                    self.diff[i] = self.diff[i] + 65535
            self.u = 2* pi * ca.DM([self.diff[0],self.diff[1]]) /  DT * self.ppr
            ## complimentary filter
            self.old_state[3] = (1 - delta) * self.old_state[3] + delta * self.feedback_yaw
            ##########
            self.new_time = time()
            self.DT = self.new_time - self.old_time

            self.new_state = self.shift_timestep(self.old_state,self.diff,self.f)

            self.old_tick = self.new_tick
            ########
            self.old_time = self.new_time

    def shift_timestep(self , state_init, u, f):
        # range-kutta 
        #DX            X           Dx
        k1 = f(state_init, (u* 2* pi)/self.ppr) 
        k2 = f(state_init + 1/2*k1, (u* 2* pi)/self.ppr)
        k3 = f(state_init + 1/2*k2, (u* 2* pi)/self.ppr)
        k4 = f(state_init + 1 * k3, (u* 2* pi)/self.ppr)
        st_next_RK4 = state_init + (1 / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        f_value = st_next_RK4
        return f_value


def main(args=None):
    rclpy.init(args=args)
    odometry_node = odometry()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()