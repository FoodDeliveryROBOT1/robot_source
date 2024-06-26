import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from time import time,sleep

import math
import numpy as np
import casadi as ca
from casadi import sin, cos, pi, arctan2

wheel_radius = 0.169/2      # in meters
D = 0.35                    # in meters

x_init = 0.0
y_init = 0.0
theta_init = 0.0

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


class odometry(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.feedback_sub = self.create_subscription(UInt16MultiArray, 'feedback', self.feedback_callback, 50)
        self.odometry_pub = self.create_publisher(Odometry, '/odom', 10)
        self.DT = 0.04
        self.odometry_timer = self.create_timer(self.DT, self.odometry_callback)   # 50 Hz
        self.tf_broadcaster = TransformBroadcaster(self)
        self.feedback_data = [0, 0]
        self.old_tick = ca.DM([0, 0])
        self.new_tick = ca.DM([0, 0])
        self.old_tick_2 = ca.DM([0, 0])
        self.new_tick_2 = ca.DM([0, 0])
        self.diff = ca.DM([0, 0])
        self.diff_2 = ca.DM([0, 0])
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
            [1/2,      -1/2],
            [0.0,      0.0],
            [-1/(D), -1/(D)]
        ])
        self.first_init = True
        self.new_state = ca.DM([x_init, y_init, theta_init])        # initial state
        self.old_state = ca.DM([x_init, y_init, theta_init])        # initial state
        self.speed_init = ca.DM([0.0, 0.0, 0.0])                    # initial state
        RHS = rot_3d_z @ J @ controls
        self.f = ca.Function('f', [states, controls], [RHS])
        self.u = ca.DM([0.0, 0.0])
        

    def odometry_callback(self):
        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base_footprint"
        # speed 
        

        if (self.first_init == False ):
            self.diff_2 = self.new_tick - self.old_tick_2
            for i in range(2):
                if (self.diff_2[i] > 32768):
                    self.diff_2[i] = self.diff_2[i] - 65535
                elif (self.diff_2[i] < -32768):
                    self.diff_2[i] = self.diff_2[i] + 65535
            self.u = 2* pi * ca.DM([self.diff_2[0],self.diff_2[1]]) / (self.DT * self.ppr)
            state_speed = self.f(ca.DM([0.0, 0.0, 0.0]), self.u)

            # update on old state
            self.old_tick_2 = self.new_tick

            odometry_msg.twist.twist.linear.x = float(state_speed[0])
            odometry_msg.twist.twist.linear.y = float(state_speed[1])
            odometry_msg.twist.twist.angular.z = float(state_speed[2])
        # Position 

        odometry_msg.pose.pose.position.x = float(self.new_state[0])
        odometry_msg.pose.pose.position.y = float(self.new_state[1])
        odometry_msg.pose.pose.position.z = 0.0
        self.q = quaternion_from_euler(0, 0, self.new_state[2])

        odometry_msg.pose.pose.orientation.x = self.q[0]
        odometry_msg.pose.pose.orientation.y = self.q[1]
        odometry_msg.pose.pose.orientation.z = self.q[2]
        odometry_msg.pose.pose.orientation.w = self.q[3]
        self.odometry_pub.publish(odometry_msg)
                
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = 'odom'
        # t.child_frame_id =  'base_footprint'
        # t.transform.translation.x = float(self.xEst[0])
        # t.transform.translation.y = float(self.xEst[1])
        # t.transform.translation.z = 0.0
        # # quat = quaternion_from_euler(0.0, 0.0, self.xEst[2])
        # t.transform.rotation.x = self.q[0]
        # t.transform.rotation.y = self.q[1]
        # t.transform.rotation.z = self.q[2]
        # t.transform.rotation.w = self.q[3]
        # self.tf_broadcaster.sendTransform(t)

        ########################
        
    
    def feedback_callback(self, tick_msg):
        if (self.first_init):
            self.first_init = False 
            self.new_tick = ca.DM([(tick_msg.data[0]),(tick_msg.data[1])])   ## first init
            self.old_tick = self.new_tick

            ## for derivative
            self.old_tick2 =  self.new_tick

        else :
            # self.new_time = time()
            # print('Total time: ', (self.new_time - self.old_time)*1000)
            # DT = self.new_time - self.old_time


            self.new_tick = ca.DM([tick_msg.data[0],tick_msg.data[1]])
            self.diff = self.new_tick - self.old_tick
            for i in range(2):
                if (self.diff[i] > 32768):
                    self.diff[i] = self.diff[i] - 65535
                elif (self.diff[i] < -32768):
                    self.diff[i] = self.diff[i] + 65535
            
            self.new_state = self.shift_timestep(self.old_state,self.diff,self.f)
            self.old_state = self.new_state
            # print(self.new_tick ,'\t',self.diff ,'\t', self.new_state)

            self.old_tick = self.new_tick
            ########
            # self.old_time = self.new_time
            

    def shift_timestep(self , state_init, u, f):
        f_value = f(state_init, (u* 2* pi)/self.ppr) + state_init
        return f_value


def main(args=None):
    rclpy.init(args=args)
    odometry_node = odometry()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()