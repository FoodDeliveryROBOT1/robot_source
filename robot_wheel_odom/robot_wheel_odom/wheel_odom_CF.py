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

wheel_radius = 0.169/2 # in meters
D = 0.35                    # in meters

x_init = 0.0
y_init = 0.0
theta_init = 0.0


# def pi_2_pi(angle):
#     while(angle > math.pi):
#         angle = angle - 2.0 * math.pi

#     while(angle < -math.pi):
#         angle = angle + 2.0 * math.pi

#     return angle

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



class odometry(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.feedback_sub = self.create_subscription(UInt16MultiArray, 'feedback', self.feedback_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "hfi_imu", self.imu_callback,10)
        self.odometry_pub = self.create_publisher(Odometry, '/odom', 10)
        self.DT = 0.04
        self.odometry_timer = self.create_timer(self.DT , self.odometry_callback)
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.feedback_data = [0, 0]
        self.old_tick = ca.DM([0, 0])
        self.new_tick = ca.DM([0, 0])
        self.old_tick_2 = ca.DM([0, 0])
        self.new_tick_2 = ca.DM([0, 0])
        self.diff = ca.DM([0, 0])
        self.q = [0.0, 0.0, 0.0, 0.0]
        self.ppr = 4096 # tick per revolution
        ## time compare
        self.init_param()
        self.DT = 0.02
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
        self.feedback_yaw = 0.0

    def imu_callback(self,imu_msg):
        self.feedback_yaw =  euler_from_quaternion(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)

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
            ## for derivative
            self.old_tick2 =  self.new_tick
        else :

            self.new_tick = ca.DM([tick_msg.data[0],tick_msg.data[1]])
            self.diff = self.new_tick - self.old_tick
            for i in range(2):
                if (self.diff[i] > 32768):
                    self.diff[i] = self.diff[i] - 65535
                elif (self.diff[i] < -32768):
                    self.diff[i] = self.diff[i] + 65535


            ## complimentary filter
            self.old_state[2] = (1 - delta) * self.pi_2_pi(self.old_state[2]) + delta * self.pi_2_pi(self.feedback_yaw)
            ##########


            self.new_state = self.shift_timestep(self.old_state,self.diff,self.f)
            self.old_state = self.new_state       
            self.old_tick = self.new_tick


    # def shift_timestep(self , state_init, u, f):
    #     # range-kutta 
    #     #DX            X           Dx
    #     k1 = f(state_init, (u* 2* pi)/self.ppr) 
    #     k2 = f(state_init + 1/2*k1, (u* 2* pi)/self.ppr)
    #     k3 = f(state_init + 1/2*k2, (u* 2* pi)/self.ppr)
    #     k4 = f(state_init + 1 * k3, (u* 2* pi)/self.ppr)
    #     st_next_RK4 = state_init + (1 / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    #     f_value = st_next_RK4
    #     return f_value
            
    def shift_timestep(self , state_init, u, f):
        f_value = f(state_init, (u* 2* pi)/self.ppr) + state_init
        return f_value
    
    def pi_2_pi(self, angle):
        return self.angle_mod(angle)


    def angle_mod(self, x, zero_2_2pi=False, degree=False):
        """
        Angle modulo operation
        Default angle modulo range is [-pi, pi)

        Parameters
        ----------
        x : float or array_like
            A angle or an array of angles. This array is flattened for
            the calculation. When an angle is provided, a float angle is returned.
        zero_2_2pi : bool, optional
            Change angle modulo range to [0, 2pi)
            Default is False.
        degree : bool, optional
            If True, then the given angles are assumed to be in degrees.
            Default is False.

        Returns
        -------
        ret : float or ndarray
            an angle or an array of modulated angle.

        Examples
        --------
        >>> angle_mod(-4.0)
        2.28318531

        >>> angle_mod([-4.0])
        np.array(2.28318531)

        >>> angle_mod([-150.0, 190.0, 350], degree=True)
        array([-150., -170.,  -10.])

        >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
        array([300.])

        """
        if isinstance(x, float):
            is_float = True
        else:
            is_float = False

        x = np.asarray(x).flatten()
        if degree:
            x = np.deg2rad(x)

        if zero_2_2pi:
            mod_angle = x % (2 * np.pi)
        else:
            mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

        if degree:
            mod_angle = np.rad2deg(mod_angle)

        if is_float:
            return mod_angle.item()
        else:
            return mod_angle
        



def main(args=None):
    rclpy.init(args=args)
    odometry_node = odometry()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()