import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist , TransformStamped
import numpy as np
import math
from time import time,sleep
from tf2_ros import TransformBroadcaster
# Covariance for EKF simulation
Q = np.diag([
    10.0,  # variance of location on x-axis
    10.0,  # variance of location on y-axis
    np.deg2rad(3.0),  # variance of yaw angle
    0.5,  # variance of velocity
    np.deg2rad(0.5)
]) ** 2  # predict state covariance
 ##             yaw             v           dot_yaw

R1 = np.diag([np.deg2rad(0.01)]) ** 2  # Observation yaw covariance
R2 = np.diag([ 0.01, np.deg2rad(0.1)]) ** 2  # Observation Vx,omega position covariance

# #  Simulation parameter
# INPUT_NOISE = np.diag([0.3, np.deg2rad(1.0)]) ** 2
# IMU_NOISE = np.diag([np.deg2rad(1.0)]) ** 2
# ODOM_NOISE = np.diag([0.05 ,np.deg2rad(1.0)]) ** 2

# DT = 0.02  # time tick [s]
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

class odometry_class(Node):
    def __init__(self):
        super().__init__('odometry_class')
        self.create_timer = self.create_timer(0.02 ,self.callback_timer)
        self.imu_subscriber = self.create_subscription(Imu, "/hfi_imu", self.imu_callback, 10)
        self.subscribe_wheel_odom = self.create_subscription(Odometry, '/odom',self.odom_callback, 10)
        self.subscribe_twist = self.create_subscription(Twist, '/cmd_vel',self.control_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_ekf = self.create_publisher(Odometry,'/odom_ekf', 10)
        self.init_ekf()

    def imu_callback(self, imu_msg):
        # imu_msg = Imu()
        self.feedback_yaw =  euler_from_quaternion(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
        self.z_imu = np.array([[self.feedback_yaw]])
        # print(self.feedback_yaw)
    def odom_callback(self, odom_msg):
        # odom_msg = Odometry()
        self.vx = odom_msg.twist.twist.angular.x
        self.omega = odom_msg.twist.twist.angular.z
        self.z_odom = np.array([[self.vx], [self.omega]])
        # print(self.z_odom)
    
    def control_callback(self, control_msg):
        # control_msg = Twist()
        self.u_vx = control_msg.linear.x
        self.u_w = control_msg.angular.z
        self.u = np.array([[self.u_vx], [self.u_w]])

    def init_ekf(self):
        self.time = 0.0
        # State Vector [x y yaw v]'
        self.xEst = np.zeros((5, 1))
        self.xTrue = np.zeros((5, 1))
        self.PEst = np.eye(5)

        self.xDR = np.zeros((5, 1))  # Dead reckoning

        # history
        self.hxEst = self.xEst
        self.hxTrue = self.xTrue
        self.hxDR = self.xTrue
        self.hz1 = np.zeros((1, 1))
        self.hz2 = np.zeros((2, 1))
        ##
        self.u = np.array([[0.0], [0.0]])
        self.z_odom = np.array([[0.0], [0.0]])
        self.z_imu = np.array([[0.0]])

        ## time compare
        self.new_time = time()
        self.old_time = time()

    def callback_timer(self):
        self.new_time = time()
        print('Total time: ', (self.new_time - self.old_time)*1000)
        DT = self.new_time - self.old_time


        # self.xTrue, z1, z2, self.xDR, ud = self.observation(self.xTrue, self.xDR, u)  # feedback here <>

        self.xEst, self.PEst = self.ekf_estimation(self.xEst, self.PEst, self.z_imu, self.z_odom, self.u, DT)  # input control here <ud>
        

        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base_footprint"
        # # speed 
        # state_speed = self.f(ca.DM([0.0, 0.0, 0.0]), self.u)
        # odometry_msg.twist.twist.linear.x = float(state_speed[0])
        # odometry_msg.twist.twist.linear.y = float(state_speed[1])
        # odometry_msg.twist.twist.angular.z = float(state_speed[2])
        # Position 
        odometry_msg.pose.pose.position.x = float(self.xEst[0])
        odometry_msg.pose.pose.position.y = float(self.xEst[1])
        odometry_msg.pose.pose.position.z = 0.0
        self.q = quaternion_from_euler(0, 0, self.xEst[2])

        odometry_msg.pose.pose.orientation.x = self.q[0]
        odometry_msg.pose.pose.orientation.y = self.q[1]
        odometry_msg.pose.pose.orientation.z = self.q[2]
        odometry_msg.pose.pose.orientation.w = self.q[3]

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id =  'base_footprint'
        t.transform.translation.x = float(self.xEst[0])
        t.transform.translation.y = float(self.xEst[1])
        t.transform.translation.z = 0.0
        # quat = quaternion_from_euler(0.0, 0.0, self.xEst[2])
        t.transform.rotation.x = self.q[0]
        t.transform.rotation.y = self.q[1]
        t.transform.rotation.z = self.q[2]
        t.transform.rotation.w = self.q[3]
        self.tf_broadcaster.sendTransform(t)

        self.publish_ekf.publish(odometry_msg)

        self.old_time = self.new_time


    # def observation(self, xTrue, xd, u):
    #     xTrue = self.motion_model(xTrue, u)

    #     # add noise to gps x-y
    #     z1 = self.imu_observation_model(xTrue) + IMU_NOISE @ np.random.randn(1, 1)
    #     z2 = self.odom_observation_model(xTrue) + ODOM_NOISE @ np.random.randn(2, 1)

    #     # add noise to input
    #     ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    #     xd = self.motion_model(xd, ud)

    #     return xTrue, z1, z2, xd, ud


    def motion_model(self, x, u, DT): ##  ok
        F = np.array([[1.0, 0, 0, 0, 0],
                    [0, 1.0, 0, 0, 0],
                    [0, 0, 1.0, 0, 0],
                    [0, 0, 0  , 0, 0],
                    [0, 0, 0  , 0, 0]])

        B = np.array([[DT * math.cos(x[2, 0]), 0],
                    [DT * math.sin(x[2, 0]), 0],
                    [0.0, DT],
                    [1.0, 0.0],
                    [0.0, 1.0]])

        x = F @ x + B @ u

        return x


    def imu_observation_model(self, x):
        H1 = np.array([
            [0, 0, 1.0, 0, 0]
        ])

        z1 = H1 @ x

        return z1

    def odom_observation_model(self, x):
        H2 = np.array([
            [0, 0, 0, 1.0, 0],
            [0, 0, 0, 0, 1.0]
        ])

        z2 = H2 @ x

        return z2


    def jacob_f(self, x, u, DT):
        """
        Jacobian of Motion Model

        motion model
        x_{t+1} = x_t+v*dt*cos(yaw)
        y_{t+1} = y_t+v*dt*sin(yaw)
        yaw_{t+1} = yaw_t+omega*dt
        v_{t+1} = v{t}
        so
        dx/dyaw = -v*dt*sin(yaw)
        dx/dv = dt*cos(yaw)
        dy/dyaw = v*dt*cos(yaw)
        dy/dv = dt*sin(yaw)
        """
        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw), 0.0],
            [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw), 0.0],
            [0.0, 0.0, 1.0, 0.0, DT],
            [0.0, 0.0, 0.0, 1.0 , 0.0],
            [0.0, 0.0, 0.0, 0.0 , 1.0]])

        return jF


    def jacob_h1(self):
        # Jacobian of Observation Model
        jH1 = np.array([
            [0, 0, 1.0, 0, 0]
        ])

        return jH1

    def jacob_h2(self):
        # Jacobian of Observation Model
        jH2 = np.array([
            [0, 0, 0, 1.0, 0],
            [0, 0, 0, 0, 1.0]
        ])

        return jH2

    def ekf_estimation(self, xEst, PEst, z1, z2, u,DT):
        #  Predict
        xPred = self.motion_model(xEst, u, DT)
        jF = self.jacob_f(xEst, u, DT)
        PPred = jF @ PEst @ jF.T + Q

        #  Update
        jH1 = self.jacob_h1()
        jH2 = self.jacob_h2()
        ## imu
        zPred_imu = self.imu_observation_model(xPred)
        y1 = z1 - zPred_imu
        S1 = jH1 @ PPred @ jH1.T + R1
        K1 = PPred @ jH1.T @ np.linalg.inv(S1)
        ## odom
        zPred_odom = self.odom_observation_model(xPred)
        y2 = z2 - zPred_odom
        S2 = jH2 @ PPred @ jH2.T + R2
        K2 = PPred @ jH2.T @ np.linalg.inv(S2)

        xEst = xPred + K1 @ y1 + K2 @ y2
        PEst = (np.eye(len(xEst)) - K1 @ jH1 - K2 @ jH2) @ PPred
        return xEst, PEst
    



def main(args=None):
    rclpy.init(args=args)

    odom_test = odometry_class()

    rclpy.spin(odom_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()