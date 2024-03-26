import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math 
from time import time

# def pi_2_pi(angle):
#     while(angle > math.pi):
#         angle = angle - 2.0 * math.pi

#     while(angle < -math.pi):
#         angle = angle + 2.0 * math.pi

#     return angle

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

# Covariance for EKF simulation
Q = np.diag([
    0.7,  # variance of location on x-axis
    0.7,  # variance of location on y-axis
    np.deg2rad(35.0),  # variance of yaw angle about 0.5
    0.2,  # variance of velocity
    np.deg2rad(3.0)
]) ** 2  # predict state covariance
 ##             imu_yaw         x    y        yaw       V           dot_yaw
R = np.diag([np.deg2rad(1.0), 2.0, 2.0,np.deg2rad(10.0)]) ** 2  # Observation yaw covariance , 5.0, np.deg2rad(10.0)


def map(Input, min_input, max_input, min_output, max_output):
    value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
    return value 

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
        self.pos_x = odom_msg.pose.pose.position.x
        self.pos_y = odom_msg.pose.pose.position.y
        self.odom_yaw =  euler_from_quaternion(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.yaw = self.pi_2_pi(self.odom_yaw)
        self.vx = odom_msg.twist.twist.angular.x
        self.omega = odom_msg.twist.twist.angular.z
        self.z_odom = np.array([[self.pos_x],[self.pos_y],[self.odom_yaw]]) # ,[self.vx], [self.omega]
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
        self.hz = np.zeros((3, 1))
        ##
        self.u = np.array([[0.0], [0.0]])
        self.z_imu = np.array([[0.0]])
        self.z_odom = np.array([[0.0], [0.0] , [0.0] ]) # , [0.0], [0.0]

        self.vx,self.omega,self.feedback_yaw = 0.0,0.0,0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        ## time compare
        self.new_time = time()
        self.old_time = time()

    def callback_timer(self):
        self.new_time = time()  # use this to compare time in ros timer
        # print('Total time: ', (self.new_time - self.old_time)*1000)
        DT = self.new_time - self.old_time
        # DT = 0.02
        # self.xTrue, z1, z2, self.xDR, ud = self.observation(self.xTrue, self.xDR, u)  # feedback here <>

        self.xEst, self.PEst = self.ekf_estimation(self.xEst, self.PEst, self.z_imu ,self.z_odom , self.u, DT)  # input control here <ud>
        # print("------------xEst-------")
        # print(self.xEst)

        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base_footprint"
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
        
        # pi 2 pi
        x[2,0] = self.pi_2_pi(x[2,0])

        return x


    def imu_observation_model(self,x):
        H = np.array([
            [0, 0, 1, 0, 0]
        ])

        z = H @ x

        return z

    def odom_observation_model(self,x):
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0]
        ])

        z = H @ x

        return z


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
        yaw = self.pi_2_pi(x[2, 0])
        
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw), 0.0],
            [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw), 0.0],
            [0.0, 0.0, 1.0, 0.0, DT],
            [0.0, 0.0, 0.0, 1.0 , 0.0],
            [0.0, 0.0, 0.0, 0.0 , 1.0]])

        return jF


    def jacob_h_imu(self):
        # Jacobian of Observation Model
        jH = np.array([
            [0, 0, 0, 0, 1]
        ])

        return jH

    def jacob_h_odom(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0]
        ])

        return jH

    def ekf_estimation(self, xEst, PEst, z1, z2, u,DT):
        #  Predict
        xPred = self.motion_model(xEst, u, DT)
        jF = self.jacob_f(xEst, u, DT)
        PPred = jF @ PEst @ jF.T + Q

        #  Update
        jH_imu = self.jacob_h_imu()
        jH_odom = self.jacob_h_odom()
        jH = np.concatenate((jH_imu, jH_odom), axis=0)

        zPred_imu = self.imu_observation_model(xPred)
        zPred_odom = self.odom_observation_model(xPred)
        zPred = np.concatenate((zPred_imu, zPred_odom), axis=0)

        z = np.concatenate((z1, z2), axis=0)

        y = z - zPred
        S = jH @ PPred @ jH.T + R
        K = PPred @ jH.T @ np.linalg.inv(S)

        xEst = xPred + K @ y
        PEst = (np.eye(len(xEst)) - K @ jH ) @ PPred
        return xEst, PEst
    

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

    odom_test = odometry_class()

    rclpy.spin(odom_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()