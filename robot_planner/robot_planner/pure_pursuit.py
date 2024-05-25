import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int16
import numpy as np
import math

lookahead_distance = 0.12

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

def pure_pursuit1(current_x, current_y, current_heading, path_x, path_y, index):
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

    return v, desired_steering_angle/3, index


class Controller(Node):
    def __init__(self):
        super().__init__('control_node')

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 100)
        self.path_sub = self.create_subscription(Path, '/path1', self.path_callback, 100)
        self.dt = 0.01

        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 100)
        self.flag_pub = self.create_publisher(Int16, 'flag', 100)
        self.flag = 0

        self.control_timer = self.create_timer(self.dt, self.control_callback)

        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.i = 0

    def odom_callback(self, odom_msg):
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )

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
            flag_msg = Int16()
            v, angle, self.i = pure_pursuit1(self.x, self.y, self.yaw,
                                            self.path_x, self.path_y, self.i)

            cmd_msg.linear.x = v
            cmd_msg.angular.z = angle
            self.twist_pub.publish(cmd_msg)
            if(abs(self.x - self.path_x[len(self.path_x) - 1])) < 0.05 and abs(self.y - self.path_y[len(self.path_y) - 1])< 0.05:
                flag_msg.data = 1
                self.flag = 0
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.twist_pub.publish(cmd_msg)
                print("Stop")
                self.flag_pub.publish(flag_msg)


def main(args= None):
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

