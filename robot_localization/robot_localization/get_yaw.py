import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

from tf_transformations import euler_from_quaternion


class get_yaw(Node):
    def __init__(self):
        super().__init__('get_yaw_node')
        self.feedback_sub = self.create_subscription(Odometry, "/odom_ekf",  self.callback, 10) #接受topic名称
    
    def callback(self, odom_msg):
        # print(data)
        (r,p,y) = euler_from_quaternion((odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w))
        #由于是弧度制，下面将其改成角度制看起来更方便
        # node.get_logger().info("Roll = %f, Pitch = %f, Yaw = %f",r*180/3.1415926,p*180/3.1415926,y*180/3.1415926)
        # node.get_logger().info("Roll = %f, Pitch = %f, Yaw = %f",r,p,y)
        print(f"Yaw = {y*180/3.1415926}") #Roll = {r}, Pitch = {p}, 



def main(args=None):
    rclpy.init(args=args)
    get_yaw_node = get_yaw()
    rclpy.spin(get_yaw_node)
    get_yaw_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()