import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber_node')
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom_ekf',
            self.listener_callback_odom,
            10)
        self.subscription_odom  # prevent unused variable warning

        self.subscription_pose = self.create_subscription(
            PoseStamped,
            '/mcl_pose',
            self.listener_callback_pose,
            10)
        self.subscription_pose  # prevent unused variable warning

        self.odom_x = []
        self.odom_y = []
        self.pose_x = []
        self.pose_y = []
        
        # Setup a periodic timer to plot the data every 10 seconds
        self.timer = self.create_timer(10, self.plot_positions)

    def listener_callback_odom(self, msg):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)
        self.get_logger().info(f'Received /odom position: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})')

    def listener_callback_pose(self, msg):
        self.pose_x.append(msg.pose.position.x)
        self.pose_y.append(msg.pose.position.y)
        self.get_logger().info(f'Received /pose position: ({msg.pose.position.x}, {msg.pose.position.y})')

    def plot_positions(self):
        plt.figure(figsize=(10, 5))
        plt.plot(self.odom_x, self.odom_y, label='EKF Path', marker='o')
        plt.plot(self.pose_x, self.pose_y, label='MCL Path', marker='x')
        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.title('EKF and MCL')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    position_subscriber = PositionSubscriber()
    rclpy.spin(position_subscriber)
    # Clean up
    position_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
