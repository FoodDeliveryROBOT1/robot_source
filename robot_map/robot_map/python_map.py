import rclpy
from rclpy.node import Node
from rclpy.time import Time
import threading ,time
# Ros2 messages
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped

# Python libraries
import numpy as np
import math
import matplotlib.pyplot as plt


expansion_size = 1

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

class Obstacle(Node):
    
    def __init__(self):
        super().__init__('obstacle_node')
        self.map_sub = self.create_subscription(OccupancyGrid, "map", self.map_callback, 100)
        # self.localmap_sub = self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.localmap_callback, 100)
        # self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 100)
        self.odom_sub = self.create_subscription(Odometry, 'odom_ekf', self.odom_callback, 100)
        # self.path_pub = self.create_publisher(Path, 'path1', 100)
        self.timer = self.create_timer(0.001, self.timer_callback)
        # self.path_timer = self.create_timer(0.001, self.path_callback)
        self.stage = 0
        self.resolution = 0.0
        self.path_x, self.path_y = [], []
        self.lox, self.loy = [], []
        self.ox, self.oy = [], []
        self.sx = 0.0
        self.sy = 0.0

    def odom_callback(self, msg):
        self.sx = msg.pose.pose.position.x
        self.sy = msg.pose.pose.position.y
        


    def map_callback(self,msg):
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.data = costmap(msg.data, msg.info.width, msg.info.height, self.resolution)
        self.map_data = [item for i in self.data for item in i]
        self.ox, self.oy = [], []
        for i in range(self.width):
            for j in range(self.height):
                if(self.map_data[j*self.width+ i] >= 100*self.resolution):
                    x = i 
                    y = j
                    self.ox.append(int(x)), self.oy.append(int(y))
        print(self.ox, self.oy)
        self.stage = 1
        


    def timer_callback(self):
        if self.stage == 1:
            plt.cla()
            
            plt.plot(self.ox, self.oy, 's')
            
            plt.grid(True)
            plt.show()
            plt.pause(0.001)
            self.stage = 0


    
        
            

def main(args = None):
    rclpy.init(args=args)
    obstacle_node = Obstacle()
    try:
        
        rclpy.spin(obstacle_node)
        
    except KeyboardInterrupt:
        pass
    obstacle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

    


