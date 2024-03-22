import rclpy
from rclpy.node import Node

# Ros 2 message
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray

# Library 
import can
import math
# Robot dimension
r = 0.169/2 # [m]
d = 0.35 # [m]

def map(Input, min_input, max_input, min_output, max_output):
    value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
    return value

def inverse_kinematic(vx, w):

    vr = ( 2 / r )*( vx + d * w )
    vl = ( 2 / r )*( vx - d * w )
    
    v1 = int(vl*60/(6.28))# rpm
    v2 = int(vr*60/(6.28))# rpm

    return v1, v2


class movement(Node):
    def __init__(self):
        super().__init__('control_node')
        self.twist_sub = self.create_subscription(Twist, "/cmd_vel", self.twist_callback, 10)
        self.bus = can.interface.Bus(channel='can0',interface = 'socketcan', bitrate= 1000000)
        self.commend_vel = [0.0, 0.0]
        self.wheel_vel = [0.0, 0.0]
        self.wheel_mapped_vel = [0.0, 0.0]
        self.wheel_mapped_vel_abs = [0, 0]
        self.wheel_mapped_vel1 = [0, 0]
        self.commend = 3
        self.can_timer = self.create_timer(1/2000, self.can_callback)

        self.TxData0 = [43, 11, 32, 27, self.commend, 0, 0, 0]
        self.TxData1 = [43, 11, 32, 28, self.wheel_mapped_vel_abs[0], 0, 0, 0]
        self.TxData2 = [43, 11, 32, 29, self.wheel_mapped_vel_abs[1], 0, 0, 0]

        self.TxData3 = [64, 10, 32, 8, 0, 0 ,0 ,0]
        self.TxData4 = [64, 12, 32, 8, 0, 0 ,0 ,0]

        self.feedback_pub = self.create_publisher(UInt16MultiArray, '/feedback',20)

        self.Data = [0, 0, 0, 0]

    def twist_callback(self, twist_msg):
        self.commend_vel = [twist_msg.linear.x, twist_msg.angular.z]
        self.wheel_vel[0], self.wheel_vel[1] = inverse_kinematic(self.commend_vel[0], self.commend_vel[1])
        # self.get_logger().info("%s" % self.commend_vel)
        
        
        for i in range(2):
            self.get_logger().info("%d" % self.wheel_vel[i])
            self.wheel_mapped_vel[i] =  (self.wheel_vel[i])
            # self.get_logger().info("%d" % self.wheel_mapped_vel[i])
            # self.wheel_mapped_vel = [0, 0]
           
            if(self.wheel_mapped_vel[i] > 200):
                self.wheel_mapped_vel =  200
            elif(self.wheel_mapped_vel[i] < -200):
                self.wheel_mapped_vel[i] = -200
        if(self.wheel_mapped_vel[0] > 0 and self.wheel_mapped_vel[1] > 0):
            self.commend = 3
        elif(self.wheel_mapped_vel[0] >0 and self.wheel_mapped_vel[1] < 0):
            self.commend = 9
        elif(self.wheel_mapped_vel[0] <0 and self.wheel_mapped_vel[1] > 0):
            self.commend = 7
        else:
            self.commend = 13
        self.wheel_mapped_vel_abs[0] = abs(self.wheel_mapped_vel[0])
        self.wheel_mapped_vel_abs[1] = abs(self.wheel_mapped_vel[1])
        # print(self.wheel_mapped_vel_abs)
        # print(self.commend)

        self.TxData0 = [43, 11, 32, 27, self.commend, 0, 0, 0]
        self.TxData1 = [43, 11, 32, 28, self.wheel_mapped_vel_abs[0], 0, 0, 0]
        self.TxData2 = [43, 11, 32, 29, self.wheel_mapped_vel_abs[1], 0, 0, 0]
        self.TxData3 = [64, 10, 32, 8, 0, 0 ,0 ,0]
        self.TxData4 = [64, 12, 32, 8, 0, 0 ,0 ,0]
        
    def can_callback(self):
        self.command_msg0 = can.Message(arbitration_id=0x601, data=self.TxData0, dlc=8, is_extended_id=False)
        self.command_msg1 = can.Message(arbitration_id=0x601, data=self.TxData1, dlc=8, is_extended_id=False)
        self.command_msg2 = can.Message(arbitration_id=0x601, data=self.TxData2, dlc=8, is_extended_id=False)
        # Data position feedback
        self.command_msg3 = can.Message(arbitration_id=0x601, data=self.TxData3, dlc=8, is_extended_id=False)
        self.command_msg4 = can.Message(arbitration_id=0x601, data=self.TxData4, dlc=8, is_extended_id=False)

        try:
            self.bus.send( self.command_msg0, 0.01 )
            self.bus.send( self.command_msg1, 0.01 )
            self.bus.send( self.command_msg2, 0.01 )
            self.bus.send( self.command_msg3, 0.01 )
            self.bus.send( self.command_msg4, 0.01 )

        except can.CanError :
            pass

        try :
            for i in range(5):
                can_msg = self.bus.recv(0.01)
                if(can_msg != None):
                    if(can_msg.arbitration_id == 0x581):
                        if(can_msg.data[1] == 10):
                            self.Data[1] = (can_msg.data[5] << 8) + can_msg.data[4]

                        elif(can_msg.data[1] == 12):
                            self.Data[3] = (can_msg.data[5] << 8) + can_msg.data[4]
                        
                        
                else:
                    self.get_logger().error('time out on msg recv!')
            feedback_msg = UInt16MultiArray()
            feedback_msg.data = [self.Data[1], self.Data[3]]
            # print(feedback_msg.data)
            self.feedback_pub.publish(feedback_msg)

        except can.CanOperationError:
                pass
        


def main(args=None):
    rclpy.init(args=args)
    movement_node = movement()
    rclpy.spin(movement_node)
    movement_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
