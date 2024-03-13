import rclpy
from rclpy.node import Node

import math
import serial
import struct
import serial.tools.list_ports
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from time import time,sleep

global init_value
init_value = True


def eul_to_qua(Eular):
    eular_div = [0, 0, 0]   
    eular_div[0], eular_div[1], eular_div[2] = Eular[0] / -2.0, Eular[1] / 2.0, Eular[2] / -2.0

    ca, cb, cc = math.cos(eular_div[0]), math.cos(eular_div[1]), math.cos(eular_div[2])
    sa, sb, sc = math.sin(eular_div[0]), math.sin(eular_div[1]), math.sin(eular_div[2])

    x = sa * cb * cc - ca * sb * sc
    y = ca * sb * cc + sa * cb * sc
    z = ca * cb * sc - sa * sb * cc
    w = ca * cb * cc + sa * sb * sc

    orientation = Quaternion()
    orientation.x, orientation.y, orientation.z, orientation.w = x, y, z, w
    return orientation

def find_ttyUSB():
    print('imu defauld port /dev/ttyUSB0')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('connected to  {} serial port {} 个: {}'.format('USB', len(posts), posts))

# crc 
def checkSum(list_data, check_data):
    data = bytearray(list_data)
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])


def hex_to_ieee(raw_data):
    ieee_data = []
    raw_data.reverse()
    for i in range(0, len(raw_data), 4):
        data2str =hex(raw_data[i] | 0xff00)[4:6] + hex(raw_data[i + 1] | 0xff00)[4:6] + hex(raw_data[i + 2] | 0xff00)[4:6] + hex(raw_data[i + 3] | 0xff00)[4:6]
        ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
    ieee_data.reverse()
    return ieee_data


def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag, data_right_count, init_value, init_yaw

    if data_right_count > 200000:
        print("The device has an error in transmitting data,, exits.")
        exit(0)


    buff[key] = raw_data

    key += 1
    if buff[0] != 0xaa:
        data_right_count += 1
        key = 0
        return
    if key < 3:
        return
    if buff[1] != 0x55:
        key = 0
        return
    if key < buff[2] + 5:  # Get the corresponding length data based on the judgment of the data length bits
        return

    else:
        data_right_count = 0
        data_buff = list(buff.values())  # get value

        if buff[2] == 0x2c and pub_flag[0]:
            if checkSum(data_buff[2:47], data_buff[47:49]):
                data = hex_to_ieee(data_buff[7:47])
                angularVelocity = data[1:4]
                acceleration = data[4:7]
                magnetometer = data[7:10]
            else:
                print('Verification failed!')
            pub_flag[0] = False
        elif buff[2] == 0x14 and pub_flag[1]:
            if checkSum(data_buff[2:23], data_buff[23:25]):
                data = hex_to_ieee(data_buff[7:23])
                angle_degree = data[1:4]
            else:
                print('Verification failed!')
            pub_flag[1] = False
        else:
            print("This data processing class does not provide the " + str(buff[2]) )
            print("data error!!")
            buff = {}
            key = 0

        buff = {}
        key = 0
        #if pub_flag[0] == True or pub_flag[1] == True:
        #    return
        pub_flag[0] = pub_flag[1] = True
        current_time = node.get_clock().now()
        stamp = current_time.to_msg()

        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = "base_link"

        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = "base_link"

        # print(angle_degree)

        acc_k = math.sqrt(acceleration[0] ** 2 + acceleration[1] ** 2 + acceleration[2] ** 2)
        if acc_k == 0:
           acc_k = 1

        if (init_value):
            init_yaw = angle_degree[2]
            print("initialing")
            if (time() - time_init > 3):    # 3 second to init yaw
                init_value = False
        else :

            angle_degree_modify = (angle_degree[0],angle_degree[1],angle_degree[2] - init_yaw)
            # print("-------------")
            # print(angle_degree_modify[2])
            angle_radian = [angle_degree_modify[i] * math.pi / 180 for i in range(3)]
            # qua = Quaternion()
            qua = eul_to_qua(angle_radian)

            # imu_msg.orientation = qua
            imu_msg.orientation.x = qua.x
            imu_msg.orientation.y = qua.y
            imu_msg.orientation.z = qua.z
            imu_msg.orientation.w = qua.w

            imu_msg.angular_velocity.x = angularVelocity[0]
            imu_msg.angular_velocity.y = angularVelocity[1]
            imu_msg.angular_velocity.z = angularVelocity[2]
            
            
            
            if gra_normalization:
                imu_msg.linear_acceleration.x = acceleration[0] * -9.8 / acc_k
                imu_msg.linear_acceleration.y = acceleration[1] * -9.8 / acc_k
                imu_msg.linear_acceleration.z = acceleration[2] * -9.8 / acc_k
            else:
                imu_msg.linear_acceleration.x = acceleration[0] * -9.8
                imu_msg.linear_acceleration.y = acceleration[1] * -9.8
                imu_msg.linear_acceleration.z = acceleration[2] * -9.8

            imu_publisher.publish(imu_msg)
        
        mag_msg.magnetic_field.x = magnetometer[0]
        mag_msg.magnetic_field.y = magnetometer[1]
        mag_msg.magnetic_field.z = magnetometer[2]

        

        mag_publisher.publish(mag_msg)



## node init
node: Node
# publisher: rclpy.Publisher[Imu]

key = 0
flag = 0
buff = {}
angularVelocity = [0.0, 0.0, 0.0]
acceleration = [0.0, 0.0, 0.0]
magnetometer = [0.0, 0.0, 0.0]
angle_degree = [0.0, 0.0, 0.0]
pub_flag = [True, True]
data_right_count = 0


def main():
    find_ttyUSB()
    rclpy.init()
    global node, imu_publisher, mag_publisher 
    node = Node("imu_node")
    

    port = "/dev/ttyUSB1"
    baudrate = 921600
    global gra_normalization,imu_msg,mag_msg  , time_init
    gra_normalization =  False
    
    imu_msg = Imu()
    mag_msg = MagneticField()
    try :
        hf_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        time_init = time()
        if hf_imu.isOpen():
            node.get_logger().info("IMU connect success")
        else:
            hf_imu.open()
            node.get_logger().info("IMU Open")
    except Exception as e:   
        print(e)
        node.get_logger().error("error port!!!") 
        node.destroy_node()
        exit(0)
    else :
        imu_publisher = node.create_publisher(Imu, "hfi_imu", 10)
        mag_publisher = node.create_publisher(MagneticField, "hfi_mag", 10)
        while rclpy.ok():
            try:
                buff_count = hf_imu.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("imu Loss of connection, poor contact, or disconnection")
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = hf_imu.read(buff_count)
                    for i in range(0, buff_count):
                        handleSerialData(buff_data[i])
        rclpy.spin(node)
        # time.sleep(0.001)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
