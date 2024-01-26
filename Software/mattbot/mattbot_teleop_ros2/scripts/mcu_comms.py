import rclpy
from rclpy.node import Node

import time
import spidev
import numpy as np
import struct
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from sensor_msgs import Imu
from tf2_msgs import TFMessage

BAUD_RATE = 1000000


class MCU_Comms(Node):
    """
    Sets up communication between the Jetson and the MCU
    """
    def __init__(self):
        super().__init__('mattbot_mcu_comms')

        # Create the SPI object to facilitate SPI communication via Jetson and MCU
        self.spi = spidev.SpiDev()  # Create SPI object
        self.spi.open(0,0)  # open spi port 0, device (CS) 0
        self.spi.max_speed_hz = BAUD_RATE  
        self.spi.mode = 0b11  # CPOL = 1, CPHA = 1 (i.e. clock is high when idle, data is clocked in on rising edge)

        # Bringup message to MCU
        bringup_message = [99, 0b1111111111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Send bringup message to MCU unti received and confirmed
        bringup_confirmed = False
        while not bringup_confirmed:
            bringup_message = [99, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            rcvd = self.spi.xfer(bringup_message)
            print(rcvd)
            if rcvd[0] == 0 and rcvd[1] == 255 and rcvd[2] == 0:
                bringup_confirmed = True  # MattBot is active
            time.sleep(0.1)
        confirmation_message = [99, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        rcvd = self.spi.xfer2(confirmation_message)
        print(rcvd)

        # Now create publishers for data collected from MCU
        self.odom_pub = self.create_publisher(Odometry, "/odom",queue_size=10)
        self.imu_pub = self.create_publisher(Imu, "/mobile_base/sensors/imu_data", queue_size=10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", queue_size=10)

        self.cmd_vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.cmd_vel_subscriber  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
    """
    Takes the twist message which prescribes the linear/angular velocity of the robot, and sends
    to the robot via SPI
    """
        lin_vel = np.float32(msg.linear.x)
        ang_vel = np.float32(msg.angular.z)

        lin_vel_bytes = [struct.pack('f', lin_vel)[i:i + 1] for i in range(0, 4, 1)]
        ang_vel_bytes = [struct.pack('f', ang_vel)[i:i + 1] for i in range(0, 4, 1)]

        # Byte in message 0 should be 0 to indicate velocity command
        vel_message = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Add the four bytes representing the float32 linear velocity command
        vel_message[1] = int.from_bytes(lin_vel_bytes[0], byteorder='big', signed=False)
        vel_message[2] = int.from_bytes(lin_vel_bytes[1], byteorder='big', signed=False)
        vel_message[3] = int.from_bytes(lin_vel_bytes[2], byteorder='big', signed=False)
        vel_message[4] = int.from_bytes(lin_vel_bytes[3], byteorder='big', signed=False)

        # Add the four bytes representing the float32 angular velocity command
        vel_message[5] = int.from_bytes(ang_vel_bytes[0], byteorder='big', signed=False)
        vel_message[6] = int.from_bytes(ang_vel_bytes[1], byteorder='big', signed=False)
        vel_message[7] = int.from_bytes(ang_vel_bytes[2], byteorder='big', signed=False)
        vel_message[8] = int.from_bytes(ang_vel_bytes[3], byteorder='big', signed=False)

        # Now transfer the velocity command message
        rcvd = self.spi.xfer(vel_message)


def main(args=None):
    rclpy.init(args=args)

    comm = MCU_Comms()

    rclpy.spin(comm)

    comm.spi.close()
    comm.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

