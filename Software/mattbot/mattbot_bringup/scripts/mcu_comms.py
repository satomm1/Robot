#!/usr/bin/env python3

import time
import rospy
import spidev
import math
import struct
import numpy as np
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from sensor_msgs import Imu
from tf2_msgs import TFMessage
from tf.transformations import quaternion_from_euler

BAUD_RATE = 1000000  # 1 MHz baud rate

class MCU_Comms:
    """
    Sets up communication between the Jetson and the MCU
    """
    def __init__(self):

        # Create the spi object to facilitate SPI communication via Jetson and MCU
        self.spi = spidev.SpiDev()  # create spi object
        self.spi.open(0, 0)
        self.spi.max_speed_hz = BAUD_RATE

        # Bringup Message to MCU
        bringup_message = [99, 0b11111111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Send bringup message to MCU until received and confirmed
        bringup_confirmed = False
        while not bringup_confirmed:
            rcvd = self.spi.xfer(bringup_message)
            if rcvd[0] == 0 and rcvd[1] == 255 and rcvd[2] == 0:
                bringup_confirmed = True  # Mattbot is active
                confirmation_message = [99, 0b10101010, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                rcvd2 = self.spi.xfer(confirmation_message)

        # The node for SPI Comms
        rospy.init_node("mattbot_mcu_comms", anonymous=True)

        # Publish Sensor Readings
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher("/mobile_base/sensors/imu_data", Imu, queue_size=10)
        self.tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)

        # Subscribe to velocity commands
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        """
        Takes the twist message which prescribes the linear/angular velocity of the robot, and sends
        to the robot via SPI
        """

        lin_vel = np.float32(msg.linear.x)
        ang_vel = np.float32(msg.angular.z)

        lin_vel_bytes = [struct.pack('f', lin_vel)[i:i+1] for i in range(0, 4, 1)]
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

    def shutdown_callback(self):
        """
        publishes a shutdown message to MCU via SPI
        """
        shutdown_message = [99, 0b11110000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # shutdown message
        rcvd = self.spi.xfer(shutdown_message)
        rcvd = self.spi.xfer(shutdown_message) # Send twice just to be sure

        self.spi.close()

    def run(self):
        """
        Periodically retrieves odometry data from the sensors/MCU via SPI
        """
        rate = rospy.Rate(10)  # 10 Hz
        sensor_sequence = 0

        setup_message = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        accel_message = [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        gyro_message = [3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        position_message = [4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        dead_reckoning_message = [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        while not rospy.is_shutdown():

            # FIXME: Do we need a pause between these messages to allow time for MCU to process and upload its data to its buffer?
            rcvd = self.spi.xfer(setup_message)
            accel = self.spi.xfer(accel_message)
            gyro = self.spi.xfer(gyro_message)
            position = self.spi.xfer(position_message)
            dead_reckoning = self.spi.xfer(dead_reckoning_message)

            # Now extract the odom/position data
            accel_x = struct.unpack('f', bytes(accel[1:5]))[0]
            accel_y = struct.unpack('f', bytes(accel[5:9]))[0]
            accel_z = struct.unpack('f', bytes(accel[9:13]))[0]

            gyro_x = struct.unpack('f', bytes(gyro[1:5]))[0]
            gyro_y = struct.unpack('f', bytes(gyro[5:9]))[0]
            gyro_z = struct.unpack('f', bytes(gyro[9:13]))[0]

            pos_x = struct.unpack('f', bytes(position[1:5]))[0]
            pos_y = struct.unpack('f', bytes(position[5:9]))[0]
            pos_theta = struct.unpack('f', bytes(position[9:13]))[0]

            V_dr = struct.unpack('f', bytes(dead_reckoning[1:5]))[0]
            w_dr = struct.unpack('f', bytes(dead_reckoning[5:9]))[0]


            # Publish the relevant information to the proper topics

            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.header.seq = sensor_sequence
            odom.child_frame_id = "base_footprint"

            odom.pose.pose = Pose(Point(pos_x, pos_y, 0), quaternion_from_euler(0,0,pos_theta*3.14159/180))  # position/orientation
            odom.twist.twist = Twist(Vector3(V_dr, 0, 0), Vector3(0, 0, w_dr))  # linear/angular velocity

            self.odom_pub.publis(odom)  # actually publish the data


            imu = Imu()
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = "gyro_link"
            imu.header.seq = sensor_sequence
            imu.orientation = quaternion_from_euler(0,0,pos_theta*3.14159/180)
            imu.angular_velocity = Vector3(gyro_x, gyro_y, gyro_z)*3.14159/180
            imu.linear_acceleration = Vector3(accel_x*9.81, accel_y*9.81, accel_z*9.81)

            tf = TFMessage()
            tf.transforms.header.stamp = rospy.Time.now()
            tf.transforms.header.frame_id = "odom"
            tf.transforms.header.seq = sensor_sequence
            tf.transforms.child_frame_id = "base_footprint"
            tf.transforms.transform.translation = Vector3(pos_x, pos_y, 0)
            tf.transforms.transform.rotations = quaternion_from_euler(0,0,pos_theta*3.14159/180)

            sensor_sequence += 1  # increment for next time
            rate.sleep()


if __name__ == "__main__":
    req = MCU_Comms()
    rospy.on_shutdown(req.shutdown_callback)
    req.run()