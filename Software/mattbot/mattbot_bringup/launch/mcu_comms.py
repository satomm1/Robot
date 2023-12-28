#!/usr/bin/env python3

import time
import rospy
import spidev
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

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
        bringup_message = [0b11111111, 0, 0, 0, 0, 0, 0, 0]

        # Send bringup message to MCU until received and confirmed
        bringup_confirmed = False
        while not bringup_confirmed:
            rcvd = self.spi.xfer(bringup_message)
            if rcvd[0] == 0 and rcvd[1] == 255 and rcvd[2] == 0:
                bringup_confirmed = True  # Mattbot is active

        # The node for SPI Comms
        rospy.init_node("mattbot_mcu_comms", anonymous=True)

        # Publish Sensor Readings
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        # Subscribe to velocity commands
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        """
        Takes the twist message which prescribes the linear/angular velocity of the robot, and sends
        to the robot via SPI
        """

        lin_vel = msg.linear.x
        ang_vel = msg.angular.z

        if lin_vel >= 10:
            lin_vel = 9.9
        if ang_vel >= 10:
            ang_vel = 9.9

        vel_message = [0b00000010, 0, 0, 0, 0]
        vel_message[1] = math.floor(lin_vel)  # ones digit of linear velocity
        vel_message[2] = math.floor((lin_vel % 1) * 10)  # tenths digit of linear velocity
        vel_message[3] = math.floor(ang_vel)  # ones digit of angular velocity
        vel_message[4] = math.floor((ang_vel % 1) * 10)  # tenths digit of angular velocity
        rcvd = self.spi.xfer(vel_message)

    def shutdown_callback(self):
        """
        publishes a shutdown message to MCU via SPI
        """
        shutdown_message = [0b10101010, 0, 0, 0, 0]  # shutdown message
        rcvd = self.spi.xfer(shutdown_message)

        self.spi.close()

    def run(self):
        """
        Periodically retrieves odometry data from the sensors/MCU via SPI
        """
        rate = rospy.Rate(10)  # 10 Hz
        odom_message = [0b00000001, 0, 0, 0, 0]
        while not rospy.is_shutdown():
            rcvd = self.spi.xfer(odom_message)
            # TODO: Now do something withe the odom data

            rate.sleep()


if __name__ == "__main__":
    req = MCU_Comms()
    rospy.on_shutdown(req.shutdown_callback)
    req.run()