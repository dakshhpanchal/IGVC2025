#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial
import time

class MDDS30Controller:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB1")  # Updated to working port
        self.baudrate = rospy.get_param("~baudrate", 9600)
        self.wheel_base = rospy.get_param("~wheel_base", 0.61)  # meters
        self.max_speed = rospy.get_param("~max_speed", 1.0)     # m/s

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Let the serial port settle
            rospy.loginfo("Connected to MDDS30 on %s", self.port)
        except serial.SerialException as e:
            rospy.logerr("Could not open serial port %s: %s", self.port, str(e))
            exit(1)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.on_shutdown(self.stop_motors)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x * 3
        w = msg.angular.z * 3

        left_speed = v - (w * self.wheel_base / 2)
        right_speed = v + (w * self.wheel_base / 2)

        # Clamp and scale speed to range 0–63
        left_cmd = self.make_command_byte(1, left_speed)
        right_cmd = self.make_command_byte(0, right_speed)

        # Log raw speeds and commands
        rospy.loginfo(f"Left Speed: {left_speed:.2f} m/s → Byte: {left_cmd}")
        rospy.loginfo(f"Right Speed: {right_speed:.2f} m/s → Byte: {right_cmd}")

        try:
            self.ser.write(bytes([left_cmd]))
            time.sleep(0.01)
            self.ser.write(bytes([right_cmd]))
        except serial.SerialException as e:
            rospy.logerr("Serial write failed: %s", str(e))

    def make_command_byte(self, motor, speed):
        # motor: 0=Left, 1=Right
        direction = 0 if speed >= 0 else 1
        abs_speed = abs(speed)
        scaled = int((min(abs_speed, self.max_speed) / self.max_speed) * 63)
        scaled = min(scaled, 63)
        return (motor << 7) | (direction << 6) | scaled

    def stop_motors(self):
        rospy.loginfo("Stopping motors")
        try:
            self.ser.write(bytes([0b00000000]))  # Left stop
            time.sleep(0.01)
            self.ser.write(bytes([0b10000000]))  # Right stop
        except serial.SerialException as e:
            rospy.logerr("Failed to stop motors: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('mdds30_cmdvel_controller')
    controller = MDDS30Controller()
    rospy.spin()