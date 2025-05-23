#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial

# Parameters for conversion
MAX_PWM = 254
MAX_LINEAR_VEL = 1  # m/s
MAX_ANGULAR_VEL = 1.0  # rad/s
WHEEL_DISTANCE = 0.65  # distance between wheels (meters)
 
class MotorDriver:
    def __init__(self):
        rospy.init_node("motor_driver_node")
        self.serial_port = rospy.get_param("~serial_port", "/dev/ttyACM0")
        self.baud_rate = rospy.get_param("~baud_rate", 9600)
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.loginfo("Motor driver node started. Listening to /cmd_vel")

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Compute wheel speeds
        left_speed = linear - angular * WHEEL_DISTANCE / 2.0
        right_speed = linear + angular * WHEEL_DISTANCE / 2.0

        # Convert to PWM values
        left_pwm = max(min(int((abs(left_speed) / MAX_LINEAR_VEL) * MAX_PWM), MAX_PWM), 0)
        right_pwm = max(min(int((abs(right_speed) / MAX_LINEAR_VEL) * MAX_PWM), MAX_PWM), 0)

        # Determine directions: 0 = forward (LOW), 1 = reverse (HIGH)
        left_dir = 0 if left_speed >= 0 else 1
        right_dir = 0 if right_speed >= 0 else 1

        # Format message
        command = f"<{left_dir},{left_pwm},{right_dir},{right_pwm}>\n"
        self.ser.write(command.encode('utf-8'))

        rospy.loginfo(f"Sent: {command.strip()}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        motor_driver = MotorDriver()
        motor_driver.run()
    except rospy.ROSInterruptException:
        pass
