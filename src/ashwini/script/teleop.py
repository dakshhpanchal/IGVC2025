'''IF USING MDDS30
USE THIS CODE
'''
'''
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
'''



'''IF USING MDDS20
USE FOLLOWING CODE'''

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial

# Parameters for conversion
MAX_PWM = 255
MAX_LINEAR_VEL = 1    # m/s
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
