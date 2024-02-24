import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class RoverOutput(Node):
    def __init__(self):
        super().__init__('rover_output')
        GPIO.setmode(GPIO.BCM)  # Set the GPIO pin numbering mode

        # Servo motor setup 1
        self.servo_pin1 = 4 # Use a different pin for the servo
        GPIO.setup(self.servo_pin1, GPIO.OUT)
        self.servo1 = GPIO.PWM(self.servo_pin1, 50)  # 50Hz pulse for servo
        self.servo1.start(0)  # Initialization

        # Subscription
        self.subscription = self.create_subscription(
            String,
            'stepper_motor_command',
            self.command_callback,
            10)
        self.get_logger().info("RoverOutput node has been initialized")

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        if command.startswith('S'):
            parts = command.split(':')
            angle = int(parts[1])
            servo_number = int(parts[0][1:])
            if servo_number == 1:
                self.set_servo_angle(angle, self.servo1)
            else:
                self.get_logger().info("Invalid servo number")

    def set_servo_angle(self, angle, servo):
        duty = float(angle) / 18.0 + 2.5
        servo.ChangeDutyCycle(duty)

    def __del__(self):
        if hasattr(self, 'servo1'):
            self.servo1.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    rover_output = RoverOutput()
    rclpy.spin(rover_output)
    # Explicitly destroy the node before shutting down rclpy
    rover_output.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
