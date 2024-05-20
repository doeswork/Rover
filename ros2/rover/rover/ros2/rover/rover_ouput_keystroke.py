import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class RoverOutput(Node):
    def __init__(self):
        super().__init__('rover_output')
        GPIO.setmode(GPIO.BCM)  # Set the GPIO pin numbering mode

        self.servo_pin1 = 4  # Use a different pin for the servo
        GPIO.setup(self.servo_pin1, GPIO.OUT)
        self.servo1 = GPIO.PWM(self.servo_pin1, 50)  # 50Hz pulse for servo
        self.servo1.start(0)  # Start servo in neutral position (usually 7.5 for continuous servos)

        self.servo_pin2 = 27  # Use a different pin for the servo
        GPIO.setup(self.servo_pin2, GPIO.OUT)
        self.servo2 = GPIO.PWM(self.servo_pin2, 50)  # 50Hz pulse for servo
        self.servo2.start(0)  # Start servo in neutral position (usually 7.5 for continuous servos)

        self.servo_pin3 = 17  # Use a different pin for the servo
        GPIO.setup(self.servo_pin3, GPIO.OUT)
        self.servo3 = GPIO.PWM(self.servo_pin3, 50)  # 50Hz pulse for servo
        self.servo3.start(0)  # Start servo in neutral position (usually 7.5 for continuous servos)

        self.servo_pin4 = 22  # Use a different pin for the servo
        GPIO.setup(self.servo_pin4, GPIO.OUT)
        self.servo4 = GPIO.PWM(self.servo_pin4, 50)  # 50Hz pulse for servo
        self.servo4.start(0)  # Start servo in neutral position (usually 7.5 for continuous servos)

        self.subscription = self.create_subscription(
            String,
            'live_topic',
            # 'stepper_motor_command',
            self.command_callback,
            10)

        self.rotation_time_per_rotation = 2  # Time in seconds for one full rotation; adjust this based on your servo's performance

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        if command.startswith('w'): # F
            parts = command.split(':')
            rotations = 1 
            # rotations = int(parts[1])
            self.spin_servo_forward(rotations)
        elif command.startswith('s'): # B
            parts = command.split(':')
            # rotations = int(parts[1])
            rotations = 1 
            self.spin_servo_backward(rotations)
        elif command.startswith('a'): # L
            parts = command.split(':')
            # rotations = int(parts[1])
            rotations = 1 
            self.spin_servo_left(rotations)
        elif command.startswith('d'): # R
            parts = command.split(':')
            # rotations = int(parts[1])
            rotations = 1 
            self.spin_servo_right(rotations)
        else:
            self.get_logger().info("Invalid Action")

    def spin_servo_left(self, rotations):
        # Assuming a duty cycle of 6.5 spins the servo in one direction and 8.5 in the opposite
        # Adjust these values based on your servo's calibration
        self.servo1.ChangeDutyCycle(5.0)
        self.servo2.ChangeDutyCycle(5.0)
        self.servo3.ChangeDutyCycle(5.0)
        self.servo4.ChangeDutyCycle(5.0)
        time.sleep(self.rotation_time_per_rotation * rotations)
        self.servo1.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo2.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo3.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo4.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position

    def spin_servo_right(self, rotations):
        # Assuming a duty cycle of 6.5 spins the servo in one direction and 8.5 in the opposite
        # Adjust these values based on your servo's calibration
        self.servo1.ChangeDutyCycle(10.0)
        self.servo2.ChangeDutyCycle(10.0)
        self.servo3.ChangeDutyCycle(10.0)
        self.servo4.ChangeDutyCycle(10.0)
        time.sleep(self.rotation_time_per_rotation * rotations)
        self.servo1.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo2.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo3.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo4.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position

    def spin_servo_forward(self, rotations):
        # Assuming a duty cycle of 6.5 spins the servo in one direction and 8.5 in the opposite
        # Adjust these values based on your servo's calibration
        self.servo1.ChangeDutyCycle(5.0)
        self.servo2.ChangeDutyCycle(5.0)
        self.servo3.ChangeDutyCycle(10.0)
        self.servo4.ChangeDutyCycle(10.0)
        time.sleep(self.rotation_time_per_rotation * rotations)
        self.servo1.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo2.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo3.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo4.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position

    def spin_servo_backward(self, rotations):
        # Assuming a duty cycle of 6.5 spins the servo in one direction and 8.5 in the opposite
        # Adjust these values based on your servo's calibration
        self.servo1.ChangeDutyCycle(10.0)
        self.servo2.ChangeDutyCycle(10.0)
        self.servo3.ChangeDutyCycle(5.0)
        self.servo4.ChangeDutyCycle(5.0)
        time.sleep(self.rotation_time_per_rotation * rotations)
        self.servo1.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo2.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo3.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position
        self.servo4.ChangeDutyCycle(0)  # Stop the servo by setting it to its neutral position

    def __del__(self):
        if hasattr(self, 'servo1'):
            self.servo1.stop()
        if hasattr(self, 'servo2'):
            self.servo2.stop()
        if hasattr(self, 'servo3'):
            self.servo3.stop()
        if hasattr(self, 'servo4'):
            self.servo4.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    rover_output = RoverOutput()
    rclpy.spin(rover_output)
    rover_output.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
