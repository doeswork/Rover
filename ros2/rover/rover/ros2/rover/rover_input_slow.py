import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RoverInput(Node):
    def __init__(self):
        super().__init__('rover_input')
        self.publisher_ = self.create_publisher(String, 'stepper_motor_command', 10)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {command}")

def main(args=None):
    rclpy.init(args=args)
    rover_input = RoverInput()

    def is_number(value):
        try:
            float(value)  # Attempt to convert the value to a float
            return True
        except ValueError:
            return False

    while rclpy.ok():
        command = input("Enter command (L/R/F/B:<number>): ")
        parts = command.split(':')
        if len(parts) == 2 and parts[0] in ['L', 'R', 'F', 'B'] and is_number(parts[1]):
            try:
                rover_input.publish_command(command)
            except ValueError:
                print("Invalid angle")
        else:
            print("Invalid command")

    rover_input.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
