import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RoverInput(Node):
    def __init__(self):
        super().__init__('rover_input')
        self.publisher_ = self.create_publisher(String, 'live_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        command = input("Enter command (w/s/a/d): ")
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    rover_input = RoverInput()
    rclpy.spin(rover_input)
    rover_input.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

