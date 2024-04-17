import rclpy
from rclpy.node import Node
from maze_interfaces.msg import CardinalDist
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.subscription= self.create_subscription(
            CardinalDist,
            '/cardinal',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Twist,'cmd_vel',10)
    def listener_callback(self, msg):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = msg.back_right-msg.front_right
        self.publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init()
    controller = Controller()
    controller.get_logger().info("Controller Started")
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
