import rclpy
from rclpy.node import Node
from maze_interfaces.msg import CardinalDist

import sensor_msgs.msg


class ReadingLaser(Node):

    def __init__(self):
        super().__init__('reading_laser')

        self.subscription= self.create_subscription(
            sensor_msgs.msg.LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(CardinalDist,'cardinal',10)


    def listener_callback(self, msg):
        num_angles = len(msg.ranges) # 1080
        front = msg.ranges[0]
        left = msg.ranges[270]
        back = msg.ranges[540]
        right = msg.ranges[810]
        front_right = msg.ranges[810+100]
        back_right = msg.ranges[810-100]
        out = CardinalDist()
        out.front = front
        out.right = right
        out.back = back
        out.left = left
        out.front_right = front_right
        out.back_right = back_right
        self.publisher.publish(out)
        
def main(args=None):
    rclpy.init()
    reading_laser = ReadingLaser()               
    reading_laser.get_logger().info("Lidar Started")
    rclpy.spin(reading_laser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reading_laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
