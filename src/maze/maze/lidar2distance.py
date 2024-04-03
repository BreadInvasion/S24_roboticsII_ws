import rclpy
from rclpy.node import Node


import sensor_msgs.msg


class ReadingLaser(Node):

    def __init__(self):
        super().__init__('reading_laser')

        self.subscription= self.create_subscription(
            sensor_msgs.msg.LaserScan,
            '/scan'
            self.listener_callback
        )


    def listener_callback(self, msg):
        self.get_logger().info(str(msg))


def main(args=None):
    rclpy.init()
    reading_laser = ReadingLaser()                  
    reading_laser.get_logger().info("Hello friend!")
    rclpy.spin(reading_laser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reading_laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
