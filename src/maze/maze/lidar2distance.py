import rclpy
from rclpy.node import Node


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


    def listener_callback(self, msg):
        num_angles = len(msg.ranges) # 1080
        # TODO: Figure out which directions are which
        dir1 = msg.ranges[0]
        dir2 = msg.ranges[270]
        dir3 = msg.ranges[540]
        dir4 = msg.ranges[810]
        dir5 = msg.ranges[1079]
        self.get_logger().info(str((dir1,dir2,dir3,dir4,dir5)))

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
