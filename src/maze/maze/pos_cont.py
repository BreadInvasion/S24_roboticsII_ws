import rclpy
from rclpy.node import Node
from maze_interfaces.msg import CardinalDist
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import numpy as np
CELL_SIZE = 0.508

class Controller(Node):
    def __init__(self):
        super().__init__('pos_controller')
        self.pos_sub= self.create_subscription(
            Twist,
            '/des_pose',
            self.pose_callback,
            10
        )
        self.timer = self.create_timer(.01, self.timer_callback)
        self.tim = 0
        self.stop = 0
        self.maze_control = True
        self.cmd_pub = self.create_publisher(Twist,'cmd_vel',10)
    def pose_callback(self, msg):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = 0.0
        if z != 0:
            if z<0.0:
                cmd_vel.angular.z = -1.0
            if z>0.0:
                cmd_vel.angular.z = 1.0
            self.stop = 217
            self.tim = 0
            self.cmd_pub.publish(cmd_vel)
        elif x!=0:
            if x >0.0:
                cmd_vel.linear.x = .2
            if x <0.0:
                cmd_vel.linear.x = -.2
            self.stop = 290
            self.tim = 0
            self.cmd_pub.publish(cmd_vel)
    def timer_callback(self):
        if self.stop !=0:
            self.tim+=1
            if self.tim>=self.stop:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0
                cmd_vel.linear.y = 0.0
                cmd_vel.angular.z = 0.0
                self.stop = 0
                self.tim = 0
                self.cmd_pub.publish(cmd_vel)



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
