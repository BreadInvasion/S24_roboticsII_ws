import rclpy
from rclpy.node import Node
from maze_interfaces.msg import CardinalDist
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import numpy as np
CELL_SIZE = 0.508
SCAN = 0
FORWARD = 1
ROTATE = 2
CENTER = 3
ALIGN = 4
ROTATE_THEN_FORWARD = 5

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.lidar_sub= self.create_subscription(
            CardinalDist,
            '/cardinal',
            self.cardinal_callback,
            10
        )
        self.control_sub = self.create_subscription(
            Bool,
            '/control',
            self.control_callback,
            10
        )
        self.des_pose_pub = self.create_publisher(Twist,'des_pose',10)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.state = SCAN
        self.maze_control = True
        self.current_cardinal = -999
    def cardinal_callback(self, msg):
        self.current_cardinal = msg
    def control_callback(self,msg):
        self.maze_control = msg.data
        
    def timer_callback(self):
        des_pose = Twist()
        des_pose.linear.x = 0.0
        des_pose.linear.y = 0.0
        des_pose.angular.z = 0.0
        if self.current_cardinal==-999:
            return
        if self.maze_control:
            self.get_logger().info(str(self.state))
            if self.state == SCAN:
                if self.current_cardinal.right>CELL_SIZE:
                    self.state = ROTATE_THEN_FORWARD
                    des_pose.angular.z = -1.0
                    self.des_pose_pub.publish(des_pose)
                elif self.current_cardinal.front>CELL_SIZE:
                    # self.state = FORWARD
                    des_pose.linear.x = 1
                    self.des_pose_pub.publish(des_pose)
                # elif self.current_cardinal.left>CELL_SIZE:
                #     self.state = ROTATE
                #     self.start = self.current_angle 
                #     self.goal = trim_angle(self.start+90)
                #     cmd_vel.angular.z = spin_speed
                #     self.cmd_pub.publish(cmd_vel)
                else:
                    # self.state = ROTATE
                    des_pose.angular.z = -1.0
                    self.des_pose_pub.publish(des_pose)
            elif self.state == ROTATE_THEN_FORWARD:
                des_pose.linear.x = 1
                self.des_pose_pub.publish(des_pose)
                self.state=SCAN
            # elif self.state == FORWARD:
            #     if self.current_cardinal.front < self.goal:
            #         self.state = SCAN #FIXME change to Center & Align
            #         self.cmd_pub.publish(cmd_vel)
            # elif self.state == ROTATE:
            #     # cmd_vel.angular.z = (self.goal-self.current_angle)/50
            #     diff = angle_dist(self.current_angle,self.goal)
            #     self.get_logger().info(str(diff))
            #     self.get_logger().info(str(self.current_angle))
            #     self.get_logger().info(str(self.goal))
            #     if diff<10.0: # FIXME tune degree requirement
            #         self.get_logger().info("rotate done")
            #         self.get_logger().info(str(diff))
            #         self.get_logger().info(str(self.current_angle))
            #         self.get_logger().info(str(self.goal))
            #         self.state = ALIGN
            #         self.cmd_pub.publish(cmd_vel)
            # elif self.state ==ROTATE_THEN_FORWARD:
            #     # cmd_vel.angular.z = (self.goal-self.current_angle)/50
            #     diff = angle_dist(self.current_angle,self.goal)
            #     self.get_logger().info(str(diff))
            #     self.get_logger().info(str(self.current_angle))
            #     self.get_logger().info(str(self.goal))
            #     if diff<10.0:
            #         self.get_logger().info("rotate done")
            #         self.get_logger().info(str(diff))
            #         self.get_logger().info(str(self.current_angle))
            #         self.get_logger().info(str(self.goal))
            #         self.state = FORWARD
            #         cmd_vel.linear.x = linear_speed
            #         self.cmd_pub.publish(cmd_vel)
            # elif self.state ==  ALIGN:
            #     aligned_tolerance = 1.0 # degrees
            #     if self.current_cardinal.right<CELL_SIZE:
            #         diff = self.current_cardinal.right_plus-self.current_cardinal.right_minus
            #         if abs(diff) <aligned_tolerance:
            #             self.state = SCAN
            #             self.cmd_pub.publish(cmd_vel)
            #         cmd_vel.angular.z = -diff
            #         self.cmd_pub.publish(cmd_vel)
            #     elif self.current_cardinal.front<CELL_SIZE:
            #         diff = self.current_cardinal.front_plus-self.current_cardinal.front_minus
            #         if abs(diff) <aligned_tolerance:
            #             self.state = SCAN
            #             self.cmd_pub.publish(cmd_vel)
            #         cmd_vel.angular.z = -diff
            #         self.cmd_pub.publish(cmd_vel)
            #     elif self.current_cardinal.left<CELL_SIZE:
            #         diff = self.current_cardinal.left_plus-self.current_cardinal.left_minus
            #         if abs(diff) <aligned_tolerance:
            #             self.state = SCAN
            #             self.cmd_pub.publish(cmd_vel)
            #         cmd_vel.angular.z = -diff
            #         self.cmd_pub.publish(cmd_vel)
            #     else:
            #         self.state = SCAN
            # elif self.state ==  CENTER:
                    
        
            
                
                
                
                
    
        # self.cardinal = msg
        # cmd_vel = Twist()
        # cmd_vel.linear.x = 0.0
        # cmd_vel.linear.y = 0.0
        # cmd_vel.angular.z = 0.0
        # cmd_vel.angular.z = msg.back_right-msg.front_right
        # self.cmd_pub.publish(cmd_vel)
            


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
