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

## Functions for quaternion and rotation matrix conversion
## The code is adapted from the general_robotics_toolbox package
## Code reference: https://github.com/rpiRobotics/rpi_general_robotics_toolbox_py
def hat(k):
    """
    Returns a 3 x 3 cross product matrix for a 3 x 1 vector

             [  0 -k3  k2]
     khat =  [ k3   0 -k1]
             [-k2  k1   0]

    :type    k: numpy.array
    :param   k: 3 x 1 vector
    :rtype:  numpy.array
    :return: the 3 x 3 cross product matrix
    """

    khat=np.zeros((3,3))
    khat[0,1]=-k[2]
    khat[0,2]=k[1]
    khat[1,0]=k[2]
    khat[1,2]=-k[0]
    khat[2,0]=-k[1]
    khat[2,1]=k[0]
    return khat

def q2R(q):
    """
    Converts a quaternion into a 3 x 3 rotation matrix according to the
    Euler-Rodrigues formula.
    
    :type    q: numpy.array
    :param   q: 4 x 1 vector representation of a quaternion q = [q0;qv]
    :rtype:  numpy.array
    :return: the 3x3 rotation matrix    
    """
    
    I = np.identity(3)
    qhat = hat(q[1:4])
    qhat2 = qhat.dot(qhat)
    return I + 2*q[0]*qhat + 2*qhat2
def R2rpy(R):
    assert np.linalg.norm(R[0:2,0]) > np.finfo(float).eps * 10.0, "Singular rpy requested"
    
    r=np.arctan2(R[2,1],R[2,2])
    y=np.arctan2(R[1,0], R[0,0])
    p=np.arctan2(-R[2,0], np.linalg.norm(R[2,1:3]))
        
    return (r,p,y)  
######################
def trim_angle(angle):
    if angle < -180.0:
        return angle+360.0
    if angle > 180.0:
        return angle-360.0
    return angle
def angle_dist(ang1,ang2):
    ang1 = ang1%360
    ang2 = ang2%360
    diff = ang1-ang2
    return min(abs(diff),diff%360)

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.lidar_sub= self.create_subscription(
            CardinalDist,
            '/cardinal',
            self.cardinal_callback,
            10
        )
        self.angle_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.angle_callback,
            10
        )
        self.control_sub = self.create_subscription(
            Bool,
            '/control',
            self.control_callback,
            10
        )
        self.timer = self.create_timer(.01, self.timer_callback)
        self.state = SCAN
        self.start = 0
        self.goal = 0
        self.maze_control = True
        self.cmd_pub = self.create_publisher(Twist,'cmd_vel',10)
        self.current_angle = -999
        self.current_cardinal = -999
    def angle_callback(self,msg):
        quat = msg.pose.pose.orientation
        orientation_list = [quat.x, quat.y, quat.z, quat.w]
        angle = R2rpy(q2R(orientation_list))
        yaw = angle[0]*180/3.1415 # FIXME (lazy pi)
        # self.get_logger().info(str(yaw))
        self.current_angle = yaw
    def cardinal_callback(self, msg):
        self.current_cardinal = msg
    def control_callback(self,msg):
        self.maze_control = msg.data
        
    def timer_callback(self):
        if self.current_angle == -999 or self.current_cardinal==-999:
            return
        if self.maze_control:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0
            spin_speed = 0.1
            linear_speed = 0.1
            self.get_logger().info(str(self.state))
            if self.state == SCAN:
                if self.current_cardinal.right>CELL_SIZE:
                    self.state = ROTATE_THEN_FORWARD
                    self.start = self.current_angle
                    self.goal = trim_angle(self.start-90.0) # want to turn to -90 degrees
                    cmd_vel.angular.z = -spin_speed
                    self.cmd_pub.publish(cmd_vel)
                elif self.current_cardinal.front>CELL_SIZE:
                    self.state = FORWARD
                    self.start = self.current_cardinal.front
                    self.goal = self.current_cardinal.front-CELL_SIZE
                    cmd_vel.linear.x = linear_speed
                    self.cmd_pub.publish(cmd_vel)
                elif self.current_cardinal.left>CELL_SIZE:
                    self.state = ROTATE
                    self.start = self.current_angle 
                    self.goal = trim_angle(self.start+90)
                    cmd_vel.angular.z = spin_speed
                    self.cmd_pub.publish(cmd_vel)
                else:
                    self.state = ROTATE
                    self.start = self.current_angle
                    self.goal = trim_angle(self.start+180)
                    cmd_vel.angular.z = spin_speed
                    self.cmd_pub.publish(cmd_vel)
            elif self.state == FORWARD:
                if self.current_cardinal.front < self.goal:
                    self.state = SCAN #FIXME change to Center & Align
                    self.cmd_pub.publish(cmd_vel)
            elif self.state == ROTATE:
                # cmd_vel.angular.z = (self.goal-self.current_angle)/50
                diff = angle_dist(self.current_angle,self.goal)
                if diff<10.0: # FIXME tune degree requirement
                    self.get_logger().info("rotate done")
                    self.get_logger().info(str(diff))
                    self.get_logger().info(str(self.current_angle))
                    self.get_logger().info(str(self.goal))
                    self.state = ALIGN
                    self.cmd_pub.publish(cmd_vel)
            elif self.state ==ROTATE_THEN_FORWARD:
                # cmd_vel.angular.z = (self.goal-self.current_angle)/50
                diff = angle_dist(self.current_angle,self.goal)
                if diff<10.0:
                    self.get_logger().info("rotate done")
                    self.get_logger().info(str(diff))
                    self.get_logger().info(str(self.current_angle))
                    self.get_logger().info(str(self.goal))
                    self.state = FORWARD
                    cmd_vel = linear_speed
                    self.cmd_pub.publish(cmd_vel)
            elif self.state ==  ALIGN:
                aligned_tolerance = 1.0 # degrees
                if self.current_cardinal.right<CELL_SIZE:
                    diff = self.current_cardinal.right_plus-self.current_cardinal.right_minus
                    if abs(diff) <aligned_tolerance:
                        self.state = SCAN
                        self.cmd_pub.publish(cmd_vel)
                    cmd_vel.angular.z = -diff
                    self.cmd_pub.publish(cmd_vel)
                elif self.current_cardinal.front<CELL_SIZE:
                    diff = self.current_cardinal.front_plus-self.current_cardinal.front_minus
                    if abs(diff) <aligned_tolerance:
                        self.state = SCAN
                        self.cmd_pub.publish(cmd_vel)
                    cmd_vel.angular.z = -diff
                    self.cmd_pub.publish(cmd_vel)
                elif self.current_cardinal.left<CELL_SIZE:
                    diff = self.current_cardinal.left_plus-self.current_cardinal.left_minus
                    if abs(diff) <aligned_tolerance:
                        self.state = SCAN
                        self.cmd_pub.publish(cmd_vel)
                    cmd_vel.angular.z = -diff
                    self.cmd_pub.publish(cmd_vel)
                else:
                    self.state = SCAN
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