import rclpy
from rclpy.node import Node
from maze_interfaces.msg import CardinalDist
from geometry_msgs.msg import Twist
CELL_SIZE = .5
SCAN = 0
FORWARD = 1
ROTATE = 2
CENTER = 3
ALIGN = 4
class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.subscription= self.create_subscription(
            CardinalDist,
            '/cardinal',
            self.listener_callback,
            10
        )
        self.state = SCAN
        self.start = 0
        self.goal = 0
        self.publisher = self.create_publisher(Twist,'cmd_vel',10)
    def listener_callback(self, msg):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = 0.0
        if self.state == SCAN:
            if msg.right>CELL_SIZE:
                self.state = ROTATE
                self.start = 0#FIXME self.start = imu.angle
                self.goal = self.start-90.0 # want to turn to -90 degrees
                cmd_vel.angular.z = -0.1
                self.publisher.publish(cmd_vel)
            elif msg.forward>CELL_SIZE:
                self.state = FORWARD
                self.start = msg.forward
                self.goal = msg.forward-CELL_SIZE
                cmd_vel.linear.x = 0.1
                self.publisher.publish(cmd_vel)
            elif msg.left>CELL_SIZE:
                self.state = ROTATE
                self.start = 0 #FIXME self.start = imu.angle
                self.goal = self.start+90
                cmd_vel.angular.z = 0.1
                self.publisher.publish(cmd_vel)
            else:
                self.state = ROTATE
                self.start = 0 #FIXME self.start = imu.angle
                self.goal = self.start+180
                cmd_vel.angular.z = 0.1
                self.publisher.publish(cmd_vel)
        elif self.state == FORWARD:
            if msg.forward < self.goal:
                self.state = SCAN #FIXME change to Center & Align
                self.publisher.publish(cmd_vel)
        elif self.state == ROTATE:
            # if imu.angle close to self.goal: # FIXME
                self.state = SCAN
                self.publisher.publish(cmd_vel)
        elif self.state ==  ALIGN:
            if msg.right<CELL_SIZE:
                cmd_vel.angular.z = -(msg.right_plus-msg.right_minus)
                self.publisher.publish(cmd_vel)
            elif msg.front<CELL_SIZE:
                cmd_vel.angular.z = -(msg.front_plus-msg.front_minus)
                self.publisher.publish(cmd_vel)
            elif msg.left<CELL_SIZE:
                cmd_vel.angular.z = -(msg.left_plus-msg.left_minus)
                self.publisher.publish(cmd_vel)
            else:
                self.state = SCAN
        # elif self.state ==  CENTER:
                
        
            
                
                
                
                
    
        # self.cardinal = msg
        # cmd_vel = Twist()
        # cmd_vel.linear.x = 0.0
        # cmd_vel.linear.y = 0.0
        # cmd_vel.angular.z = 0.0
        # cmd_vel.angular.z = msg.back_right-msg.front_right
        # self.publisher.publish(cmd_vel)
    # def timer_callback(self):
    #     if self.state == SCAN:
            


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