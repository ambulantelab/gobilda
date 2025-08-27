import rclpy
from rclpy.node import Node


'''
    We're going to implement an open loop controller/FSM to have the turtlebot
    draw a square on the screen. Below I have commented parts of the code that
    you need to fill in to make the logic complete.
'''

# We have to use the geometry_msgs/msg/Twist to control robots
# Write in here what the correct import should be
from geometry_msgs.msg import TwistStamped

class DrawSquare(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('draw_square_gobilda')
        
        self.publisher_ = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)

        # Functions running at 1Hz
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.turning = False

        self.forward_msg = TwistStamped()
        self.forward_msg.twist.linear.x = 1.0

        # What if I want the robot to turn 90 degrees?
        # Along which axis?
        # (Note that values are in rad/s)
        self.turn_msg = TwistStamped()
        self.turn_msg.twist.angular.z = 3.14

    # Callback for the events
    def timer_callback(self):
        # If robot is turining
        if (self.turning):
            self.get_logger().info('Robot is Turning!')
            self.publisher_.publish(self.turn_msg)
        
        else:
            self.get_logger().info('Robot is going forward!')
            self.publisher_.publish(self.forward_msg)
        
        # Flip the mode of the robot
        # self.turning = not self.turning


def main(args=None):
    rclpy.init(args=args)

    draw_square = DrawSquare()
    rclpy.spin(draw_square)

    draw_square.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
