import rclpy
from rclpy.node import Node
import casadi
from std_msgs.msg import String


class MPC(Node):

    def __init__(self):
        super().__init__('mpc')
        self.solver = casadi.Function.load("quadrotor_nlp.casadi")
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.25, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    MPC = MPC()

    rclpy.spin(MPC)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    MPC.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()