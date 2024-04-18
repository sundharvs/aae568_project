import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
import math

class Radar(Node):

    def __init__(self):
        super().__init__('radar')
        self.odometry_subscriber_ = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odometry_callback, qos_profile_sensor_data
        )
        self.publisher_ = self.create_publisher(Point, 'radar', 10)

    def odometry_callback(self, odometry_msg):
        x = odometry_msg.position[0]
        y = odometry_msg.position[1]
        z = odometry_msg.position[2]

        r = math.sqrt(x**2 + y**2 + z**2)
        theta = math.atan2(y,x)
        phi = math.acos(z/r)

        msg = Point()
        msg.x = r
        msg.y = theta
        msg.z = phi

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    radar = Radar()

    rclpy.spin(radar)

    radar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()