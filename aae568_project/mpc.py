import rclpy
from rclpy.node import Node
import casadi
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry, ActuatorMotors, OffboardControlMode, VehicleThrustSetpoint, VehicleTorqueSetpoint
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from scipy.spatial.transform import Rotation

class MPC(Node):

    def __init__(self):
        super().__init__('mpc')
        self.control_publisher_ = self.create_publisher(String, 'topic', 10)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.motor_publisher_ = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", 10)

        self.odometry_subscriber_ = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odometry_callback, qos_profile_sensor_data
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = np.zeros(12)
        self.target = [0,0,3,0,0,0,0,0,0,0,0,0]
        self.nlp_solver = casadi.Function.load("quadrotor_nlp.casadi")

    def timer_callback(self):
        hover_thrust = 1.84*9.81/4
        self.publish_offboard_control_mode()
        optimal_input = self.nlp_solver(self.target - self.state).elements()
        self.publish_motor_thrust(optimal_input) # 0.726 ish to hover

    def odometry_callback(self, odometry_msg):
        rot = Rotation.from_quat([odometry_msg.q[1], odometry_msg.q[2],odometry_msg.q[3], odometry_msg.q[0]])
        self.state[0:3] = odometry_msg.position
        self.state[3:6] = rot.as_euler("xyz",degrees=False)
        self.state[6:9] = odometry_msg.velocity
        self.state[9:] = odometry_msg.angular_velocity

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)
    
    def publish_motor_thrust(self, thrusts):
        msg = ActuatorMotors()

        thrusts = np.array(thrusts,np.float32)
        motor_constant = 8.54858e-06
        motor_velocity = np.sqrt(abs(thrusts/motor_constant))
        motor_inputs = motor_velocity / 1000
        motor_inputs = np.pad(motor_inputs, (0,8))

        msg.control = motor_inputs
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.motor_publisher_.publish(msg)

    def publish_thrust_torque(self, inputs):
        thrust_msg = VehicleThrustSetpoint()
        torque_msg = VehicleTorqueSetpoint()


def main(args=None):
    rclpy.init(args=args)

    mpc = MPC()

    rclpy.spin(mpc)

    MPC.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()