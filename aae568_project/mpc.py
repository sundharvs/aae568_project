import rclpy
from rclpy.node import Node
import casadi
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry, ActuatorMotors, OffboardControlMode, VehicleThrustSetpoint, VehicleTorqueSetpoint, TrajectorySetpoint, VehicleStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.io import savemat
# from solver import M

class MPC(Node):

    def __init__(self):
        super().__init__('mpc')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.motor_publisher_ = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.odometry_subscriber_ = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odometry_callback, qos_profile
        )
        self.euler_publisher_ = self.create_publisher(Point, "/euler_angles", 10)
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = np.zeros(12)
        self.target = [0,5,-10]
        self.nlp_solver = casadi.Function.load("quadrotor_nlp.casadi")
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

    def timer_callback(self):
        hover_thrust = 2.06430769*9.8/4
        self.publish_offboard_control_mode()
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            (optimal_input, next_state) = self.nlp_solver(self.state, self.target)
            optimal_input = optimal_input.elements()
            next_state = next_state.elements()
            self.publish_motor_thrust(optimal_input) # 0.726 ish to hover
        # self.publish_motor_thrust([hover_thrust+0.45, hover_thrust+0.43, hover_thrust+0.45, hover_thrust+0.43])
        # self.publish_motor_thrust(optimal_input) # 0.726 ish to hover
        # self.publish_position_setpoint(optimal_state)

    def odometry_callback(self, odometry_msg):
        rot = Rotation.from_quat([odometry_msg.q[1], odometry_msg.q[2],odometry_msg.q[3], odometry_msg.q[0]])
        if odometry_msg.position[2] > 0:
            odometry_msg.position[2] = 0
        self.state[0:3] = odometry_msg.position
        self.state[3:6] = -1*rot.as_euler("xyz",degrees=False)
        self.state[6:9] = odometry_msg.velocity
        self.state[9:] = -1*odometry_msg.angular_velocity

        # euler = Point()
        # euler.x = self.state[3]
        # euler.y = self.state[4]
        # euler.z = self.state[5]
        # self.euler_publisher_.publish(euler)


    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

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

        motor_constant = 8.54858e-06
        max_motor_vel = 1000
        min_motor_vel = 150
        thrusts = np.array(thrusts, np.float32)

        motor_inputs = (np.sqrt(thrusts/motor_constant)-min_motor_vel)/(max_motor_vel-min_motor_vel)
        motor_inputs = np.pad(motor_inputs, (0,8))

        msg.control = motor_inputs
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.motor_publisher_.publish(msg)

    def publish_thrust_torque(self, inputs):
        thrust_msg = VehicleThrustSetpoint()
        torque_msg = VehicleTorqueSetpoint()

    def publish_position_setpoint(self, target_state):
        msg = TrajectorySetpoint()
        msg.position = target_state[0:3]
        msg.yaw = float(target_state[5])
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # self.trajectory_setpoint_publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    mpc = MPC()

    rclpy.spin(mpc)

    MPC.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()