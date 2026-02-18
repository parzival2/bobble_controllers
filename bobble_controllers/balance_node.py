"""Effort-based cascaded PID balance controller for BobbleBot.

Subscribes to EKF-filtered odometry for orientation and joint_states for wheel
velocities. Runs a cascaded PID loop (velocity → tilt → effort) and publishes
direct effort commands to the forward_command_controller.

Also publishes wheel odometry on /bobble_controller/odom so the EKF can fuse it
(replacing what the old diff_drive_controller provided).
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from bobble_controllers.pid_controller import PIDController


# State machine
IDLE = 0
STARTUP = 1
BALANCE = 2


class BalanceNode(Node):
    def __init__(self):
        super().__init__('balance_node')

        # Declare all parameters
        self.declare_parameter('control_rate', 500.0)
        self.declare_parameter('wheel_radius', 0.0275)
        self.declare_parameter('wheel_separation', 0.16)
        self.declare_parameter('max_tilt_safety', 0.349)
        self.declare_parameter('startup_tilt_threshold', 0.175)
        self.declare_parameter('max_effort', 0.4)

        # Velocity PID
        self.declare_parameter('velocity_pid.kp', 0.125)
        self.declare_parameter('velocity_pid.ki', 0.01)
        self.declare_parameter('velocity_pid.kd', 0.05)
        self.declare_parameter('velocity_pid.output_limit', 0.1484)
        self.declare_parameter('velocity_pid.max_integral', 0.025)
        self.declare_parameter('velocity_pid.output_filter', 0.99)

        # Tilt PID
        self.declare_parameter('tilt_pid.kp', 2.5)
        self.declare_parameter('tilt_pid.ki', 0.0)
        self.declare_parameter('tilt_pid.kd', 0.5)
        self.declare_parameter('tilt_pid.output_filter', 0.05)

        # Turning PID
        self.declare_parameter('turning_pid.kp', 0.25)
        self.declare_parameter('turning_pid.ki', 0.05)
        self.declare_parameter('turning_pid.kd', 0.0)
        self.declare_parameter('turning_pid.output_filter', 0.5)
        self.declare_parameter('turning_pid.max_integral', 1.0)

        # Load parameters and build PID controllers
        self._load_params()

        # State machine
        self._state = IDLE
        self._startup_cmd = True  # Auto-start on launch

        # EKF state
        self._pitch = 0.0
        self._pitch_rate = 0.0
        self._has_odom = False

        # Joint state
        self._left_wheel_vel = 0.0
        self._right_wheel_vel = 0.0
        self._left_wheel_pos = 0.0
        self._right_wheel_pos = 0.0
        self._has_joint_states = False

        # Wheel odom integration state
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._prev_left_pos = None
        self._prev_right_pos = None

        # Desired velocity from teleop
        self._desired_linear = 0.0
        self._desired_angular = 0.0

        # Control loop timing
        self._prev_time = None

        # Subscribers
        self._odom_sub = self.create_subscription(
            Odometry, '/bobble/odom/filtered', self._odom_callback, 10)
        self._joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10)
        self._vel_cmd_sub = self.create_subscription(
            Twist, '~/velocity_cmd', self._velocity_cmd_callback, 10)

        # Publishers
        self._effort_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10)
        self._odom_pub = self.create_publisher(
            Odometry, '/bobble_controller/odom', 10)
        self._debug_pub = self.create_publisher(
            Float64MultiArray, '~/debug', 10)

        # Control timer
        rate = self.get_parameter('control_rate').value
        self._timer = self.create_timer(1.0 / rate, self._control_loop)

        # Dynamic reconfigure
        self.add_on_set_parameters_callback(self._param_callback)

        self.get_logger().info(
            f'Effort balance controller started at {rate} Hz')

    def _load_params(self):
        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._wheel_separation = self.get_parameter('wheel_separation').value
        self._max_tilt_safety = self.get_parameter('max_tilt_safety').value
        self._startup_tilt_threshold = self.get_parameter('startup_tilt_threshold').value
        self._max_effort = self.get_parameter('max_effort').value

        self._vel_pid = PIDController(
            kp=self.get_parameter('velocity_pid.kp').value,
            ki=self.get_parameter('velocity_pid.ki').value,
            kd=self.get_parameter('velocity_pid.kd').value,
            output_limit=self.get_parameter('velocity_pid.output_limit').value,
            max_integral=self.get_parameter('velocity_pid.max_integral').value,
            output_filter=self.get_parameter('velocity_pid.output_filter').value,
        )
        self._tilt_pid = PIDController(
            kp=self.get_parameter('tilt_pid.kp').value,
            ki=self.get_parameter('tilt_pid.ki').value,
            kd=self.get_parameter('tilt_pid.kd').value,
            output_limit=self._max_effort,
            output_filter=self.get_parameter('tilt_pid.output_filter').value,
        )
        self._turn_pid = PIDController(
            kp=self.get_parameter('turning_pid.kp').value,
            ki=self.get_parameter('turning_pid.ki').value,
            kd=self.get_parameter('turning_pid.kd').value,
            output_limit=self._max_effort / 2.0,
            max_integral=self.get_parameter('turning_pid.max_integral').value,
            output_filter=self.get_parameter('turning_pid.output_filter').value,
        )

    def _param_callback(self, params):
        for p in params:
            name = p.name
            v = p.value
            # Velocity PID
            if name == 'velocity_pid.kp':
                self._vel_pid.kp = v
            elif name == 'velocity_pid.ki':
                self._vel_pid.ki = v
                self._vel_pid.reset()
            elif name == 'velocity_pid.kd':
                self._vel_pid.kd = v
            elif name == 'velocity_pid.output_limit':
                self._vel_pid.output_limit = v
            elif name == 'velocity_pid.max_integral':
                self._vel_pid.max_integral = v
            elif name == 'velocity_pid.output_filter':
                self._vel_pid.output_filter = v
            # Tilt PID
            elif name == 'tilt_pid.kp':
                self._tilt_pid.kp = v
            elif name == 'tilt_pid.ki':
                self._tilt_pid.ki = v
                self._tilt_pid.reset()
            elif name == 'tilt_pid.kd':
                self._tilt_pid.kd = v
            elif name == 'tilt_pid.output_filter':
                self._tilt_pid.output_filter = v
            # Turning PID
            elif name == 'turning_pid.kp':
                self._turn_pid.kp = v
            elif name == 'turning_pid.ki':
                self._turn_pid.ki = v
                self._turn_pid.reset()
            elif name == 'turning_pid.kd':
                self._turn_pid.kd = v
            elif name == 'turning_pid.output_filter':
                self._turn_pid.output_filter = v
            elif name == 'turning_pid.max_integral':
                self._turn_pid.max_integral = v
            # Safety / physical
            elif name == 'max_tilt_safety':
                self._max_tilt_safety = v
            elif name == 'startup_tilt_threshold':
                self._startup_tilt_threshold = v
            elif name == 'max_effort':
                self._max_effort = v
                self._tilt_pid.output_limit = v
                self._turn_pid.output_limit = v / 2.0
            elif name == 'wheel_radius':
                self._wheel_radius = v
            elif name == 'wheel_separation':
                self._wheel_separation = v
            elif name == 'control_rate':
                self._timer.cancel()
                self._timer = self.create_timer(1.0 / v, self._control_loop)

        self.get_logger().info(f'Parameters updated')
        return SetParametersResult(successful=True)

    # ── Callbacks ──

    def _odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # Extract pitch from quaternion: pitch = asin(2*(w*y - z*x))
        self._pitch = math.asin(
            max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        # Pitch rate from EKF (fused gyro)
        self._pitch_rate = msg.twist.twist.angular.y
        self._has_odom = True

    def _joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'left_wheel_hinge':
                self._left_wheel_vel = msg.velocity[i] if msg.velocity else 0.0
                self._left_wheel_pos = msg.position[i] if msg.position else 0.0
            elif name == 'right_wheel_hinge':
                self._right_wheel_vel = msg.velocity[i] if msg.velocity else 0.0
                self._right_wheel_pos = msg.position[i] if msg.position else 0.0
        self._has_joint_states = True

    def _velocity_cmd_callback(self, msg: Twist):
        self._desired_linear = msg.linear.x
        self._desired_angular = msg.angular.z

    # ── Control Loop ──

    def _control_loop(self):
        if not self._has_odom or not self._has_joint_states:
            return

        now = self.get_clock().now()

        # Compute dt
        if self._prev_time is None:
            self._prev_time = now
            return
        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        pitch = self._pitch
        pitch_rate = self._pitch_rate
        abs_pitch = abs(pitch)

        # Forward velocity from wheel encoders
        fwd_velocity = self._wheel_radius * (
            self._right_wheel_vel + self._left_wheel_vel) / 2.0
        # Turn rate from wheel encoders
        turn_rate = self._wheel_radius * (
            self._right_wheel_vel - self._left_wheel_vel) / self._wheel_separation

        # Publish wheel odometry (replaces diff_drive_controller odom)
        self._publish_wheel_odom(dt)

        # ── State Machine ──
        if self._state == IDLE:
            self._publish_effort(0.0, 0.0)
            if self._startup_cmd:
                self._state = STARTUP
                self._startup_cmd = False
                self.get_logger().info('IDLE → STARTUP')
            self._publish_debug(pitch, 0.0, 0.0, 0.0, 0.0)
            return

        if self._state == STARTUP:
            self._publish_effort(0.0, 0.0)
            if abs_pitch < self._startup_tilt_threshold:
                self._state = BALANCE
                self._vel_pid.reset()
                self._tilt_pid.reset()
                self._turn_pid.reset()
                self.get_logger().info('STARTUP → BALANCE')
            self._publish_debug(pitch, 0.0, 0.0, 0.0, 0.0)
            return

        # BALANCE state
        # Safety check: if tilt exceeds limit, go back to IDLE
        if abs_pitch > self._max_tilt_safety:
            self.get_logger().warn(
                f'Tilt safety triggered ({math.degrees(abs_pitch):.1f} deg). '
                'BALANCE → IDLE')
            self._state = IDLE
            self._startup_cmd = True  # Will auto-restart
            self._vel_pid.reset()
            self._tilt_pid.reset()
            self._turn_pid.reset()
            self._publish_effort(0.0, 0.0)
            self._publish_debug(pitch, 0.0, 0.0, 0.0, 0.0)
            return

        # ── Cascaded PID ──

        # 1. Velocity PID: desired_velocity → desired_tilt
        vel_error = self._desired_linear - fwd_velocity
        desired_tilt = self._vel_pid.compute(vel_error, dt)

        # 2. Tilt PID: tilt_error → effort
        #    error = pitch - desired_tilt (positive pitch = forward lean)
        #    D term uses pitch_rate directly from EKF (not error derivative)
        tilt_error = pitch - desired_tilt
        tilt_effort = self._tilt_pid.compute(tilt_error, dt,
                                              derivative=pitch_rate)

        # 3. Turning PID: turn_rate_error → heading_effort
        turn_error = self._desired_angular - turn_rate
        heading_effort = self._turn_pid.compute(turn_error, dt)

        # ── Motor Mixing ──
        # Reference convention: Left = -TiltEffort - HeadingEffort
        #                       Right = -TiltEffort + HeadingEffort
        #
        # In our URDF (both wheels axis +Y), positive effort → forward torque.
        # Forward lean → positive pitch → positive tilt_error → positive tilt_effort.
        # To correct forward lean, we need forward wheel torque (wheels drive forward
        # to move base of support under COM). So we want positive tilt_effort to
        # produce positive effort on both wheels:
        #   left  = tilt_effort - heading_effort
        #   right = tilt_effort + heading_effort
        left_effort = tilt_effort - heading_effort
        right_effort = tilt_effort + heading_effort

        # Clamp individual motor efforts
        left_effort = max(-self._max_effort, min(self._max_effort, left_effort))
        right_effort = max(-self._max_effort, min(self._max_effort, right_effort))

        self._publish_effort(left_effort, right_effort)
        self._publish_debug(pitch, desired_tilt, tilt_effort, heading_effort,
                            fwd_velocity)

    # ── Publishers ──

    def _publish_effort(self, left: float, right: float):
        msg = Float64MultiArray()
        msg.data = [left, right]
        self._effort_pub.publish(msg)

    def _publish_wheel_odom(self, dt):
        """Compute and publish diff-drive odometry from joint positions."""
        left_pos = self._left_wheel_pos
        right_pos = self._right_wheel_pos

        if self._prev_left_pos is None:
            self._prev_left_pos = left_pos
            self._prev_right_pos = right_pos
            return

        # Delta wheel travel
        d_left = (left_pos - self._prev_left_pos) * self._wheel_radius
        d_right = (right_pos - self._prev_right_pos) * self._wheel_radius
        self._prev_left_pos = left_pos
        self._prev_right_pos = right_pos

        d_center = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / self._wheel_separation

        # Integrate pose
        self._odom_yaw += d_yaw
        self._odom_x += d_center * math.cos(self._odom_yaw)
        self._odom_y += d_center * math.sin(self._odom_yaw)

        # Linear and angular velocity from wheel velocities
        fwd_vel = self._wheel_radius * (
            self._right_wheel_vel + self._left_wheel_vel) / 2.0
        yaw_rate = self._wheel_radius * (
            self._right_wheel_vel - self._left_wheel_vel) / self._wheel_separation

        # Build odometry message
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'bobble_chassis_link'

        msg.pose.pose.position.x = self._odom_x
        msg.pose.pose.position.y = self._odom_y
        msg.pose.pose.position.z = 0.0

        # Yaw-only quaternion
        half_yaw = self._odom_yaw / 2.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        msg.twist.twist.linear.x = fwd_vel
        msg.twist.twist.angular.z = yaw_rate

        # Covariances (match what diff_drive used)
        msg.pose.covariance[0] = 0.001   # x
        msg.pose.covariance[7] = 0.001   # y
        msg.pose.covariance[14] = 0.001  # z
        msg.pose.covariance[21] = 0.001  # roll
        msg.pose.covariance[28] = 0.001  # pitch
        msg.pose.covariance[35] = 0.01   # yaw
        msg.twist.covariance[0] = 0.001  # vx
        msg.twist.covariance[7] = 0.001  # vy
        msg.twist.covariance[14] = 0.001 # vz
        msg.twist.covariance[21] = 0.001 # vroll
        msg.twist.covariance[28] = 0.001 # vpitch
        msg.twist.covariance[35] = 0.01  # vyaw

        self._odom_pub.publish(msg)

    def _publish_debug(self, pitch, desired_tilt, tilt_effort,
                       heading_effort, fwd_velocity):
        """Publish debug: [state, pitch_deg, desired_tilt_deg, tilt_effort,
                           heading_effort, fwd_velocity]"""
        msg = Float64MultiArray()
        msg.data = [
            float(self._state),
            math.degrees(pitch),
            math.degrees(desired_tilt),
            tilt_effort,
            heading_effort,
            fwd_velocity,
        ]
        self._debug_pub.publish(msg)

    def destroy_node(self):
        self._publish_effort(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BalanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
