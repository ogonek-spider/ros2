#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
import math
from spider_walker.kinematics_solver import HexapodKinematicsSolver
from spider_walker.trajectory_generator import TripodTrajectoryGenerator


class SpiderWalker(Node):
    IDLE      = 'idle'
    STARTUP   = 'startup'
    WALKING   = 'walking'
    RETURNING = 'returning'
    
    STEP_MIN_DURATION = 1.5

    def __init__(self):
        super().__init__('spider_walker')

        self.trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/spider_controller/follow_joint_trajectory'
        )

        joint_names = [f'{leg}-{joint}' for leg in range(1, 7) for joint in range(1, 4)]

        kinematics = HexapodKinematicsSolver.create_default()
        self.traj = TripodTrajectoryGenerator(kinematics, joint_names)

        self.get_logger().info(f'Spider Walker initialized with {len(joint_names)} joints')

        # Current walking direction (degrees, 0 = forward)
        self.step_angle = 0.0

        # Velocity tracking
        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_angular_z = 0.0

        # State machine
        self.state = self.IDLE
        self._action_in_flight = False

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.cmd_linear_x = msg.linear.x
        self.cmd_linear_y = msg.linear.y
        self.cmd_angular_z = msg.angular.z

    def timer_callback(self):
        if self._action_in_flight:
            return

        moving_now = (abs(self.cmd_linear_x) > 0.05 or
                      abs(self.cmd_linear_y) > 0.05 or
                      abs(self.cmd_angular_z) > 0.05)

        if moving_now:
            self.step_angle = math.degrees(math.atan2(self.cmd_linear_y, self.cmd_linear_x))
            self.step_duration = self.STEP_MIN_DURATION / min(1.0, math.sqrt(self.cmd_linear_x**2 + self.cmd_linear_y**2))

            if self.state == self.IDLE:
                self.get_logger().info('Starting movement: transitioning to stance...')
                self.state = self.STARTUP
                self._send(self.traj.make_startup(self.step_angle, self.step_duration/2))
            elif self.state == self.STARTUP:
                self.state = self.WALKING
                self._send(self.traj.make_walk_cycle(self.step_angle, self.step_duration))
            elif self.state == self.WALKING:
                self._send(self.traj.make_walk_cycle(self.step_angle, self.step_duration))
        else:
            if self.state in (self.STARTUP, self.WALKING):
                self.get_logger().info('Stopping: returning to neutral...')
                self.state = self.RETURNING
                self._send(self.traj.make_return_to_neutral())
            elif self.state == self.RETURNING:
                self.state = self.IDLE

    # ------------------------------------------------------------------
    # Action client (non-blocking)
    # ------------------------------------------------------------------

    def _send(self, trajectory_msg):
        if not self.trajectory_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg

        self._action_in_flight = True
        future = self.trajectory_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self._action_in_flight = False
            return
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_code}')
        self._action_in_flight = False


def main(args=None):
    rclpy.init(args=args)
    spider_walker = SpiderWalker()

    try:
        spider_walker._send(spider_walker.traj.make_return_to_neutral())
        spider_walker.get_logger().info('Spider Walker ready, listening to /cmd_vel...')
        rclpy.spin(spider_walker)
    except KeyboardInterrupt:
        spider_walker.get_logger().info('Shutting down...')
    finally:
        spider_walker._send(spider_walker.traj.make_return_to_neutral())
        spider_walker.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
