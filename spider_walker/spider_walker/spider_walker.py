#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
import numpy as np
import math
import time
from pprint import pprint
from spider_walker.kinematics_solver import HexapodKinematicsSolver


class SpiderWalker(Node):
    # States for the non-blocking state machine
    IDLE = 'idle'
    STARTUP = 'startup'
    WALKING = 'walking'
    RETURNING = 'returning'

    def __init__(self):
        super().__init__('spider_walker')

        # Create action client for trajectory commands
        self.trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/spider_controller/follow_joint_trajectory'
        )

        # Define all 18 joints (6 legs × 3 joints each)
        self.joint_names = []
        for leg in range(1, 7):
            for joint in range(1, 4):
                self.joint_names.append(f'{leg}-{joint}')

        self.get_logger().info(f'Spider Walker initialized with {len(self.joint_names)} joints')

        # Walking parameters
        self.step_height = 0.5
        self.step_length = 0.4
#        self.body_height = 0.15
#        self.leg_radius = 0.1  # Distance from body center to leg base

        # Leg groups for tripod gait
        self.tripod1 = [1, 3, 5]
        self.tripod2 = [2, 4, 6]

        self.kinematics = HexapodKinematicsSolver.create_default()
        self.neutral_positions = self.kinematics.NEUTRAL * 6

        # Current walking direction (degrees, 0 = forward)
        self.step_angle = 0.0

        # Velocity Tracking
        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_angular_z = 0.0

        # State machine
        self.state = self.IDLE
        self._action_in_flight = False

        # Subscriber setup
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Control timer (runs frequently to check if we need to take a step)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.cmd_linear_x = msg.linear.x
        self.cmd_linear_y = msg.linear.y
        self.cmd_angular_z = msg.angular.z

    def timer_callback(self):
        # Don't issue a new command while an action is still in flight
        if self._action_in_flight:
            return

        moving_now = (abs(self.cmd_linear_x) > 0.05 or
                      abs(self.cmd_linear_y) > 0.05 or
                      abs(self.cmd_angular_z) > 0.05)

        if moving_now:
            rad = math.atan2(self.cmd_linear_y, self.cmd_linear_x)
            self.step_angle = math.degrees(rad)

            if self.state == self.IDLE:
                self.get_logger().info('Starting movement: transitioning to stance...')
                self.state = self.STARTUP
                self.startup_step(step_duration=1.0)
            elif self.state == self.STARTUP:
                # startup just finished (action_in_flight is False), move to walking
                self.state = self.WALKING
                self.walk_step(num_steps=1, step_duration=1.5)
            elif self.state == self.WALKING:
                self.walk_step(num_steps=1, step_duration=1.5)
        else:
            if self.state in (self.STARTUP, self.WALKING):
                self.get_logger().info('Stopping movement: returning to neutral...')
                self.state = self.RETURNING
                self.return_to_neutral()
            elif self.state == self.RETURNING:
                self.state = self.IDLE

    def create_walking_trajectory(self, phase, step_progress):
        """
        Create walking trajectory points
        phase: 0 or 1 (which tripod is swinging)
        step_progress: 0 to 1 (progress through step)
        """
        trajectory_points = []
        
        # Time for this trajectory point
        point_time = Duration(sec=0, nanosec=int(step_progress * 1e9))
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.time_from_start = point_time
        
        # Calculate positions for all joints
        positions = []
        
        for leg_idx in range(1, 7):
            # Determine if this leg is in swing or stance phase
            is_swing = (leg_idx in self.tripod1 and phase == 0) or \
                      (leg_idx in self.tripod2 and phase == 1)
            
            xyz = self.kinematics.forward(leg_idx, self.kinematics.NEUTRAL)
            
            lift_height = 0
            step_distance = 0

            if is_swing:
                # Swing leg: Lift, move forward, lower
                if step_progress < 0.5:
                    # Lift phase
                    lift_height = self.step_height * (step_progress * 2)
                else:
                    # Lower phase
                    lift_height = self.step_height * (2 - step_progress * 2)
                
                # Centered swing: starts at -step_length/2, ends at +step_length/2
                step_distance = self.step_length * (step_progress - 0.5)
            else:
                # Centered stance: starts at +step_length/2, ends at -step_length/2
                step_distance = self.step_length * (0.5 - step_progress)
            
            # Apply heading angle to get X and Y components
            angle_rad = math.radians(self.step_angle)
            step_x = step_distance * math.cos(angle_rad)
            step_y = step_distance * math.sin(angle_rad)
            
            xyz[0] += step_x
            xyz[1] += step_y
            xyz[2] += lift_height
            self.get_logger().info(f'PH {phase} {step_progress*100}% L{leg_idx} {is_swing}: dist {step_distance:.2f}, x {step_x:.2f}, y {step_y:.2f}, z {lift_height:.2f}')
            # Calculate joint angles from foot position
            joint_angles = self.kinematics.inverse(leg_idx, xyz)
            positions.extend(joint_angles)
        
        point.positions = positions
        # Add some velocity for smooth motion
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        
        return point

    def get_foot_position(self, leg_index, progress, lift_height):
        """
        Calculate foot position for a given leg during walking cycle
        progress: 0 to 1 (position in step cycle)
        lift_height: additional height for swing leg
        """
        # Leg positions around the body (hexagonal arrangement)
        angles = [0, 60, 120, 180, 240, 300]  # Degrees
        angle_rad = math.radians(angles[leg_index])
        
        # Base position of leg (relative to body center)
        base_x = self.leg_radius * math.cos(angle_rad)
        base_y = self.leg_radius * math.sin(angle_rad)
        base_z = -self.body_height
        
        # Foot movement during step
        # Swing leg moves forward, stance leg moves backward
        step_x = self.step_length * (2 * progress - 1)  # -1 to 1
        
        # Position relative to leg base
        if leg_index in self.tripod1:
            # These legs move differently in tripod gait
            rel_x = step_x * math.cos(angle_rad)
            rel_y = step_x * math.sin(angle_rad)
        else:
            rel_x = -step_x * math.cos(angle_rad)
            rel_y = -step_x * math.sin(angle_rad)
        
        # Total foot position
        foot_pos = [
            base_x + rel_x,
            base_y + rel_y,
            base_z + lift_height
        ]
        
        return foot_pos
        
    def startup_step(self, step_duration):
        """Transition smoothly from neutral into the starting pose for walking"""
        self.get_logger().info('Executing startup step to transition into gait stance')
        
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        
        # We need 1 motion point that pulls leg groups into their starting phase positions
        # Without lifting them (they slide into position) or perhaps a slight lift
        # The first step of walk_step assumes:
        # Phase 0 progress 0 -> Tripod 1 is at -step_length/2 (about to swing forward)
        #                    -> Tripod 2 is at +step_length/2 (about to push backward)
        
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(
            sec=int(step_duration),
            nanosec=int((step_duration % 1) * 1e9)
        )
        
        positions = []
        angle_rad = math.radians(self.step_angle)
        
        for leg_idx in range(1, 7):
            xyz = self.kinematics.forward(leg_idx, self.kinematics.NEUTRAL)
            
            # Distance offset based on tripod startup
            if leg_idx in self.tripod1:
                dist_offset = -self.step_length * 0.5
            else:
                dist_offset = self.step_length * 0.5
                
            xyz[0] += dist_offset * math.cos(angle_rad)
            xyz[1] += dist_offset * math.sin(angle_rad)
                
            joint_angles = self.kinematics.inverse(leg_idx, xyz)
            positions.extend(joint_angles)
            
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        
        trajectory_msg.points.append(point)
        self.send_trajectory_action(trajectory_msg)

    def send_trajectory_action(self, trajectory_msg):
        """Send trajectory as a non-blocking action. Sets _action_in_flight=True until done."""
        if not self.trajectory_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg

        self._action_in_flight = True
        send_goal_future = self.trajectory_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self._action_in_flight = False
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result error code: {result.error_code}')
        self._action_in_flight = False

    def walk_step(self, num_steps=4, step_duration=2.0):
        """Execute walking steps"""
        self.get_logger().info(f'Starting {num_steps} walking steps')
        
        for step in range(num_steps):
            self.get_logger().info(f'Step {step + 1}/{num_steps}')
            
            # Create trajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = self.joint_names
            
            # Create points for a complete step cycle (2 phases)
            num_points_per_phase = 5
            points = []
            
            # Phase 1: Tripod 1 swings, Tripod 2 stances
            for i in range(num_points_per_phase):
                progress = i / (num_points_per_phase - 1)
                point = self.create_walking_trajectory(0, progress)
                
                # Calculate time for this point (convert to integer nanoseconds)                
                time_from_start = int((progress * step_duration / 2) * 1e9)
                point.time_from_start = Duration(
                    sec=int(time_from_start // 1e9),
                    nanosec=int(time_from_start % 1e9)
                )
                points.append(point)
            
            # Phase 2: Tripod 2 swings, Tripod 1 stances
            for i in range(num_points_per_phase):
                progress = i / (num_points_per_phase - 1)
                point = self.create_walking_trajectory(1, progress)
                
                # Calculate time for this point (starts from halfway)
                time_from_start = int((step_duration / 2 + step_duration/(num_points_per_phase - 1) + progress * step_duration / 2) * 1e9)
                point.time_from_start = Duration(
                    sec=int(time_from_start // 1e9),
                    nanosec=int(time_from_start % 1e9)
                )
                points.append(point)
            trajectory_msg.points = points
            
            # Send trajectory using action client
            #pprint(trajectory_msg.points)
            self.send_trajectory_action(trajectory_msg)
            
        # Return to neutral position
        # self.return_to_neutral()

    def return_to_neutral(self):
        """Return to neutral position"""
        self.get_logger().info('Returning to neutral position')
        
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.neutral_positions
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        trajectory_msg.points.append(point)
        self.send_trajectory_action(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    
    spider_walker = SpiderWalker()
    
    try:
        spider_walker.return_to_neutral()
        spider_walker.get_logger().info('Spider Walker is ready and listening to /cmd_vel...')
        
        # Spin keeps the node alive, timers, and callbacks running
        rclpy.spin(spider_walker)
        
    except KeyboardInterrupt:
        spider_walker.get_logger().info('Shutting down...')
    finally:
        spider_walker.return_to_neutral()
        spider_walker.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()