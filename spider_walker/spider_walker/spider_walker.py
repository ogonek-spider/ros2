#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import math
import time
from pprint import pprint
from spider_walker.kinematics_solver import HexapodKinematicsSolver


class SpiderWalker(Node):
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
            sec=int(step_duration // 1e9),
            nanosec=int(step_duration % 1e9)
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
        """Send trajectory as an action and wait for completion"""
        self.get_logger().info('Waiting for action server...')
        self.trajectory_action_client.wait_for_server()
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg
        
        self.get_logger().info('Sending goal request...')
        send_goal_future = self.trajectory_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return False
            
        self.get_logger().info('Goal accepted :)')
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        self.get_logger().info(f'Result error code: {result.error_code}')
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

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

    def simple_wave_motion(self):
        """Simple wave motion for testing - moves each leg sequentially"""
        self.get_logger().info('Starting simple wave motion')
        
        for leg in range(6):
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = self.joint_names
            
            # Start from neutral
            point1 = JointTrajectoryPoint()
            point1.positions = self.neutral_positions
            point1.time_from_start = Duration(sec=0, nanosec=0)
            
            # Move one leg
            point2 = JointTrajectoryPoint()
            positions = self.neutral_positions.copy()
            leg_offset = leg * 3
            positions[leg_offset] += 0.5  # Rotate base joint
            positions[leg_offset + 2] -= 1.5  # Lift knee
            
            point2.positions = positions
            point2.time_from_start = Duration(sec=1, nanosec=0)
            
            # Return to neutral
            point3 = JointTrajectoryPoint()
            point3.positions = self.neutral_positions
            point3.time_from_start = Duration(sec=2, nanosec=0)
            
            trajectory_msg.points = [point1, point2, point3]            
            self.send_trajectory_action(trajectory_msg)

    def tripod_pose_test(self, phase):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        
        # Start from neutral
        point1 = JointTrajectoryPoint()
        point1.positions = self.neutral_positions
        point1.time_from_start = Duration(sec=0, nanosec=0)
        
        point2 = JointTrajectoryPoint()
        positions = list(self.neutral_positions)

        if phase == 0:
            swing_legs = [2, 4, 6]
        else:
            swing_legs = [1, 3, 5]
 
        for leg in range(1, 7):            
            leg_offset = (leg - 1) * 3
            xyz = self.kinematics.forward(leg, self.kinematics.NEUTRAL)
            if leg in swing_legs:
                xyz[2] += self.step_height                
                xyz[0] += self.step_length
                pass
            else:
                xyz[0] -= self.step_length
                pass
            angles = self.kinematics.inverse(leg, xyz)
            positions[leg_offset] = angles[0]
            positions[leg_offset + 1] = angles[1]
            positions[leg_offset + 2] = angles[2]
        
        point2.positions = positions
        point2.time_from_start = Duration(sec=0, nanosec=500000000)
        
        # # Return to neutral
        point3 = JointTrajectoryPoint()
        point3.positions = point2.positions
        for leg in range(1, 7):            
            leg_offset = (leg - 1) * 3
            if leg in swing_legs:
                positions[leg_offset + 2] += 1.5 #put need on the ground
            else:
                positions[leg_offset] -= 0.5
        point3.time_from_start = Duration(sec=0, nanosec=1000000000)
        
        trajectory_msg.points = [point1, point2]#, point3]   
        #pprint(trajectory_msg.points)
        self.send_trajectory_action(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    
    spider_walker = SpiderWalker()
    
    try:
        spider_walker.return_to_neutral()  
        # Test with simple motion first
        #spider_walker.get_logger().info('Testing with simple wave motion...')
        # spider_walker.simple_wave_motion()
        #spider_walker.tripod_pose_test(0)
        # while True:
        #     spider_walker.return_to_neutral()
        #     spider_walker.tripod_pose_test(0)
        #     spider_walker.tripod_pose_test(1)
        
        # Then try walk
        # ing  
        spider_walker.get_logger().info('Starting walking pattern...')
        
        # Try walking forward (0 degrees)
        spider_walker.step_angle = 0.0
        spider_walker.startup_step(step_duration=2e9)
        spider_walker.walk_step(num_steps=2, step_duration=3)
        
        # Walk right (90 degrees or -90 degrees depending on axis frame)
        # Assuming typical ROS: X forward, Y left. So -90 is Right.
        spider_walker.get_logger().info('Walking Right...')
        spider_walker.step_angle = -90.0
        spider_walker.startup_step(step_duration=2e9)
        spider_walker.walk_step(num_steps=2, step_duration=3)
        
        # Walk left
        spider_walker.get_logger().info('Walking Left...')
        spider_walker.step_angle = 90.0
        spider_walker.startup_step(step_duration=2e9)
        spider_walker.walk_step(num_steps=2, step_duration=3)
        
        # Walk backward
        spider_walker.get_logger().info('Walking Backward...')
        spider_walker.step_angle = 180.0
        spider_walker.startup_step(step_duration=2e9)
        spider_walker.walk_step(num_steps=2, step_duration=3)
        
        while True:
            spider_walker.step_angle = 0.0
            spider_walker.walk_step(num_steps=1, step_duration=3)
        #spider_walker.walk_step(num_steps=1, step_duration=300.0)
        
    except KeyboardInterrupt:
        spider_walker.get_logger().info('Shutting down...')
    finally:
        spider_walker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()