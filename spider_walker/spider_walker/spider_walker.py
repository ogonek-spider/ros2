#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
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
        
        # Create publisher for trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/spider_controller/joint_trajectory',
            10
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

            # not supper clean, but we need to save leg positions between phases
            if phase == 1:
                if is_swing:
                    xyz[0] -= self.step_length
                else:
                    xyz[0] += self.step_length
            
            lift_height = 0
            step_x = 0

            if is_swing:
                # Swing leg: Lift, move forward, lower
                if step_progress < 0.5:
                    # Lift phase
                    lift_height = self.step_height * (step_progress * 2)
                    #step_x = self.step_length * (step_progress - 1)
                else:
                    # Lower phase
                    lift_height = self.step_height * (2 - step_progress * 2)
                    #step_x = self.step_length * (step_progress - 0.5)
                step_x = self.step_length * step_progress
            else:
                # Stance leg: Push body forward
                #step_x = - self.step_length * (2 * step_progress - 1)            
                step_x = -self.step_length * step_progress
            
            xyz[0] += step_x
            xyz[2] += lift_height
            self.get_logger().info(f'PH {phase} {step_progress*100}% L{leg_idx} {is_swing}: x {step_x}, z {lift_height}')
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
            
            # Publish trajectory
            #pprint(trajectory_msg.points)
            self.trajectory_pub.publish(trajectory_msg)
            
            # Wait for step to complete
            #time.sleep(step_duration * 1.2)
            time.sleep(step_duration * 1)
        
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
        self.trajectory_pub.publish(trajectory_msg)
        
        time.sleep(2.5)

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
            self.trajectory_pub.publish(trajectory_msg)
            
            time.sleep(2.5)

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
        # self.trajectory_pub.publish(trajectory_msg)
        # time.sleep(2.5)
        self.trajectory_pub.publish(trajectory_msg)
            
        time.sleep(2)


def main(args=None):
    rclpy.init(args=args)
    
    spider_walker = SpiderWalker()
    
    try:
        spider_walker.return_to_neutral()      
        spider_walker.return_to_neutral()        
        time.sleep(2)
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
        while True:
            spider_walker.walk_step(num_steps=1, step_duration=10)
        #spider_walker.walk_step(num_steps=1, step_duration=300.0)
        
    except KeyboardInterrupt:
        spider_walker.get_logger().info('Shutting down...')
    finally:
        spider_walker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()