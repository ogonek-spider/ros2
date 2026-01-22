import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
import PyKDL as kdl
import time
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF

from pprint import pprint

# Add this at the VERY BEGINNING of your file
import sys
import traceback
import faulthandler

# Enable faulthandler to get traceback on segfault
faulthandler.enable()

# Set Python recursion limit
sys.setrecursionlimit(10000)

LEG_CHAIN_ROOT = "body_plate_deepseek"
LEG_CHAIN_TIP = lambda leg_id: '3limb' if leg_id == 1 else f'3limb_{leg_id}'

class HexapodKDLController(Node):
    def __init__(self):
        super().__init__('hexapod_kdl_controller')
        
        self.declare_parameter('robot_description', 'string')
        urdf_string = self.get_parameter('robot_description').get_parameter_value().string_value
        
        if urdf_string:
            self.robot_urdf = URDF.from_xml_string(urdf_string)
            self.get_logger().info(f'Successfully loaded URDF from parameter, joints count {len(self.robot_urdf.joints)}, test leg chain {self.robot_urdf.get_chain(LEG_CHAIN_ROOT, LEG_CHAIN_TIP(1))}')

            (ok, self.kdl_tree) = treeFromUrdfModel(self.robot_urdf, False)
            if not ok:
                raise Exception("Failed to extract KDL tree from URDF")
            elif self.kdl_tree.getNrOfJoints() == 0:
                raise Exception("No joints found in KDL Tree")
            else:
                self.get_logger().info(f"Extracted KDL tree from URDF joints {self.kdl_tree.getNrOfJoints()}, segments {self.kdl_tree.getNrOfSegments()}")
            
            # Create chains and solvers for all legs
            self.chains = {}
            self.fk_solvers = {}
            self.ik_solvers = {}
            self.joint_limits = {}
            
            for leg_id in range(1, 7):
                self.setup_leg_solver(leg_id)
            
            # Current joint positions
            self.q_current = self.initialize_joint_positions()                        
        else:
            self.get_logger().warn('No URDF found on parameter server')

    def setup_leg_solver(self, leg_id):
        """Setup KDL solvers for a single leg"""
        # Get chain from base to foot
        chain = self.kdl_tree.getChain(LEG_CHAIN_ROOT, LEG_CHAIN_TIP(leg_id))
        if not chain:
            self.get_logger().error(f"Failed to get chain for leg {leg_id}")
            return
        elif chain.getNrOfJoints() != 3:            
            self.get_logger().error(f"Leg {leg_id} (chain from {LEG_CHAIN_ROOT} to {LEG_CHAIN_TIP(leg_id)}), wrong num of joints {chain.getNrOfJoints()}")
            self.get_logger().error(f"Segments {chain.getNrOfSegments()}")
            return
        
        self.chains[leg_id] = chain
        
        # Create forward kinematics solver
        self.fk_solvers[leg_id] = kdl.ChainFkSolverPos_recursive(chain)
        
        # Create inverse kinematics velocity solver
        ik_vel_solver = kdl.ChainIkSolverVel_pinv(chain)
        
        # Get joint limits from URDF
        joint_min, joint_max = self.get_joint_limits(leg_id)
        self.joint_limits[leg_id] = (joint_min, joint_max)
        
        # Create numerical IK solver (Newton-Raphson)
        self.ik_solvers[leg_id] = kdl.ChainIkSolverPos_NR_JL(
            chain,
            joint_min,
            joint_max,
            self.fk_solvers[leg_id],
            ik_vel_solver,
            100,  # max iterations
            1e-6  # epsilon
        )
        self.ik_solvers[leg_id] = kdl.ChainIkSolverPos_NR (
            chain, 
            self.fk_solvers[leg_id], 
            ik_vel_solver, 100, 1e-6)

        self.ik_solvers[leg_id] = kdl.ChainIkSolverPos_LMA (chain)

    def get_joint_limits(self, leg_id):
        """Extract joint limits from URDF"""
        
        q_min = kdl.JntArray(3)
        q_max = kdl.JntArray(3)
        for joint_id in range(3):
            joint = self.robot_urdf.joint_map[f"{leg_id}-{joint_id + 1}"]
            
            if joint.limit:
                q_min[joint_id] = joint.limit.lower
                q_max[joint_id] = joint.limit.upper
            else:
                q_min[joint_id] = -np.pi
                q_max[joint_id] = np.pi
        
        #self.get_logger().info(f"Leg {leg_id} joint limits {q_min}, {q_max}")
        return q_min, q_max
    
    def forward_kinematics(self, leg_id, q):
        """Calculate foot position from joint angles"""
        frame = kdl.Frame()
        self.fk_solvers[leg_id].JntToCart(q, frame)
        
        position = [
            frame.p.x(), 
            frame.p.y(), 
            frame.p.z()
        ]
        
        # Extract orientation (quaternion)
        rotation = frame.M
        orientation = rotation.GetQuaternion()
        
        return position, orientation
    
    def inverse_kinematics(self, leg_id, target_position, target_orientation=None):
        """Calculate joint angles for desired foot position"""
        # Create target frame
        if target_orientation is None:
            # Default orientation (foot pointing down)
            rot = kdl.Rotation.RPY(0, np.pi/2, 0)
            rot = kdl.Rotation.Identity()
        else:
            rot = kdl.Rotation.Quaternion(*target_orientation)
        
        target_frame = kdl.Frame(
            rot,
            kdl.Vector(*target_position)
        )
        
        # Solve IK
        q_out = kdl.JntArray(3)

        q_init = kdl.JntArray(3)
        q_init[0] = self.q_current[f"{leg_id}_1"]
        q_init[1] = self.q_current[f"{leg_id}_2"]
        q_init[2] = self.q_current[f"{leg_id}_3"]

        # pprint(q_init)
        # pprint(rot.GetRPY())
        # pprint(target_position)
        result = self.ik_solvers[leg_id].CartToJnt(q_init, target_frame, q_out)

        if result >= 0:  # Success
            return [q_out[0], q_out[1], q_out[2]]
        else:
            self.get_logger().warn(f"IK failed for leg {leg_id}")
            return None
    
    def generate_foot_trajectory(self, leg_id, start_pos, end_pos, steps=10):
        """Generate smooth foot trajectory"""
        trajectory = []
        
        for i in range(steps):
            t = i / (steps - 1)
            # Linear interpolation
            current_pos = [
                start_pos[0] + (end_pos[0] - start_pos[0]) * t,
                start_pos[1] + (end_pos[1] - start_pos[1]) * t,
                start_pos[2] + (end_pos[2] - start_pos[2]) * t
            ]
            
            # Add parabolic lift for swing phase
            if t < 0.5:  # First half: lift
                current_pos[2] += 0.05 * np.sin(t * np.pi)
            else:  # Second half: lower
                current_pos[2] += 0.05 * np.sin(t * np.pi)
            
            trajectory.append(current_pos)
        
        return trajectory
    
    def initialize_joint_positions(self):
        """Initialize all joint positions to a safe standing position"""
        joint_positions = {}
        
        # Hexapod standing position configuration
        # Adjust these based on your robot's geometry
        default_angles = {
            1: 0,  # Legs pointing straight out
            2: 0,    # Slightly bent up
            3: 0     # Leg extended downward
        }
        
        # Initialize all 18 joints
        for leg_id in range(1, 7):
            for joint_id in range(1, 4):
                joint_name = f"{leg_id}_{joint_id}"
                
                joint_positions[joint_name] = default_angles[joint_id]

        return joint_positions    
    

def main(args=None):
    # rclpy.init()
    # node = HexapodKDLController()
    
    # Spin for 30 seconds max
    # from rclpy.executors import SingleThreadedExecutor
    # executor = SingleThreadedExecutor()
    # executor.add_node(node)
    
    # try:
    #     # executor.spin_once(timeout_sec=130.0)

    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

    rclpy.init(args=args)
    
    ik_solver = HexapodKDLController()
    
    try:
        position, orientation = ik_solver.forward_kinematics(1, kdl.JntArray(3))
        print(position, orientation)
    
        new_pos = kdl.JntArray(3)
        new_pos[0] = 0.1
        new_pos[1] = 0.1
        new_pos[2] = 0.1
        print(ik_solver.forward_kinematics(1, new_pos))
        
        print(ik_solver.inverse_kinematics(1, position, orientation))
        print(ik_solver.inverse_kinematics(1, position))
        print(ik_solver.inverse_kinematics(1, [0.5, 1.79, 0.619], orientation))

        # while True:
        #     time.sleep(1)
    except KeyboardInterrupt:
        ik_solver.get_logger().info('Shutting down...')
    finally:
        ik_solver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()