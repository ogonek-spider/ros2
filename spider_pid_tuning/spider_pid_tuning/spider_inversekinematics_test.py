import rclpy
from rclpy.node import Node
import numpy as np
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF


class HexapodKDLController(Node):
    def __init__(self):
        super().__init__('hexapod_kdl_controller')
        
        # Load URDF and create KDL tree
        self.robot_urdf = URDF.from_parameter_server()
        self.kdl_tree = kdl.Tree()
        if not treeFromUrdfModel(self.robot_urdf, self.kdl_tree):
            self.get_logger().error("Failed to extract KDL tree from URDF")
            return
        
        # Create chains and solvers for all legs
        self.chains = {}
        self.fk_solvers = {}
        self.ik_solvers = {}
        self.joint_limits = {}
        
        for leg_id in range(1, 7):
            self.setup_leg_solver(leg_id)
        
        # Current joint positions
        self.q_current = self.initialize_joint_positions()
        
    def setup_leg_solver(self, leg_id):
        """Setup KDL solvers for a single leg"""
        # Get chain from base to foot
        chain = kdl.Chain()
        if not self.kdl_tree.getChain("base_link", f"leg{leg_id}_foot", chain):
            self.get_logger().error(f"Failed to get chain for leg {leg_id}")
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
    
    def get_joint_limits(self, leg_id):
        """Extract joint limits from URDF"""
        joint_names = [
            f"leg{leg_id}_shoulder",
            f"leg{leg_id}_femur",
            f"leg{leg_id}_tibia"
        ]
        
        q_min = kdl.JntArray(3)
        q_max = kdl.JntArray(3)
        
        for i, joint_name in enumerate(joint_names):
            joint = self.robot_urdf.joint_map[joint_name]
            if joint.limit:
                q_min[i] = joint.limit.lower
                q_max[i] = joint.limit.upper
            else:
                q_min[i] = -np.pi
                q_max[i] = np.pi
        
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
        else:
            rot = kdl.Rotation.Quaternion(*target_orientation)
        
        target_frame = kdl.Frame(
            rot,
            kdl.Vector(*target_position)
        )
        
        # Solve IK
        q_out = kdl.JntArray(3)
        q_init = kdl.JntArray(3)
        q_init[0] = self.q_current[f"leg{leg_id}_shoulder"]
        q_init[1] = self.q_current[f"leg{leg_id}_femur"]
        q_init[2] = self.q_current[f"leg{leg_id}_tibia"]
        
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
    

def main(args=None):
    rclpy.init(args=args)
    
    ik_solver = HexapodKDLController()
    
    try:
        #spider_walker.return_to_neutral()
        ik_solver.get_logger().info(ik_solver.inverse_kinematics(1, [0, 0, 0]))

        
    except KeyboardInterrupt:
        ik_solver.get_logger().info('Shutting down...')
    finally:
        ik_solver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()