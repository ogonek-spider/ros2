import rclpy
import math
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
import PyKDL as kdl
import time
from urdf_parser_py.urdf import URDF
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


from pprint import pprint


LEG_CHAIN_ROOT = "body_plate_deepseek"
LEG_CHAIN_TIP = lambda leg_id: '3limb' if leg_id == 1 else f'3limb_{leg_id}'

class HexapodLegKinematics():
    @classmethod
    def create_from_urdf(self, urdf, leg_id):
        params = self.extract_kinematics()

        L1, L2, L3 = params['link_lengths']
        mount_angle = self.params['mount_transform']['rpy'][2]
        return HexapodLegKinematics(leg_id, L1, L2, L3, mount_angle)

    @classmethod
    def extract_kinematics(urdf):
        joint_by_child = {j.child: j for j in self.robot_urdf.joints}

        # Walk from tip to base
        chain = []
        current = LEG_CHAIN_TIP(self.leg_id)
        
        while current != LEG_CHAIN_ROOT:
            if current not in joint_by_child:
                raise Exception(f"❌ Error: No joint found for child link '{current}'")
            
            joint = joint_by_child[current]
            chain.insert(0, (joint, current))
            current = joint.parent
        
        # for joint, child in chain:
        #     print(f"  {joint.parent} -> [{joint.name}] -> {child}")
        #     print(f"    Type: {joint.type}")
        #     if joint.origin:
        #         print(f"    Origin xyz: {joint.origin.xyz}")
        #         print(f"    Origin rpy: {joint.origin.rpy}")
        #     # if joint.axis:
        #     #     print(f"    Axis: {joint.axis[0].xyz}")

        # Extract parameters
        params = {
            'link_lengths': [],
            'mount_transform': None
        }
        
        # First joint's origin gives mount transform
        if chain and chain[0][0].origin:
            params['mount_transform'] = {
                'xyz': chain[0][0].origin.xyz,
                'rpy': chain[0][0].origin.rpy
            }
        
        # Calculate distances between consecutive joint origins
        prev_xyz = None
        for joint, child in chain:
            if joint.origin:
                if prev_xyz is not None:
                    # Calculate Euclidean distance
                    dist = np.linalg.norm(np.array(joint.origin.xyz))# - np.array(prev_xyz))
                    params['link_lengths'].append(dist)
                    print(f"📏 Distance to {joint.name}: {dist:.4f}m")
                prev_xyz = joint.origin.xyz

        chain = self.robot_urdf.get_chain(self.robot_urdf.joint_map["1-3"], LEG_CHAIN_TIP(self.leg_id))
        pprint(chain)
        tip = self.robot_urdf.link_map[LEG_CHAIN_TIP(self.leg_id)]
        current_node = tip
        dist = 0
        while current_node != self.robot_urdf.joint_map["1-3"]:
            dist += np.linalg.norm(np.array(current_node.visual.origin.xyz))# - np.array(prev_xyz))
            current_node = current_node.parent
        
        params['link_lengths'].append(dist)
        print(f"📏 Distance to {tip.name}: {dist:.4f}m")

        return params

    def __init__(self, leg_id, L1, L2, L3, mount_angle, joints_offset_angles):
        self.leg_id = leg_id
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.mount_angle = mount_angle
        self.L_coxa = self.L1
        self.L_femur = self.L2
        self.L_tibia = self.L3
        self.joint_offset_angles = joints_offset_angles

        #fixme get from urdf
        self.joint_limits = {
            (-math.pi/2, math.pi/2),    # ±90°
            (-math.pi/4, math.pi/2),   # -45° to 90°
            (-math.pi/2, 0)            # -90° to 0° (bent knee)
        }        

    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp value between min and max."""
        return max(min_val, min(value, max_val))

    def rotation_x(self, angle):
        """Rotation matrix around X-axis"""
        c, s = math.cos(angle), math.sin(angle)
        return np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])
    
    def rotation_y(self, angle):
        """Rotation matrix around Y-axis"""
        c, s = math.cos(angle), math.sin(angle)
        return np.array([
            [c, 0, s],
            [0, 1, 0],
            [-s, 0, c]
        ])
    
    def rotation_z(self, angle):
        """Rotation matrix around Z-axis"""
        c, s = math.cos(angle), math.sin(angle)
        return np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])
        
    def forward(self, angles):
        """
        Compute foot position relatice to coxa joint from joint angles.
        
        Args:
            theta_coxa: Coxa joint angle (radians)
            theta_femur: Femur joint angle (radians)
            theta_tibia: Tibia joint angle (radians)
            
        Returns:
            foot_position: [x, y, z] position of foot
            joint_positions: Dictionary of all joint positions for visualization
        """
        angles = list(angles)
        for n in range(len(angles)):
            angles[n] += self.joint_offset_angles[n]
        
        theta_coxa, theta_femur, theta_tibia = angles
    
        # 1. Start at the coxa joint (origin of leg coordinate system)
        # Position of coxa joint relative to leg base (usually at origin in leg frame)
        P_coxa = np.array([0.0, 0.0, 0.0])
        
        # 2. Femur joint position (after coxa rotation and coxa link translation)
        # Coxa rotates around Z-axis in leg plane
        R_coxa = self.rotation_z(theta_coxa)
        
        # Femur joint is at end of coxa link, along local X-axis after rotation
        P_femur_leg = P_coxa + R_coxa @ np.array([self.L_coxa, 0.0, 0.0])
        
        # 3. Tibia joint position (after femur rotation)
        # Femur rotates around local Y-axis (perpendicular to leg plane)
        # We need to apply rotations in the correct order
        
        # First, get the rotation matrix for femur rotation
        # Since femur rotates in the leg plane after coxa rotation,
        # we need to combine rotations: R_coxa then R_femur
        R_femur = self.rotation_y(theta_femur)
        R_coxa_femur = R_coxa @ R_femur
        
        # Tibia joint is at end of femur link
        # In the coxa-femur frame, femur link is along X-axis
        P_tibia_leg = P_femur_leg + R_coxa_femur @ np.array([self.L_femur, 0.0, 0.0])
        
        # 4. Foot position (after tibia rotation)
        # Tibia also rotates around local Y-axis
        R_tibia = self.rotation_y(theta_tibia)
        R_total = R_coxa_femur @ R_tibia
        
        # Foot is at end of tibia link
        P_foot_leg = P_tibia_leg + R_total @ np.array([self.L_tibia, 0.0, 0.0])
        
        # Store all joint positions for visualization
        joint_positions = {
            'coxa': P_coxa,
            'femur': P_femur_leg,
            'tibia': P_tibia_leg,
            'foot': P_foot_leg
        }

        # Transform from leg plane coordinates
        # x = P_foot_leg[0]
        # x = P_foot_leg[0]
        # x_leg = x * math.cos(self.mount_angle) + y * math.sin(self.mount_angle)
        # y_leg = -x * math.sin(self.mount_angle) + y * math.cos(self.mount_angle)

        return P_foot_leg, joint_positions
        
    def inverse(self, point):
        """
        Solve IK for target position in body frame.
        
        Args:
            x, y, z: Target foot position in body frame (meters)

        x = axis parralel to body (forward step)
        y = axis perpendicular to body (move closer father away from body)
        z = top down
            
        """
        x, y, z = point
        # 1. Transform to leg plane coordinates
        x_leg = x * math.cos(self.mount_angle) + y * math.sin(self.mount_angle)
        y_leg = -x * math.sin(self.mount_angle) + y * math.cos(self.mount_angle)
        z_leg = z

        print(f"body2leg {point} -> {x_leg:.2f}, {y_leg:.2f}, {z_leg:.2f}")

        # 2. Calculate coxa angle
        theta_coxa = math.atan2(y_leg, x_leg)

        # 3. Project to femur-tibia plane
        X_proj = math.sqrt(x_leg*x_leg + y_leg*y_leg) - self.L_coxa
        Z_proj = -z_leg  # Negative for downward leg (adjust if your Z is up)

        # 4. 2D geometric solve for femur & tibia
        D_sq = X_proj*X_proj + Z_proj*Z_proj
        D = math.sqrt(D_sq)

        # Check reachability
        max_reach = self.L_femur + self.L_tibia
        min_reach = abs(self.L_femur - self.L_tibia)
        
        print(f"x_proj {X_proj:.2f} z_proj {Z_proj:.2f} {D}")
        if D > max_reach or D < min_reach:
            self.logger.error(f"D not in reach. {min_reach} > {D} < {max_reach}")
            return None

        # Law of cosines for tibia
        cos_tibia = (D_sq - self.L_femur*self.L_femur - self.L_tibia*self.L_tibia) / (2.0 * self.L_femur * self.L_tibia)
        cos_tibia = self._clamp(cos_tibia, -1.0, 1.0)
        # theta_tibia = math.pi - math.acos(cos_tibia)  # Bent knee configuration
        theta_tibia = math.acos(cos_tibia)

        # Solve for femur
        gamma = math.atan2(Z_proj, X_proj)
        cos_femur = (D_sq + self.L_femur*self.L_femur - self.L_tibia*self.L_tibia) / (2.0 * self.L_femur * D)
        cos_femur = self._clamp(cos_femur, -1.0, 1.0)
        beta = math.acos(cos_femur)

        theta_femur = gamma - beta
        
        # Adjust sign if needed (some robots have different joint direction)
        # Uncomment if your femur/tibia rotate opposite direction:
        # theta_femur = -theta_femur
        # theta_tibia = -theta_tibia

        print(f"before offset {theta_coxa:.2f}, {theta_femur:.2f}, {theta_tibia:.2f}")
        return theta_coxa - self.joint_offset_angles[0], theta_femur - self.joint_offset_angles[1], theta_tibia - self.joint_offset_angles[2]
                
    
class HexapodKinematics(Node):
    def __init__(self, L1=None, L2=None, L3=None, legs_mount_angles=None, joint_offset_angles=None):
        super().__init__('hexapod_kdl_controller')

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/spider_controller/joint_trajectory',
            10
        )

        self.joint_names = []
        for leg in range(1, 7):
            for joint in range(1, 4):
                self.joint_names.append(f'{leg}-{joint}')

        self.neutral_positions = self.calculate_neutral_positions()

        if L1 is None:
            self.declare_parameter('robot_description', 'string')
            urdf_string = self.get_parameter('robot_description').get_parameter_value().string_value
            
            if urdf_string:
                self.robot_urdf = URDF.from_xml_string(urdf_string)
                self.get_logger().info(f'Successfully loaded URDF from parameter, joints count {len(self.robot_urdf.joints)}')

                self.leg_kinematics = {}
                for legn in range(1, 7):
                    self.leg_kinematics[legn] = HexapodLegKinematics(self.robot_urdf, legn)
            else:
                raise Exception("No URDF found in params")
        else:
            self.L1 = L1
            self.L2 = L2
            self.L3 = L3
            self.legs_mount_angles = legs_mount_angles
            self.leg_kinematics = {}
            for legn in range(1, 7):
                self.leg_kinematics[legn] = HexapodLegKinematics(legn, L1, L2, L3, legs_mount_angles[legn - 1], joint_offset_angles)
                self.leg_kinematics[legn].logger = self.get_logger()


    def extract_kinematics_from_model(self):
        self.leg_params = {}
        self.leg_params[1] = self.extract_kinematics_for_legn(1)
        return True

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

    def test_joint(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.neutral_positions

        for legn in range(6):
            point.positions[legn*3 + 1] = 1
            point.positions[legn*3 + 2] = 1.7
        point.time_from_start = Duration(sec=5, nanosec=0)

        trajectory_msg.points.append(point)
        self.trajectory_pub.publish(trajectory_msg)
        
        time.sleep(5+2)

    def calculate_neutral_positions(self):
        """Calculate initial neutral positions for all joints"""
        positions = []
        for leg_idx in range(6):
            # Base joint (horizontal rotation)
            positions.append(0)  # joint1
            
            # Shoulder joint (vertical rotation)
            positions.append(1)  # joint2
            
            # Knee joint
            positions.append(1.7)  # joint3 - slightly bent
        return positions

def main(args=None):
    rclpy.init(args=args)
    
    #fixme parse urdf
    L1 = 0.25
    L2 = 0.718
    L3 = 1.71
    legs_mount_angles = [-math.pi/4, 0, 0, 0, 0, 0]

    joints_offset_angles = [math.pi/2, -math.pi/2, 0]

    ik_solver = HexapodKinematics(L1, L2, L3, legs_mount_angles, joints_offset_angles)
    
    def print_forward_kinematics(point):
        xyz, debug = ik_solver.leg_kinematics[2].forward(point)
        print(f"({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) -> x {xyz[0]:.3f} y {xyz[1]:.3f} z {xyz[2]:.3f}")
        return xyz

    def print_inverse_kinematics(xyz):
        angles = ik_solver.leg_kinematics[2].inverse(xyz)
        print(f"x {xyz[0]:.3f} y {xyz[1]:.3f} z {xyz[2]:.3f} -> ({angles[0]:.3f}, {angles[1]:.3f}, {angles[2]:.3f})")
        return angles

    try:        
        LEGS_UP = (0, 0, 0)
        NEUTRAL = (0, 1, 1.7)
        #ik_solver.get_logger().info(r"{}".format(ik_solver.leg_kinematics[1].inverse(point)))
        
        
        #pprint(ik_solver.leg_kinematics[2].forward((0, 1, 1.7)))
        xyz = print_forward_kinematics(LEGS_UP)
        print_inverse_kinematics(xyz)
        
        xyz = print_forward_kinematics(NEUTRAL)
        print_inverse_kinematics(xyz)

        step_forward = xyz
        step_forward[0] += 0.3
        step_forward_joints = print_inverse_kinematics(step_forward)
        print_forward_kinematics(step_forward_joints)
        

        # xyz, debug = ik_solver.leg_kinematics[2].forward(NEUTRAL)
        # pprint(ik_solver.leg_kinematics[2].inverse(xyz))

        # while True:
        #     # ik_solver.return_to_neutral()
        #     # ik_solver.test_joint()
        #     time.sleep(1)
    except KeyboardInterrupt:
        ik_solver.get_logger().info('Shutting down...')
        ik_solver.return_to_neutral()
    finally:
        ik_solver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()