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

    def __init__(self, leg_id, L1, L2, L3, mount_angle):
        self.leg_id = leg_id
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.mount_angle = mount_angle

        #fixme get from urdf
        self.joint_limits = {
            (-math.pi/2, math.pi/2),    # ±90°
            (-math.pi/4, math.pi/2),   # -45° to 90°
            (-math.pi/2, 0)            # -90° to 0° (bent knee)
        }        

    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp value between min and max."""
        return max(min_val, min(value, max_val))
    
    def inverse(self, point):
        """
        Solve IK for target position in body frame.
        
        Args:
            x, y, z: Target foot position in body frame (meters)
            
        Returns:
            tuple: (success, theta_coxa, theta_femur, theta_tibia)
                   success: True if IK solution found
                   angles in radians
        """
        x, y, z = point
        self.L_coxa = self.L1
        self.L_femur = self.L2
        self.L_tibia = self.L3
        # 1. Transform to leg plane coordinates
        x_leg = x * math.cos(self.mount_angle) + y * math.sin(self.mount_angle)
        y_leg = -x * math.sin(self.mount_angle) + y * math.cos(self.mount_angle)
        z_leg = z

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
        
        print(D)
        if D > max_reach or D < min_reach:
            self.logger.error(f"D not in reach. {min_reach} > {D} < {max_reach}")
            return None

        # Law of cosines for tibia
        cos_tibia = (D_sq - self.L_femur*self.L_femur - self.L_tibia*self.L_tibia) / (2.0 * self.L_femur * self.L_tibia)
        cos_tibia = self._clamp(cos_tibia, -1.0, 1.0)
        theta_tibia = math.pi - math.acos(cos_tibia)  # Bent knee configuration

        # Solve for femur
        gamma = math.atan2(Z_proj, X_proj)
        cos_femur = (D_sq + self.L_femur*self.L_femur - self.L_tibia*self.L_tibia) / (2.0 * self.L_femur * D)
        cos_femur = self._clamp(cos_femur, -1.0, 1.0)
        beta = math.acos(cos_femur)

        theta_femur = gamma + beta
        
        # Adjust sign if needed (some robots have different joint direction)
        # Uncomment if your femur/tibia rotate opposite direction:
        # theta_femur = -theta_femur
        # theta_tibia = -theta_tibia

        return theta_coxa, theta_femur, theta_tibia
                
    
class HexapodKinematics(Node):
    def __init__(self, L1=None, L2=None, L3=None, legs_mount_angles=None):
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
                self.leg_kinematics[legn] = HexapodLegKinematics(legn, L1, L2, L3, legs_mount_angles[legn - 1])
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
            positions.append(0.0)  # joint1
            
            # Shoulder joint (vertical rotation)
            positions.append(0)  # joint2
            
            # Knee joint
            positions.append(0)  # joint3 - slightly bent
        return positions

def main(args=None):
    rclpy.init(args=args)
    
    #fixme parse urdf
    L1 = 0.25
    L2 = 0.718
    L3 = 1.71
    legs_mount_angles = [0, 0, 0, 0, 0, 0]

    joints_offset_angles = [math.pi/2, 0, math.pi/2]

    ik_solver = HexapodKinematics(L1, L2, L3, legs_mount_angles)
    
    try:        
        point = [0, 1, -0.7]
        ik_solver.get_logger().info(r"{}".format(ik_solver.leg_kinematics[1].inverse(point)))

        while True:
            ik_solver.return_to_neutral()
            ik_solver.test_joint()
            time.sleep(1)
    except KeyboardInterrupt:
        ik_solver.get_logger().info('Shutting down...')
        ik_solver.return_to_neutral()
    finally:
        ik_solver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()