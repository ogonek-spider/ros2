import math 
import numpy as np

class HexapodKinematicsSolver:
    """
    converts joint angles to foot position and vise versa

    # joint angles

    joint_angles[0] = 1-1 (coxa)
    joint_angles[1] = 1-2 (femur)
    joint_angles[2] = 1-3 (tibia)

    # xyz

    x = axis parralel to body (forward step)
    y = axis perpendicular to body (move closer father away from body)
    z = top down

    if leg is mounted 45 degree to body leg coordinates will be converted to body frame

    ```x += step``` always will result in a step forward

    # legn = 1..6
    
    leg number from 0, legs_mount_angles starts from 1
    """

    
    NEUTRAL = (0, 1, 1.7)
    LEGSUP = (0, 0, 0)

    @classmethod
    def create_default(cls):
        #fixme parse urdf
        L1 = 0.25
        L2 = 0.718
        L3 = 1.71
        #legs_mount_angles = [None, -math.pi/4, -math.pi/2, - 3/4 * math.pi, math.pi/4, math.pi/2, 3/4 * math.pi]
        legs_mount_angles = [None, 
                             -math.pi/4,  #1
                             0,     #2
                             math.pi/4,   #3
                             math.pi/4 + math.pi,   #4
                             -math.pi,    #5
                             3/4 * math.pi#6
        ]
        joints_offset_angles = [math.pi/2, -math.pi/2, 0]

        return cls(L1, L2, L3, legs_mount_angles, joints_offset_angles)

    def __init__(self, L1, L2, L3, legs_mount_angles, joint_offset_angles):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L_coxa = L1
        self.L_femur = L2
        self.L_tibia = L3
        self.legs_mount_angles = legs_mount_angles
        self.joint_offset_angles = joint_offset_angles
    
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
        
    def forward(self, legn, joint_angles):
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
        angles = list(joint_angles)
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
        x_leg, y_leg, z = P_foot_leg
        
        ma = self.legs_mount_angles[legn]
        x = x_leg * math.cos(ma) - y_leg * math.sin(ma)
        y = x_leg * math.sin(ma) + y_leg * math.cos(ma)
        return [x, y, z]
    
        
    def inverse(self, legn, point):
        """
        Solve IK for target position in body frame.
        
        Args:
            x, y, z: Target foot position in body frame (meters)

        y = axis parralel to body (forward step)
        x = axis perpendicular to body (move closer father away from body)
        z = top down
            
        """
        x, y, z = point
        # 1. Transform to leg plane coordinates
        ma = self.legs_mount_angles[legn]
        x_leg = x * math.cos(ma) + y * math.sin(ma)
        y_leg = -x * math.sin(ma) + y * math.cos(ma)
        z_leg = z

        #print(f"body2leg {point} -> {x_leg:.2f}, {y_leg:.2f}, {z_leg:.2f}")

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
        
        #print(f"x_proj {X_proj:.2f} z_proj {Z_proj:.2f} {D}")
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

        #print(f"before offset {theta_coxa:.2f}, {theta_femur:.2f}, {theta_tibia:.2f}")
        angles = (theta_coxa - self.joint_offset_angles[0], theta_femur - self.joint_offset_angles[1], theta_tibia - self.joint_offset_angles[2])
        #print(f"x {point[0]:.3f} y {point[1]:.3f} z {point[2]:.3f} -> ({angles[0]:.3f}, {angles[1]:.3f}, {angles[2]:.3f})")
        return angles

    
