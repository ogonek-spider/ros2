import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TripodTrajectoryGenerator:
    """
    Builds JointTrajectory messages for hexapod tripod gait.
    Pure data — no ROS node, no logging, no action clients.
    """

    TRIPOD1 = [1, 3, 5]
    TRIPOD2 = [2, 4, 6]    

    def __init__(self, kinematics, joint_names, step_height=0.5, step_length=0.4):
        self.kinematics = kinematics
        self.joint_names = joint_names
        self.step_height = step_height
        self.step_length = step_length
        self.neutral_positions = list(kinematics.NEUTRAL) * 6

    # ------------------------------------------------------------------
    # Public builders
    # ------------------------------------------------------------------

    def make_startup(self, step_angle_deg: float, duration_sec: float) -> JointTrajectory:
        """Slide legs into tripod start positions without lifting."""
        angle_rad = math.radians(step_angle_deg)
        positions = []

        for leg_idx in range(1, 7):
            xyz = self.kinematics.forward(leg_idx, self.kinematics.NEUTRAL)
            offset = -self.step_length * 0.5 if leg_idx in self.TRIPOD1 else self.step_length * 0.5
            xyz[0] += offset * math.cos(angle_rad)
            xyz[1] += offset * math.sin(angle_rad)
            positions.extend(self.kinematics.inverse(leg_idx, xyz))

        point = self._make_point(positions, duration_sec)
        return self._make_msg([point])

    def make_walk_cycle(self, step_angle_deg: float, step_duration_sec: float,
                        num_points_per_phase: int = 5) -> JointTrajectory:
        """One full tripod gait cycle (phase 0 + phase 1)."""
        half = step_duration_sec / 2.0
        gap = step_duration_sec / (num_points_per_phase - 1)
        points = []

        for i in range(num_points_per_phase):
            progress = i / (num_points_per_phase - 1)
            t = progress * half
            points.append(self._walking_point(0, progress, step_angle_deg, t))

        for i in range(num_points_per_phase):
            progress = i / (num_points_per_phase - 1)
            t = half + gap + progress * half
            points.append(self._walking_point(1, progress, step_angle_deg, t))

        return self._make_msg(points)

    def make_return_to_neutral(self, duration_sec: float = 2.0) -> JointTrajectory:
        point = self._make_point(self.neutral_positions, duration_sec)
        return self._make_msg([point])

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _walking_point(self, phase: int, progress: float,
                       step_angle_deg: float, time_sec: float) -> JointTrajectoryPoint:
        angle_rad = math.radians(step_angle_deg)
        positions = []

        for leg_idx in range(1, 7):
            is_swing = (leg_idx in self.TRIPOD1 and phase == 0) or \
                       (leg_idx in self.TRIPOD2 and phase == 1)

            xyz = self.kinematics.forward(leg_idx, self.kinematics.NEUTRAL)

            if is_swing:
                lift = self.step_height * (progress * 2 if progress < 0.5 else 2 - progress * 2)
                dist = self.step_length * (progress - 0.5)
            else:
                lift = 0.0
                dist = self.step_length * (0.5 - progress)

            xyz[0] += dist * math.cos(angle_rad)
            xyz[1] += dist * math.sin(angle_rad)
            xyz[2] += lift
            positions.extend(self.kinematics.inverse(leg_idx, xyz))

        return self._make_point(positions, time_sec)

    def _make_point(self, positions: list, time_sec: float) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        point.time_from_start = Duration(
            sec=int(time_sec),
            nanosec=int((time_sec % 1) * 1e9)
        )
        return point

    def _make_msg(self, points: list) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = points
        return msg
