import pytest
from spider_walker.kinematics_solver import HexapodKinematicsSolver

def assert_vectors_equal(v1, v2):    
    assert v1[0] == pytest.approx(v2[0])
    assert v1[1] == pytest.approx(v2[1])
    assert v1[2] == pytest.approx(v2[2])

class TestHexapodKinematicsSolver:
    def test_inverse_is_forward(self):
        self.solver = HexapodKinematicsSolver.create_default()

        for legn in range(1,7):
            xyz = self.solver.forward(legn, HexapodKinematicsSolver.NEUTRAL)
            assert_vectors_equal(HexapodKinematicsSolver.NEUTRAL, self.solver.inverse(legn, xyz))

            xyz = self.solver.forward(legn, HexapodKinematicsSolver.LEGSUP)
            assert_vectors_equal(HexapodKinematicsSolver.LEGSUP, self.solver.inverse(legn, xyz))


    def test_step_straight_leg(self):
        self.solver = HexapodKinematicsSolver.create_default()

        xyz = self.solver.forward(2, HexapodKinematicsSolver.NEUTRAL)
        xyz[1] += 0.3
        angles = self.solver.inverse(2, xyz)
        v1 = HexapodKinematicsSolver.NEUTRAL
        v2 = angles
        assert v1[0] != pytest.approx(v2[0]) #just coxa joint should move (dont true, another joint move too!)
        assert v1[1] == pytest.approx(v2[1], 0.1)
        assert v1[2] == pytest.approx(v2[2], 0.1)

    def test_step_diagonal_leg(self):
        self.solver = HexapodKinematicsSolver.create_default()

        xyz = self.solver.forward(1, HexapodKinematicsSolver.NEUTRAL)
        xyz[1] += 0.3
        angles = self.solver.inverse(1, xyz)
        v1 = HexapodKinematicsSolver.NEUTRAL
        v2 = angles
        assert v1[0] != pytest.approx(v2[0], 0.1) #all joints should move
        assert v1[1] != pytest.approx(v2[1], 0.1)
        assert v1[2] != pytest.approx(v2[2], 0.1)

    def test_step_mirror_legs(self):
        self.solver = HexapodKinematicsSolver.create_default()

        xyz1 = self.solver.forward(2, HexapodKinematicsSolver.NEUTRAL)
        xyz1[1] += 0.3
        xyz2 = self.solver.forward(5, HexapodKinematicsSolver.NEUTRAL)
        xyz2[1] += 0.3
        v1 = self.solver.inverse(2, xyz1)
        v2 = self.solver.inverse(5, xyz2)

        assert -v1[0] == pytest.approx(v2[0]) #coxa should move in opposite direction
        assert v1[1] == pytest.approx(v2[1], 0.1)
        assert v1[2] == pytest.approx(v2[2], 0.1)



