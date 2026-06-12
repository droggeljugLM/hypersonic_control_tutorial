import math
import unittest

from hgv_control.models.control_allocation import (
    AllocationParams,
    AllocatorState,
    MomentDemand,
    allocate_moment,
    apply_limits,
    solve_3x3,
    state_to_tuple,
)
from hgv_control.simulation.control_allocation import run


class ControlAllocationTests(unittest.TestCase):
    def test_solve_3x3_solves_small_system(self):
        solution = solve_3x3(((3.0, 1.0, 0.0), (1.0, 4.0, 1.0), (0.0, 1.0, 2.0)), (1.0, 2.0, 3.0))
        self.assertAlmostEqual(3.0 * solution[0] + solution[1], 1.0)
        self.assertAlmostEqual(solution[0] + 4.0 * solution[1] + solution[2], 2.0)
        self.assertAlmostEqual(solution[1] + 2.0 * solution[2], 3.0)

    def test_allocation_tracks_modest_moment(self):
        params = AllocationParams(deflection_rate_limit=math.radians(1000.0))
        result = allocate_moment(MomentDemand(1_000.0, -700.0, 800.0), AllocatorState(), 0.1, params)
        self.assertLess(result.residual_norm, 80.0)
        self.assertLessEqual(max(abs(value) for value in state_to_tuple(result.actual)), params.deflection_limit)

    def test_rate_limit_is_enforced(self):
        params = AllocationParams(deflection_rate_limit=0.5)
        target = AllocatorState(0.2, -0.2, 0.1, -0.1)
        actual, _saturated, rate_limited = apply_limits(AllocatorState(), target, 0.1, params)
        self.assertTrue(rate_limited)
        self.assertLessEqual(max(abs(value) for value in state_to_tuple(actual)), 0.05 + 1e-12)

    def test_efficiency_loss_increases_yaw_residual(self):
        healthy = AllocationParams(deflection_rate_limit=math.radians(1000.0))
        degraded = AllocationParams(efficiency=(1.0, 1.0, 0.2, 1.0), deflection_rate_limit=math.radians(1000.0))
        demand = MomentDemand(0.0, 0.0, 4_000.0)
        healthy_result = allocate_moment(demand, AllocatorState(), 0.1, healthy)
        degraded_result = allocate_moment(demand, AllocatorState(), 0.1, degraded)
        self.assertGreater(degraded_result.residual_norm, healthy_result.residual_norm)

    def test_allocation_run_reports_passing_metrics(self):
        trace, metrics = run(duration=8.0, dt=0.04)
        self.assertGreater(len(trace), 100)
        self.assertIn("allocation_residual_rms_nm", metrics)
        self.assertTrue(metrics["deflection_pass"])
        self.assertTrue(metrics["rate_pass"])


if __name__ == "__main__":
    unittest.main()
