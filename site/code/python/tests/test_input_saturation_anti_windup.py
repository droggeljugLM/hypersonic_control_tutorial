import unittest

from hgv_control.simulation.input_saturation_anti_windup import run


class InputSaturationAntiWindupTests(unittest.TestCase):
    def test_anti_windup_reduces_integrator_peak_and_recovery_error(self):
        _, no_aw = run("no_anti_windup", duration=32.0, dt=0.05)
        _, aw = run("anti_windup", duration=32.0, dt=0.05)

        self.assertGreater(no_aw["speed_integral_abs_max"], aw["speed_integral_abs_max"])
        self.assertGreater(no_aw["post_release_v_rms"], aw["post_release_v_rms"])
        self.assertGreater(no_aw["throttle_excess_integral"], aw["throttle_excess_integral"])
        self.assertFalse(no_aw["tracking_pass"])
        self.assertTrue(aw["tracking_pass"])

    def test_trace_records_saturation_and_unsaturated_command(self):
        trace, metrics = run("anti_windup", duration=4.0, dt=0.05)

        self.assertGreater(len(trace), 10)
        self.assertIn("throttle_unsat", trace[0])
        self.assertIn("speed_integral", trace[0])
        self.assertGreater(metrics["saturation_fraction"], 0.0)


if __name__ == "__main__":
    unittest.main()
