import tempfile
from pathlib import Path
import unittest

from hgv_control.simulation.igc_step_summary import build_summary, write_gap_markdown, write_summary_csv


class IgcStepSummaryTests(unittest.TestCase):
    def test_summary_contains_teaching_steps_and_explicit_gaps(self):
        rows = build_summary()
        modules = {row["module"] for row in rows}
        self.assertIn("guidance_3d", modules)
        self.assertIn("attitude_inner_loop", modules)
        self.assertIn("control_allocation", modules)
        self.assertIn("guidance_attitude_interface", modules)
        self.assertIn("coupled_six_dof_skeleton", modules)
        self.assertIn("aero_table_demo", modules)
        self.assertIn("six_dof_rigid_body_demo", modules)
        self.assertTrue(any(row["metric"] == "terminal_range_m" and row["step"] == "Step 5" for row in rows))
        self.assertTrue(any(row["metric"] == "clift_alpha_slope_per_rad" and row["module"] == "aero_table_demo" for row in rows))
        self.assertTrue(any(row["metric"] == "body_rate_max_rad_s" and row["module"] == "six_dof_rigid_body_demo" for row in rows))
        gap_rows = [row for row in rows if row["step"] == "Step 5 gap"]
        self.assertGreaterEqual(len(gap_rows), 3)
        self.assertTrue(any(row["metric"] == "formal_igc_claim" for row in gap_rows))

    def test_writers_create_csv_and_gap_markdown(self):
        rows = build_summary()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            csv_path = root / "summary.csv"
            md_path = root / "gaps.md"
            write_summary_csv(rows, csv_path)
            write_gap_markdown(rows, md_path)
            self.assertIn("step,module,metric", csv_path.read_text(encoding="utf-8"))
            markdown = md_path.read_text(encoding="utf-8")
            self.assertIn("IGC 教学扩展指标汇总", markdown)
            self.assertIn("coupled_six_dof_skeleton", markdown)
            self.assertIn("aero_table_demo", markdown)
            self.assertIn("six_dof_rigid_body_demo", markdown)
            self.assertIn("formal_igc_claim", markdown)


if __name__ == "__main__":
    unittest.main()
