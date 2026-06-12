import tempfile
from pathlib import Path
import unittest

from hgv_control.plotting.make_fault_figures import generate_fault_figures
from hgv_control.plotting.make_figures import comparison_label
from hgv_control.plotting.make_aero_table_figures import generate_aero_table_figures
from hgv_control.plotting.make_attitude_figures import generate_attitude_figures
from hgv_control.plotting.make_allocation_figures import generate_allocation_figures
from hgv_control.plotting.make_coupled_six_dof_figures import generate_coupled_figures
from hgv_control.plotting.make_guidance_figures import generate_guidance_figures
from hgv_control.plotting.make_interface_figures import generate_interface_figures
from hgv_control.plotting.make_six_dof_rigid_body_figures import generate_six_dof_rigid_body_figures
from hgv_control.plotting.svg import bar_chart, line_plot, read_numeric_csv


class PlottingTests(unittest.TestCase):
    def test_read_numeric_csv_parses_numbers(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "data.csv"
            path.write_text("time,value,label\n0,1.5,a\n", encoding="utf-8")
            rows = read_numeric_csv(path)
            self.assertEqual(rows[0]["time"], 0.0)
            self.assertEqual(rows[0]["value"], 1.5)
            self.assertEqual(rows[0]["label"], "a")

    def test_svg_outputs_are_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            line_path = Path(tmp) / "line.svg"
            bar_path = Path(tmp) / "bar.svg"
            line_plot(line_path, "Line", "x", "y", [("a", [0.0, 1.0], [2.0, 3.0], "#000")])
            bar_chart(bar_path, "Bar", "value", ["s1", "s2"], [1.0, -2.0])
            self.assertIn("<svg", line_path.read_text(encoding="utf-8"))
            self.assertIn("<svg", bar_path.read_text(encoding="utf-8"))

    def test_comparison_label_uses_pair_metadata(self):
        rows = [{"baseline": "baseline", "candidate": "lqr"}]
        self.assertEqual(comparison_label(rows), "lqr - baseline")

    def test_fault_figures_are_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            metrics = root / "fault_metrics.csv"
            pairs = root / "fault_pairs.csv"
            output_dir = root / "figures"
            metrics.write_text(
                "controller,fault_type,fault_detection_delay\n"
                "baseline,alpha_sensor_bias,0.2\n"
                "guarded,alpha_sensor_bias,0.1\n",
                encoding="utf-8",
            )
            pairs.write_text(
                "fault_type,baseline,candidate,delta_post_fault_q_violation_integral,delta_post_fault_h_rms\n"
                "alpha_sensor_bias,baseline,guarded,-10.0,3.0\n",
                encoding="utf-8",
            )

            outputs = generate_fault_figures(metrics, pairs, output_dir, candidate="guarded")

            self.assertEqual(len(outputs), 3)
            for output in outputs:
                self.assertIn("<svg", output.read_text(encoding="utf-8"))

    def test_attitude_figures_are_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            trace = root / "attitude.csv"
            output_dir = root / "figures"
            trace.write_text(
                "time,phi,theta,psi,phi_ref,theta_ref,psi_ref,p,q,r,mx,my,mz,mx_cmd,my_cmd,mz_cmd,mx_rate,my_rate,mz_rate\n"
                "0,0,0,0,0.1,0.0,0.2,0,0,0,0,0,0,10,0,20,0,0,0\n"
                "1,0.05,0,0.1,0.1,0.0,0.2,0.01,0,0.02,5,0,8,10,0,20,5,0,8\n",
                encoding="utf-8",
            )

            outputs = generate_attitude_figures(trace, output_dir)

            self.assertEqual(len(outputs), 3)
            for output in outputs:
                self.assertIn("<svg", output.read_text(encoding="utf-8"))

    def test_guidance_figures_are_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            trace = root / "guidance.csv"
            output_dir = root / "figures"
            trace.write_text(
                "time,east,north,altitude,velocity,gamma,heading,target_east,target_north,target_altitude,"
                "horizontal_range,range_to_target,altitude_error,gamma_rate_cmd,heading_rate_cmd,throttle_cmd,"
                "qbar,load_factor\n"
                "0,0,0,30000,1750,0,0,52000,14000,29000,53000,53010,-1000,0,0,0.5,32000,1.0\n"
                "1,1000,100,29950,1748,0,0.01,52000,14000,29000,52000,52010,-950,0,0,0.5,32100,1.1\n",
                encoding="utf-8",
            )

            outputs = generate_guidance_figures(trace, output_dir)

            self.assertEqual(len(outputs), 4)
            for output in outputs:
                self.assertIn("<svg", output.read_text(encoding="utf-8"))

    def test_allocation_figures_are_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            trace = root / "allocation.csv"
            output_dir = root / "figures"
            trace.write_text(
                "time,mx_cmd,my_cmd,mz_cmd,mx_achieved,my_achieved,mz_achieved,"
                "mx_residual,my_residual,mz_residual,residual_norm,left_elevon,right_elevon,rudder,body_flap,"
                "left_elevon_target,right_elevon_target,rudder_target,body_flap_target,"
                "left_elevon_rate,right_elevon_rate,rudder_rate,body_flap_rate,saturated,rate_limited\n"
                "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n"
                "1,100,50,80,95,45,75,5,5,5,8.7,0.01,-0.01,0.02,0.0,0.01,-0.01,0.02,0.0,0.01,-0.01,0.02,0.0,0,0\n",
                encoding="utf-8",
            )

            outputs = generate_allocation_figures(trace, output_dir)

            self.assertEqual(len(outputs), 3)
            for output in outputs:
                self.assertIn("<svg", output.read_text(encoding="utf-8"))

    def test_interface_figures_are_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            trace = root / "interface.csv"
            output_dir = root / "figures"
            trace.write_text(
                "time,gamma_error,heading_error,roll_ref,roll,pitch_ref,pitch,yaw_ref,yaw,allocation_residual_norm\n"
                "0,0.1,0.2,0.0,0.0,0.01,0.01,0.02,0.02,10\n"
                "1,0.08,0.15,0.05,0.04,0.02,0.018,0.03,0.028,12\n",
                encoding="utf-8",
            )

            outputs = generate_interface_figures(trace, output_dir)

            self.assertEqual(len(outputs), 3)
            for output in outputs:
                self.assertIn("<svg", output.read_text(encoding="utf-8"))

    def test_coupled_figures_are_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            trace = root / "coupled.csv"
            output_dir = root / "figures"
            trace.write_text(
                "time,east,north,gamma_rate_cmd,gamma_rate_achieved,heading_rate_cmd,heading_rate_achieved,"
                "roll_ref,roll,pitch_ref,pitch,yaw_ref,yaw,allocation_residual_norm,"
                "alpha_rad,beta_rad,mx_cmd,my_cmd,mz_cmd,mx_aero,my_aero,mz_aero,force_tangent,force_normal,force_lateral,"
                "quaternion_norm_error,dcm_orthogonality_error,force_east,force_up,dcm_force_east,dcm_force_up,dcm_force_delta_norm\n"
                "0,0,0,0.01,0.008,0.02,0.018,0.0,0.0,0.01,0.01,0.02,0.02,10,0.03,0.01,100,80,60,90,70,50,-3000,8000,200,0,0,-3000,8000,-2800,7900,224\n"
                "1,1000,100,0.01,0.009,0.02,0.019,0.05,0.04,0.02,0.018,0.03,0.028,12,0.04,0.015,110,85,65,95,75,55,-3200,8200,250,1e-15,2e-15,-3200,8200,-2950,8000,320\n",
                encoding="utf-8",
            )

            outputs = generate_coupled_figures(trace, output_dir)

            self.assertEqual(len(outputs), 9)
            for output in outputs:
                self.assertIn("<svg", output.read_text(encoding="utf-8"))

    def test_six_dof_rigid_body_figures_are_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            trace = root / "sixdof.csv"
            output_dir = root / "figures"
            trace.write_text(
                "time,east,north,altitude,speed,p,q,r,quaternion_norm_error,dcm_orthogonality_error\n"
                "0,0,0,30000,1700,0,0,0,0,0\n"
                "1,1700,5,30001,1699,0.01,0.02,0.03,1e-15,2e-15\n",
                encoding="utf-8",
            )

            outputs = generate_six_dof_rigid_body_figures(trace, output_dir)

            self.assertEqual(len(outputs), 3)
            for output in outputs:
                self.assertIn("<svg", output.read_text(encoding="utf-8"))

    def test_aero_table_figures_are_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            trace = root / "aero.csv"
            output_dir = root / "figures"
            trace.write_text(
                "time,clift,cdrag,cy,cm_pitch,lift,drag,side_force,mx,my,mz\n"
                "0,0.2,0.06,0.01,-0.02,12000,3600,600,100,-200,50\n"
                "1,0.25,0.07,-0.02,-0.03,14000,3900,-800,120,-250,70\n",
                encoding="utf-8",
            )

            outputs = generate_aero_table_figures(trace, output_dir)

            self.assertEqual(len(outputs), 3)
            for output in outputs:
                self.assertIn("<svg", output.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
