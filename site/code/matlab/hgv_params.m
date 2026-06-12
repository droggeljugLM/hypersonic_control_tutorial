function params = hgv_params()
%HGV_PARAMS Return teaching-model parameters.
% The values mirror the compact Python example. They are for control
% education only, not for vehicle design or flight qualification.

params.mass = 1200.0;
params.iy = 8.5e4;
params.ref_area = 1.2;
params.ref_chord = 3.0;
params.gravity = 9.80665;
params.rho0 = 1.225;
params.scale_height = 7200.0;
params.thrust_max = 55000.0;
params.cl0 = 0.04;
params.cl_alpha = 2.2;
params.cl_delta = 0.25;
params.cd0 = 0.045;
params.cd_alpha2 = 0.85;
params.cm_alpha = -0.42;
params.cm_q = -5.0;
params.cm_delta = -0.75;
params.actuator_wn = 22.0;
params.actuator_zeta = 0.75;
params.delta_limit = deg2rad(18.0);
params.delta_rate_limit = deg2rad(80.0);
params.throttle_min = 0.0;
params.throttle_max = 1.0;
params.q_limit = 70000.0;
params.q_warning = 62000.0;
end
