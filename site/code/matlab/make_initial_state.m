function state = make_initial_state()
%MAKE_INITIAL_STATE Return nominal longitudinal initial condition.

state.velocity = 1750.0;
state.altitude = 30000.0;
state.gamma = deg2rad(-0.5);
state.alpha = deg2rad(2.0);
state.pitch_rate = 0.0;
state.theta = deg2rad(1.5);
state.elevator = 0.0;
state.elevator_rate = 0.0;
end
