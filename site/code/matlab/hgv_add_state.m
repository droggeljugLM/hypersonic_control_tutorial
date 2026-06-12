function next = hgv_add_state(state, delta, scale)
%HGV_ADD_STATE Add scaled state derivative for RK4 stages.

next.velocity = state.velocity + scale * delta.velocity;
next.altitude = state.altitude + scale * delta.altitude;
next.gamma = state.gamma + scale * delta.gamma;
next.alpha = state.alpha + scale * delta.alpha;
next.pitch_rate = state.pitch_rate + scale * delta.pitch_rate;
next.theta = state.theta + scale * delta.theta;
next.elevator = clamp_value(state.elevator + scale * delta.elevator, deg2rad(-25.0), deg2rad(25.0));
next.elevator_rate = clamp_value(state.elevator_rate + scale * delta.elevator_rate, deg2rad(-120.0), deg2rad(120.0));
end
