function qbar = hgv_dynamic_pressure(state, params)
%HGV_DYNAMIC_PRESSURE Compute qbar = 0.5*rho*V^2.

rho = hgv_atmosphere(state.altitude, params);
qbar = 0.5 * rho * state.velocity * state.velocity;
end
