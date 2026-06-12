function [altitude_cmd, velocity_cmd] = controller_guarded_ref(state, altitude_ref, velocity_ref, params)
%CONTROLLER_GUARDED_REF Dynamic-pressure reference governor.

qbar = hgv_dynamic_pressure(state, params);
if qbar <= params.q_warning
    altitude_cmd = altitude_ref;
    velocity_cmd = velocity_ref;
    return;
end

severity = min(1.0, (qbar - params.q_warning) / max(1.0, params.q_limit - params.q_warning));
altitude_cmd = altitude_ref + severity * 7000.0;
velocity_cmd = max(900.0, velocity_ref - severity * 500.0);
end
