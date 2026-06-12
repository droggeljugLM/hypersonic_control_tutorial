function [altitude_ref, velocity_ref] = reference_profile(time_s)
%REFERENCE_PROFILE Nominal speed-altitude reference.

altitude_ref = 30000.0 - 900.0 * min(1.0, time_s / 35.0);
velocity_ref = 1750.0 + 80.0 * min(1.0, time_s / 25.0);
end
