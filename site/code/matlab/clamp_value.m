function value = clamp_value(value, lower, upper)
%CLAMP_VALUE Saturate a scalar.

value = max(lower, min(upper, value));
end
