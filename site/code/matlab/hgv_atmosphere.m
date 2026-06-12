function rho = hgv_atmosphere(altitude, params)
%HGV_ATMOSPHERE Exponential atmosphere for the teaching model.

altitude = max(0.0, altitude);
rho = params.rho0 * exp(-altitude / params.scale_height);
end
