% Backlash Runner
params.R_A = 6; % (in) Fixed Upper Mount Radius
params.R_B = 4; % (in) Fixed Lower Mount Radius
params.Z_A = 0.5; % (in) Fixed Upper Mount Height
params.Z_B = -12; % (in) Fixed Lower Mount Height

theta_x_desired = deg2rad(5);
theta_y_desired = deg2rad(-5);

backlash = 0.200;

[theta_x_range,theta_y_range] = tvc_backlash(theta_x_desired,theta_y_desired,backlash,params);

disp(rad2deg(theta_x_range))
disp(rad2deg(theta_y_range))