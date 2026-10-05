% TVC Error Runner 

params.R_A = 6; % (in) Fixed Upper Mount Radius
params.R_B = 4; % (in) Fixed Lower Mount Radius
params.Z_A = 0.5; % (in) Fixed Upper Mount Height
params.Z_B = -12; % (in) Fixed Lower Mount Height
params.Vfeed = 2.6; % (in/s) Fixed Feed Speed

Fparams.D_cm = 9; % (in) Distance between O and the engines CM
Fparams.mu_cg = 0.2; % friction in the central bearing joint
Fparams.r_bear = 1; % (in) radius of the central bearing joint

theta_x_desired = deg2rad(5); % input desired angle for theta_x
theta_y_desired = deg2rad(-5); % input desired angle for theta_y

backlash = 0.002; % (in)
stiffness = 20000; % (lbf/in)

[theta_x_range,theta_y_range] = tvc_error (theta_x_desired,theta_y_desired,backlash,stiffness,params,Fparams);

disp(rad2deg(theta_x_range)) % return actual angle range for theta_x
disp(rad2deg(theta_y_range)) % return actual angle range for theta_y
