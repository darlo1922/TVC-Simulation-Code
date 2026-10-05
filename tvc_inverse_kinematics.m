function [L1, L2] = tvc_inverse_kinematics(theta_x, theta_y, params)

R_A = params.R_A; % (in) Upper Mount Radius 
Z_A = params.Z_A; % (in) Upper Mount Height
R_B = params.R_B; % (in) Lower Mount Radius 
Z_B = params.Z_B; % (in) Lower Mount Height

A_1 = [R_A; 0; Z_A]; % (in) First Upper Mount Position Vector
A_2 = [0; R_A; Z_A]; % (in) Second Upper Mount Position Vector

B_0_1 = [R_B; 0; Z_B]; % (in) First Lower Mount Position Vector when theta_x and theta_y = 0
B_0_2 = [0; R_B; Z_B]; % (in) Second Lower Mount Position Vector when theta_x and theta_y = 0

% Rotational Matrix for 2 degrees of freedom R = RyRx
R = [cos(theta_y), sin(theta_x).*sin(theta_y), cos(theta_x).*sin(theta_y);
    0, cos(theta_x), -sin(theta_x);
    -sin(theta_y), sin(theta_x).*cos(theta_y), cos(theta_x).*cos(theta_y)];

B_1 = R * B_0_1; % First Lower Position Vector when theta_x and theta_y /= 0
B_2 = R * B_0_2; % Second Lower Position Vector when theta_x and theta_y /= 0

v1 = B_1 - A_1; % Vector between position vector A_1 and position vector B_1
v2 = B_2 - A_2; % Vector between position vector A_2  and position vector B_2

L1 = norm(v1); % Magnitude of v1 (distance between A_1 and B_1)
L2 = norm(v2); % Magnitude of v2 (distance between A_2 and B_2)

end