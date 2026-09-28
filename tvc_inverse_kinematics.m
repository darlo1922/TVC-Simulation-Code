function [L1, L2, tau1_per_lbf, tau2_per_lbf] = tvc_inverse_kinematics(theta_x, theta_y, params)

R_A = params.R_A;
Z_A = params.Z_A;
R_B = params.R_B;
Z_B = params.Z_B;

A_1 = [R_A; 0; Z_A];
A_2 = [0; R_A; Z_A];

B_0_1 = [R_B; 0; Z_B];
B_0_2 = [0; R_B; Z_B];

R = [cos(theta_y), sin(theta_x).*sin(theta_y), cos(theta_x).*sin(theta_y);
    0, cos(theta_x), -sin(theta_x);
    -sin(theta_y), sin(theta_x).*cos(theta_y), cos(theta_x).*cos(theta_y)];

B_1 = R * B_0_1;
B_2 = R * B_0_2;

v1 = B_1 - A_1;
v2 = B_2 - A_2;

L1 = norm(v1);
L2 = norm(v2);

f1_hat = -v1 / L1;
f2_hat = -v2 / L2;

tau1_per_lbf = cross(B_1, f1_hat);
tau2_per_lbf = cross(B_2, f2_hat);

end