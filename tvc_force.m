function [F1, F2, J] = tvc_force(theta_x, theta_y, params, T_req)

dtheta = 1e-6;

[L1,L2] = tvc_inverse_kinematics(theta_x,theta_y,params);

[L1_x,L2_x] = tvc_inverse_kinematics(theta_x + dtheta, theta_y, params);
[L1_y,L2_y] = tvc_inverse_kinematics(theta_x, theta_y + dtheta, params);

dL1_dtheta_x = (L1_x - L1)/dtheta;
dL2_dtheta_x = (L2_x - L2)/dtheta;
dL1_dtheta_y = (L1_y - L1)/dtheta;
dL2_dtheta_y = (L2_y - L2)/dtheta;

J = [dL1_dtheta_x, dL1_dtheta_y;
    dL2_dtheta_x, dL2_dtheta_y];

F = J' \ T_req;
F1 = F(1);
F2 = F(2);

end