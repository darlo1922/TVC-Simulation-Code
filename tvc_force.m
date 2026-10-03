function [F1, F2, J] = tvc_force(theta_x, theta_y, params, T_req)

dtheta = 1e-6; % small increment for finding numerical partial derivatives

[L1,L2] = tvc_inverse_kinematics(theta_x,theta_y,params); % get lengths at given angles

[L1_x,L2_x] = tvc_inverse_kinematics(theta_x + dtheta, theta_y, params); % find lengths for given angles+dtheta for x
[L1_y,L2_y] = tvc_inverse_kinematics(theta_x, theta_y + dtheta, params); % find lengths for given angles+dtheta for y

dL1_dtheta_x = (L1_x - L1)/dtheta; % find partial derivative
dL2_dtheta_x = (L2_x - L2)/dtheta; % find partial derivative
dL1_dtheta_y = (L1_y - L1)/dtheta; % find partial derivative
dL2_dtheta_y = (L2_y - L2)/dtheta; % find partial derivative

% Assign partial derivatives to get the Jacobian
J = [dL1_dtheta_x, dL1_dtheta_y; 
    dL2_dtheta_x, dL2_dtheta_y];

F = J' \ T_req; % Left divide the maximum torque by the transpose of the Jacobian to find Force Vector
F1 = F(1); % Divide force vector into component (1)
F2 = F(2); % Divide force vector into component (2)

end