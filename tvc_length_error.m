function [error] = tvc_length_error (theta,L1_tar,L2_tar,params)

theta_x = theta(1);
theta_y = theta(2);

[L1,L2] = tvc_inverse_kinematics (theta_x,theta_y,params);

error = [L1-L1_tar; L2-L2_tar];

end