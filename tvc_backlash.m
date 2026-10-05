function [theta_x_range,theta_y_range] = tvc_backlash (theta_x_des, theta_y_des, b, params)


[L1_des,L2_des] = tvc_inverse_kinematics(theta_x_des, theta_y_des, params);

L1_min = L1_des - b/2;
L1_max = L1_des + b/2;

L2_min = L2_des - b/2;
L2_max = L2_des + b/2;

[theta_x_1,theta_y_1] = tvc_kinematics (L1_min,L2_min,params);
[theta_x_2,theta_y_2] = tvc_kinematics (L1_max,L2_min,params);
[theta_x_3,theta_y_3] = tvc_kinematics (L1_min,L2_max,params);
[theta_x_4,theta_y_4] = tvc_kinematics (L1_max,L2_max,params);

theta_x_values = [theta_x_1,theta_x_2,theta_x_3,theta_x_4];
theta_y_values = [theta_y_1,theta_y_2,theta_y_3,theta_y_4];

theta_x_range = [min(theta_x_values), max(theta_x_values)];
theta_y_range = [min(theta_y_values), max(theta_y_values)];

end
