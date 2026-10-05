function [theta_x_range,theta_y_range] = tvc_error (theta_x_des, theta_y_des, b, k, params,Fparams)

[T_tot] = tvc_load (params,Fparams);
T_req = [T_tot;0]; % Loaded in X axis
%T_req = [0;T_tot]; % Loaded in Y axis
[F1,F2,~] = tvc_force (theta_x_des,theta_y_des,params,T_req);

L1_del = F1 / k;
L2_del = F2 / k;

[L1_des,L2_des] = tvc_inverse_kinematics(theta_x_des, theta_y_des, params);

L1_min = L1_des - b/2 - L1_del;
L1_max = L1_des + b/2 + L1_del;

L2_min = L2_des - b/2 - L2_del;
L2_max = L2_des + b/2 + L2_del;

[theta_x_1,theta_y_1] = tvc_kinematics (L1_min,L2_min,params);
[theta_x_2,theta_y_2] = tvc_kinematics (L1_max,L2_min,params);
[theta_x_3,theta_y_3] = tvc_kinematics (L1_min,L2_max,params);
[theta_x_4,theta_y_4] = tvc_kinematics (L1_max,L2_max,params);

theta_x_values = [theta_x_1,theta_x_2,theta_x_3,theta_x_4];
theta_y_values = [theta_y_1,theta_y_2,theta_y_3,theta_y_4];

theta_x_range = [min(theta_x_values), max(theta_x_values)];
theta_y_range = [min(theta_y_values), max(theta_y_values)];

end