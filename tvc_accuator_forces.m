function [max_F1_Y,max_F2_Y,theta1_max_Y,theta2_max_Y,max_F1_X,max_F2_X,theta1_max_X,theta2_max_X,F_MAX] = tvc_accuator_forces (params,Fparams)

theta_max = deg2rad(15);
theta_x_vec = linspace(-theta_max, theta_max, 31);
theta_y_vec = linspace(-theta_max, theta_max, 31);

[T_tot_ft] = tvc_load (params,Fparams); % (ft*lbf)
T_tot_in = T_tot_ft.*12; % (in*lbf)


%% All of the torque along the Y axis

T_req = [0; T_tot_in]; % (in*lbf)

F1_values = zeros(length(theta_x_vec),length(theta_y_vec));
F2_values = zeros(length(theta_x_vec),length(theta_y_vec));

for i = 1:length(theta_x_vec)

    for j = 1:length(theta_y_vec)

        theta_x = theta_x_vec(i);
        theta_y = theta_y_vec(j);

        [F1,F2,~] = tvc_force(theta_x,theta_y,params,T_req);

        F1_values(i,j) = F1;
        F2_values(i,j) = F2;

    end
end

[max_F1_Y, index1] = max(abs(F1_values(:)));
[max_F2_Y, index2] = max(abs(F2_values(:)));

[i1,j1] = ind2sub(size(F1_values), index1);
[i2,j2] = ind2sub(size(F2_values), index2);

theta1_max_Y = [rad2deg(theta_x_vec(i1)), rad2deg(theta_y_vec(j1))];
theta2_max_Y = [rad2deg(theta_x_vec(i2)), rad2deg(theta_y_vec(j2))];


%% All of the torque along the X axis

T_req = [T_tot_in; 0]; % (in*lbf)

F1_values = zeros(length(theta_x_vec),length(theta_y_vec));
F2_values = zeros(length(theta_x_vec),length(theta_y_vec));

for i = 1:length(theta_x_vec)

    for j = 1:length(theta_y_vec)

        theta_x = theta_x_vec(i);
        theta_y = theta_y_vec(j);

        [F1,F2,~] = tvc_force(theta_x,theta_y,params,T_req);

        F1_values(i,j) = F1;
        F2_values(i,j) = F2;

    end
end

[max_F1_X, index1] = max(abs(F1_values(:)));
[max_F2_X, index2] = max(abs(F2_values(:)));

[i1,j1] = ind2sub(size(F1_values), index1);
[i2,j2] = ind2sub(size(F2_values), index2);

theta1_max_X = [rad2deg(theta_x_vec(i1)), rad2deg(theta_y_vec(j1))];
theta2_max_X = [rad2deg(theta_x_vec(i2)), rad2deg(theta_y_vec(j2))];

%% Absolute Max Force on Actuator
F_MAX = max([max_F1_Y,max_F2_Y,max_F1_X,max_F2_X]);

