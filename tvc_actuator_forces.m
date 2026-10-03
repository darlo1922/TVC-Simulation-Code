function [max_F1_Y,max_F2_Y,theta1_max_Y,theta2_max_Y,max_F1_X,max_F2_X,theta1_max_X,theta2_max_X,F_MAX] = tvc_actuator_forces (params,Fparams)

theta_max = deg2rad(15); % convert max theta into rads
theta_x_vec = linspace(-theta_max, theta_max, 31); % create a vector for varying theta_x
theta_y_vec = linspace(-theta_max, theta_max, 31); % create a vector for varying theta_y

[T_tot] = tvc_load (params,Fparams); % (ft*lbf) get maximum torque

%% All of the torque along the Y axis

T_req = [0; T_tot]; % (in*lbf)

F1_values = zeros(length(theta_x_vec),length(theta_y_vec)); % create vector for storing F1 values (1)
F2_values = zeros(length(theta_x_vec),length(theta_y_vec)); % create vector for storing F2 values (2)

for i = 1:length(theta_x_vec)

    for j = 1:length(theta_y_vec)

        theta_x = theta_x_vec(i); % get value for theta_x
        theta_y = theta_y_vec(j); % get value for theta_y

        [F1,F2,~] = tvc_force(theta_x,theta_y,params,T_req); % get forces for given combination of theta_x and theta_y

        F1_values(i,j) = F1; % store force (1)
        F2_values(i,j) = F2; % store force (2)

    end
end

[max_F1_Y, index1] = max(abs(F1_values(:))); % find and store the maximum force (1)
[max_F2_Y, index2] = max(abs(F2_values(:))); % find and store the maximum force (2)

[i1,j1] = ind2sub(size(F1_values), index1); % find the position when the force is at maximum (1)
[i2,j2] = ind2sub(size(F2_values), index2); % find the position when the force is at maximum (2)

theta1_max_Y = [rad2deg(theta_x_vec(i1)), rad2deg(theta_y_vec(j1))]; % store the values of theta_x and theta_y when force is max (1)
theta2_max_Y = [rad2deg(theta_x_vec(i2)), rad2deg(theta_y_vec(j2))]; % store the values of theta_x and theta_y when force is max (2)


%% All of the torque along the X axis

T_req = [T_tot; 0]; % (in*lbf)

F1_values = zeros(length(theta_x_vec),length(theta_y_vec)); % create vector for storing F1 values (1)
F2_values = zeros(length(theta_x_vec),length(theta_y_vec)); % create vector for storing F2 values (2)

for i = 1:length(theta_x_vec)

    for j = 1:length(theta_y_vec)

        theta_x = theta_x_vec(i); % get value for theta_x
        theta_y = theta_y_vec(j); % get value for theta_y

        [F1,F2,~] = tvc_force(theta_x,theta_y,params,T_req); % get force components for given combination of theta_x and theta_y

        F1_values(i,j) = F1; % store force (1)
        F2_values(i,j) = F2; % store force (2)

    end
end

[max_F1_X, index1] = max(abs(F1_values(:))); % find and store the maximum force (1)
[max_F2_X, index2] = max(abs(F2_values(:))); % find and store the maximum force (1)

[i1,j1] = ind2sub(size(F1_values), index1); % find the position when the force is at maximum (1)
[i2,j2] = ind2sub(size(F2_values), index2); % find the position when the force is at maximum (2)

theta1_max_X = [rad2deg(theta_x_vec(i1)), rad2deg(theta_y_vec(j1))]; % store the values of theta_x and theta_y when force is max (1)
theta2_max_X = [rad2deg(theta_x_vec(i2)), rad2deg(theta_y_vec(j2))]; % store the values of theta_x and theta_y when force is max (2)

% Absolute Max Force on Actuator
F_MAX = max([max_F1_Y,max_F2_Y,max_F1_X,max_F2_X]); % find and store the absolute maximum force

