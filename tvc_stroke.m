function [stroke_L1,stroke_L2,stroke, L_MAX, L_MIN] = tvc_stroke (params)

theta_max = deg2rad(15); % convert max theta into rads

theta_x_vec = linspace(-theta_max, theta_max, 31); % create a vector for varying theta_x
theta_y_vec = linspace(-theta_max, theta_max, 31); % create a vector for varying theta_y

L1_values = zeros(length(theta_x_vec), length(theta_y_vec)); % create vector for storing L1 values (1)
L2_values = zeros(length(theta_x_vec), length(theta_y_vec)); % create vector for storing L2 values (1)

for i = 1:length(theta_x_vec)
    for j = 1:length(theta_y_vec)

        theta_x = theta_x_vec(i); % get value for theta_x
        theta_y = theta_y_vec(j); % get value for theta_y

        [L1,L2] = tvc_inverse_kinematics(theta_x,theta_y,params); % get lengths for given combination of theta_x and theta_y

        L1_values(i,j) = L1; % store length (1)
        L2_values(i,j) = L2; % store length (2)

    end
end

L1_min = min(L1_values(:)); % find and store the minimum length (1)
L1_max = max(L1_values(:)); % find and store the maximum length (1)

L2_min = min(L2_values(:)); % find and store the minimum length (2)
L2_max = max(L2_values(:)); % find and store the maximum length (2)

L_MIN = min([L1_min,L2_min]); % find the absolute minimum length
L_MAX = max([L1_max,L2_max]); % find the absolute maximum length

stroke_L1 = L1_max - L1_min; % find the difference between max and min lengths (1)
stroke_L2 = L2_max - L2_min; % find the difference between max and min lengths (2)

stroke = max(stroke_L1, stroke_L2); % find the absolute maximum stroke