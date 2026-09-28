function [stroke_L1,stroke_L2,stroke] = tvc_stroke (params)

theta_max = deg2rad(15);

theta_x_vec = linspace(-theta_max, theta_max, 31);
theta_y_vec = linspace(-theta_max, theta_max, 31);

L1_values = zeros(length(theta_x_vec), length(theta_y_vec));
L2_values = zeros(length(theta_x_vec), length(theta_y_vec));

for i = 1:length(theta_x_vec)
    for j = 1:length(theta_y_vec)

        theta_x = theta_x_vec(i);
        theta_y = theta_y_vec(j);

        [L1,L2] = tvc_inverse_kinematics(theta_x,theta_y,params);

        L1_values(i,j) = L1;
        L2_values(i,j) = L2;

    end
end

L1_min = min(L1_values(:));
L1_max = max(L1_values(:));

L2_min = min(L2_values(:));
L2_max = max(L2_values(:));

stroke_L1 = L1_max - L1_min;
stroke_L2 = L2_max - L2_min;

stroke = max(stroke_L1, stroke_L2);