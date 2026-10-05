%% TVC Force vs Mount Heights

Z_A_vec = -3:0.1:3; % (in) Varying Upper Mount Height (-3,3)
Z_B_vec = -18:0.5:-9; % (in) Varying Lower Mount Height (-18,-9)

params.R_A = 6; % (in) Fixed Upper Mount Radius 
params.R_B = 4; % (in) Fixed Lower Mount Radius
params.Vfeed = 2.6; % (in/s) Fixed Feed Speed

Fparams.D_cm = 9; % (in) Distance between O and the engines CM
Fparams.mu_cg = 0.2; % friction in the central bearing joint
Fparams.r_bear = 1; % (in) radius of the central bearing joint

backlash = 0.002; % (in)
stiffness = 20000; % (lbf/in)
theta_x_des = deg2rad(15);
theta_y_des = deg2rad(15);

F_MAX_matrix = zeros(length(Z_A_vec),length(Z_B_vec)); % Maximum Force Matrix for each combination of Z_A and Z_B
stroke_matrix = zeros(length(Z_A_vec),length(Z_B_vec)); % Maximum Stroke Matrix for each combination of Z_A and Z_B
theta_MIN_matrix = zeros(length(Z_A_vec),length(Z_B_vec)); % Minimum Slew Rate Matrix for each combination of Z_A and Z_B
L_MAX_matrix = zeros(length(Z_A_vec),length(Z_B_vec)); % Max Actuator Length Matrix for each combination of R_A and R_B
L_MIN_matrix = zeros(length(Z_A_vec),length(Z_B_vec)); % Min Actuator Length Matrix for each combination of R_A and R_B
backlash_theta_x_matrix = zeros(length(R_A_vec),length(R_B_vec));
backlash_theta_y_matrix = zeros(length(R_A_vec),length(R_B_vec));
stiffness_theta_x_matrix = zeros(length(R_A_vec),length(R_B_vec));
stiffness_theta_y_matrix = zeros(length(R_A_vec),length(R_B_vec));
total_error_theta_x_matrix = zeros(length(R_A_vec),length(R_B_vec));
total_error_theta_y_matrix = zeros(length(R_A_vec),length(R_B_vec));

for i = 1:length(Z_A_vec)

    for j = 1:length(Z_B_vec)

        params.Z_A = Z_A_vec(i); % Pass each Z_A entry into params 
        params.Z_B = Z_B_vec(j); % Pass each Z_B entry into params 

        [~,~,~,~,~,~,~,~,F_MAX] = tvc_actuator_forces (params,Fparams); % get max force for each combination
        [~,~,stroke,~,~] = tvc_stroke (params); % get max stroke for each combination
        [~,~,~,theta_MIN] = tvc_speed (params); % get min slew rate for each combination
        [~,~,~,L_MAX,L_MIN] = tvc_stroke (params); % get max and min actuator length for each combination


        F_MAX_matrix(i,j) = F_MAX; % store each max force value inside of its combination location in the matrix
        stroke_matrix(i,j) = stroke; % store each max stroke value inside of its combination location in the matrix
        theta_MIN_matrix(i,j) = theta_MIN; % store each min slew rate value inside of its combination location in the matrix
        L_MAX_matrix(i,j) = L_MAX; % store max actuator length 
        L_MIN_matrix(i,j) = L_MIN; % store min actuator length
    end

end

% 3D Force Surface

figure ('Color','k') % create a new figure
grid on; % turn grid on
[Z_A_grid,Z_B_grid] = meshgrid(Z_A_vec,Z_B_vec); % create a grid for Z_A and Z_B 
surf(Z_A_grid,Z_B_grid,F_MAX_matrix.'); % graph the surface of max force vs Z_A and Z_B

xlabel('Z_A (in)') % label x axis
ylabel('Z_B (in)') % label y axis
zlabel('Maximum Actuator Force (lbf)') % label z axis
title('Maximum Actuator Force vs Z_A and Z_B when R_A = 6 and R_B = 4 (2.6in/s)') % title

% 3D Stroke Surface

figure ('Color','k') % create a new figure
grid on; % turn grid on
[Z_A_grid,Z_B_grid] = meshgrid(Z_A_vec,Z_B_vec); % create a grid for Z_A and Z_B 
surf(Z_A_grid,Z_B_grid,stroke_matrix.'); % graph the surface of max stroke vs Z_A and Z_B

xlabel('Z_A (in)') % label x axis
ylabel('Z_B (in)') % label y axis
zlabel('Maximum Stroke (in)') % label z axis
title('Maximum Stroke vs Z_A and Z_B when R_A = 6 and R_B = 4 (2.6in/s)') % title

% 3D Actuator Speed Surface

figure('Color','k'); % create a new figure
grid on; % turn grid on
[Z_A_grid,Z_B_grid] = meshgrid(Z_A_vec,Z_B_vec); % create a grid for Z_A and Z_B 
surf(Z_A_grid,Z_B_grid,theta_MIN_matrix.'); % graph the surface of min slew rate vs Z_A and Z_B

xlabel('Z_A (in)'); % label x axis
ylabel('Z_B (in)'); % label y axis
zlabel('Minimum Actuator Speed (deg/s)'); % label z axis
title('Minimum Actuator Speed vs Z_A and Z_B when R_A = 6 and R_B = 4 (2.6in/s)'); % title

% 3D Minimum Actuator Length Surface

figure('Color','k');  % create a new figure
grid on; % turn grid on
[Z_A_grid,Z_B_grid] = meshgrid(Z_A_vec,Z_B_vec); % create a grid for Z_A and Z_B 
surf(Z_A_grid,Z_B_grid,L_MIN_matrix.'); % graph the surface of min length vs R_A and R_B

xlabel('Z_A (in)'); % label x axis
ylabel('Z_B (in)'); % label y axis
zlabel('Minimum Actuator Length (in)'); % label z axis
title('Minimum Actuator Length vs Z_A and Z_B when R_A = 6 and R_B = 4 (2.6in/s)'); % title

% 3D Maximum Actuator Length Surface

figure('Color','k');  % create a new figure
grid on; % turn grid on
[Z_A_grid,Z_B_grid] = meshgrid(Z_A_vec,Z_B_vec); % create a grid for Z_A and Z_B 
surf(Z_A_grid,Z_B_grid,L_MAX_matrix.'); % graph the surface of max length vs R_A and R_B

xlabel('Z_A (in)'); % label x axis
ylabel('Z_B (in)'); % label y axis
zlabel('Maximum Actuator Length (in)'); % label z axis
title('Maximum Actuator Length vs Z_A and Z_B when R_A = 6 and R_B = 4 (2.6in/s)'); % title