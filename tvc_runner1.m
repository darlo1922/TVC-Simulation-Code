%% TVC vs Mount Radii

clc; clf; clear;

% Create Ranges for R_A and R_B
R_A_vec = 5:0.1:8; % (in) Varying Upper Mount Radius for (5,8)
R_B_vec = 3:0.1:5; % (in) Varying Lower Mount Radius for (3,5)

params.Z_A = 0; % (in) Fixed Upper Mount Height
params.Z_B = -12; % (in) Fixed Lower Mount Height
params.Vfeed = 2.6; % (in/s) Fixed Feed Speed

Fparams.D_cm = 9; % (in) Distance between O and the engines CM
Fparams.mu_cg = 0.2; % friction in the central bearing joint
Fparams.r_bear = 1; % (in) radius of the central bearing joint

F_MAX_matrix = zeros(length(R_A_vec),length(R_B_vec)); % Maximum Force Matrix for each combination of R_A and R_B
stroke_matrix = zeros(length(R_A_vec),length(R_B_vec)); % Maximum Stroke Matrix for each combination of R_A and R_B
theta_MIN_matrix = zeros(length(R_A_vec),length(R_B_vec)); % Minimum Slew Rate Matrix for each combination of R_A and R_B
L_MAX_matrix = zeros(length(R_A_vec),length(R_B_vec)); % Max Actuator Length Matrix for each combination of R_A and R_B
L_MIN_matrix = zeros(length(R_A_vec),length(R_B_vec)); % Min Actuator Length Matrix for each combination of R_A and R_B

for i = 1:length(R_A_vec)
    
    for j = 1:length(R_B_vec)
    
    params.R_A = R_A_vec(i); % Pass each R_A entry into params 
    params.R_B = R_B_vec(j); % Pass each R_B entry into params
    
    [~,~,~,~,~,~,~,~,F_MAX] = tvc_actuator_forces (params,Fparams); % get max force for each combination
    [~,~,stroke,~,~] = tvc_stroke (params); % get max stroke for each combination
    [~,~,~,theta_MIN] = tvc_speed (params); % get min slew rate for each combination
    [~,~,~,L_MAX,L_MIN] = tvc_stroke (params); % get max and min actuator length for each combination

    F_MAX_matrix(i,j) = F_MAX; % store each max force value
    stroke_matrix(i,j) = stroke; % store each max stroke value
    theta_MIN_matrix(i,j) = theta_MIN; % store each min slew rate value 
    L_MAX_matrix(i,j) = L_MAX; % store max actuator length 
    L_MIN_matrix(i,j) = L_MIN; % store min actuator length
    
    end

end

% 3D Force Surface

figure ('Color','k') % create a new figure
grid on; % turn grid on
[R_A_grid,R_B_grid] = meshgrid(R_A_vec,R_B_vec); % create a grid for R_A and R_B 
surf(R_A_grid,R_B_grid,F_MAX_matrix.'); % graph the surface of max force vs R_A and R_B

xlabel('R_A (in)') % label x axis
ylabel('R_B (in)') % label y axis
zlabel('Maximum Actuator Force (lbf)') % label z axis
title('Maximum Actuator Force vs R_A and R_B when Z_A = 0 and Z_B = -12 (2.6in/s)') % title

% 3D Stroke Surface

figure ('Color','k') % create a new figure
grid on; % turn grid on
[R_A_grid,R_B_grid] = meshgrid(R_A_vec,R_B_vec); % create a grid for R_A and R_B 
surf(R_A_grid,R_B_grid,stroke_matrix.'); % graph the surface of max stroke vs R_A and R_B

xlabel('R_A (in)') % label x axis
ylabel('R_B (in)') % label y axis
zlabel('Maximum Stroke (in)') % label z axis
title('Maximum Stroke vs R_A and R_B when Z_A = 0 and Z_B = -12 (2.6in/s)') % title

% 3D Actuator Speed Surface

figure('Color','k');  % create a new figure
grid on; % turn grid on
[R_A_grid,R_B_grid] = meshgrid(R_A_vec,R_B_vec); % create a grid for R_A and R_B 
surf(R_A_grid,R_B_grid,theta_MIN_matrix.'); % graph the surface of min speed vs R_A and R_B

xlabel('R_A (in)'); % label x axis
ylabel('R_B (in)'); % label y axis
zlabel('Minimum Actuator Speed (deg/s)'); % label z axis
title('Minimum Actuator Speed vs R_A and R_B when Z_A = 0 and Z_B = -12 (2.6in/s)'); % title

% 3D Minimum Actuator Length Surface

figure('Color','k');  % create a new figure
grid on; % turn grid on
[R_A_grid,R_B_grid] = meshgrid(R_A_vec,R_B_vec); % create a grid for R_A and R_B 
surf(R_A_grid,R_B_grid,L_MIN_matrix.'); % graph the surface of min length vs R_A and R_B

xlabel('R_A (in)'); % label x axis
ylabel('R_B (in)'); % label y axis
zlabel('Minimum Actuator Length (in)'); % label z axis
title('Minimum Actuator Length vs R_A and R_B when Z_A = 0 and Z_B = -12 (2.6in/s)'); % title

% 3D Maximum Actuator Length Surface

figure('Color','k');  % create a new figure
grid on; % turn grid on
[R_A_grid,R_B_grid] = meshgrid(R_A_vec,R_B_vec); % create a grid for R_A and R_B 
surf(R_A_grid,R_B_grid,L_MAX_matrix.'); % graph the surface of max length vs R_A and R_B

xlabel('R_A (in)'); % label x axis
ylabel('R_B (in)'); % label y axis
zlabel('Maximum Actuator Length (in)'); % label z axis
title('Maximum Actuator Length vs R_A and R_B when Z_A = 0 and Z_B = -12 (2.6in/s)'); % title