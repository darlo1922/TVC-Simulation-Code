%% TVC vs Mount Radii

clc; clf; clear;

R_A_vec = 5:0.1:8; % (in)
R_B_vec = 3:0.1:5; % (in)

params.Z_A = 0; % (in)
params.Z_B = -12; % (in)
params.Vfeed = 2.6; % (in/s)

Fparams.D_cm = 9; % (in)
Fparams.mu_cg = 0.2;
Fparams.r_bear = 1; % (in)


F_MAX_matrix = zeros(length(R_A_vec),length(R_B_vec));
stroke_matrix = zeros(length(R_A_vec),length(R_B_vec));
theta_MIN_matrix = zeros(length(R_A_vec),length(R_B_vec));

for i = 1:length(R_A_vec)
    
    for j = 1:length(R_B_vec)
    
    params.R_A = R_A_vec(i);
    params.R_B = R_B_vec(j);
    
    [~,~,~,~,~,~,~,~,F_MAX] = tvc_accuator_forces (params,Fparams);
    [~,~,stroke] = tvc_stroke (params);
    [~,~,~,theta_MIN] = tvc_speed (params);

    F_MAX_matrix(i,j) = F_MAX;
    stroke_matrix(i,j) = stroke;
    theta_MIN_matrix(i,j) = theta_MIN;

    end

end

% 3D Force Surface

figure ('Color','k')
grid on;
[R_A_grid,R_B_grid] = meshgrid(R_A_vec,R_B_vec);
surf(R_A_grid,R_B_grid,F_MAX_matrix.');

xlabel('R_A (in)')
ylabel('R_B (in)')
zlabel('Maximum Actuator Force (lbf)')
title('Maximum Actuator Force vs R_A and R_B when Z_A = 0 and Z_B = -12 (3in/s)')

% 3D Stroke Surface

figure ('Color','k')
grid on;
[R_A_grid,R_B_grid] = meshgrid(R_A_vec,R_B_vec);
surf(R_A_grid,R_B_grid,stroke_matrix.');

xlabel('R_A (in)')
ylabel('R_B (in)')
zlabel('Maximum Stroke (in)')
title('Maximum Stroke vs R_A and R_B when Z_A = 0 and Z_B = -12 (3in/s)')
grid on

% 3D Actuator Speed Surface

figure('Color','k');
[R_A_grid,R_B_grid] = meshgrid(R_A_vec,R_B_vec);
surf(R_A_grid,R_B_grid,theta_MIN_matrix.');

grid on;
xlabel('R_A (in)');
ylabel('R_B (in)');
zlabel('Minimum Actuator Speed (deg/s)');
title('Minimum Actuator Speed vs R_A and R_B when Z_A = 0 and Z_B = -12 (3in/s)');

%% TVC Force vs Mount Heights

Z_A_vec = -3:0.1:3; % (in)
Z_B_vec = -18:0.5:-9; % (in)

params.R_A = 6; % (in)
params.R_B = 4; % (in)
params.Vfeed = 2; % (in/s)

F_MAX_matrix = zeros(length(Z_A_vec),length(Z_B_vec));
stroke_matrix = zeros(length(Z_A_vec),length(Z_B_vec));
theta_MIN_matrix = zeros(length(Z_A_vec),length(Z_B_vec));

for i = 1:length(Z_A_vec)

    for j = 1:length(Z_B_vec)

        params.Z_A = Z_A_vec(i);
        params.Z_B = Z_B_vec(j);

        [~,~,~,~,~,~,~,~,F_MAX] = tvc_accuator_forces (params,Fparams);
        [~,~,stroke] = tvc_stroke (params);
        [~,~,~,theta_MIN] = tvc_speed (params);

        F_MAX_matrix(i,j) = F_MAX;
        stroke_matrix(i,j) = stroke;
        theta_MIN_matrix(i,j) = theta_MIN;

    end

end

% 3D Force Surface

figure ('Color','k')
grid on;
[Z_A_grid,Z_B_grid] = meshgrid(Z_A_vec,Z_B_vec);
surf(Z_A_grid,Z_B_grid,F_MAX_matrix.');

xlabel('Z_A (in)')
ylabel('Z_B (in)')
zlabel('Maximum Actuator Force (lbf)')
title('Maximum Actuator Force vs Z_A and Z_B when R_A = 6 and R_B = 4 (3in/s)')

% 3D Stroke Surface

figure ('Color','k')
grid on;
[Z_A_grid,Z_B_grid] = meshgrid(Z_A_vec,Z_B_vec);
surf(Z_A_grid,Z_B_grid,stroke_matrix.');

xlabel('Z_A (in)')
ylabel('Z_B (in)')
zlabel('Maximum Stroke (in)')
title('Maximum Stroke vs Z_A and Z_B when R_A = 6 and R_B = 4 (3in/s)')
grid on

% 3D Actuator Speed Surface

figure('Color','k');
[Z_A_grid,Z_B_grid] = meshgrid(Z_A_vec,Z_B_vec);
surf(Z_A_grid,Z_B_grid,theta_MIN_matrix.');

grid on;
xlabel('Z_A (in)');
ylabel('Z_B (in)');
zlabel('Minimum Actuator Speed (deg/s)');
title('Minimum Actuator Speed vs Z_A and Z_B when R_A = 6 and R_B = 4 (3in/s)');

%% Feed Speed vs Deg/s

Vfeed_vec = 0.5:0.25:4;
params.R_A = 6;
params.R_B = 4;
params.Z_A = 0.5;
params.Z_B = -12;

theta_MIN_vec = zeros(size(Vfeed_vec));

for i = 1:length(Vfeed_vec)
    
    params.Vfeed = Vfeed_vec(i);

    [~,~,~,theta_MIN] = tvc_speed (params);

    theta_MIN_vec(i) = theta_MIN;

end

figure;
plot(Vfeed_vec,theta_MIN_vec,'-o','LineWidth',2);
grid on;
xlabel('Feed Speed (in/s)');
ylabel('Minimum Degrees per Second');
title(sprintf('Feed Speed vs Minimum Degrees per Second at R_A = 6 & R_B = 4 '));
 

 
 
 
 
 
 
 
