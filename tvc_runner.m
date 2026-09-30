%% TVC Force vs Upper Mount Radius R_A

clc; clf; clear;

R_A_vec = 2:0.5:20; % (in)
R_B_vec = 2:0.5:20; % (in)

params.Z_A = 6; % (in)
params.Z_B = -4; % (in)
params.Vfeed = 4; % (in/s)

Fparams.D_cm = 9; % (in)
Fparams.freq = 1; % (Hz)
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


%%
figure;
plot(R_B_vec,F_MAX_matrix(9,:),'-o','LineWidth',2);
grid on;

xlabel('R_B (in)');
ylabel('Maximum Actuator Force (lbf)');
title(sprintf('Force vs R_B at R_A = %.1f in',R_A_vec(9)));

figure;
plot(R_A_vec,F_MAX_matrix(:,9),'-o','LineWidth',2);
grid on;

xlabel('R_A (in)');
ylabel('Maximum Actuator Force (lbf)');
title(sprintf('Force vs R_A at R_B = %.1f in',R_B_vec(9)));

% 2D Stroke Slices

figure;
plot(R_B_vec,stroke_matrix(9,:),'-o','LineWidth',2);
grid on;

xlabel('R_B (in)');
ylabel('Maximum Stroke (in)');
title(sprintf('Stroke vs R_B at R_A = %.1f in',R_A_vec(9)));

figure;
plot(R_A_vec,stroke_matrix(:,9),'-o','LineWidth',2);
grid on;

xlabel('R_A (in)');
ylabel('Maximum Stroke (in)');
title(sprintf('Stroke vs R_A at R_B = %.1f in',R_B_vec(9)));

% 3D Force Surface

figure ('Color','k')
grid on;
[R_A_grid,R_B_grid] = meshgrid(R_A_vec,R_B_vec);
surf(R_A_grid,R_B_grid,F_MAX_matrix.');

xlabel('R_A (in)')
ylabel('R_B (in)')
zlabel('Maximum Actuator Force (lbf)')
title('Maximum Actuator Force vs R_A and R_B')

% 3D Stroke Surface

figure ('Color','k')
grid on;
[R_A_grid,R_B_grid] = meshgrid(R_A_vec,R_B_vec);
surf(R_A_grid,R_B_grid,stroke_matrix.');

xlabel('R_A (in)')
ylabel('R_B (in)')
zlabel('Maximum Stroke (in)')
title('Maximum Stroke vs R_A and R_B')
grid on

% 3D Actuator Speed Surface
figure('Color','k');
surf(R_A_grid,R_B_grid,theta_MIN_matrix.');
grid on;
xlabel('R_A (in)');
ylabel('R_B (in)');
zlabel('Minimum Actuator Speed (deg/s)');
title('Minimum Actuator Speed vs R_A and R_B');

%% Feed Speed vs Deg/s

Vfeed_vec = 0.5:0.25:4;
params.R_A = 7;
params.R_B = 4;

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
title(sprintf('Feed Speed vs Minimum Degrees per Second at R_A = 7 & R_B = 4 '));
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
