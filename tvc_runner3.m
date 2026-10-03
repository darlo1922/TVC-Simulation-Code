%% Feed Speed vs Deg/s

Vfeed_vec = 0.5:0.25:4; % (in/s) Varying Feed Speed (0.5,4)
params.R_A = 6; % (in) Fixed Upper Mount Radius
params.R_B = 4; % (in) Fixed Lower Mount Radius
params.Z_A = 0.5; % (in) Fixed Upper Mount Height
params.Z_B = -12; % (in) Fixed Lower Mount Height

theta_MIN_vec = zeros(size(Vfeed_vec)); % Minimum Slew Rate Vector for each Vfeed

for i = 1:length(Vfeed_vec)

    params.Vfeed = Vfeed_vec(i); % Pass each Vfeed entry into params 

    %[~,~,~,theta_MIN] = tvc_speed (params); % get min slew rate for each combination

    theta_MIN_vec(i) = theta_MIN; % store each min slew rate value inside of its Vfeed location in the vector

end

figure ('Color','k') % create a new figure
grid on; % turn grid on
plot(Vfeed_vec,theta_MIN_vec,'-o','LineWidth',2); % graph of min slew rate vs Vfeed
xlabel('Feed Speed (in/s)'); % label x axis
ylabel('Minimum Degrees per Second'); % label y axis
title(sprintf('Feed Speed vs Minimum Degrees per Second at R_A = 6 & R_B = 4 ')); % title

params.Vfeed = 2.6; % Pass each Vfeed entry into params 
[~,~,~,theta_MIN] = tvc_speed (params); % get min slew rate for each combination
