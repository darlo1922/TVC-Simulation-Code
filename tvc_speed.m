function [theta_dot_X_deg,theta_dot_Y_deg,freq_MAX,theta_MIN] = tvc_speed (params)

Vfeed = params.Vfeed;
theta_max = deg2rad(15);

theta_x_vec = linspace(-theta_max, theta_max, 31);
theta_y_vec = linspace(-theta_max, theta_max, 31);

speed_gain_X = 0;
speed_gain_Y = 0;

for i = 1:length(theta_x_vec)

    for j = 1:length(theta_y_vec)

        theta_x = theta_x_vec(i);
        theta_y = theta_y_vec(j);

        [~,~,J] = tvc_force(theta_x,theta_y,params,[0;0]);

        speed_gain_X = max(speed_gain_X,max(abs(J(:,1))));

        speed_gain_Y = max(speed_gain_Y,max(abs(J(:,2))));
    end
end

theta_dot_X = Vfeed/speed_gain_X;
theta_dot_Y = Vfeed/speed_gain_Y;

theta_dot_X_deg = rad2deg(theta_dot_X);
theta_dot_Y_deg = rad2deg(theta_dot_Y);

theta_MIN = min([theta_dot_Y_deg,theta_dot_X_deg]);

freq_X = theta_dot_X/(2*pi*theta_max);
freq_Y = theta_dot_Y/(2*pi*theta_max);

freq_MAX = max([freq_Y,freq_X]);