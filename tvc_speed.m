function [theta_dot_X_deg,theta_dot_Y_deg,freq_MAX,theta_MIN] = tvc_speed (params)

Vfeed = params.Vfeed; % get the feed speed
theta_max = deg2rad(15); % convert the max angle to rads

theta_x_vec = linspace(-theta_max, theta_max, 31); % create a vector for varying theta_x
theta_y_vec = linspace(-theta_max, theta_max, 31); % create a vector for varying theta_y

speed_gain_X = 0; % maximum actuator-length sensitivity to x-axis rotation
speed_gain_Y = 0; % maximum actuator-length sensitivity to y-axis rotation

for i = 1:length(theta_x_vec)

    for j = 1:length(theta_y_vec)

        theta_x = theta_x_vec(i); % set the current theta_x_value
        theta_y = theta_y_vec(j); % set the current theta_y_value

        [~,~,J] = tvc_force(theta_x,theta_y,params,[0;0]); % get the jacobian

        speed_gain_X = max(speed_gain_X,max(abs(J(:,1)))); % find the largest length change per radian for x-axis rotation

        speed_gain_Y = max(speed_gain_Y,max(abs(J(:,2)))); % find the largest length change per radian for y-axis rotation
    end
end

theta_dot_X = Vfeed/speed_gain_X; % estimate the maximum x-axis angular rate 
theta_dot_Y = Vfeed/speed_gain_Y; % estimate the maximum y-axis angular rate 

theta_dot_X_deg = rad2deg(theta_dot_X); % convert to deg
theta_dot_Y_deg = rad2deg(theta_dot_Y); % convert to deg

theta_MIN = min([theta_dot_Y_deg,theta_dot_X_deg]); % get the minimum angular rate

freq_X = theta_dot_X/(2*pi*theta_max); % find the x frequency
freq_Y = theta_dot_Y/(2*pi*theta_max); % find the y frequency

freq_MAX = max([freq_Y,freq_X]); % get the maximum frequency

