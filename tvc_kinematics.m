function [theta_x, theta_y] = tvc_kinematics(L1_tar, L2_tar, params)

theta_guess = [0,0];

func = @(theta) tvc_length_error(theta, L1_tar,L2_tar,params);

options = optimoptions('fsolve','Display','off');
theta = fsolve(func,theta_guess,options);

theta_x = theta(1);
theta_y = theta(2);

end