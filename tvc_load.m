function [T_tot] = tvc_load (params,Fparams)

D_cm = Fparams.D_cm; % (in) distance from the CM of the engine to O
mu_cg = Fparams.mu_cg; % friction coefficient for central gimbal bearing
r_bear = Fparams.r_bear; % (in) radius of the bearing contact in central gimbal

F_thrust = 1600; % (lbf) maximum force of thrust 
r_off = 0.250; % (in) worst case scenario radius offset
m_e = 0.1036; %((lbf*s^2)/(in)) mass of the engine assembly below A for W_engine = 40 lbf
R_i = 2.89; % (in) ID of engine
R_o = 3.00; % (in) OD of engine
L_e = 18.00; % (in) length of the engine
theta_max = 15; % maximum gimbal angle
k_hose = 12; % ((in*lbf)/(deg)) rotational stiffness of hoses

% Frequency
[~,~,Freq_MAX] = tvc_speed (params); % get the frequency given a certain Vfeed using sinusoidal motion model
% This frequency does not relate to electronic frequency, this is an internal frequency from the motion model
freq = Freq_MAX; % assign frequency value

% Torque from offset thrust
T_off = r_off .* F_thrust; % (in*lbf)

% Torque from inertia of engine
I_gimbal = m_e.*((1/12).*L_e.^2 +0.25.*(R_i.^2 + R_o^2)) + m_e.*(D_cm).^2;
thetadd_max = (theta_max.*(pi/180)).*(2.*pi.*freq).^2;

T_inertia = I_gimbal .* thetadd_max; % (in*lbf)

% Torque from friction in gimbal pivot bearing
T_friction = mu_cg.*F_thrust.*r_bear; % (in*lbf)

% Torque from hose stiffness
T_hose = k_hose.*theta_max; % (in*lbf)

% Total Torque on actuator:
T_tot = T_off + T_inertia + T_friction + T_hose; % (in*lbf)







