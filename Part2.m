%% AER1216 Project - Part 2
% Group Members:
% David Rolko
% Farhan Wadia
% Shahzeb Mahmood

%% Parameters
close all
clear all
clc

m = 0.420; % kg
g = 9.81; % N/kg
C_D  = 0.97;
S = 0.01; % m^2
rho = 1.225; % kg/m^3

% APC 8x6 SF Properties
%https://m-selig.ae.illinois.edu/props/volume-1/data/apcsf_8x6_static_2783rd.txt
diam = 8*0.0254; % m

num_cells = 3;
battery_life = 1500; % mA-hr
battery_voltage = 3.7; % V, assumed from propulsion lecture slide 53

%% Question 1
W = m*g; % N

P = sqrt(W^3 / (2*rho*(4*pi/4)*(diam)^2)); % W

E_b = num_cells * battery_voltage * (battery_life/1000) * 3600; % J

eta_m = 0.75;
eta_e = 0.85;

t_e = (E_b * eta_m * eta_e)/P; % s
sprintf('The flight endurance is %.2f s.', t_e)

% Find max range
V = 0:0.5:20;

D = 0.5 * rho * S * C_D .* V.^2;
alpha_D = atan2(D, W);
T = sqrt(W^2 + D.^2);

v = zeros(1, length(V));
P_ind = zeros(1, length(V));
P_tot = zeros(1, length(V));
for i = 1:length(V)
    % Solve 4th order polynomial for real, positive roots at each V 
    a4 = 1;
    a3 = 2*V(i)*sin(alpha_D(i));
    a2 = V(i)^2;
    a1 = 0;
    a0 = -(W^2 + D(i)^2) / (2*rho*(4*pi/4)*diam^2)^2; % use area of the 4 props
    
    v_all = roots([a4 a3 a2 a1 a0]);
    v(i) = v_all(real(v_all)>0 & imag(v_all)==0);
    
    % Find P_ind and P_tot at each V and save the results
    P_ind(i) = T(i) * v(i);
    P_tot(i) = T(i) * (v(i) + V(i)*sin(alpha_D(i))); 
end

[min_P_tot_over_V, min_P_tot_over_V_idx] = min(P_tot ./ V);

t_e_losses_for_max_range = (E_b * eta_m * eta_e) / P_tot(min_P_tot_over_V_idx);
max_range = t_e_losses_for_max_range * V(min_P_tot_over_V_idx);

sprintf('The flight range is %.2f m', max_range)
sprintf('The corresponding forward speed is %.2f m/s', V(min_P_tot_over_V_idx))
%% Question 2

% % Roll
% phi_desired = 0;
% 
% A_roll = [-4.2683 -3.1716; 4 0];
% B_roll = [2; 0];
% C_roll = [0.7417 0.4405];
% D_roll = 0;
% 
% x_roll = [0; 0];
% u_roll = phi_desired;
% 
% x_roll_dot = A_roll * x_roll + B_roll * u_roll;
% phi_actual = C_roll * x_roll + D_roll * u_roll;
% 
% % Pitch
% theta_desired = 0;
% 
% A_pitch = [-3.9784 -2.9796; 4 0];
% B_pitch = [2; 0];
% C_pitch = [1.2569 0.6083];
% D_pitch = 0;
% 
% x_pitch = [0; 0];
% u_roll = theta_desired;
% 
% x_pitch_dot = A_pitch * x_pitch + B_pitch * u_pitch;
% theta_actual = C_pitch * x_pitch + D_pitch * u_pitch;
% 
% % Yaw
% psi_dot_desired = 0;
% 
% A_yaw = -0.0059;
% B_yaw = 1;
% C_yaw = 1.2653;
% D_yaw = 0;
% 
% x_yaw = [0; 0];
% u_yaw = psi_dot_desired;
% 
% x_yaw_dot = A_yaw * x_yaw + B_yaw * u_yaw;
% psi_actual = C_pitch * x_pitch + D_pitch * u_pitch;
% 
% % Height
% w_desired = 0;
% 
% A_height = [-5.82 -3.6046*exp(1)^-6; 3.8147*exp(1)^-6 0];
% B_height = [1024; 0];
% C_height = [1.4907*exp(1)^-4 1.3191*exp(1)^3];
% D_height = 0;
% 
% x_height = [0; 0];
% u_height = w_desired;
% 
% x_height_dot = A_height * x_height + B_height * u_height;
% h_actual = C_height * x_height + D_height * u_height;
% 
% % Pitch to u
% 
% x_dot = -0.6665x
% u_actual = -3.0772 * x + 0 * theta_actual

