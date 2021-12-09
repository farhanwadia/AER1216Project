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