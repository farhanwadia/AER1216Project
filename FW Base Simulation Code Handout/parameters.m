% AER1216 Fall 2021 
% Fixed Wing Project Code
%
% parameters.m
%
% Initialization file which generates and stores all required data into the 
% structure P, which is then stored in the workspace. Simulink model calls 
% on this function at the start of every simulation. Code structure adapted
% from Small Unmanned Aircraft: Theory and Practice by R.W. Beard and T. W. 
% McLain. 
% 
% Inputs: 
% N/A
%
% Outputs:
% P                 structure that contains all aerodynamic, geometric, and
%                   initial condition data for the aircraft and simulation.
%
% Last updated: Pravin Wedage 2021-11-09

%% TA NOTE
% An easy way to store parameters for use in simulink is through the use of
% a structure. For example, P.g = 9.81 stores the value of gravitational
% acceleration in the field g that is contained within the structure P.
% Anytime P is called anywhere in the simulation code, the value of P.g is
% accessible. 

%% Parameter Computation
% Initial Conditions
clear all
% compute trim conditions            
P.Va0 = 15;         % initial airspeed (also used as trim airspeed)
P.Va_trim = 15.00; 
P.Va = P.Va_trim;


P.gravity = 9.81;
P.g = 9.81; 

% Aerosonde UAV Data
% physical parameters of airframe

% aerodynamic coefficients

% Control Input limits 
P.delta_e_max = deg2rad(45); % assumed symmetric
P.delta_a_max = deg2rad(45); 
P.delta_r_max = deg2rad(25);

% Initial Conditions % connects with aircraft_dynamics.m, do not modify
% structure field names
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -1000;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axisu_trim
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate
P.delta_e0 =0;
P.delta_a0 =0;
P.delta_r0 =0;
P.delta_t0 =0;
                         
% Aerosonde UAV Geometric Data

P.m = 13.5;                          % UAV weight in kg
P.Ixx = 0.8244;                      % Moment of inertia in x-axis [kg m^2]
P.Iyy = 1.135;                       % Moment of inertia in y-axis [kg m^2]
P.Izz = 1.759;                       % Moment of inertia in z-axis [kg m^2]
P.Ixz = 0.1204;                      % Moment of inertia in xz frame [kg m^2]
P.S = 0.55;                          % Surface area of the wing in m^2
P.b = 2.8956;                        % Span of the wing in m
P.c = 0.18994;                       % Chord length of the wing in m
P.Sprop = pi*((16/2)*0.0254)^2;      % Disc area of the prop (in m^2) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! FIX
P.D=16*0.0254;                       % Prop diameter
P.e = 0.9;                           % Oswald's efficiency factor ??????????????
P.OmegaMax_RPM = 7000;               % Max RPM 
P.Fuel_Cap_L = 5.7;                  % Fuel Capacity in Liters

% Prop data obtained from https://m-selig.ae.illinois.edu/props/volume-3/propDB-volume-3.html

current_folder = pwd;

P.RPM_values = [3017,3043,3967,4034,4994,5027]; % the RPMs which we have data for
Num_files = length(P.RPM_values);

Prop_16x8_file_Location_static = append(current_folder,'\Propeller Data\16x8 - static.txt');   % make the path to the file
Prop_16x8_table_static = readtable(Prop_16x8_file_Location_static, 'HeaderLines', 1);      % put the contents of the file into a table
P.Prop_static = Prop_16x8_table_static{:,:};                                           % convert the table into an array

Prop_16x8_file_locations = strings([1,Num_files]);

for i = 1:Num_files   
    
    Prop_16x8_file_locations(i) = append(current_folder,'\Propeller Data\16x8 - ', int2str(P.RPM_values(i)), '.txt');   % make the path to the file
    Prop_16x8_tables{i} = readtable(Prop_16x8_file_locations(i), 'HeaderLines', 1);      % put the contents of the file into a table
    Array_temp = Prop_16x8_tables{i};  % convert the table into an array
    P.Props{i} = Array_temp{:,:};  % convert the table into an array

end

% aerodynamic Longitudinal coefficients

P.C_Lo = 0.28;
P.C_Do = 0.03;
P.C_mo = -0.02388;
P.C_Lalpha = 3.45;
P.C_Dalpha = 0.3;
P.C_malpha = -0.38;
P.C_Lq = 0;
P.C_Dq = 0;
P.C_mq = -3.6;
P.C_Ldelta_e = -0.36;
P.C_Ddelta_e_ = 0;
P.C_mdelta_e_ = -0.5;
P.C_Epsilon = 0.1592;

% aerodynamic Lateral coefficients

P.C_Yo = 0;
P.C_lo = 0;
P.C_no = 0;
P.C_Ybeta = -0.98;
P.C_lbeta = -0.12;
P.C_nbeta = 0.25;
P.C_Yp = 0;
P.C_lp = -0.26;
P.C_np = 0.022;
P.C_Yr = 0;
P.C_lr = 0.14;
P.C_nr = -0.35;
P.C_Ydelta_a = 0;
P.C_ldelta_a = 0.08;
P.C_ndelta_a = 0.06;
P.C_Ydelta_r = -0.17;
P.C_ldelta_r = 0.105;
P.C_ndelta_r = -0.032;