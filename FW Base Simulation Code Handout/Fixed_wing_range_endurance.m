% AER1216 project
% David Rolko, Shahzeb Mahmood, Farhan Wadia
% 1.1 (fixed wing) question 1

% This code calculates the max range and endurance for the UAV

% For max range we find velocity to give the best fuel comsumption /
% velocity.  This is done each time the weight of the plane changes.  
% This is then integrated over the weight of the plane

% For max endurance we find velocity to give the best fuel comsumption.
% This is done each time the weight of the plane changes.  
% This is then integrated over the weight of the plane

%%
% Input Data

clc
fprintf(' Code is initializing'); 
parameters % get the data for the UAV

% what info to show
verbose_minP_T = false;
verbose_graph = true;
verbose = true;

% assumed flight level above sea level in (m)
Altitude = 500;

Fuel_L = P.Fuel_Cap_L; % amount of fuel in L

Fuel_density = (775.0 + 840.0) / 2; % density of Jet A-1 fuel (kg/m^3)
%https://code7700.com/fuel_density.htm

% info about the engine fuel consumption
Data_fuel_RPM = [1;4250;5000];
Data_fuel_Power = [1;1340; 2240];
fuel_comsump_P_kg= [0;0.48;0.78];

% step size to increment through the weight (in N), small is better but slower
W_fuel_step = 1; % can use 1
% step size to increment through the velocity (in m/s), small is better but slower
Velocity_step = 0.1; % can use 0.1

%%
% Initial general calculations

%calculate density of air
if Altitude < 11000
    % Use this if below 11km (in the troposphere)
    Temperature = 15.04 - .00649 * Altitude;
    Pressure = 101.29 * [(Temperature + 273.1)/288.08]^5.256;       
else
    % Use this if between 11km and 25km (entering lower stratosphere)
    Temperature = -56.46;
    Pressure = 22.65 * exp(1.73 - .000157 * Temperature);
end

row = Pressure / (0.2869 * (Temperature + 273.1));

% weight of the fuel in N
W_fuel_full = (Fuel_L/1000)*Fuel_density*P.gravity;

% Calculate some properties of the UAV
AR = P.b/P.c;
K = 1 / (pi*P.e*AR);

Prop_R = sqrt(P.Sprop/pi);
Prop_diam = 2*Prop_R;

% Build the curve for the Fuel consumption

fuel_comsump_P = fuel_comsump_P_kg * P.gravity;
Fuel_comsump_curve = fit(Data_fuel_Power,fuel_comsump_P,'poly1'); % use 'power1' for a exp^x fit

if verbose_graph
    figure
    plot(Fuel_comsump_curve)
    xlim([1 2800])
    ylim([0 8])
    hold on
    plot(Data_fuel_Power,fuel_comsump_P, 'bo')
    title('Fuel comsumption vs. Power output of the Engine')
    xlabel('Engine Output Power (W)')
    ylabel('Fuel comsumption (N/hr)')
    legend('Fitted Curve', 'Provided Data')
    grid on
end

% build curves of best fit for the prop data
Num_RPMS = length(P.RPM_values);
syms J_variable CT_variable 'real'

for i = 1:Num_RPMS

    CT_J_Curve{i} = fit(P.Props{i}(:,1), P.Props{i}(:,2),'poly2');
    CT_J_Curve_f{i}(J_variable) = CT_J_Curve{i}.p1*(J_variable)^2 + CT_J_Curve{i}.p2*(J_variable) + CT_J_Curve{i}.p3;
 
    CP_J_Curve{i} = fit(P.Props{i}(:,1), P.Props{i}(:,3),'exp2');
    CP_J_Curve_f{i}(J_variable) = CP_J_Curve{i}.a*exp(CP_J_Curve{i}.b*J_variable) + CP_J_Curve{i}.c*exp(CP_J_Curve{i}.d*J_variable);

end

%%
% Setup initial guess curves

init_guess_prop_data = 3;
title_RPM = P.RPM_values(init_guess_prop_data);

Init_CT_J_Curve = CT_J_Curve{init_guess_prop_data};
Init_CT_J_Curve_f(J_variable) = CT_J_Curve_f{init_guess_prop_data};

if verbose_graph
    figure
    plot(P.Props{init_guess_prop_data}(:,1), P.Props{init_guess_prop_data}(:,2),'bo')
    xlim([0 0.8])
    ylim([0 0.1])
    hold on
    plot(Init_CT_J_Curve)
    title("CT vs J curve for the prop at " + title_RPM + " RPM")
    xlabel('J')
    ylabel('CT')
    legend('Provided Data', 'Fitted Curve')
end

Init_CP_J_Curve = CP_J_Curve{init_guess_prop_data};
Init_CP_J_Curve_f(J_variable) = CP_J_Curve_f{init_guess_prop_data};

if verbose_graph
    figure
    plot(P.Props{init_guess_prop_data}(:,1), P.Props{init_guess_prop_data}(:,3),'bo')
    xlim([0 1.0])
    ylim([0 0.035])
    hold on
    plot(Init_CP_J_Curve)
    title("CP vs J curve for the prop at " + title_RPM + " RPM")
    xlabel('J')
    ylabel('CP')
    legend('Provided Data', 'Fitted Curve')
end

%%
% Loop through the data and calculate

% Fuel weights to try
W_fuel = W_fuel_full:-W_fuel_step:0;   
Weight = (P.m * P.gravity) + W_fuel;
Num_W = length(W_fuel);

Velocity_limit_min = 14;     % Min velocity to test at, 14 works well
Velocity_limit_max = 23;     % Max velocity to test at, 23 works well

Velocities = Velocity_limit_min:Velocity_step:Velocity_limit_max;   % Velocities to test at
Num_V = length(Velocities);

%initialize several arrays
min_Fuel_consump_E = double.empty(Num_W,0);
min_Fuel_consump_R = double.empty(Num_W,0);

min_Fuel_velocity_E = double.empty(Num_W,0);
min_Fuel_velocity_R = double.empty(Num_W,0);

Power_E = double.empty(Num_W,0);
Power_R = double.empty(Num_W,0);

Thrust_E = double.empty(Num_W,0);
Thrust_R = double.empty(Num_W,0);

RPM_E = double.empty(Num_W,0);
RPM_R = double.empty(Num_W,0);

for w = 1:Num_W
    
    clc
    fprintf(' The current loop is: %d out of %d ',w,Num_W); 
    

    Thrust_Req = double.empty(Num_V,0);
    Prop_RPM = double.empty(Num_V,0);
    Prop_P = double.empty(Num_V,0);
    Fuel_consump_V = double.empty(Num_V,0);
    Fuel_consump_perV_V = double.empty(Num_V,0);

    for v = 1:Num_V

        % calculate the required thrust to fly at desired speed
        q = (1/2)*row*(Velocities(v))^2;
        CL = Weight(w) / (q*P.S);
        CD = P.C_Do + K*CL^2;
        Thrust_Req(v) = Weight(w) / (CL / CD);

        % Find an inital estimate for J -> RPM to know which table is best

        init_eqn1 = [CT_variable == Init_CT_J_Curve_f(J_variable)];
        init_eqn2 = [CT_variable == Thrust_Req(v) / (row * (Velocities(v) / (Prop_diam * (J_variable)))^2 * Prop_diam^4)];
        init_eqns = [init_eqn1(1) init_eqn2(1)];
        vars = [CT_variable J_variable]; 

        [Init_Ct_temp Init_J_temp] = vpasolve(init_eqns,vars,[0.05; 0.5]);
        Init_J = double(max(Init_J_temp));

        Rev_sec = Velocities(v) / (Prop_diam * Init_J);
        Prop_RPM_init = Rev_sec*60;
        
        min_diff_RPM = 1/0;

        for i = 1:Num_RPMS  
                        
            diff_RPM_chart = abs(Prop_RPM_init - P.RPM_values(i));
            
            if diff_RPM_chart < min_diff_RPM 
                min_diff_RPM = diff_RPM_chart;
                Best_table = i;                      
            end  
        end

        % now using the best table, recalculate for the CT, J, and Power

        Num_J = length(P.Props{Best_table}(:,1));
               
        eqn1 = [CT_variable == CT_J_Curve_f{Best_table}(J_variable)];
        eqn2 = [CT_variable == Thrust_Req(v) / (row * (Velocities(v) / (Prop_diam * (J_variable)))^2 * Prop_diam^4)];
        eqns = [eqn1(1) eqn2(1)];
        vars = [CT_variable J_variable]; 

        [Actual_Ct_temp, Actual_J_temp] = vpasolve(eqns,vars,[0.05; 0.5]);

        Actual_J = double(max(Actual_J_temp));

        Rev_sec = Velocities(v) / (Prop_diam * Actual_J);
        Prop_RPM_temp = Rev_sec*60;
      
        CP_temp = CP_J_Curve_f{Best_table}(Actual_J);
        P_temp = CP_temp * row*Rev_sec^3*Prop_diam^5;

        Prop_RPM(v) = Prop_RPM_temp;
        Prop_P(v) = P_temp;

        Fuel_consump_V(v) = Fuel_comsump_curve(Prop_P(v));
        Fuel_consump_perV_V(v) = Fuel_consump_V(v) / Velocities(v);

    end
    
    % find the velocity that gives best endurance based on the descrete chart
    [Best_E_fuelrate_approx Best_E_I] = min(Fuel_consump_V);
    
    % find the velocity that gives best range based on the descrete chart
    [Best_R_fuelrate_V_approx Best_R_I] = min(Fuel_consump_perV_V);
    Best_R_fuelrate_approx = Fuel_consump_V(Best_R_I);
    
    % find the velocity that gives best endurance based on curve fitting the
    % bottom 3 points and making a continous function
    V_values_E = [Velocities(Best_E_I-1); Velocities(Best_E_I); Velocities(Best_E_I+1)];
    fuelrate_values_E = [Fuel_consump_V(Best_E_I-1); Fuel_consump_V(Best_E_I); Fuel_consump_V(Best_E_I+1)];
    fuelrate_Curve_E = fit(V_values_E,fuelrate_values_E,'poly2');    
    [Best_E_velocity , Best_E_fuelrate] = fminbnd(fuelrate_Curve_E,V_values_E(1),V_values_E(3));
    
    min_Fuel_consump_E(w) = Best_E_fuelrate;
    min_Fuel_velocity_E(w) = Best_E_velocity;
    
    % find the velocity that gives best range based on curve fitting the
    % bottom 3 points and making a continous function
    V_values_R = [Velocities(Best_R_I-1); Velocities(Best_R_I); Velocities(Best_R_I+1)];
    fuelrate_values_R = [Fuel_consump_V(Best_R_I-1); Fuel_consump_V(Best_R_I); Fuel_consump_V(Best_R_I+1)];
    fuelrate_Curve_R = fit(V_values_R,fuelrate_values_R,'poly2');    
    [Best_R_velocity , Best_R_fuelrate] = fminbnd(fuelrate_Curve_R,V_values_R(1),V_values_R(3));
    
    min_Fuel_consump_R(w) = Best_R_fuelrate;
    min_Fuel_velocity_R(w) = Best_R_velocity;
    
    % store the propeller power
    Power_E(w) = Prop_P(Best_E_I);
    Power_R(w) = Prop_P(Best_R_I);
    
    % store thrust
    Thrust_E(w) = Thrust_Req(Best_E_I);
    Thrust_R(w) = Thrust_Req(Best_R_I);

    %store the RPM
    RPM_E(w) = Prop_RPM(Best_E_I);
    RPM_R(w) = Prop_RPM(Best_R_I);

    if verbose_graph && w == 1
        
        % plot the curves at the inital weight
        figure
        plot(Velocities, Prop_P,'b-o')
        title('Power Required vs. Velocity for initial weight')
        xlabel('Velocity (m/s)')
        ylabel('Power (W)')
        grid on

        figure
        plot(Velocities, Fuel_consump_V,'b-o')
        title('Fuel comsumption vs. Velocity for initial weight')
        xlabel('Velocity (m/s)')
        ylabel('Fuel comsumption (N/hr)')
        grid on

        figure
        plot(Velocities, Fuel_consump_perV_V,'b-o')
        title('Fuel comsumption over Velocity vs. Velocity for initial weight')
        xlabel('Velocity (m/s)')
        ylabel('Fuel comsumption over Velocity (N/hr)/(m/s)')
        grid on

        figure
        plot(Velocities, Thrust_Req,'b-o')
        title('Thrust Required vs. Velocity for initial weight')
        xlabel('Velocity (m/s)')
        ylabel('Thrust Required (N)')
        grid on

        figure
        plot(Velocities, Prop_RPM,'b-o')
        title('Propeller RPM vs. Velocity for initial weight')
        xlabel('Velocity (m/s)')
        ylabel('Propeller angular velocity (RPM)')
        grid on
    
    end
    
end

% build a temporary weight vector, used for debugging when length of weight
% doesn't equal the lenght of other arrays
weight_temp = double.empty(length(min_Fuel_consump_R), 0);
for i = 1:length(min_Fuel_consump_R)
    weight_temp(i) = Weight(i);
end

% Build the range equation
Range_eqn = min_Fuel_velocity_R./(min_Fuel_consump_R./3600);
% Do trapezoidal numerical integration over the weight
Range = -trapz(weight_temp, Range_eqn, 2); 

% Build the endurance equation
Endurance_eqn = 1./(min_Fuel_consump_E./3600);
% Do trapezoidal numerical integration over the weight
Endurance = -trapz(weight_temp, Endurance_eqn, 2); 

clc
fprintf(' The loop is now complete \n'); 

fprintf('\n')
fprintf(' The max range is: %d (m) or %d (km)\n',Range,Range/1000); 
fprintf(' The max endurance is: %d (sec) or %d (hr)\n',Endurance,Endurance/3600); 

%%
% Provide extra info

if verbose
    
    % print extra info about the plane
    fprintf('\n'); 
    
    avg_fuel_rate_R = mean(min_Fuel_consump_R);
    avg_fuel_rate_E = mean(min_Fuel_consump_E);
    fprintf(' The average fuel consumption for max range is: %d (N/hr) and for max endurance is %d (N/hr)\n',avg_fuel_rate_R,avg_fuel_rate_E); 
    fprintf(' The average fuel consumption for max range is: %d (kg/hr) and for max endurance is %d (kg/hr)\n', avg_fuel_rate_R/P.gravity, avg_fuel_rate_E/P.gravity); 
    
    avg_velocity_R = mean(min_Fuel_velocity_R);
    avg_velocity_E = mean(min_Fuel_velocity_E);
    fprintf(' The average velocity for max range is: %d (m/s) and for max endurance is %d (m/s)\n',avg_velocity_R,avg_velocity_E); 
    
    avg_power_R = mean(Power_R);
    avg_power_E = mean(Power_E);
    fprintf(' The average power for max range is: %d (W) and for max endurance is %d (W)\n',avg_power_R,avg_power_E); 

    avg_thrust_R = mean(Thrust_E);
    avg_thrust_E = mean(Thrust_R);
    fprintf(' The average thrust for max range is: %d (N) and for max endurance is %d (N)\n',avg_thrust_R,avg_thrust_E); 

    avg_RPM_R = mean(RPM_E);
    avg_RPM_E = mean(RPM_R);
    fprintf(' The average RPM for max range is: %d (RPM) and for max endurance is %d (RPM)\n',avg_RPM_R,avg_RPM_E); 

end

if verbose_graph
    
    figure
    plot(weight_temp, min_Fuel_consump_R,'r-o')
    hold on
    plot(weight_temp, min_Fuel_consump_E,'b-o')
    title('Fuel consumption vs. Weight')
    xlabel('Weight (N)')
    ylabel('Fuel consumption (N/hr)')
    legend('Max Range', 'Max Endurance')
    grid on
    
    figure
    plot(weight_temp, min_Fuel_velocity_R,'r-o')
    hold on
    plot(weight_temp, min_Fuel_velocity_E,'b-o')
    title('Velocity vs. Weight')
    xlabel('Weight (N)')
    ylabel('Velocity (m/s)')
    legend('Max Range', 'Max Endurance')
    grid on
    
    figure
    plot(weight_temp, Power_R,'r-o')
    hold on
    plot(weight_temp, Power_E,'b-o')
    title('Power vs. Weight')
    xlabel('Weight (N)')
    ylabel('Power (W)')
    legend('Max Range', 'Max Endurance')
    grid on
    
    figure
    plot(weight_temp, Thrust_R,'r-o')
    hold on
    plot(weight_temp, Thrust_E,'b-o')
    title('Thrust vs. Weight')
    xlabel('Weight (N)')
    ylabel('Thrust (N)')
    legend('Max Range', 'Max Endurance')
    grid on
    
    figure
    plot(weight_temp, RPM_R,'r-o')
    hold on
    plot(weight_temp, RPM_E,'b-o')
    title('Prop RPM vs. Weight')
    xlabel('Weight (N)')
    ylabel('Prop angular velocity (RPM)')
    legend('Max Range', 'Max Endurance')
    grid on
    
end


if verbose_minP_T
    
    % used for comparison
    fprintf('\n');
    
    CL_TRmin = sqrt(P.C_Do / K);
    V_TRmin = sqrt( (2 *(Weight(1)/P.S)) / (row * CL_TRmin) );
    CD_TRmin = P.C_Do + K*(CL_TRmin)^2;
    D_TRmin = (1/2)*row*(V_TRmin)^2 * P.S*CD_TRmin;
    T_TRmin = D_TRmin;
    fprintf(' The minimum thrust required is: %d (N) which occurs when the speed is %d (m/s)\n',T_TRmin,V_TRmin); 

    CL_Pmin = sqrt(3*P.C_Do / K);
    V_Pmin = sqrt( (2 *(Weight(1)/P.S)) / (row * CL_Pmin) );
    CD_Pmin = P.C_Do + K*(CL_Pmin)^2;
    D_Pmin = (1/2)*row*(V_Pmin)^2 * P.S*CD_Pmin;
    T_Pmin = D_Pmin;
    fprintf(' The minimum power required is: %d (N) which occurs when the speed is %d (m/s)\n',T_Pmin,V_Pmin); 
end
