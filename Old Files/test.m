parameters

u=15;
v=0;
w=0;

h=1500;
Va = sqrt(u^2+v^2+w^2);
alpha = atan(w/u);
beta = asin(v/Va);
Velocities=Va;

if h < 11000
    % Use this if below 11km (in the troposphere)
    Temperature = 15.04 - .00649 * h;
    Pressure = 101.29 * [(Temperature + 273.1)/288.08]^5.256;       
else
    % Use this if between 11km and 25km (entering lower stratosphere)
    Temperature = -56.46;
    Pressure = 22.65 * exp(1.73 - .000157 * Temperature);
end

% propulsion forces and moments

density = Pressure / (0.2869 * (Temperature + 273.1));

Fuel_density = (775.0 + 840.0) / 2; % density of Jet A-1 fuel (kg/m^3)
%https://code7700.com/fuel_density.htm

Fuel_L = P.Fuel_Cap_L /2 ; % amount of fuel in L -> half a tank
W_fuel_half = (Fuel_L/1000)*Fuel_density*P.gravity; % weight of gas in N

Weight = (P.m * P.gravity) + W_fuel_half;

syms J_variable CT_variable 'real'

CT_J_Curve = fit(P.Props{3}(:,1), P.Props{3}(:,2),'poly2');
CT_J_Curve_f(J_variable) = CT_J_Curve.p1*(J_variable)^2 + CT_J_Curve.p2*(J_variable) + CT_J_Curve.p3;

CP_J_Curve = fit(P.Props{3}(:,1), P.Props{3}(:,3),'exp2');
CP_J_Curve_f(J_variable) = CP_J_Curve.a*exp(CP_J_Curve.b*J_variable) + CP_J_Curve.c*exp(CP_J_Curve.d*J_variable);

AR = P.b/P.c;
K = 1 / (pi*P.e*AR);

dyn_pressure2 = (1/2)*density*(Velocities)^2;
CL = Weight / (dyn_pressure2*P.S);
CD = P.C_Do + K*CL^2;
Thrust_Req = Weight / (CL / CD);

eqn1 = [CT_variable == CT_J_Curve_f(J_variable)];
eqn2 = [CT_variable == Thrust_Req / (density * (Velocities / (Prop_diam * (J_variable)))^2 * Prop_diam^4)];
eqns = [eqn1(1) eqn2(1)];
vars = [CT_variable J_variable]; 

[Init_Ct_temp Init_J_temp] = double(vpasolve(eqns,vars,[0.05; 0.5]));
Init_J = double(max(Init_J_temp));

Rev_sec = Velocities / (Prop_diam * Init_J);
RPM = Rev_sec*60; 

CP = CP_J_Curve_f(Init_J);
Power = CP * density*Rev_sec^3*Prop_diam^5;
Power = double(Power);

% compute the propulsion forces and moments here

fp=CT*density*Rev_sec^2*Prop_diam^4;

CQ=CP/(2*pi);
Mp=CQ*density*Rev_sec^2*Prop_diam^5;