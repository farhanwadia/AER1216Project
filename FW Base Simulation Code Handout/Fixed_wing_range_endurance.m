% AER1216 project
% David Rolko
% 1.1 (fixed wing) question 1

parameters

% for max range find best fuel comsumption / velocity per velocity
% for max endurance find best fuel consumption per velocity

% assume flight at sea level
row = 1.225;

W = P.m * P.gravity;
AR = P.b/P.c;
K = 1 / (pi*P.e*AR);

%%
CL_Pmin = sqrt(3*P.C_Do / K);
V_Pmin = sqrt( (2 *(W/P.S)) / (row * CL_Pmin) );

CD_Pmin = P.C_Do + K*(CL_Pmin)^2;
D_Pmin = (1/2)*row*(V_Pmin)^2 * P.S*CD_Pmin;
T_Pmin = D_Pmin;
%%

Velocity_limit_min = 1;     % Min velocity to test at
Velocity_limit_max = 20;    % Max velocity to test at
Velocity_step = 0.5;        % Step size velocity to test at

Velocities = Velocity_limit_min:Velocity_step:Velocity_limit_max;   % Velocties to test at

Num_V = length(Velocities);

for v = 1:Num_V
    
    q = (1/2)*row*(v)^2;
    CL = W / (q*P.S);
    CD = P.C_Do + K*CL^2;
    Thrust_R = W / (CL / CD);
    
    %CT = Thrust_R / 
    
    
end

eta = 0; % prop efficiency


