function aircraft_dynamics(block)
%MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl_basic' with the 
%   name of your S-function.
%
%   It should be noted that the MATLAB S-function is very similar
%   to Level-2 C-Mex S-functions. You should be able to get more
%   information for each of the block methods by referring to the
%   documentation for C-Mex S-functions.
%
%   Copyright 2003-2010 The MathWorks, Inc.

% AER1216 Fall 2021 
% Fixed Wing Project Code
%
% aircraft_dynamics.m
%
% Fixed wing simulation model file, based on the Aerosonde UAV, with code
% structure adapted from Small Unmanned Aircraft: Theory and Practice by 
% R.W. Beard and T. W. McLain. 
% 
% Inputs: 
% delta_e           elevator deflection [deg]
% delta_a           aileron deflection [deg]
% delta_r           rudder deflection [deg]
% delta_t           normalized thrust []
%
% Outputs:
% pn                inertial frame x (north) position [m]
% pe                inertial frame y (east) position [m]
% pd                inertial frame z (down) position [m]
% u                 body frame x velocity [m/s]
% v                 body frame y velocity [m/s]
% w                 body frame z velocity [m/s]
% phi               roll angle [rad]
% theta             pitch angle [rad]
% psi               yaw angle [rad]
% p                 roll rate [rad/s]
% q                 pitch rate [rad/s]
% r                 yaw rate [rad/s]
%
% Last updated: Pravin Wedage 2021-11-09

%% TA NOTE
% The code segements you must modify are located in the derivatives
% function in this .m file. Modify other sections at your own risk. 


%
% The setup method is used to set up the basic attributes of the
% S-function such as ports, parameters, etc. Do not add any other
% calls to the main body of the function.
%
setup(block);

end 


%% Function: setup ===================================================
% Abstract:
%   Set up the basic characteristics of the S-function block such as:
%   - Input ports
%   - Output ports
%   - Dialog parameters
%   - Options
%
%   Required         : Yes
%   C-Mex counterpart: mdlInitializeSizes
%
function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
for i = 1:block.NumInputPorts
    block.InputPort(i).Dimensions        = 4;
    block.InputPort(i).DatatypeID  = 0;  % double
    block.InputPort(i).Complexity  = 'Real';
    block.InputPort(i).DirectFeedthrough = false; % important to be false 
end

% Override output port properties
for i = 1:block.NumOutputPorts
    block.OutputPort(i).Dimensions       = 12;
    block.OutputPort(i).DatatypeID  = 0; % double
    block.OutputPort(i).Complexity  = 'Real';
%     block.OutputPort(i).SamplingMode = 'Sample';
end

% Register parameters
block.NumDialogPrms     = 1;
P = block.DialogPrm(1).Data; % must duplicate this line in each function

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Register multiple instances allowable
% block.SupportMultipleExecInstances = true;

% Register number of continuous states
block.NumContStates = 12;

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

% -----------------------------------------------------------------
% The MATLAB S-function uses an internal registry for all
% block methods. You should register all relevant methods
% (optional and required) as illustrated below. You may choose
% any suitable name for the methods and implement these methods
% as local functions within the same file. See comments
% provided for each function for more information.
% -----------------------------------------------------------------

% block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup); % discrete states only
block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
block.RegBlockMethod('InitializeConditions',    @InitializeConditions);
% block.RegBlockMethod('Start',                   @Start); % Initialize Conditions is used
block.RegBlockMethod('Outputs',                 @Outputs); % Required
% block.RegBlockMethod('Update',                  @Update); % only required for discrete states
block.RegBlockMethod('Derivatives',             @Derivatives); % Required for continuous states
block.RegBlockMethod('Terminate',               @Terminate); % Required

end 


%% PostPropagationSetup:
%   Functionality    : Setup work areas and state variables. Can
%                      also register run-time methods here
%   Required         : No
%   C-Mex counterpart: mdlSetWorkWidths
%
function DoPostPropSetup(block)
block.NumDworks = 1;
  
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;

end


%% InitializeConditions:
%   Functionality    : Called at the start of simulation and if it is 
%                      present in an enabled subsystem configured to reset 
%                      states, it will be called when the enabled subsystem
%                      restarts execution to reset the states.
%   Required         : No
%   C-MEX counterpart: mdlInitializeConditions
%
function InitializeConditions(block)

% Rename parameters
P = block.DialogPrm(1).Data; % must duplicate this line in each function

% Initialize continuous states
block.ContStates.Data(1) = P.pn0; 
block.ContStates.Data(2) = P.pe0;
block.ContStates.Data(3) = P.pd0;
block.ContStates.Data(4) = P.u0;
block.ContStates.Data(5) = P.v0;
block.ContStates.Data(6) = P.w0;
block.ContStates.Data(7) = P.phi0;
block.ContStates.Data(8) = P.theta0;
block.ContStates.Data(9) = P.psi0;
block.ContStates.Data(10) = P.p0;
block.ContStates.Data(11) = P.q0;
block.ContStates.Data(12) = P.r0;

end 

%% Start:
%   Functionality    : Called once at start of model execution. If you
%                      have states that should be initialized once, this 
%                      is the place to do it.
%   Required         : No
%   C-MEX counterpart: mdlStart
%
function Start(block)

block.Dwork(1).Data = 0;

end 

%% Input Port Sampling Method:
function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = 'Sample';
  for i = 1:block.NumOutputPorts
    block.OutputPort(i).SamplingMode  = 'Sample';   
  end
end

%% Outputs:
%   Functionality    : Called to generate block outputs in
%                      simulation step
%   Required         : Yes
%   C-MEX counterpart: mdlOutputs
%
function Outputs(block)

temp_mat = zeros(block.NumContStates,1); % thirteen states
for i = 1:block.NumContStates
     temp_mat(i) = block.ContStates.Data(i);
end

block.OutputPort(1).Data = temp_mat; % states

% for i = 1:block.NumOutputPorts
%     block.OutputPort(1).Data(i) = block.ContStates.Data(i);
% end

end 


%% Update:
%   Functionality    : Called to update discrete states
%                      during simulation step
%   Required         : No
%   C-MEX counterpart: mdlUpdate
%
function Update(block)

block.Dwork(1).Data = block.InputPort(1).Data;

end 


%% Derivatives:
%   Functionality    : Called to update derivatives of
%                      continuous states during simulation step
%   Required         : No
%   C-MEX counterpart: mdlDerivatives
%
function Derivatives(block)

% Rename parameters

P = block.DialogPrm(1).Data; % must duplicate this line in each function

% Trim conditions and general data

Va0=P.Va0;
g=P.g;

% Aerosonde UAV Data

% physical parameters of airframe

m=P.m;
Ixx=P.Ixx;
Iyy=P.Iyy;
Izz=P.Izz;
Ixz=P.Ixz;
S=P.S;
b=P.b;
c=P.c;
Sprop=P.Sprop;
e=P.e;
Omega_max=P.OmegaMax_RPM; % rad/s
FuelCapacity=P.Fuel_Cap_L; % L

% Longitudinal

Cl0=P.C_Lo;
CD0=P.C_Do;
Cm0=P.C_mo;
Clalpha=P.C_Lalpha;
CDalpha=P.C_Dalpha;
Cmalpha=P.C_malpha;
Clq=P.C_Lq;
CDq=P.C_Dq;
Cmq=P.C_mq;
Cldelta_e=P.C_Ldelta_e;
CDdelta_e=P.C_Ddelta_e_;
Cmdelta_e=P.C_mdelta_e_;
epsilon=P.C_Epsilon;

% Lateral

CY0=P.C_Yo;
Cl0=P.C_lo;
Cn0=P.C_no;
CYbeta=P.C_Ybeta;
Clbeta=P.C_lbeta;
Cnbeta=P.C_nbeta;
CYp=P.C_Yp;
Clp=P.C_lp;
Cnp=P.C_np;
CYr=P.C_Yr;
Clr=P.C_lr;
Cnr=P.C_nr;
CYdelta_a=P.C_Ydelta_a;
Cldelta_a=P.C_ldelta_a;
Cndelta_a=P.C_ndelta_a;
CYdelta_r=P.C_Ydelta_r;
Cldelta_r=P.C_ldelta_r;
Cndelta_r=P.C_ndelta_r;

% compute inertial constants

K = Ixx*Izz-Ixz^2;
k1 = Ixz*(Ixx-Iyy+Izz)/K;
k2 = (Izz*(Izz-Iyy)+Ixz^2)/K;
k3 = Izz/K;
k4 = Ixz/K;
k5 = (Izz-Ixx)/Iyy;
k6 = Ixz/Iyy;
k7 = ((Ixx-Iyy)*Ixx+Ixz^2)/K;
k8 = Ixx/K;

% map states and inputs

pn    = block.ContStates.Data(1);
pe    = block.ContStates.Data(2);
pd    = block.ContStates.Data(3);
u     = block.ContStates.Data(4);
v     = block.ContStates.Data(5);
w     = block.ContStates.Data(6);
phi   = block.ContStates.Data(7);
theta = block.ContStates.Data(8);
psi   = block.ContStates.Data(9);
p     = block.ContStates.Data(10);
q     = block.ContStates.Data(11);
r     = block.ContStates.Data(12);
delta_e = block.InputPort(1).Data(1)*pi/180 ; % converted inputs to radians
delta_a = block.InputPort(1).Data(2)*pi/180 ; % converted inputs to radians
delta_r = block.InputPort(1).Data(3)*pi/180 ; % converted inputs to radians
delta_t = block.InputPort(1).Data(4);

%calculate density of air

Altitude=-pd;

if Altitude < 11000
    % Use this if below 11km (in the troposphere)
    Temperature = 15.04 - .00649 * Altitude;
    Pressure = 101.29 * [(Temperature + 273.1)/288.08]^5.256;       
else
    % Use this if between 11km and 25km (entering lower stratosphere)
    Temperature = -56.46;
    Pressure = 22.65 * exp(1.73 - .000157 * Temperature);
end

rho = Pressure / (0.2869 * (Temperature + 273.1));
% Air Data

Va = sqrt(u^2+v^2+w^2);
alpha = atan(w/u);
beta = asin(v/Va);

% rotation matrix

C_bv=[cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];

% aerodynamic coefficients 

% compute the nondimensional aerodynamic coefficients here

% aerodynamic forces and moments

% compute the aerodynamic forces and moments here

% Longitudinal aerodynamic forces and moments

F_lift=(1/2)*rho*Va^2*S*(Cl0+Clalpha*alpha+Clq*(c/(2*Va))*q+Cldelta_e*delta_e);
F_drag=(1/2)*rho*Va^2*S*(CD0+CDalpha*alpha+CDq*(c/(2*Va))*q+CDdelta_e*delta_e);

fx=-F_drag*cos(alpha)+F_lift*sin(alpha);
fz=-F_drag*sin(alpha)-F_lift*cos(alpha);

M=(1/2)*rho*Va^2*S*c*(Cm0+Cmalpha*alpha+Cmq*(c/(2*Va))*q+Cmdelta_e*delta_e);

% Lateral aerodynamic forces and moments

fy=(1/2)*rho*Va^2*S*(CY0+CYbeta*beta+CYp*(b/(2*Va))*p+CYr*(b/(2*Va))*r+CYdelta_a*delta_e+CYdelta_r*delta_r);
l=(1/2)*rho*Va^2*S*b*(Cl0+Clbeta*beta+Clp*(b/(2*Va))*p+Clr*(b/(2*Va))*r+Cldelta_a*delta_e+Cldelta_r*delta_r);
n=(1/2)*rho*Va^2*S*b*(Cn0+Cnbeta*beta+Cnp*(b/(2*Va))*p+Cnr*(b/(2*Va))*r+Cndelta_a*delta_a+Cndelta_r*delta_r);

% propulsion forces and moments

% compute the propulsion forces and moments here

Velocities = Va;

Prop_R = sqrt(P.Sprop/pi);
Prop_diam = 2*Prop_R;

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

q = (1/2)*rho*(Velocities)^2;
CL = Weight / (q*P.S);
CD = P.C_Do + K*CL^2;
Thrust_Req = Weight / (CL / CD);

eqn1 = [CT_variable == CT_J_Curve_f(J_variable)];
eqn2 = [CT_variable == Thrust_Req / (rho * (Velocities / (Prop_diam * (J_variable)))^2 * Prop_diam^4)];
eqns = [eqn1(1) eqn2(1)];
vars = [CT_variable J_variable]; 

[Init_Ct_temp, Init_J_temp] = vpasolve(eqns,vars,[0.05; 0.5]);
Init_J = double(max(Init_J_temp));
CT = double(max(Init_Ct_temp));

Rev_sec = Velocities / (Prop_diam * Init_J);
RPM = Rev_sec*60; 

CP = CP_J_Curve_f(Init_J);
Power = double(CP * rho*Rev_sec^3*Prop_diam^5);

fp=CT*rho*Rev_sec^2*Prop_diam^4;

CQ=CP/(2*pi);
Mp=CQ*rho*Rev_sec^2*Prop_diam^5;

% gravity

% compute the gravitational forces here

fg_v=[0; 0; m*g];
fg_b=C_bv*fg_v;

% total forces and moments (body frame)

f_total=fg_b+[fx; fy; fz]+[fp; 0; 0]

M_total=[l; m; n]+[Mp; 0; 0];

% state derivatives

% the full aircraft dynamics model is computed here

%pdot = ;    % extra?
pndot = (cos(theta)*cos(psi))*u+(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v+(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w;
pedot = (cos(theta)*sin(psi))*u+(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v+(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w;
pddot = -u*sin(theta)+v*sin(phi)*cos(theta)+w*cos(phi)*cos(theta);

udot = r*v-q*w+(1/m)*f_total(1);
vdot = p*w-r*u+(1/m)*f_total(2);
wdot = q*u-p*v+(1/m)*f_total(3);

phidot = p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
thetadot = q*cos(phi)-r*sin(phi);
psidot = q*sin(phi)*sec(theta)+r*cos(phi)*tan(theta);

pdot = k1*p*q-k2*q*r+k3*l+k4*n;
qdot = k5*p*r-k6*(p^2-r^2)+(1/Iyy)*m;
rdot = k7*p*q-k1*q*r+k4*l+k8*n;

% map derivatives

block.Derivatives.Data(1) = pndot;
block.Derivatives.Data(2) = pedot;
block.Derivatives.Data(3) = pddot;
block.Derivatives.Data(4) = udot;
block.Derivatives.Data(5) = vdot;
block.Derivatives.Data(6) = wdot;
block.Derivatives.Data(7) = phidot;
block.Derivatives.Data(8) = thetadot;
block.Derivatives.Data(9) = psidot;
block.Derivatives.Data(10)= pdot;
block.Derivatives.Data(11)= qdot;
block.Derivatives.Data(12)= rdot;

end 


%% Terminate:
%   Functionality    : Called at the end of simulation for cleanup
%   Required         : Yes
%   C-MEX counterpart: mdlTerminate
%
function Terminate(block)

end 

