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

pn0=P.pn0;
pe0=P.pe0;
pd0=P.pd0;
u0=P.u0;
v0=P.v0;
w0=P.w0;
phi0=P.phi0;
theta0=P.theta0;
psi0=P.psi0;
p0=P.p0;
q0=P.q0;
r0=P.r0;
delta_e0=P.delta_e0;
delta_a0=P.delta_a0;
delta_r0=P.delta_r0;
delta_t0=P.delta_t0;

% General Data

Va0=P.Va0;
g=P.g;
rho=1.225;
beta0=0;
alpha0=0;        
Prop_R = sqrt(P.Sprop/pi);
Prop_diam = 2*Prop_R;
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

% Compute CL and CD

Fuel_density = (775.0 + 840.0) / 2; % density of Jet A-1 fuel (kg/m^3)
%https://code7700.com/fuel_density.htm

Fuel_L = P.Fuel_Cap_L /2 ; % amount of fuel in L -> half a tank
W_fuel_half = (Fuel_L/1000)*Fuel_density*P.gravity; % weight of gas in N

Weight = (P.m * P.gravity) + W_fuel_half;

AR = P.b/P.c;
K = 1 / (pi*P.e*AR);

dyn_pressure = (1/2)*rho*(Va0)^2;
CL = Weight / (dyn_pressure*P.S);
CD = P.C_Do + K*CL^2;

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
K = ;
k1 = ;
k2 = ;
k3 = ;
k4 = ;
k5 = ;
k6 = ;
k7 = ;
k8 = ;

% Additional Coefficients

Cp0=k3*Cl0+k4*Cn0;
Cpbeta=k3*Clbeta+k4*Cnbeta;
Cpp=k3*Clp+k4*Cnp;
Cpr=k3*Clr+k4*Cnr;
Cpdelta_a=k3*Cldelta_a+k4*Cndelta_a;
Cpdelta_r=k3*Cldelta_r+k4*Cndelta_r;
Cr0=k4*Cl0+k8*Cn0;
Crbeta=k4*Clbeta+k8*Cnbeta;
Crp=k4*Clp+k8*Cnp;
Crr=k4*Clr+k8*Cnr;
Crdelta_a=k4*Cldelta_a+k8*Cndelta_a;
Crdelta_r=k4*Cldelta_r+k8*Cndelta_r;

CX0=-CD0*cos(alpha0)+CL*sin(alpha0);
CXalpha=0;  % ???
CXq=-CDq*cos(alpha0)+Clq*sin(alpha0);
CXdelta_e=-CDdelta_e*cos(alpha0)+Cldelta_e*sin(alpha0);
CZ0=-CD0*sin(alpha0)-CL*cos(alpha0);
CZalpha=0; % ???
CZq=-CDq*sin(alpha0)-Clq*cos(alpha0);
CZdelta_e=-CDdelta_e*sin(alpha0)-Cldelta_e*cos(alpha0);
        
% Longitudinal Aerodynamic Derivatives

Xu=(u0*rho*S)/m*(CX0+CXalpha*alpha0+CXdelta_e*delta_e0)-(rho*S*w0*CXalpha)/(2*m)+(rho*S*c*CXq*u0*q0)/(4*m*Va0)-(rho*Sprop*u0)/m;
Xw=-q0+(w0*rho*S)/m*(CX0+CXalpha*alpha0+CXdelta_e*delta_e0)+(rho*S*c*CXq*w0*q0)/(4*m*Va0)+(rho*S*CXalpha*u0)/(2*m)-(rho*Sprop*w0)/m;
Xq=-w0+(rho*Va0*S*CXq*c)/(4*m);
Xdelta_e=(rho*Va0^2*S*CXdelta_e)/(2*m);
Xdelta_t=(rho*Sprop*delta_t0^2)/m;

Zu=q0+(u0*rho*S)/m*(CZ0+CZalpha*alpha0+CZdelta_e*delta_e0)-(rho*S*w0*CZalpha)/(2*m)+(rho*S*c*CZq*u0*q0)/(4*m*Va0);
Zw=(w0*rho*S)/m*(CZ0+CZalpha*alpha0+CZdelta_e*delta_e0)+(rho*S*c*CZq*w0*q0)/(4*m*Va0)+(rho*S*CZalpha*u0)/(2*m);
Zq=u0+(rho*Va0*S*CZq*c)/(4*m);
Zdelta_e=(rho*Va0^2*S*CZdelta_e)/(2*m);

Mu=(u0*rho*S*c)/Iyy*(Cm0+Cmalpha*alpha0+Cmdelta_e*delta_e0)+(rho*S*c*Cmalpha*w0)/(2*Iyy)+(rho*S*c^2*Cmq*q0*u0)/(4*Iyy*Va0);
Mw=(w0*rho*S*c)/Iyy*(Cm0+Cmalpha*alpha0+Cmdelta_e*delta_e0)+(rho*S*c*Cmalpha*u0)/(2*Iyy)+(rho*S*c^2*Cmq*q0*w0)/(4*Iyy*Va0);
Mq=(rho*Va0*S*c^2*Cmq)/(4*Iyy);
Mdelta_e=(rho*Va0*2*S*c*Cmdelta_e)/(2*Iyy);

% Lateral Aerodynamic Derivatives

Yv=(rho*S*b*v0)/(4*m*Va0)*(CYp*p0+CYr*r0)+(rho*S*v0)/m*(CY0+CYbeta*beta0+CYdelta_a*delta_a0+CYdelta_r*delta_r0)+(rho*S*CYbeta)/(2*m)*sqrt(u0^2+w0^2);
Yp=w0+(rho*Va0*S*b)/(4*m)*CYp;
Yr=-u0+(rho*Va0*S*b)/(4*m)*CYdelta_r;
Ydelta_a=(rho*Va0^2*S)/(2*m)*CYdelta_a;
Ydelta_r=(rho*Va0^2*S)/(2*m)*CYdelta_r;

Lv=(rho*S*b^2*v0^2)/(4*Va0)*(Cpp*p0+Cpr*r0)+rho*S*b*v0*(Cp0+Cpbeta*beta0+Cpdelta_a*delta_a0+Cpdelta_r*delta_r0)+(rho*S*b*Cpbeta)/2*sqrt(u0^2+v0^2);
Lp=k1*q0+(rho*Va0*S*b^2)/4*Cpp;
Lr=-k2*q0+(rho*Va0*S*b^2)/4*Cpr;
Ldelta_a=(rho*Va0^2*S*b)/2*Cpdelta_a;
Ldelta_r=(rho*Va0^2*S*b)/2*Cpdelta_r;

Nv=(rho*S*b^2*v0^2)/(4*Va0)*(Crp*p0+Crr*r0)+rho*S*b*v0*(Cr0+Crbeta*beta0+Crdelta_a*delta_a0+Crdelta_r*delta_r0)+(rho*S*b*Crbeta)/2*sqrt(u0^2+v0^2);
Np=k7*q0+(rho*Va0*S*b^2)/4*Crp;
Nr=-k1*q0+(rho*Va0*S*b^2)/4*Crr;
Ndelta_a=(rho*Va0^2*S*b)/2*Crdelta_a;
Ndelta_r=(rho*Va0^2*S*b)/2*Crdelta_r;

% Longitudinal Model

A_long=[Xu Xw Xq -g*cos(theta0) 0;...
    Zu Zw Zq -g*sin(theta0) 0;...
    Mu Mw Mq 0 0;
    0 0 1 0 0;
    sin(theta0) -cos(theta0) 0 u0*cos(theta0)+w0*sin(theta0) 0];

B_long=[Xdelta_e 1; Zdelta_e 0; Mdelta_e 0; 0 0; 0 0];


% Lateral Model

A_latr=[Yv Yp Yr g*cos(theta0)*cos(phi0) 0;...
    Lv Lp Lr 0 0;...
    Nv Np Nr 0 0;...
    0 1 cos(phi0)*tan(theta0) q0*cos(phi0)*tan(theta0)-r0*sin(phi0)*tan(theta0) 0;...
    0 0 cos(phi0)*sec(theta0) p0*cos(phi0)*sec(theta0)-r0*sin(phi0)*sec(theta0) 0];

B_latr=[Ydelta_a Ydelta_r; Ldelta_a Ldelta_r; Ndelta_a Ndelta_r; 0 0; 0 0];

% Inputs and States

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
delta_e = block.InputPort(1).Data(1)*pi/180;  % converted inputs to radians
delta_a = block.InputPort(1).Data(2)*pi/180;  % converted inputs to radians
delta_r = block.InputPort(1).Data(3)*pi/180;  % converted inputs to radians
delta_t = block.InputPort(1).Data(4);
h=-pd;

% Full dynamics model

x_dot_long=A_long*[u; w; q; theta; h]+B_long*[delta_e; delta_t];
x_dot_latr=A_latr*[v; p; r; phi; psi]+B_latr*[delta_a; delta_r];

pndot = (cos(theta)*cos(psi))*u+(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v+(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w;
pedot = (cos(theta)*sin(psi))*u+(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v+(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w;
pddot = -x_dot_long(5,1);

udot = x_dot_long(1,1);
vdot = x_dot_latr(1,1);
wdot = x_dot_long(2,1);

phidot = x_dot_latr(4,1);
thetadot = x_dot_long(4,1);
psidot = x_dot_latr(5,1);

pdot = x_dot_latr(2,1);
qdot = x_dot_long(3,1);
rdot = x_dot_latr(3,1);

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

