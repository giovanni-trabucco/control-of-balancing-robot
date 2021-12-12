function sfun_balrob_long_dyn(block)

setup(block);

end %function


function setup(block)

% Register number of ports
block.NumInputPorts = 1;
block.NumOutputPorts = 2*3;

% Register number of continuous states
block.NumContStates = 2*2;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
for k = 1:block.NumInputPorts,
    block.InputPort(k).Dimensions = 1;
    block.InputPort(k).DatatypeID = 0; % double
    block.InputPort(k).Complexity = 'Real';
    block.InputPort(k).DirectFeedthrough = false;
end;

% Override output port properties
for k = 1:block.NumOutputPorts,
    block.OutputPort(k).Dimensions = 1;

    block.OutputPort(k).DatatypeID = 0; % double
    block.OutputPort(k).Complexity = 'Real';
end;

% Register parameters
block.NumDialogPrms = 5;

% Register sample times
% [0 offset] : Continuous sample time
block.SampleTimes = [0 0];

% Specify the block simStateCompliance.
% 'DefaultSimState', < Same sim state as a built´in block
block.SimStateCompliance = 'DefaultSimState';

% Register block methods
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('SetInputPortSamplingMode', @SetInputPortSamplingMode);
block.RegBlockMethod('Outputs', @Outputs); % Required
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required

for k = 1:block.NumOutputPorts,
    block.OutputPort(k).SamplingMode = 0;
end;

end % setup


function SetInputPortSamplingMode(block, port, mode)

block.InputPort(port).SamplingMode = mode;

end % SetInputPortSamplingMode


function Start(block)

%% Get init state

x0 = block.DialogPrm(1).Data; % get init state

%% Set init state

block.ContStates.Data(1) = x0(1); % gam(0)
block.ContStates.Data(2) = x0(2); % th(0)

block.ContStates.Data(3) = x0(3); % dot_gam(0)
block.ContStates.Data(4) = x0(4); % dot_th(0)

end % Start


function Outputs(block)

%% Extract state components

gam = block.ContStates.Data(1);
th = block.ContStates.Data(2);

dot_gam = block.ContStates.Data(3);
dot_th = block.ContStates.Data(4);


%% Get accelerations

[ddot_gam, ddot_th] = get_acc(block);

%% Set outputs

block.OutputPort(1).Data = gam;
block.OutputPort(2).Data = th;

block.OutputPort(3).Data = dot_gam;
block.OutputPort(4).Data = dot_th;

block.OutputPort(5).Data = ddot_gam;
block.OutputPort(6).Data = ddot_th;

end % Outputs


function Derivatives(block)

%% Extract state components

dot_gam = block.ContStates.Data(3);
dot_th = block.ContStates.Data(4);

%% Get accelerations

[ddot_gam, ddot_th] = get_acc(block);

%% Set state derivative

block.Derivatives.Data(1) = dot_gam;
block.Derivatives.Data(2) = dot_th;

block.Derivatives.Data(3) = ddot_gam;
block.Derivatives.Data(4) = ddot_th;

end % Derivatives


function Terminate(block)

end % Terminate


function [ddot_gam, ddot_th] = get_acc(block)

%% Get inputs and parameter structs

% parameters
body = block.DialogPrm(2).Data; % body data struct
mot = block.DialogPrm(3).Data; % mot data struct
gbox = block.DialogPrm(4).Data; % gbox data struct
wheel = block.DialogPrm(5).Data; % wheel data struct

% input voltages
ua = block.InputPort(1).Data; % armature voltage (right/left motor)

%% Extract state components

th = block.ContStates.Data(2);
dot_gam = block.ContStates.Data(3);
dot_th = block.ContStates.Data(4);


%% Extract params

% body params
l = body.zb;
mb = body.m;
Ibyy = body.Iyy;

% wheel params
w = 2*wheel.yb;
r = wheel.r;
mw = wheel.m;
Iwyy = wheel.Iyy;

% (motor) rotor params
zbrot = mot.rot.zb;
mrot = mot.rot.m;
Irotyy = mot.rot.Iyy;

% gear ratio
n = gbox.N;

% friction params
bw = wheel.B; % wheel viscous fric coeff
bm = mot.B; % motor viscous fric coeff (motor side)
bg = gbox.B; % gbox viscous fric coeff (load side)
b = n^2*bm+bg; % motor+gbox viscous fric coeff (load side)

% gravity acc
g = 9.81;

%% Get motor torques

% back´EMFs
ue = mot.Ke * n*(dot_gam-dot_th);

% motor torque (single motor)
tau = n*mot.Kt * (ua-ue)/mot.R;

%% Evaluate accelerations of generalised coords

% inertia matrix
MM = zeros(2,2);

MM(1,1) = 2*Iwyy + 2*Irotyy*n^2 + (mb + 2*(mrot+mw))*r^2;
MM(1,2) = 2*(1-n)*n*Irotyy + r*(l*mb + 2*mrot*zbrot)*cos(th);

MM(2,1) = MM(1,2);
MM(2,2) = Ibyy + 2*(1-n)^2*Irotyy + mb*l^2 + 2*mrot*zbrot^2;

% Coriolis + centrifugal terms matrix
CC = zeros(2,2);

CC(1,1) = 0;
CC(2,1) = 0;

CC(2,2) = 0;
CC(1,2) = -r*(mb*l + 2*mrot*zbrot)*sin(th)*dot_th;

% viscous friction matrix
Fv = zeros(2,2);

Fv(1,1) = 2*(b+bw);

Fv(1,2) = -2*b;

Fv(2,1) = Fv(1,2);
Fv(2,2) = 2*b;

% gravity loading
GG = zeros(2,1);
GG(2) = -g*(mb*l + 2*mrot*zbrot)*sin(th);

% generalized actuator forces
TT = zeros(2,1);
TT(1) = 2*tau;
TT(2) = -2*tau;

% get accelerations of generalised coords (q = [gam, th].')
dotq = [dot_gam, dot_th].';
ddotq = MM \ (TT - CC*dotq - Fv*dotq - GG);

ddot_gam = ddotq(1);
ddot_th = ddotq(2);

end % get_acc