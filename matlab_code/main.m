load balrob_params.mat;

f_c = 0.35; %Hertz 
T_c = 1/(2*pi*f_c); 
w_c =  2*pi*f_c;

C = Ts/(T_c + Ts);

% %% test 1
% x0(2) = 5*pi/180; % 5° initial tilt
% step = 0;
% dist = 0;

% %% test 2
% x0(2) = 0*pi/180; % 0° initial tilt
% step = 0.1/wheel.r;
% dist = 0;

%% test 3
x0(2) = 0*pi/180; % 0° initial tilt
step = 0.1/wheel.r;
dist = 115;

%% chain of integrators
C_11 = 0;
C_21 = 0;
C_22 = 0;
C_12 = -(body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*mot.rot.r;

%% State space model & discretization
F_v_11 = 2 *(gbox.B + wheel.B);
F_v_12 = -2*gbox.B;
F_v_21 = F_v_12;
F_v_22 = 2*gbox.B;
F_v = [ [F_v_11 F_v_12]; [F_v_21 F_v_22]];
Fv = F_v; %chain

Fv_prime = F_v + 2*gbox.N^2*mot.Kt*mot.Ke/(mot.R)*[ [1 -1]; [-1 1] ];

l = body.zb;

M_11 = 2*wheel.Iyy + 2*gbox.N^2*mot.rot.Iyy + (body.m + 2*wheel.m + 2*mot.rot.m)*mot.rot.r^2;
M_12 = 2*gbox.N*(1-gbox.N)*mot.rot.Iyy + (body.m*l + 2*mot.rot.m*mot.rot.zb)*mot.rot.r;
M_21 = M_12;
M_22 = body.Iyy + 2*(1-gbox.N)^2*mot.rot.Iyy + body.m*l^2 + 2*mot.rot.m*mot.rot.zb^2;
M = [ [M_11 M_12]; [M_21 M_22] ];

G_11 = 0;
G_12 = 0;
G_21 = 0;
G_22 = -(body.m*l + 2*mot.rot.m*mot.rot.zb)*g;
G = [ [G_11 G_12]; [G_21 G_22] ];

A = [ [zeros(2,2) eye(2)]; [-inv(M)*G -inv(M)*Fv_prime] ];
B = 2*gbox.N*mot.Kt/mot.R*[zeros(2,2);inv(M)]*[1;-1];
ss_cont = ss(A,B,eye(4),0);

ss_disc = c2d(ss_cont, Ts, 'zoh');

%% Nominal
[Phi, Gam] = ssdata(ss_disc);
H = [1,0,0,0];
gains = [ [Phi - eye(4) Gam]; [H 0] ] \ [zeros(4,1);1];
Nx = gains(1:4);
Nu = gains(5);

%% LQR Nominal
gamma_bar = pi/18;
theta_bar = pi/360;
u_bar = 1;
rho = [500 5000];

Q = diag([1/gamma_bar^2 1/theta_bar^2 0 0]);
r = 1/u_bar^2;
K = dlqr(Phi, Gam, Q, r*rho(1));
Nr = Nu + K*Nx;

%% Robust 
Phi_e = [ [1 H]; [zeros(4,1) Phi] ];
Gam_e = [0;Gam];
q_11 = [0.1 1];
Q_e = diag([q_11(1) Q(1,1) Q(2,2) 0 0]);
K_e = dlqr(Phi_e, Gam_e, Q_e, r*rho(1));
K_I = K_e(1);
K_e_state = K_e(2:5);
