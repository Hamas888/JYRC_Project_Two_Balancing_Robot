clc
clear
close all
%% Defining Parameters
mw=2;                  % Mass of the wheel base
mp=1;                  % Mass of the pendulum body
g =9.8;                % Gravity
L =1;                  % Length of pendulum body
d1=0.01;               % Damping of wheel base displacement
d2=0.01;               % Damping in the pendulum body and wheel base joint
%% Defining Matrices
A=[0,                  0,            1,                          0;
   0,                  0,            0,                          1;
   0,        (g*mp)/(mw),   (-d1)/(mw),               (-d2)/(L*mw);
   0, (g*(mw+mp))/(L*mw), (-d1)/(L*mw), (-d2*mw-d2*mp)/(L^2*mw*mp)];

B=[0 ; 0 ; 1/(mw) ; 1/(L*mw)];

%% Defining Output
C=[1 ; 0 ; 0 ; 0]; %q1 output using wheels to balance
D=0;

%% Building System
sys=ss(A,B,C',D);
%('eigen_values') 
eig(A);
%('poles')
pole(sys);
sc=ctrb(sys);
so=obsv(sys);
%rlocus(sys);
%% Defining Initial Condition
x0=[0 ; 5*pi/180 ; 0 ; 0]; %initial condition for intigrators
%% Designing Controller
des_pole=[-3 ; -3 ; -3 ; -3];
k= acker(A,B,des_pole);
Q1= 10*eye(4);
R1=0.1;
k_lqr = lqr(A,B,Q1,R1);
%% Discrete Time Controller
Ts =0.1;
sys_d=c2d(sys,Ts);
Ad=sys_d.a;
Bd=sys_d.b;
Cd=sys_d.c;
Q=diag([1,10,1,1]);
R=0.02;
N=zeros(length(B),1);
[k_d,~,~]=dlqr(Ad,Bd,Q,R,N);
[Ob,~,~]=dlqr(Ad',Cd',Q,R,N);
















