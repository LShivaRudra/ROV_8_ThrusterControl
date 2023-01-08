clc;
clear all;
close all;
syms x y z phi u v w p q r theta psi t1 t2 t3 t4 t5 t6 real;
syms s complex
Tau=[t1;t2;t3;t4;t5;t6];
vb=[u;v;w;p;q;r];
%-----------------------------------------------------------------------------------------------------------------------------
eta=[x;y;z;phi;theta;psi];
eta_lap=laplace(eta);
eta_dot=ilaplace(s*eta_lap);

R1=[cos(psi)*cos(theta),-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi),sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
    sin(psi)*cos(theta),cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi),-cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
    -sin(theta),cos(theta)*sin(phi),cos(theta)*cos(phi)];

T=[1,sin(phi)*tan(theta),cos(phi)*tan(theta);
    0,cos(phi),-sin(phi);
    0,sin(phi)/cos(theta),cos(phi)/cos(theta)];

J=[R1,zeros(3,3);zeros(3,3),T];
% J_lap=laplace(J);
% J_diff=ilaplace(s*J_lap);

%vb=inv(J)*eta_dot;

%-----------------------------------------------------------------------------------------------------------------------------

Ix=0.16;
Iy=0.16;
Iz=0.16;
zg=0.02;
m=11.5;
W=112.8;
% Buo=114.8;
Buo=150;
% load('WBparams.mat')
M=[m,0,0,0,m*zg,0;
    0,m,0,-m*zg,0,0;
    0,0,m,0,0,0;
    0,-m*zg,0,Ix,0,0;
    m*zg,0,0,0,Iy,0;
    0,0,0,0,0,Iz
    ];

CRB=[0,0,0,0,m*w,0;
    0,0,0,-m*w,0,0;
    0,0,0,m*v,-m*u,0;
    0,m*w,-m*v,0,Iz*r,-Iy*q;
    -m*w,0,-m*u,-Iz*r,0,Ix*p;
    m*v,-m*u,0,Iy*q,-Ix*p,0];
CA=[0,0,0,0,-14.57*w,0;
    0,0,0,14.57*w,0,5.5*u;
    0,0,0,12.7*v,-5.5*u,0;
    0,14.57*w,-12.7*v,0,0.12*r,-0.12*q;
    -14.57*w,0,5.5*u,-0.12*r,0,0.12*p;
    12.57*v,-5.5*u,0,0.12*q,-0.12*p,0
    ];
C=CRB+CA;
D=diag([4.03+18.18*abs(u),6.22+21.66*abs(v),5.18+36.99*abs(w),0.07+1.55*abs(p),0.07+1.55*abs(q),0.07+1.55*abs(r)]);
g=[(W-Buo)*sin(theta);-(W-Buo)*cos(theta)*sin(phi);-(W-Buo)*cos(theta)*cos(phi);zg*W*cos(theta)*sin(phi);zg*W*sin(theta);0];
%-----------------------------------------------------------------------------------------------------------------------------

% s1_dot=eta_dot;
% s2_dot=J*inv(M)*(Tau-g-M*J_diff*eta_dot-C*inv(J)*eta_dot-D*inv(J)*eta_dot);
% 
% A1=jacobian(s1_dot,eta)
% A2=jacobian(s1_dot,)
% B=jacobian(states_dot,Tau)
s1=eta;
s2=vb;
s1_dot=J*vb;
s2_dot=inv(M)*(Tau-C*vb-D*vb-g);
states=[s1;s2];
states_dot=[s1_dot;s2_dot];

A=jacobian(states_dot,states);
B=jacobian(states_dot,Tau);
A = subs(A,[x y z phi theta psi u v w p q r],zeros(1,12));
B = subs(B,[x y z phi theta psi u v w p q r],zeros(1,12));
A = double(A);
B = double(B);

% Q=1000*eye(12);
Q=1000*eye(12);
%R=0.9*eye(6)+0.1*ones(6,6);
% R=0.99*eye(6)+0.01*ones(6,6);

% R=[0.1 0.01 0.01 0.01 0.01 0.01;
%     0.01 0.1 0.01 0.01 0.01 0.01;
%     0.01 0.01 0.1 0.01 0.01 0.01;
%     0.01 0.01 0.01 0.1 0.01 0.01;
%     0.01 0.01 0.01 0.01 0.1 0.01;
%     0.01 0.01 0.01 0.01 0.01 0.1;]
%smaller R value, smaller the control cost and faster the states are
%reached

R=1.2*eye(6)+0.8*ones(6)
%higher R value, higher the control cost and slower the states will be
%reached

[K,S,P]=lqr(A,B,Q,R);
save('LQR_vars.mat','A','B','K','S','P','Q','R');
clear all;
load('LQR_vars.mat');
sim('LQR_1.slx');

C_=eye(12);
D_=zeros(12,6);
figure;
subplot(2,1,1);
pzmap(ss(A-B*K,zeros(12,6),C_-D_*K,zeros(12,6)));
title('Pole-zero response of closed-loop system')

subplot(2,1,2);
pzmap(ss(A,B,C_,D_));
title('Pole-zero response of open-loop system')