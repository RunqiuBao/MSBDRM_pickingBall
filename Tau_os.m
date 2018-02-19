function [ tau ] = Tau_os( u )
%TAU Summary of this function goes here
%   Detailed explanation goes here
% Times are defined in TrajGen.m file
global t0 t1 t2 t3 t4 t5 t6

t=u(1);

x1d=u(2);
x2d=u(3);
x3d=u(4);
x4d=deg2rad(u(5));
x5d=deg2rad(u(6));
x6d=deg2rad(u(7));

x1dp=u(8);
x2dp=u(9);
x3dp=u(10);
x4dp=deg2rad(u(11));
x5dp=deg2rad(u(12));
x6dp=deg2rad(u(13));


q1=u(20);
q2=u(21);
q3=u(22);

q1p=u(23);
q2p=u(24);
q3p=u(25);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              q3p=u(25);

Q=[q1;q2;q3];
Qp=[q1p;q2p;q3p];

Xd=[x1d;x2d;x3d;x4d;x5d;x6d];
Xdp=[x1dp;x2dp;x3dp;x4dp;x5dp;x6dp];

% %Joint Errors
% 
% 
% %Robot Parameters
% 
m1=u(26);
m2=u(27);
m3=u(28);
g=u(29);

L1=u(30);
L2=u(31);
L4=u(32);
L6=u(33);
L7=u(34);
L9=u(35);
L3=u(36);
L5=u(37);
L8=u(38);
L10=u(39);
L11=u(40);
L12=0.001*115.7;
L13=0;
I111=0.04;
I112=0.02;
I113=0.02;
I122=0.04;
I123=0.02;
I133=0.04;

I211=0.04;
I212=0.02;
I213=0.02;
I222=0.04;
I223=0.02;
I233=0.04;

I311=0.04;
I312=0.02;
I313=0.02;
I322=0.04;
I323=0.02;
I333=0.04;

q1i=deg2rad(u(41));
q2i=deg2rad(u(42));
q3i=deg2rad(u(43));
q1di=deg2rad(u(50));
q2di=deg2rad(u(51));
q3di=deg2rad(u(52));
Qi=[q1i;q2i;q3i];
Qdi=[q1di;q2di;q3di];

Kis=diag([u(44);u(45);u(46)]);
Ki=diag([0;0;0]);

gx=u(47);
gy=u(48);
gz=u(49);

Kd=diag([u(59);u(60);u(61)]);%;u(62);u(63);u(64)
Kp=diag([u(65);u(66);u(67);u(68);u(69);u(70)]);

%robot jacobian
Jef=[ cos(q1)*(L2 + L4 - L7 + L13) + L7*cos(q1) + L3*cos(q2)*sin(q1) + cos(q2)*sin(q1)*cos(q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2) - sin(q3 + atan(L12/(L5 - L12)))*sin(q1)*sin(q2)*(L12^2 + (L5 - L12)^2)^(1/2), cos(q1)*(L3*sin(q2) + sin(q2 + q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2)), sin(q2 + q3 + atan(L12/(L5 - L12)))*cos(q1)*(L12^2 + (L5 - L12)^2)^(1/2);
    sin(q1)*(L2 + L4 - L7 + L13) + L7*sin(q1) - L3*cos(q1)*cos(q2) - cos(q1)*cos(q2)*cos(q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2) + sin(q3 + atan(L12/(L5 - L12)))*cos(q1)*sin(q2)*(L12^2 + (L5 - L12)^2)^(1/2), sin(q1)*(L3*sin(q2) + sin(q2 + q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2)), sin(q2 + q3 + atan(L12/(L5 - L12)))*sin(q1)*(L12^2 + (L5 - L12)^2)^(1/2);
    0,    - cos(q2 + q3 + atan(L12/(L5 - L12)))*(L5^2 - 2*L5*L12 + 2*L12^2)^(1/2) - L3*cos(q2),        -cos(q2 + q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2);
    0,                                                                                 sin(q1),                                                                  sin(q1);
    0,                                                                                -cos(q1),                                                                 -cos(q1);
    1,                                                                                       0,                                                                        0];
Jef
%current X
T3_0=[ cos(q2 + q3 + atan(L12/(L5 - L12)))*cos(q1), -sin(q2 + q3 + atan(L12/(L5 - L12)))*cos(q1),  sin(q1), sin(q1)*(L2 + L4 - L7 + L13) + L7*sin(q1) - L3*cos(q1)*cos(q2) - cos(q1)*cos(q2)*cos(q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2) + sin(q3 + atan(L12/(L5 - L12)))*cos(q1)*sin(q2)*(L12^2 + (L5 - L12)^2)^(1/2);
       cos(q2 + q3 + atan(L12/(L5 - L12)))*sin(q1), -sin(q2 + q3 + atan(L12/(L5 - L12)))*sin(q1), -cos(q1), sin(q3 + atan(L12/(L5 - L12)))*sin(q1)*sin(q2)*(L12^2 + (L5 - L12)^2)^(1/2) - L7*cos(q1) - L3*cos(q2)*sin(q1) - cos(q2)*sin(q1)*cos(q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2) - cos(q1)*(L2 + L4 - L7 + L13);
               sin(q2 + q3 + atan(L12/(L5 - L12))),          cos(q2 + q3 + atan(L12/(L5 - L12))),        0,                                                                                                                                         L1 - L3*sin(q2) - sin(q2 + q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2);
                                                 0,                                            0,        0,                                                                                                                                                                                                                          1];
[roll,pitch,yaw]=R2EulerA(T3_0(1:3,1:3));                                                      
X=[T3_0(1:3,4);roll;pitch;yaw];

%Xrp
Xrp=[0;0;0;0;0;0]-Kp*(X-Xd);
X
%Define controllers PD,PID
Jp=[                                                                                                                                                           - q1p*(L2*sin(q1) + L4*sin(q1) + L13*sin(q1) - L3*cos(q1)*cos(q2) - cos(q1)*cos(q2)*cos(q3 + atan(L12/(L5 - L12)))*(L5^2 - 2*L5*L12 + 2*L12^2)^(1/2) + sin(q3 + atan(L12/(L5 - L12)))*cos(q1)*sin(q2)*(L5^2 - 2*L5*L12 + 2*L12^2)^(1/2)) - q2p*sin(q1)*(sin(q2 + q3 + atan(L12/(L5 - L12)))*(L5^2 - 2*L5*L12 + 2*L12^2)^(1/2) + L3*sin(q2)) - q3p*sin(q2 + q3 + atan(L12/(L5 - L12)))*sin(q1)*(L5^2 - 2*L5*L12 + 2*L12^2)^(1/2), q2p*cos(q1)*(L3*cos(q2) + cos(q2 + q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2)) - q1p*sin(q1)*(L3*sin(q2) + sin(q2 + q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2)) + q3p*cos(q2 + q3 + atan(L12/(L5 - L12)))*cos(q1)*(L12^2 + (L5 - L12)^2)^(1/2), (L12^2 + (L5 - L12)^2)^(1/2)*(q2p*cos(q2 + q3 + atan(L12/(L5 - L12)))*cos(q1) + q3p*cos(q2 + q3 + atan(L12/(L5 - L12)))*cos(q1) - q1p*sin(q2 + q3 + atan(L12/(L5 - L12)))*sin(q1));
    q3p*(sin(q3 + atan(L12/(L5 - L12)))*cos(q1)*cos(q2)*(L12^2 + (L5 - L12)^2)^(1/2) + cos(q1)*sin(q2)*cos(q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2)) + q1p*(cos(q1)*(L2 + L4 - L7 + L13) + L7*cos(q1) + L3*cos(q2)*sin(q1) + cos(q2)*sin(q1)*cos(q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2) - sin(q3 + atan(L12/(L5 - L12)))*sin(q1)*sin(q2)*(L12^2 + (L5 - L12)^2)^(1/2)) + q2p*(L3*cos(q1)*sin(q2) + sin(q3 + atan(L12/(L5 - L12)))*cos(q1)*cos(q2)*(L12^2 + (L5 - L12)^2)^(1/2) + cos(q1)*sin(q2)*cos(q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2)), q2p*sin(q1)*(L3*cos(q2) + cos(q2 + q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2)) + q1p*cos(q1)*(L3*sin(q2) + sin(q2 + q3 + atan(L12/(L5 - L12)))*(L12^2 + (L5 - L12)^2)^(1/2)) + q3p*cos(q2 + q3 + atan(L12/(L5 - L12)))*sin(q1)*(L12^2 + (L5 - L12)^2)^(1/2), (L12^2 + (L5 - L12)^2)^(1/2)*(q1p*sin(q2 + q3 + atan(L12/(L5 - L12)))*cos(q1) + q2p*cos(q2 + q3 + atan(L12/(L5 - L12)))*sin(q1) + q3p*cos(q2 + q3 + atan(L12/(L5 - L12)))*sin(q1));
    0,                                                                                                     q2p*(sin(q2 + q3 + atan(L12/(L5 - L12)))*(L5^2 - 2*L5*L12 + 2*L12^2)^(1/2) + L3*sin(q2)) + q3p*sin(q2 + q3 + atan(L12/(L5 - L12)))*(L5^2 - 2*L5*L12 + 2*L12^2)^(1/2),                                                                                                       sin(q2 + q3 + atan(L12/(L5 - L12)))*(q2p + q3p)*(L12^2 + (L5 - L12)^2)^(1/2);
    0,                                                                                                                                                                                                                                                              q1p*cos(q1),                                                                                                                                                                        q1p*cos(q1);
    0,                                                                                                                                                                                                                                                              q1p*sin(q1),                                                                                                                                                                        q1p*sin(q1);
    0,                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                  0];

Xrp=Xdp-Kp*(X-Xd);
Jef_inv=pinv(Jef(1:3,:));                               
Qrp=Jef_inv*Xrp(1:3,1);
q1rp=Qrp(1);
q2rp=Qrp(2);
q3rp=Qrp(3);
Xrpp=[0;0;0]-Kp(1:3,1:3)*(Jef(1:3,:)*Qp-[0;0;0]);
Qrpp=Jef_inv*(Xrpp-Jp(1:3,:)*Qrp);
q1rpp=Qrpp(1);
q2rpp=Qrpp(2);
q3rpp=Qrpp(3);
EoMr = [I133*q1rpp + (I211*q1rpp)/2 + (I222*q1rpp)/2 + (I311*q1rpp)/2 + (I322*q1rpp)/2 - (I211*q1rpp*cos(2*q2))/2 + (I222*q1rpp*cos(2*q2))/2 + I212*q1rpp*sin(2*q2) - (I311*q1rpp*cos(2*q2 + 2*q3))/2 + (I322*q1rpp*cos(2*q2 + 2*q3))/2 + I312*q1rpp*sin(2*q2 + 2*q3) + I323*q2rpp*cos(q2 + q3) + I323*q3rpp*cos(q2 + q3) + I313*q2rpp*sin(q2 + q3) + I313*q3rpp*sin(q2 + q3) + (L3^2*m3*q1rpp)/2 + L7^2*m2*q1rpp + (L8^2*m2*q1rpp)/2 + L9^2*m3*q1rpp + (L10^2*m3*q1rpp)/2 + L11^2*m3*q1rpp + I223*q2rpp*cos(q2) + I213*q2rpp*sin(q2) + (L3^2*m3*q1rpp*cos(2*q2))/2 + (L8^2*m2*q1rpp*cos(2*q2))/2 + I313*q2p*q2rp*cos(q2 + q3) + I313*q2p*q3rp*cos(q2 + q3) + I313*q3p*q2rp*cos(q2 + q3) + I313*q3p*q3rp*cos(q2 + q3) - I323*q2p*q2rp*sin(q2 + q3) - I323*q2p*q3rp*sin(q2 + q3) - I323*q3p*q2rp*sin(q2 + q3) - I323*q3p*q3rp*sin(q2 + q3) + (L10^2*m3*q1rpp*cos(2*q2 + 2*q3))/2 + I213*q2p*q2rp*cos(q2) - I223*q2p*q2rp*sin(q2) + I212*q1p*q2rp*cos(2*q2) + I212*q2p*q1rp*cos(2*q2) + (I211*q1p*q2rp*sin(2*q2))/2 + (I211*q2p*q1rp*sin(2*q2))/2 - (I222*q1p*q2rp*sin(2*q2))/2 - (I222*q2p*q1rp*sin(2*q2))/2 + 2*L9*L11*m3*q1rpp + I312*q1p*q2rp*cos(2*q2 + 2*q3) + I312*q2p*q1rp*cos(2*q2 + 2*q3) + I312*q1p*q3rp*cos(2*q2 + 2*q3) + I312*q3p*q1rp*cos(2*q2 + 2*q3) + (I311*q1p*q2rp*sin(2*q2 + 2*q3))/2 + (I311*q2p*q1rp*sin(2*q2 + 2*q3))/2 + (I311*q1p*q3rp*sin(2*q2 + 2*q3))/2 + (I311*q3p*q1rp*sin(2*q2 + 2*q3))/2 - (I322*q1p*q2rp*sin(2*q2 + 2*q3))/2 - (I322*q2p*q1rp*sin(2*q2 + 2*q3))/2 - (I322*q1p*q3rp*sin(2*q2 + 2*q3))/2 - (I322*q3p*q1rp*sin(2*q2 + 2*q3))/2 + L9*L10*m3*q2rpp*sin(q2 + q3) + L9*L10*m3*q3rpp*sin(q2 + q3) + L10*L11*m3*q2rpp*sin(q2 + q3) + L10*L11*m3*q3rpp*sin(q2 + q3) - (L3^2*m3*q1p*q2rp*sin(2*q2))/2 - (L3^2*m3*q2p*q1rp*sin(2*q2))/2 - (L8^2*m2*q1p*q2rp*sin(2*q2))/2 - (L8^2*m2*q2p*q1rp*sin(2*q2))/2 + L3*L10*m3*q1rpp*cos(q3) + L3*L9*m3*q2rpp*sin(q2) + L3*L11*m3*q2rpp*sin(q2) + L7*L8*m2*q2rpp*sin(q2) - (L10^2*m3*q1p*q2rp*sin(2*q2 + 2*q3))/2 - (L10^2*m3*q2p*q1rp*sin(2*q2 + 2*q3))/2 - (L10^2*m3*q1p*q3rp*sin(2*q2 + 2*q3))/2 - (L10^2*m3*q3p*q1rp*sin(2*q2 + 2*q3))/2 + L3*L10*m3*q1rpp*cos(2*q2 + q3) + L3*L9*m3*q2p*q2rp*cos(q2) + L3*L11*m3*q2p*q2rp*cos(q2) + L7*L8*m2*q2p*q2rp*cos(q2) - (L3*L10*m3*q1p*q3rp*sin(q3))/2 - (L3*L10*m3*q3p*q1rp*sin(q3))/2 - L3*L10*m3*q1p*q2rp*sin(2*q2 + q3) - L3*L10*m3*q2p*q1rp*sin(2*q2 + q3) - (L3*L10*m3*q1p*q3rp*sin(2*q2 + q3))/2 - (L3*L10*m3*q3p*q1rp*sin(2*q2 + q3))/2 + L9*L10*m3*q2p*q2rp*cos(q2 + q3) + L9*L10*m3*q2p*q3rp*cos(q2 + q3) + L9*L10*m3*q3p*q2rp*cos(q2 + q3) + L9*L10*m3*q3p*q3rp*cos(q2 + q3) + L10*L11*m3*q2p*q2rp*cos(q2 + q3) + L10*L11*m3*q2p*q3rp*cos(q2 + q3) + L10*L11*m3*q3p*q2rp*cos(q2 + q3) + L10*L11*m3*q3p*q3rp*cos(q2 + q3);
    I233*q2rpp + I333*q2rpp + I333*q3rpp + I323*q1rpp*cos(q2 + q3) + I313*q1rpp*sin(q2 + q3) + L3^2*m3*q2rpp + L8^2*m2*q2rpp + L10^2*m3*q2rpp + L10^2*m3*q3rpp + I223*q1rpp*cos(q2) + I213*q1rpp*sin(q2) - L10*g*m3*cos(q2 + q3) - L3*g*m3*cos(q2) - L8*g*m2*cos(q2) - I212*q1p*q1rp*cos(2*q2) - (I211*q1p*q1rp*sin(2*q2))/2 + (I222*q1p*q1rp*sin(2*q2))/2 - I312*q1p*q1rp*cos(2*q2 + 2*q3) - (I311*q1p*q1rp*sin(2*q2 + 2*q3))/2 + (I322*q1p*q1rp*sin(2*q2 + 2*q3))/2 + L9*L10*m3*q1rpp*sin(q2 + q3) + L10*L11*m3*q1rpp*sin(q2 + q3) + (L3^2*m3*q1p*q1rp*sin(2*q2))/2 + (L8^2*m2*q1p*q1rp*sin(2*q2))/2 + 2*L3*L10*m3*q2rpp*cos(q3) + L3*L10*m3*q3rpp*cos(q3) + L3*L9*m3*q1rpp*sin(q2) + L3*L11*m3*q1rpp*sin(q2) + L7*L8*m2*q1rpp*sin(q2) + (L10^2*m3*q1p*q1rp*sin(2*q2 + 2*q3))/2 - L3*L10*m3*q2p*q3rp*sin(q3) - L3*L10*m3*q3p*q2rp*sin(q3) - L3*L10*m3*q3p*q3rp*sin(q3) + L3*L10*m3*q1p*q1rp*sin(2*q2 + q3);
    I333*q2rpp + I333*q3rpp + I323*q1rpp*cos(q2 + q3) + I313*q1rpp*sin(q2 + q3) + L10^2*m3*q2rpp + L10^2*m3*q3rpp - L10*g*m3*cos(q2 + q3) - I312*q1p*q1rp*cos(2*q2 + 2*q3) - (I311*q1p*q1rp*sin(2*q2 + 2*q3))/2 + (I322*q1p*q1rp*sin(2*q2 + 2*q3))/2 + L9*L10*m3*q1rpp*sin(q2 + q3) + L10*L11*m3*q1rpp*sin(q2 + q3) + L3*L10*m3*q2rpp*cos(q3) + (L10^2*m3*q1p*q1rp*sin(2*q2 + 2*q3))/2 + (L3*L10*m3*q1p*q1rp*sin(q3))/2 + L3*L10*m3*q2p*q2rp*sin(q3) + (L3*L10*m3*q1p*q1rp*sin(2*q2 + q3))/2];
 
EoMr=EoMr+0.1*Qp;%friction

% Jef_inv=inv(Jef'*Jef);
% Jef_inv2=inv((Jef'*Jef))*Jef';
% [u,s,v]=svd(Jef);
% z=zeros(3,3);
% Jef_inv=[inv(s(1:3,1:3))];
% Jef_inv=inv(Jef(1:3,:));
% reference=det(Jef_inv);
Sq=Qp-Jef_inv*Xrp(1:3,1);

G=[                                                     0;
 - g*m3*(L10*cos(q2 + q3) + L3*cos(q2)) - L8*g*m2*cos(q2);
                                   -L10*g*m3*cos(q2 + q3)];
                               

tau=-Kd*Sq+EoMr;
EoMr
%tau=-Kd*Sq;%PD like  %+EoMr %add friction in EoMr

%singularity
w=sqrt(det(Jef(1:3,:)'*Jef(1:3,:)));
if w<=0.01
    isSingular=1;
    tau=0;
end

  end

