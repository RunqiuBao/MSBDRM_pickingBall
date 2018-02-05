%method:find items like I or m*l1*l2 or m*l^2 from M matrix and items like
%m*l from G matrix, and stack them into theta vector
syms g z0 m1 m2 m3  L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 L11 L12 L13 q1 q2 q3 q1p q2p q3p q1pp q2pp q3pp real
syms I111 I112 I113 I122 I123 I133 real
syms I211 I212 I213 I222 I223 I233 real
syms I311 I312 I313 I322 I323 I333 real

syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9 real
syms theta10 theta11 theta12 theta13 theta14 theta15 theta16 theta17 theta18 theta19 real
syms theta20 theta21 theta22 theta23 theta24 theta25 theta26 theta27 theta28 theta29 real

% M=simplify(M);
% G=simplify(G);
% C=simplify(C);

% Yr=[1*q1pp,    0.5*q1pp+cos(2*q2)/2-q2p*sin(2*q2)/2,    -sin(2*q2)*q1pp-q2p*sin(2*q2)/2,    0,    0.5*q1pp-cos(2*q2)*q1pp/2+q2p*sin(2*q2)/2,    0,   0,   0.5+cos(2*q2 - 2*q3)/2-q2p*sin(2*q2 - 2*q3)/2+q3p*sin(2*q2 - 2*q3)/2,    sin(2*q2 - 2*q3)+q2p*cos(2*q2 - 2*q3)-q3p*cos(2*q2 - 2*q3),    0,    0.5-cos(2*q2 - 2*q3)/2+q2p*sin(2*q2 - 2*q3)/2-q3p*sin(2*q2 - 2*q3)/2,    0,    0,    1,    0.5-cos(2*q2)/2+q2p*sin(2*q2)/2,    0.5-cos(2*q2)/2+q2p*sin(2*q2)/2,   1,    0.5-cos(2*q2 - 2*q3)/2+q2p*sin(2*q2 - 2*q3)/2-q3p*sin(2*q2 - 2*q3)/2,    1,    0,    0,    cos(q3)-cos(2*q2 - q3)+q2p*sin(2*q2 - q3)-q3p*sin(2*q2 - q3)/2-q3p*sin(q3)/2,    0,    2,    0,    0,    0,   0,   0];
% 
% Yr=[0,    -q1p*sin(2*q2)/2,    -q1p*cos(2*q2),    -cos(q2)+q2p*sin(q2),    q1p*sin(2*q2)/2,    sin(q2)+q2p*cos(q2),    0,    -q1p*sin(2*q2 - 2*q3)/2,    q1p*cos(2*q2 - 2*q3),    cos(q2 - q3)-q2p*sin(q2 - q3)+q3p*sin(q2 - q3),    q1p*sin(2*q2 - 2*q3)/2,    sin(q2 - q3)+q2p*cos(q2 - q3)-q3p*cos(q2 - q3),    0,    0,    q1p*sin(2*q2)/2,    q1p*sin(2*q2)/2,    0,    q1p*sin(2*q2 - 2*q3)/2,    0,    -cos(q2)+q2p*sin(q2),    -sin(q2 - q3)+q2p*sin(q2),    q1p*sin(2*q2 - q3),    -cos(q2)+q2p*sin(q2),    -cos(q2)+q2p*sin(q2 - q3)-q3p*sin(q2 - q3),    0,    -cos(q2 - q3)+q2p*sin(q2 - q3)-q3p*sin(q2 - q3),    g*sin(q2),    g*sin(q2),    sin(q2 - q3)*g-g*sin(q2 - q3)];
% 
% Yr=[I133;
%     I211;
%     I212;
%     I213;
%     I222;
%     I223;
%     I233;
%     I311;
%     I312;
%     I313;
%     I322;
%     I323;
%     I333;
%     m2*L7^2;
%     m2*L8^2;
%     m3*L3^2;
%     m3*L9^2;
%     m3*L10^2;
%     m3*L11^2;
%     m2*L7*L8;
%     m3*L3*L9;
%     m3*L3*L10;
%     m3*L3*L11;
%     m3*L9*L10;
%     m3*L9*L11;
%     m3*L10*L11;
%     m2*L8;
%     m3*L3;
%     m3*L10];

EoM =[I133*q1pp + (I211*q1pp)/2 + (I222*q1pp)/2 + (I311*q1pp)/2 + (I322*q1pp)/2 + I313*q2pp*cos(q2 - q3) - I313*q3pp*cos(q2 - q3) + I323*q2pp*sin(q2 - q3) - I323*q3pp*sin(q2 - q3) + (I211*q1pp*cos(2*q2))/2 - (I222*q1pp*cos(2*q2))/2 + I223*q2p^2*cos(q2) - I212*q1pp*sin(2*q2) + I213*q2p^2*sin(q2) + (I311*q1pp*cos(2*q2 - 2*q3))/2 - (I322*q1pp*cos(2*q2 - 2*q3))/2 + I323*q2p^2*cos(q2 - q3) + I323*q3p^2*cos(q2 - q3) + I312*q1pp*sin(2*q2 - 2*q3) - I313*q2p^2*sin(q2 - q3) - I313*q3p^2*sin(q2 - q3) + (L3^2*m3*q1pp)/2 + L7^2*m2*q1pp + (L8^2*m2*q1pp)/2 + L9^2*m3*q1pp + (L10^2*m3*q1pp)/2 + L11^2*m3*q1pp - I213*q2pp*cos(q2) + I223*q2pp*sin(q2) - (L3^2*m3*q1pp*cos(2*q2))/2 - (L8^2*m2*q1pp*cos(2*q2))/2 - (L10^2*m3*q1pp*cos(2*q2 - 2*q3))/2 - 2*I323*q2p*q3p*cos(q2 - q3) + 2*I313*q2p*q3p*sin(q2 - q3) - 2*I212*q1p*q2p*cos(2*q2) - I211*q1p*q2p*sin(2*q2) + I222*q1p*q2p*sin(2*q2) + 2*L9*L11*m3*q1pp + 2*I312*q1p*q2p*cos(2*q2 - 2*q3) - 2*I312*q1p*q3p*cos(2*q2 - 2*q3) - I311*q1p*q2p*sin(2*q2 - 2*q3) + I311*q1p*q3p*sin(2*q2 - 2*q3) + I322*q1p*q2p*sin(2*q2 - 2*q3) - I322*q1p*q3p*sin(2*q2 - 2*q3) + L3*L9*m3*q2p^2*sin(q2) + L3*L11*m3*q2p^2*sin(q2) + L7*L8*m2*q2p^2*sin(q2) - L3*L10*m3*q1pp*cos(2*q2 - q3) + L9*L10*m3*q2p^2*sin(q2 - q3) + L9*L10*m3*q3p^2*sin(q2 - q3) + L10*L11*m3*q2p^2*sin(q2 - q3) + L10*L11*m3*q3p^2*sin(q2 - q3) + L3^2*m3*q1p*q2p*sin(2*q2) + L8^2*m2*q1p*q2p*sin(2*q2) - L3*L9*m3*q2pp*cos(q2) + L3*L10*m3*q1pp*cos(q3) - L3*L11*m3*q2pp*cos(q2) - L7*L8*m2*q2pp*cos(q2) + L10^2*m3*q1p*q2p*sin(2*q2 - 2*q3) - L10^2*m3*q1p*q3p*sin(2*q2 - 2*q3) - L9*L10*m3*q2pp*cos(q2 - q3) + L9*L10*m3*q3pp*cos(q2 - q3) - L10*L11*m3*q2pp*cos(q2 - q3) + L10*L11*m3*q3pp*cos(q2 - q3) - L3*L10*m3*q1p*q3p*sin(q3) - 2*L9*L10*m3*q2p*q3p*sin(q2 - q3) - 2*L10*L11*m3*q2p*q3p*sin(q2 - q3) + 2*L3*L10*m3*q1p*q2p*sin(2*q2 - q3) - L3*L10*m3*q1p*q3p*sin(2*q2 - q3);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                I233*q2pp + I333*q2pp - I333*q3pp + I313*q1pp*cos(q2 - q3) + I323*q1pp*sin(q2 - q3) + I212*q1p^2*cos(2*q2) + (I211*q1p^2*sin(2*q2))/2 - (I222*q1p^2*sin(2*q2))/2 - I312*q1p^2*cos(2*q2 - 2*q3) + L3^2*m3*q2pp + L8^2*m2*q2pp + L10^2*m3*q2pp - L10^2*m3*q3pp + (I311*q1p^2*sin(2*q2 - 2*q3))/2 - (I322*q1p^2*sin(2*q2 - 2*q3))/2 - I213*q1pp*cos(q2) + I223*q1pp*sin(q2) + L3*g*m3*sin(q2) + L8*g*m2*sin(q2) - (L3^2*m3*q1p^2*sin(2*q2))/2 - (L8^2*m2*q1p^2*sin(2*q2))/2 + L10*g*m3*sin(q2 - q3) - (L10^2*m3*q1p^2*sin(2*q2 - 2*q3))/2 + L3*L10*m3*q3p^2*sin(q3) - L3*L10*m3*q1p^2*sin(2*q2 - q3) - L3*L9*m3*q1pp*cos(q2) - L3*L11*m3*q1pp*cos(q2) - L7*L8*m2*q1pp*cos(q2) + 2*L3*L10*m3*q2pp*cos(q3) - L3*L10*m3*q3pp*cos(q3) - L9*L10*m3*q1pp*cos(q2 - q3) - L10*L11*m3*q1pp*cos(q2 - q3) - 2*L3*L10*m3*q2p*q3p*sin(q3);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               I333*q3pp - I333*q2pp - I313*q1pp*cos(q2 - q3) - I323*q1pp*sin(q2 - q3) + I312*q1p^2*cos(2*q2 - 2*q3) - L10^2*m3*q2pp + L10^2*m3*q3pp - (I311*q1p^2*sin(2*q2 - 2*q3))/2 + (I322*q1p^2*sin(2*q2 - 2*q3))/2 - L10*g*m3*sin(q2 - q3) + (L10^2*m3*q1p^2*sin(2*q2 - 2*q3))/2 + (L3*L10*m3*q1p^2*sin(q3))/2 + L3*L10*m3*q2p^2*sin(q3) + (L3*L10*m3*q1p^2*sin(2*q2 - q3))/2 - L3*L10*m3*q2pp*cos(q3) + L9*L10*m3*q1pp*cos(q2 - q3) + L10*L11*m3*q1pp*cos(q2 - q3)];
 



%% Set the parameter vector(29 parameters).
Theta=[I133;
       I211;
       I212;
       I213;
       I222;
       I223;
       I233;
       I311;
       I312;
       I313;
       I322;
       I323;
       I333;
       m2*L7^2;
       m2*L8^2;
       m3*L3^2;
       m3*L9^2;
       m3*L10^2;
       m3*L11^2;
       m2*L7*L8;
       m3*L3*L9;
       m3*L3*L10;
       m3*L3*L11;
       m3*L9*L10;
       m3*L9*L11;
       m3*L10*L11;
       m2*L8;
       m3*L3;
       m3*L10];
 
 %% Substitute Theta into EoM.

EoM=subs(EoM,I133,theta1);
EoM=subs(EoM,I211,theta2);
EoM=subs(EoM,I212,theta3);
EoM=subs(EoM,I213,theta4);       
EoM=subs(EoM,I222,theta5);
EoM=subs(EoM,I223,theta6);
EoM=subs(EoM,I233,theta7);
EoM=subs(EoM,I311,theta8);
EoM=subs(EoM,I312,theta9);
EoM=subs(EoM,I313,theta10);
EoM=subs(EoM,I322,theta11);
EoM=subs(EoM,I323,theta12);
EoM=subs(EoM,I333,theta13);
EoM=subs(EoM,m2*L7^2,theta14);
EoM=subs(EoM,m2*L8^2,theta15);
EoM=subs(EoM,m3*L3^2,theta16);
EoM=subs(EoM,m3*L9^2,theta17);
EoM=subs(EoM,m3*L10^2,theta18);
EoM=subs(EoM,m3*L11^2,theta19);
EoM=subs(EoM,m2*L7*L8,theta20);
EoM=subs(EoM,m3*L3*L9,theta21);
EoM=subs(EoM,m3*L3*L10,theta22);
EoM=subs(EoM,m3*L3*L11,theta23);
EoM=subs(EoM,m3*L9*L10,theta24);
EoM=subs(EoM,m3*L9*L11,theta25);
EoM=subs(EoM,m3*L10*L11,theta26);
EoM=subs(EoM,m2*L8,theta27);
EoM=subs(EoM,m3*L3,theta28); 
EoM=subs(EoM,m3*L10,theta29); 


theta=[theta1; theta2; theta3; theta4; theta5; theta6; theta7; theta8; theta9;...
       theta10; theta11; theta12; theta13; theta14; theta15; theta16; theta17; theta18; theta19;...
       theta20; theta21; theta22; theta23; theta24; theta25; theta26; theta27; theta28; theta29];

Y1=jacobian(EoM(1),theta);
Y2=jacobian(EoM(2),theta);
Y3=jacobian(EoM(3),theta);

Y=[Y1;Y2;Y3];
Y=simplify(Y);