clc;
clear;
close all;

%%
syms g z0 m1 m2 m3  L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 L11 L12 L13 q1 q2 q3 q1p q2p q3p real
syms I111 I112 I113 I122 I123 I133 real
syms I211 I212 I213 I222 I223 I233 real
syms I311 I312 I313 I322 I323 I333 real

%% Reconstruct Inertia matrices
I1=[I111 I112 I113;
    I112 I122 I123;
    I113 I123 I133];

I2=[I211 I212 I213;
    I212 I222 I223;
    I213 I223 I233];

I3=[I311 I312 I313;
    I312 I322 I323;
    I313 I323 I333];

%% Jacobian
Jcm1=[0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;1 0 0];

Jcm2=[L7*cos(q1) + L8*sin(q1)*sin(q2), -L8*cos(q1)*cos(q2),     0;
      L7*sin(q1) - L8*cos(q1)*sin(q2), -L8*cos(q2)*sin(q1),     0;
                                    0,         -L8*sin(q2),     0;
                                    0,             sin(q1),     0;
                                    0,            -cos(q1),     0;
                                    1,                   0,     0];
                                
Jcm3=[L9*cos(q1) + L11*cos(q1) + L3*sin(q1)*sin(q2) - L10*cos(q2)*sin(q1)*sin(q3) + L10*cos(q3)*sin(q1)*sin(q2),   -cos(q1)*(L3*cos(q2) + L10*cos(q2 - q3)),   L10*cos(q2 - q3)*cos(q1);
      L9*sin(q1) + L11*sin(q1) - L3*cos(q1)*sin(q2) + L10*cos(q1)*cos(q2)*sin(q3) - L10*cos(q1)*cos(q3)*sin(q2),   -sin(q1)*(L3*cos(q2) + L10*cos(q2 - q3)),   L10*cos(q2 - q3)*sin(q1);
                                                                                                              0,            - L3*sin(q2) - L10*sin(q2 - q3),           L10*sin(q2 - q3);
                                                                                                              0,                                    sin(q1),                   -sin(q1);
                                                                                                              0,                                   -cos(q1),                    cos(q1);
                                                                                                              1,                                          0,                         0];
%% Compute HT
Hcm1_0 =[ cos(q1), -sin(q1), 0,  0;
          sin(q1),  cos(q1), 0,  0;
                0,        0, 1, L6;
                0,        0, 0,  1];

Hcm2_0 =[ cos(q1)*sin(q2), cos(q1)*cos(q2),  sin(q1), L7*sin(q1) + L8*cos(q1)*sin(q2);
          sin(q1)*sin(q2), cos(q2)*sin(q1), -cos(q1), L8*sin(q1)*sin(q2) - L7*cos(q1);
                 -cos(q2),         sin(q2),        0,                 L1 - L8*cos(q2);
                        0,               0,        0,                               1];

Hcm3_0 =[ cos(q1 + q3)*sin(q2), -cos(q1 + q3)*cos(q2), -sin(q1 + q3), L10*cos(q3) + cos(q3)*(L7*sin(q1) + L3*cos(q1)*sin(q2)) + sin(q3)*(L7*cos(q1) - L3*sin(q1)*sin(q2));
          sin(q1 + q3)*sin(q2), -sin(q1 + q3)*cos(q2),  cos(q1 + q3), L10*sin(q3) - cos(q3)*(L7*cos(q1) - L3*sin(q1)*sin(q2)) + sin(q3)*(L7*sin(q1) + L3*cos(q1)*sin(q2));
                      -cos(q2),              -sin(q2),             0,                                                                     L1 + L7 - L9 - L11 - L3*cos(q2);
                             0,                     0,             0,                                                                                                   1];
                         
%% Get Rotation and Translation from HT.
Rcm1_0=Hcm1_0(1:3,1:3);
Rcm2_0=Hcm2_0(1:3,1:3);
Rcm3_0=Hcm3_0(1:3,1:3);

tcm1_0=Hcm1_0(1:3,4);
tcm2_0=Hcm2_0(1:3,4);
tcm3_0=Hcm3_0(1:3,4);

%% Compute M
M=m1*Jcm1(1:3,:)'*Jcm1(1:3,:)+...
    m2*Jcm2(1:3,:)'*Jcm2(1:3,:)+...
    m3*Jcm3(1:3,:)'*Jcm3(1:3,:)+...
    Jcm1(4:6,:)'*Rcm1_0*I1*Rcm1_0'*Jcm1(4:6,:)+...
    Jcm2(4:6,:)'*Rcm2_0*I2*Rcm2_0'*Jcm2(4:6,:)+...
    Jcm3(4:6,:)'*Rcm3_0*I3*Rcm3_0'*Jcm3(4:6,:);

M=simplify(M)

%% Compute G
% Potential energie
P=m1*[0 0 g]*tcm1_0+...
  m2*[0 0 g]*tcm2_0+...
  m3*[0 0 g]*tcm3_0;

G=[diff(P,q1);diff(P,q2);diff(P,q3)];
G=simplify(G)

%% Compute C
C=sym(zeros(3));
q=[q1;q2;q3];
qp=[q1p;q2p;q3p];

for k=1:3
    for j=1:3
        
            C(k,j)=0.5*qp'*[diff(M(k,j),q(1))+diff(M(k,1),q(j))-diff(M(1,j),q(k));
                            diff(M(k,j),q(2))+diff(M(k,2),q(j))-diff(M(2,j),q(k));
                            diff(M(k,j),q(3))+diff(M(k,3),q(j))-diff(M(3,j),q(k))];
            C(k,j)=expand(C(k,j));
            C(k,j)=simplify(C(k,j));
       
    end
end
C=simplify(C)
%% Verify if M-2C is skew symmetric.
N=[jacobian(M(:,1),q)*qp,jacobian(M(:,2),q)*qp,jacobian(M(:,3),q)*qp]-2*C;
D2=simplify(expand(N+N'))


