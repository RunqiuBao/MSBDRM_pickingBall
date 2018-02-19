function [Xdn,Xdnp] = TrajGen_os(X,Xd,tw)
persistent tLast;
persistent XdnLast;

if isempty(tLast)
    tLast=0;
end
if isempty(XdnLast)
    XdnLast=0;
end

t=0.1;

eq=[  1 0 0 0 0 0;
      0 1 0 0 0 0;
      0 0 2 0 0 0;
      1 t t^2 t^3 t^4 t^5;
      0 1 2*t 3*t^2 4*t^3 5*t^4;
      0 0 2 6*t 12*t^2 20*t^3];

a_X=inv(eq)*[X(1);0;0;Xd(1);0;0];  
a_Y=inv(eq)*[X(2);0;0;Xd(2);0;0];   
a_Z=inv(eq)*[X(3);0;0;Xd(3);0;0];   
a_row=inv(eq)*[X(4);0;0;Xd(4);0;0];   
a_pitch=inv(eq)*[X(5);0;0;Xd(5);0;0];   
a_yaw=inv(eq)*[X(6);0;0;Xd(6);0;0];    

temp=[1 t t^2 t^3 t^4 t^5];

Xdn(1)=temp*a_X;
Xdn(2)=temp*a_Y;
Xdn(3)=temp*a_Z;
Xdn(4)=temp*a_row;
Xdn(5)=temp*a_pitch;
Xdn(6)=temp*a_yaw;

Xdnp=[0,0,0,0,0,0];

if tw>=0.01
    Xdnp=(Xdn-XdnLast)/(tw-tLast);
end

XdnLast=Xdn;
tLast=tw;

Xdn=Xdn';
Xdnp=Xdnp';
end

