function [sys,y0,str,ts,simStateCompliance] = position(t,y,u,flag)
switch flag
  case 0
    [sys,y0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,y,u);
  case 3
    sys=mdlOutputs(t,y,u);
  case {2,4,9}
        sys=[];
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
function [sys,y0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 6;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 5;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1; 
sys = simsizes(sizes);
y0  = [0 0 0 0 0 0];
str = [];
ts  = [-1 0];
simStateCompliance = 'UnknownSimState';
function sys=mdlDerivatives(t,y,u)
global K0;

g = 9.8;
%u(1) d u(2) phi u(3) theta u(4) psi u(5) u
dy(1)=y(2);
dy(2)=-K0*y(2)+u(1)+u(5)*(cos(u(2))*sin(u(3))*cos(u(4))+sin(u(2))*sin(u(4)));
dy(3)=y(4);
dy(4)=-K0*y(4)+u(1)+u(5)*(cos(u(2))*sin(u(3))*sin(u(4))-sin(u(2))*cos(u(4)));
dy(5)= y(6);
dy(6)=u(5)*cos(u(3))*cos(u(2))-g-K0*y(6)+u(1);
sys(1)=dy(1);
sys(2)=dy(2);
sys(3)=dy(3);
sys(4)=dy(4);
sys(5)=dy(5);
sys(6)=dy(6);

function sys=mdlOutputs(t,y,u)
sys(1)=y(1);
sys(2)=y(2);
sys(3)=y(3);
sys(4)=y(4);
sys(5)=y(5);
sys(6)=y(6);

