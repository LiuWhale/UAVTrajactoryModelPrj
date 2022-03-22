function [sys,y0,str,ts,simStateCompliance] = theta_model(t,y,u,flag)
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
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1; 
sys = simsizes(sizes);
y0  = [-20/180*3.1415926 0];
str = [];
ts  = [-1 0];
simStateCompliance = 'UnknownSimState';
function sys=mdlDerivatives(t,y,u)
global K0;
global J1;
global l;

%u(1) u u(2) d 
dy(1)=y(2);
dy(2)=(-K0*l*y(2)+u(2)+l*u(1))/J1;
sys(1)=dy(1);
sys(2)=dy(2);

function sys=mdlOutputs(t,y,u)
sys(1) = y(1);
sys(2) = y(2);

