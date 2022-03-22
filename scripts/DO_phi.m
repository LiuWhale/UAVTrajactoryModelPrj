function [sys,y0,str,ts,simStateCompliance] = DO(t,y,u,flag)
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
sizes.NumContStates  = 3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1; 
sys = simsizes(sizes);
y0  = [0 0 0];
str = [];
ts  = [-1 0];
simStateCompliance = 'UnknownSimState';
function sys=mdlDerivatives(t,y,u)
global K0;
global J;
global l;

l0 = 22600000;
l1 = 39100;
l2 = 390;
l_1 = 1;
l_2 = 0;
l_3 = 0;
%u(1) u u(2) dot_A
dy(1)=y(2)/J+l*u(1)/J+l2*(u(2)-y(1))/10+l_1*sign((u(2)-y(1))/10);
dy(2)=y(3)+l1*(u(2)-y(1))/100+l_2*sat((u(1)-y(1))/100);
dy(3)=l0*(u(2)-y(1))/1000+l_3*sat((u(1)-y(1))/1000);
sys(1)=dy(1);
sys(2)=dy(2);
sys(3)=dy(3);
function sys=mdlOutputs(t,y,u)
sys(1) = y(2);

