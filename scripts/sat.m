function [M] = sat(x)

upperLimit = 0.25;
d=abs(x);
if d<=upperLimit
M=x;
%M=0;
else
M= sign(x)*upperLimit;
%M=0;
end

