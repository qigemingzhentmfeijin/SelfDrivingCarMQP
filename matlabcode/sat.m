function [ result ] = sat( a, b, c )
%SAT Summary of this function goes here
%   Detailed explanation goes here
if(a<b)
    result = b;
elseif(a>c)
    result = c;
else
    result = a;

end

