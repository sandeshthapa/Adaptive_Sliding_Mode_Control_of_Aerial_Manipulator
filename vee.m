%
% vee.m
%
% inverse of crossmat.m
%

function k = vee(K)

k=[-K(2,3);K(1,3);-K(1,2)];
