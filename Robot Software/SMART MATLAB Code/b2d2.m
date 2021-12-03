function y = b2d2(x)

% Convert a binary array to a decimal number
% 
% Similar to bin2dec but works with arrays instead of strings and is found to be 
% rather faster
z = 2.^(length(x)-1:-1:0);
%x=double(fliplr(x))-48;
x=double(x);
y = sum(x.*z);
