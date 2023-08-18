function [T_new] = T_inverse(T)
%T_INVERSE 齐次变换变换矩阵
%   此处显示详细说明
T_new= T;
T_new(1:3,1:3) = T(1:3,1:3)';
T_new(1:3,4) = -T_new(1:3,1:3) * T(1:3,4);
T_new(4,4) = 1;
end

