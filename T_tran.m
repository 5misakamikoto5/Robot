%定义由位姿到变换矩阵的函数
function T = T_tran(pos,eular)
    T = eul2tform(flip(eular),'ZYX');
    T(1:3,4) = pos;
end

