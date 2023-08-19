clear all;close all;
Read_Data;
%%  导入刚体a和b在动捕系统坐标系下的位置姿态数据，求得Tca和Tcb，其中Tca是基座，Tcb是末端
% A为Tca^-1 * Tcb,B为T06
% 将动捕得到的位姿数据转换为齐次变换矩阵形式
AA = [ ];
for i =1:6
    Tca = T_tran(base_Data(i,1:3),base_Data(i,4:6));
    Tcb = T_tran(penqiang_Data(i,1:3),penqiang_Data(i,4:6));
    AA = [AA T_inverse(Tca)*Tcb];
end

%% 导入机械臂DH坐标参数，求得T07
% T07 = position(Jointangle);
T06 = [];
for i =1:6
    T06 = [T06 Gen2_position(angle_Data(i,:))]
end

%% 根据公式: (Tca)^-1 * Tcb * (T7b)^-1 = Ta0 * T07 
% 需要求解出T7b和Ta0,X为T7b^-1,Y为Ta0,
% 转换用的公式 T07 = (TcaTa0)^-1 * Tcb * (T7b)^-1
% AA = (Tca) * Tcb;
BB = T06;
[X,Y] = shah(AA,BB);
T7b = T_inverse(X);
Ta0 = Y;
% disp(T_inverse(T06(1:4,1:4)));

%% 数据转换,得到基于机械臂基座的数据
process_data = [];
% T07 = inv(Tca*Ta0)*Tcb*(T7b)^-1;
for i =1:440
    T_tmp = T_tran(demonstrations_data(i,1:3),demonstrations_data(i,4:6));
    T2_tmp = T_inverse(Tca*Ta0) * T_tmp * T_inverse(T7b);      %转换后
    e = rotm2eul(T2_tmp(1:3,1:3),'ZYX');
    process_data = [process_data;e,T2_tmp(1:3,4)']    
end

