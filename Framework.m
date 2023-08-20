clear all;close all;
Read_Data;

%%  导入刚体a和b在动捕系统坐标系下的位置姿态数据，求得Tca和Tcb，其中Tca是基座刚体，Tcb是末端刚体
% A为Tca^-1 * Tcb,B为T06
% 将动捕得到的位姿数据转换为齐次变换矩阵形式
collection_size = 6;
AA = zeros(6,collection_size);
for i =1:collection_size
    Tca = T_tran(base_Data(i,1:3),base_Data(i,4:6));
    Tcb = T_tran(penqiang_Data(i,1:3),penqiang_Data(i,4:6));
    AA(:,i) = T_inverse(Tca)*Tcb;
end

%% 导入机械臂DH坐标参数，求得T06:6*n形状
T06 = zeros(6,collection_size);
for i =1:collection_size
    T06(:,i) = Gen2_position(angle_Data(i,:))
end

%% 根据公式: (Tca)^-1 * Tcb * (T7b)^-1 = Ta0 * T07 
% 需要求解出T7b和Ta0,X为T7b^-1,Y为Ta0,
% 转换用的公式 T07 = (TcaTa0)^-1 * Tcb * (T7b)^-1
% AA = (Tca) * Tcb;
BB = T06;
[X,Y] = shah(AA,BB);
T6b = T_inverse(X);
Ta0 = Y;

%% 数据转换,根据记录的末端刚体数据Tcb，得到末端关节基于机械臂基座的变换矩阵
[m,n] = size(demonstrations_data);
end_pose = zeros(4*m,n);
% T06 = inv(Tca*Ta0)*Tcb*(T6b)^-1;
for i =1:m
    T_tmp = T_tran(demonstrations_data(i,1:3),demonstrations_data(i,4:6));
    T2_tmp = T_inverse(Tca*Ta0) * T_tmp * T_inverse(T6b);      %转换后
    end_pose(4*(i-1)+1:4*i,:) = T2_tmp;  
end

%% 逆运动学求解,由机器人工具箱完成
gen2 = loadrobot('kinovaJacoJ2N6S300');
% 创建逆运动学求解器
ik = inverseKinematics('RigidBodyTree',gen2);
% 设置初始关节角度
initialguess = gen2.homeConfiguration;
% weights为每个维度允许的误差
weights = [0.25 0.25 0.25 1 1 1];
% 记录逆运动学求解的关节角
configuration = zeros(m,6);
for i=1:m
% 设置末端执行器的位姿，由4*4变换矩阵构成
tform = end_pose(4*(i-1)+1:4*i,:);
% 求解逆运动学
[config,solinfo] = ik('j2n6s300_end_effector',tform,weights,initialguess);
configuration(i,:) = config;
end
