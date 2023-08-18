%% 设置读取数据文件的路径,数据为毫米和度数，
% end为末端附近的刚体，base为基座附近的刚体
% pre_path = 'F:\动捕系统设计\2023\08_09\record_';
penqiang_path = 'end_';
base_path = 'base_';
tail_path = '.xlsx';
angle_path = '关节角.xlsx';
penqiang_Data = [];
base_Data = [];
range = 'B25:G25';
angle_range = 'A1:F6';

for i=2:7
    % 喷枪的
    path = [penqiang_path num2str(i) tail_path];
    tmp = readmatrix(path,'Range',range);
    penqiang_Data = [penqiang_Data;tmp];
    % base基座的
    path = [base_path num2str(i) tail_path];
    tmp = readmatrix(path,'Range',range);
    base_Data = [base_Data;tmp];
end

% 将数据转换为米和弧度制
penqiang_Data(:,1:3) = penqiang_Data(:,1:3) /1000;
penqiang_Data(:,4:6) = penqiang_Data(:,4:6) *pi/180;
base_Data(:,1:3) = base_Data(:,1:3) /1000;
base_Data(:,4:6) = base_Data(:,4:6) *pi/180;

%机械臂角度
angle_Data = readmatrix(angle_path,'Range',angle_range);

%% 将关节角转换为DH算法的关节角数据
angle_Data (:,1) = -angle_Data (:,1);
angle_Data (:,2) = angle_Data (:,2)-pi/2;
angle_Data (:,3) = angle_Data (:,3)+pi/2;
angle_Data (:,5) = angle_Data (:,5)-pi;
angle_Data (:,6) = angle_Data (:,6)+pi/2;

%% 末端运动过程数据
demonstrations_path = 'demonstrations_end.xlsx';
demonstrations_range = 'B25:G464';
demonstrations_data = readmatrix(demonstrations_path,'Range',demonstrations_range);

demonstrations_data(:,1:3) = demonstrations_data(:,1:3) /1000;
demonstrations_data(:,4:6) = demonstrations_data(:,4:6) *pi/180;


