function [x,y,z]= position(jointAngle)
    %通过已知关节角求机械臂的变换矩阵T07
    
    % 标准 DH参数 i=1,2,3，4,5,6,7
    
    a = [0, 0, 0, 0, 0, 0, 0 ]';  % 连杆长度，沿x_i轴方向，z_{i-1}轴到z_i轴之间的距离
    alpha = [pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, 0]'; % 扭角，绕x_{i-1}右手定则旋转方向为正，z_{i-1}轴到z_i轴的转角，且在（-pi,pi]之内
    d = [-0.2848, -0.0118, -0.4208, -0.0128, -0.3143,0,-0.1059]'; % 偏置，沿z_i轴方向，x_{i-1}轴到x_i轴的距离
    theta = jointAngle; % 关节角，绕z_i右手定则旋转方向为正，x_{i-1}轴到x_i轴的转角，且在（-pi,pi]之内
    
    % 世界坐标系Cw与坐标系C0间的DH参数
    a_0=0 ; alpha_0 = pi; d_0 = 0; theta_0 = 0;
    
    % 坐标系C7与坐标系Ce间的DH参数
    a_e=0 ; alpha_e = pi; d_e = -0.0615; theta_e = 0;
    
    Tw0=T_para(theta_0,d_0,a_0,alpha_0);
    T01=T_para(theta(1),d(1),a(1),alpha(1));
    T12=T_para(theta(2),d(2),a(2),alpha(2));
    T23=T_para(theta(3),d(3),a(3),alpha(3));
    T34=T_para(theta(4),d(4),a(4),alpha(4));
    T45=T_para(theta(5),d(5),a(5),alpha(5));
    T56=T_para(theta(6),d(6),a(6),alpha(6));
    T67=T_para(theta(7),d(7),a(7),alpha(7));
    T7e=T_para(theta_e,d_e,a_e,alpha_e);
    
    Tw1=Tw0*T01;
    Tw2=Tw1*T12;
    Tw3=Tw2*T23;
    Tw4=Tw3*T34;
    Tw5=Tw4*T45;
    Tw6=Tw5*T56;
    Tw7=Tw6*T67;
    Twe=Tw7*T7e;

%     position = Twe * [0; 0; 0; 1];

%     x = position(1);
%     y = position(2);
%     z = position(3);
end

% function [Tz] = Tz_angle(alpha)
%     Tz = [cos(alpha),-sin(alpha),0;
%         sin(alpha),cos(alpha),0;
%         0,0,1];
% end
% 
% function [Tx] = Tx_angle(alpha)
%     Tx = [1,0,0;
%         0,cos(alpha),-sin(alpha);
%         0,sin(alpha),cos(alpha)];
% end
% 
% function [Ty] = Ty_angle(alpha)
%     Ty = [cos(alpha),0,sin(alpha);
%         0,1,0;
%         -sin(alpha),0,cos(alpha)];
% end