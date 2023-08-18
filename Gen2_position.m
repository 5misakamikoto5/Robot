function [T06]= Gen2_position(jointAngle)
    %% Gen2参数
    D1 = 0.2755;
    D2 = 0.41;
    D3 = 0.2703;
    D4 = 0.0741;
    D5 = 0.0741;
    D6 = 0.16;
    e2 = 0.0098;
    aa = pi/6;
    sa = sin(aa);
    s2a = sin(2*aa);
    d4b = D3 + (sa / s2a)*D4;
    d5b = (sa / s2a)*(D4 + D5);
    d6b = (sa / s2a) * D5 + D6;

    %通过已知关节角求机械臂的变换矩阵T06
    % 标准 DH参数 i=1,2,3，4,5,6
    a = [0, D2, 0, 0, 0, 0]';  
    alpha = [pi/2, pi, pi/2, pi/3, pi/3, pi]'; 
    d = [D1, 0 -e2, -d4b, -d5b,-d6b]';
    theta = jointAngle; 
    
    T01=T_para(theta(1),d(1),a(1),alpha(1));
    T12=T_para(theta(2),d(2),a(2),alpha(2));
    T23=T_para(theta(3),d(3),a(3),alpha(3));
    T34=T_para(theta(4),d(4),a(4),alpha(4));
    T45=T_para(theta(5),d(5),a(5),alpha(5));
    T56=T_para(theta(6),d(6),a(6),alpha(6));
    
    T02=T01*T12;
    T03=T02*T23;
    T04=T03*T34;
    T05=T04*T45;
    T06=T05*T56;

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