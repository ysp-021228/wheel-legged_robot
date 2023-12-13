 %% 获取直腿模型的K矩阵
   %参数个数为1，传入的是L
function [K] = Cal_mK_In_Zhi_Tui_Mode(paras)
%% 调用函数，获取A,B矩阵
[A_ballance, B_ballance] = Cal_mA_mB_In_Zhi_Tui_Mode(paras(1));
%[A_ballance, B_ballance] = Cal_mA_mB_In_Zhi_Tui_Mode(0.18);
%% 调用lqr函数
T = 0.001;%仿真步长

Q = [1.8   0    0    0    0     0;   %腿倾角
     0    2.5   0    0    0     0;   %腿倾角速度
     0    0    10   0    0    0;   %位移
     0    0    0    1    0   0;   %速度
     0    0    0    0    2800    0;   %机体角度
     0    0    0    0    0     10.8];   %机体角速度

     
R = [5      0 ;  %轮子
     0        0.2];   %关节

K = lqr(A_ballance, B_ballance, Q, R);

%fprintf('{%f,%f,%f,%f,%f,%f},\n',K(1,1),K(1,2),K(1,3),K(1,4),K(1,5),K(1,6));
%fprintf('{%f,%f,%f,%f,%f,%f},\n\n',K(2,1),K(2,2),K(2,3),K(2,4),K(2,5),K(2,6));
end