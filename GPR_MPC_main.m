%% 非线性求解器 动力学模型 高斯过程回归 颠簸道路
function [sys,x0,str,ts] = GPR_MPC_main(t,x,u,flag)
switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
        
    case 2
        sys = mdlUpdates(t,x,u); % Update discrete states
        
    case 3
        sys = mdlOutputs(t,x,u); % Calculate outputs
        
    case {1,4,9} % Unused flags
        sys = [];
        
    otherwise
        error(['unhandled flag = ',num2str(flag)]); % Error handling
end
%% 初始化
function [sys,x0,str,ts] = mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 0;  %连续状态变量的个数
sizes.NumDiscStates  = 3;  %离散状态变量的个数
sizes.NumOutputs     = 10;  %输出的个数
sizes.NumInputs      = 7;  %输入的个数
sizes.DirFeedthrough = 1;  % Matrix D is non-empty.
sizes.NumSampleTimes = 1;  %是否直接贯通，与代数环的检测有关
sys = simsizes(sizes);
x0 =zeros(sizes.NumDiscStates,1);
str = [];
ts  = [0.1 0];       % 采样周期和mpc周期统一

%定义全局变量
%标志位
global gap_flag ;
gap_flag = 0;
global run_flag ;
run_flag = false;
%车辆状态参数 Carsim E-Class SUV
global VehiclePara;
VehiclePara.m   = 1860;  %簧载质量加悬架质量 1590+120+150
VehiclePara.g   = 9.81;  %
VehiclePara.hCG = 0.72;  %质心高度
VehiclePara.Lf  = 1.18;  %前轴轴距
VehiclePara.Lr  = 1.77;  %后轴轴距
VehiclePara.L   = 2.95;  %轴距
VehiclePara.Tr  = 1.575; %轮距
VehiclePara.mu  = 0.75;  %地面摩擦因数
VehiclePara.Iz  = 2687.1;  %车辆绕Z轴的转动惯量
VehiclePara.Ix  = 894.4;  %车辆绕x轴的转动惯量
VehiclePara.Iy  = 2687.1;  %车辆绕y轴的转动惯量
VehiclePara.Radius = 0.39;  % 轮胎滚动半径
%MPC参数
global MPCParameters;
MPCParameters.Np  = 8;% 时域
MPCParameters.Ts  = 0.1; %采样周期
MPCParameters.Nx  = 3; %状态量
MPCParameters.Ny  = 2; %输出量
MPCParameters.Nu  = 1; %控制量
MPCParameters.lowest_length = 20;  %最小预测距离
MPCParameters.model = 1;
MPCParameters.useGPonline = false;
MPCParameters.useParallel = true;
%代价权重
global CostWeights;
CostWeights.Wephi   = 1;  %航向偏差
CostWeights.Wey     = 1;  %横向偏差
CostWeights.steer = 30;  %控制量
CostWeights.steer_rate = 0;  %控制量变化率
%约束
global Constraints;
Constraints.dumax   = 0.1;  %控制增量约束 deg/s
Constraints.umax    = 0.5;  %控制量约束 deg
Constraints.ePhimax = 60*pi/180;  %航向角偏差的极限值
Constraints.eymax   = 1.5;  %横向偏差的极限值
Constraints.minsteer = -1;
Constraints.maxsteer = 1;
Constraints.amax = 4;

%参考路径  Carsim中的ALT3
global Reftraj
Reftraj = load('shuangyixian.mat');
%初始化估计线性侧偏刚度函数 动力学模型参数
% [y, e] = func_RLSFilter_Calpha_f('initial', 0.95, 10, 10);
% [y, e] = func_RLSFilter_Calpha_r('initial', 0.95, 10, 10);

%% 更新离散状态量
function sys = mdlUpdates(t,x,u)
sys = x;

%% 输出
function sys = mdlOutputs(t,x,u)

%********************************************一.参数初始化********************************************
global Reftraj;
global VehiclePara;
global MPCParameters;
global CostWeights;
global Constraints;
global gap_flag
global run_flag

Ctrl_frontwheelangle = 0;  %控制量：前轮转角
PosX = 0;  %车辆全局坐标轴下的X坐标
PosY = 0;  %车辆全局坐标轴下的Y坐标
PosPhi = 0;  %车辆航向角
PosVel = 0;
e_phi = 0;  %航向偏差
e_y = 0;  %横向偏差
yawrate = 0;  %角速度
Beta = 0;  %质心侧偏角
fwa = 0;  %观测量，前轮偏角
Next_Steer = 0; %控制量
spend_Time = 0;  %耗时
Predict_Path = [];

%忽略前两个输入量
if gap_flag < 2
    gap_flag = gap_flag + 1;
else % start control
    gap_flag = gap_flag + 1;
    %********************************************二.车辆状态更新********************************************
    t_Start = tic; %开始计时
    %更新观测量
    VehState = func_StateObserver(u);
    PosPhi = VehState.phi;
    PosVel = VehState.Vel;
    PosX = VehState.X;
    PosY = VehState.Y ;
    %更新估计量 动力学模型使用
    %前轮
    %     Arfa_f = (yawrate*VehiclePara.Lf/Vel - fwa);  %前轮侧偏角  TODO：考虑纵向影响
    %     [yf, Calpha_f1] = func_RLSFilter_Calpha_f(Arfa_f, Fyf);
    %     Caf = sum(Calpha_f1);  %前轮侧偏刚度
    %后轮
    %     Arfa_r = (yawrate*VehiclePara.Lr/Vel);  %后轮侧偏角
    %     [yr, Calpha_r1] = func_RLSFilter_Calpha_r(Arfa_r, Fyr);
    %     CarHat = sum(Calpha_r1);  %后轮侧偏刚度
    %********************************************三.参考轨迹处理********************************************
    [local_path_x,local_path_y] = func_wayPoint2VehicleCoords(VehState,Reftraj.WayPoints_Collect);
    %根据参数判断预测路径长度
    total_t = MPCParameters.Np * MPCParameters.Ts * 1.2;
    s_limit = (PosVel + Constraints.amax * total_t / 2) * total_t;
    s_limit = max(MPCParameters.lowest_length,s_limit);
    %计算参考轨迹信息
    [total_s,segment_arc_s,predict_length] = func_CurveLength(local_path_x,local_path_y,s_limit);
    local_path_x = local_path_x(1:predict_length);
    local_path_y = local_path_y(1:predict_length);
    heading = func_CurveHeading(local_path_x,local_path_y);  %车辆坐标系下参考路径航向
    curvature = func_CurveCurvature(local_path_x,local_path_y);  %参考路径曲率
    %检查数据长度是否匹配
    if (length(local_path_x) ~= length(local_path_y)|| length(local_path_x) ~= length(heading)|| ...
            length(local_path_x) ~= length(curvature) || length(local_path_x) ~= length(total_s) || ...
            length(local_path_x) ~= length(segment_arc_s) )
        run_flag = false;
        warning("Data length mismatch");
        warning on ;
    end
    %计算跟踪误差
    [e_phi,e_y] = func_TrackingError(local_path_x,local_path_y);
    %********************************************四.车辆模型与求解********************************************
    %选择车辆模型
    if (run_flag)
        switch MPCParameters.model
            case 1
                [Next_Steer,Predict_Path] = ...
                    func_Simple_Kinematic_model(local_path_x,local_path_y,heading,curvature,total_s,segment_arc_s,VehState,t);  %非线性运动学模型
            case 2
                [Next_Steer,Predict_Path] = ...
                    func_Simple_Dynamics_model(local_path_x,local_path_y,heading,curvature,total_s,segment_arc_s,VehState);  %非线性动力学模型
            case 3
                [Next_Steer,Predict_Path] = ...
                    func_GPR_Kinematic_model(local_path_x,local_path_y,heading,curvature,total_s,segment_arc_s,VehState);  %非线性高斯过程运动学模型
            case 4
                [Next_Steer,Predict_Path] = ...
                    func_GPR_Dynamics_model(local_path_x,local_path_y,heading,curvature,total_s,segment_arc_s,VehState);  %非线性高斯过程动力学模型
        end
    else
        Next_Steer = 0;
    end
%     %滤波
%     if Next_Steer < 0.001
%         Next_Steer = 0;
%     end 
Ctrl_frontwheelangle = atan(Next_Steer * VehiclePara.L)*180/pi;
spend_Time = toc(t_Start);  %计算每次控制所需时间
end
    %********************************************五.数据处理********************************************
%全局轨迹
figure(1)
plot(Reftraj.WayPoints_Collect(:,1),Reftraj.WayPoints_Collect(:,2),'k-')
hold on
%车辆轨迹
figure(1)
plot(PosX,PosY,'bo')
hold on
pause(0.01);
%绘制绿色的预测路径
global Refer_Path;
if length(Predict_Path) > 1
    figure(1)
    %车身转全局
    Pre_deltax = (Predict_Path(:,1) - Predict_Path(:,2) * tan(PosPhi)) * cos(PosPhi);
    Pre_deltay = Predict_Path(:,2) / cos(PosPhi) + (Predict_Path(:,1) - Predict_Path(:,2) * tan(PosPhi)) * sin(PosPhi);
    Pre_x = Pre_deltax + PosX;
    Pre_y = Pre_deltay + PosY;
    plot(Pre_x,Pre_y,'-g','linewidth',3);
    hold on
end
% 绘制紫色的参考路径
if length(Refer_Path) > 1
    figure(1)
    %车身转全局
    Ref_deltax = (Refer_Path(:,1) - Refer_Path(:,2) * tan(PosPhi)) * cos(PosPhi);
    Ref_deltay = Refer_Path(:,2) / cos(PosPhi) + (Refer_Path(:,1) - Refer_Path(:,2) * tan(PosPhi)) * sin(PosPhi);
    Ref_x = Ref_deltax + PosX;
    Ref_y = Ref_deltay + PosY;
    plot(Ref_x,Ref_y,'-m','linewidth',1);
    hold off
end
sys = [Ctrl_frontwheelangle,e_y,e_phi,PosX,PosY,PosPhi,PosVel,Next_Steer,spend_Time,t];



























  
    
    