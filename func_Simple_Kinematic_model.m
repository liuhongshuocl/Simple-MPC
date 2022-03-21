function [Next_Steer,Predict_Path] = ...
            func_Simple_Kinematic_model(local_path_x,local_path_y,heading,curvature,total_s,segment_arc_s,VehState,t)
%  非线性运动学模型           
global VehiclePara;
global MPCParameters;
global CostWeights;
global Constraints;
%********************************************一.车辆状态********************************************
x_init = 0;
y_init = 0;
yaw_init = 0;
speed = VehState.Vel;
yawrate = VehState.yawrate;
kappa = yawrate/speed;
N = MPCParameters.Np;

%********************************************二.约束及上下界********************************************
x_range_begin = 1;  %车辆坐标 1
y_range_begin = x_range_begin + N;  %车辆坐标 21
yaw_range_begin = y_range_begin + N;  %加速度 41
steer_range_begin = yaw_range_begin + N;  %控制量 61

state_size = MPCParameters.Nx;  %状态量个数
control_size = MPCParameters.Nu;  %控制量个数
n_vars = state_size * N + control_size * (N - 1);  %变量个数 
n_constraints = state_size * N;  %约束量个数

%约束方程
constraint_func = @(vars)MPC_constraint(vars,speed);

%约束方程上下界
constraints_lowerbound = zeros(n_constraints,1);  %约束方程下界
constraints_upperbound = zeros(n_constraints,1);  %约束方程上界

constraints_lowerbound(x_range_begin) = x_init;
constraints_upperbound(x_range_begin) = x_init;

constraints_lowerbound(y_range_begin) = y_init;
constraints_upperbound(y_range_begin) = y_init;

constraints_lowerbound(yaw_range_begin) = yaw_init;
constraints_upperbound(yaw_range_begin) = yaw_init;

%变量初始值
vars_init = zeros(n_vars,1);  %初始化变量
vars_init(x_range_begin) = x_init;
vars_init(y_range_begin) = y_init;
vars_init(yaw_range_begin) = yaw_init;
vars_init(steer_range_begin) = kappa;

%变量上下界
vars_lowerbound = zeros(n_vars,1);  %变量下界
vars_upperbound = zeros(n_vars,1);  %变量上界
for i=1:1:(steer_range_begin - 1)
    vars_lowerbound(i) = -inf;
    vars_upperbound(i) = inf;
end
for i =steer_range_begin:1:n_vars
    vars_lowerbound(i) = Constraints.minsteer;
    vars_upperbound(i) = Constraints.maxsteer;
end
%********************************************三.目标方程********************************************
objective_func = @(vars)MPC_objective_func(vars,local_path_x,local_path_y,heading,kappa);

%********************************************四.调用求解器********************************************
% 设置求解器选项
% opts = optiset('solver','ipopt','display','iter');
opts = optiset('solver','ipopt','maxiter',60,'maxtime',0.5);
% 创建求解对象 目标函数，非线性约束方程，约束下界，约束上界，变量下界，变量上界，初始值
Opt = opti('fun',objective_func,'nl',constraint_func,constraints_lowerbound,constraints_upperbound,...
    'bounds',vars_lowerbound,vars_upperbound,'x0',vars_init,'options',opts);
% 求解
[result,fval,exitflag,info] = solve(Opt);

if exitflag < 0
    Predict_Path=[];
    Next_Steer = 0;
    warn = ['IPOPT Solution failed at X:',num2str(VehState.X),',Y:',num2str(VehState.Y),',t:',num2str(t)];
    disp(warn);
else
    Next_Steer = result(steer_range_begin);
    Predict_Path = zeros(N,3);
    for i =1:1:N
        Predict_Path(i,1) = result(x_range_begin + i - 1);
        Predict_Path(i,2) = result(yaw_range_begin + i - 1);
        Predict_Path(i,3) = result(yaw_range_begin + i - 1);
    end
end
end

function constraint_func = MPC_constraint(vars,speed)
global MPCParameters;

N = MPCParameters.Np;
dt = MPCParameters.Ts;

x_range_begin = 1;  %车辆坐标 1
y_range_begin = x_range_begin + N;  %车辆坐标 21
yaw_range_begin = y_range_begin + N;  %加速度 41
steer_range_begin = yaw_range_begin + N;  %控制量 61

state_size = MPCParameters.Nx;  %状态量个数
control_size = MPCParameters.Nu;  %控制量个数
% n_vars = state_size * N + control_size * (N - 1);  %变量个数 
n_constraints = state_size * N;  %约束量个数

constraint_func = zeros(n_constraints,1);  %初始化约束方程
constraint_func(x_range_begin) = vars(x_range_begin);
constraint_func(y_range_begin) = vars(y_range_begin);
constraint_func(yaw_range_begin) = vars(yaw_range_begin);
for i = 1:1:N-1  %约束方程赋值
    %t+1时刻状态量
    x1 = vars(x_range_begin + i);
    y1 = vars(y_range_begin + i);
    yaw1 = vars(yaw_range_begin + i);
    %t时刻状态量
    x0 = vars(x_range_begin + i - 1);
    y0 = vars(y_range_begin + i - 1);
    yaw0 = vars(yaw_range_begin + i - 1);
    %t时刻控制量 n个状态量，n-1个控制量
    kappa0 = vars(steer_range_begin + i -1);
    
    %填充约束方程
    constraint_func(x_range_begin + i) = x1 - (x0 + speed * cos(yaw0) * dt);
    constraint_func(y_range_begin + i) = y1 - (y0 + speed * sin(yaw0) * dt);
    constraint_func(yaw_range_begin + i) = yaw1 -(yaw0 + speed * kappa0 * dt);
end
end

function objective_func = MPC_objective_func(vars,local_path_x,local_path_y,heading,kappa)
global CostWeights;
global MPCParameters;
global Constraints;
global Refer_Path;

N = MPCParameters.Np;
dt = MPCParameters.Ts;


%对权重进行归一化调整
R_ey = CostWeights.Wey /(Constraints.eymax * Constraints.eymax);
R_ephi = CostWeights.Wephi / (Constraints.ePhimax * Constraints.ePhimax);
R_u = CostWeights.steer / (Constraints.umax * Constraints.umax);
R_du = CostWeights.steer_rate / (Constraints.dumax * Constraints.dumax);

x_range_begin = 1;  %车辆坐标 1
y_range_begin = x_range_begin + N;  %车辆坐标 21
yaw_range_begin = y_range_begin + N;  %加速度 41
steer_range_begin = yaw_range_begin + N;  %控制量 61
Refer_Path = NaN(length(local_path_x),2);
for i= 0:1:N-1
    index = find_referpoint(local_path_x,local_path_y,vars(x_range_begin+i),vars(y_range_begin+i));
    Refer_Path(i+1,1) = local_path_x(index);
    Refer_Path(i+1,2) = local_path_y(index);
    Refer_Path(i+1,3) = heading(index);
end

objective_func = 0;
%控制输入 曲率
for i = 0:1:(N-2)
    objective_func = objective_func + R_u * power(vars(steer_range_begin + i),2);
end

%控制输入变化率
objective_func = objective_func + R_du/20 * power(vars(steer_range_begin) - kappa,2);
for i = 0:1:(N-3)
    objective_func = objective_func + R_du * ...
        power(vars(steer_range_begin + i + 1) - vars(steer_range_begin + i),2);  % 61+17+1=79 
end

%航向偏差与横向偏差
for i = 0:1:(N-1)
    objective_func = objective_func + R_ey * ...
        (power(vars(x_range_begin + i) - Refer_Path(i + 1,1),2) + power(vars(y_range_begin + i) - Refer_Path(i+1 ,2),2));
     objective_func = objective_func + R_ephi * ...
         power(vars(yaw_range_begin + i) - Refer_Path(i +1,3),2);
end
end
%找到预测点的跟踪点
function index = find_referpoint(local_path_x,local_path_y,predict_x,predict_y)
dist_min = 1e10;
path_Length = length(local_path_x);
for i=1:1:path_Length
    deltax = local_path_x(i) - predict_x;
    deltay = local_path_y(i) - predict_y;
    dist = sqrt(power(deltax,2) + power(deltay,2));
    if dist < dist_min
        dist_min = dist;
        index = i;
    end
end
% index = index+1;
end