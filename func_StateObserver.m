function VehState = func_StateObserver(CarsimOutput)
% 观测车辆状态，保留两位小数
    VehState.X = round(CarsimOutput(1),2);  %全局坐标X位置
    VehState.Y = round(CarsimOutput(2),2);  %全局坐标Y位置
    VehState.phi = round(CarsimOutput(3),2)*pi/180;  %航向角
    VehState.Vel = round(CarsimOutput(4),2)/3.6;  %速度
    VehState.yawrate = round(CarsimOutput(5),2);  %车辆角速度
    VehState.fwa = round(0.5*(CarsimOutput(6)+CarsimOutput(7)),2);  %前轮偏转角