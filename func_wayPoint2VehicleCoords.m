function [local_path_x,local_path_y] = func_wayPoint2VehicleCoords(VehState,Reftraj)
% 将全局坐标系转换为车辆坐标系 并找到索引
global run_flag;

index_min = -1;
path_Length = length(Reftraj(:,1));
dist_min = 1e10;
local_path_x = [];
local_path_y = [];
%找到距离车辆最近的参考点
for i=1:1:path_Length
    deltax = Reftraj(i,1) - VehState.X;
    deltay = Reftraj(i,2) - VehState.Y;
    dist = sqrt(power(deltax,2) + power(deltay,2));
    if dist < dist_min
        dist_min = dist;
        index_min = i;
    end
end

%设置终止条件
if (index_min < 0 || (index_min + 3) >  path_Length)
    run_flag = false;
    disp("Index error or reference path termination");
    return;
end

%将索引点之后的全局坐标系转为局部坐标系 先平移再旋转
for i=index_min:1:path_Length
    deltax = Reftraj(i,1) - VehState.X;
    deltay = Reftraj(i,2) - VehState.Y;
    next_x = deltax * cos(VehState.phi) + deltay * sin(VehState.phi);
    next_y = deltay * cos(VehState.phi) - deltax * sin(VehState.phi);
    local_path_x = [local_path_x;next_x];
    local_path_y = [local_path_y;next_y];
end
run_flag = true;
    

