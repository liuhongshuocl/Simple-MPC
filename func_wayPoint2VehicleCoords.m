function [local_path_x,local_path_y] = func_wayPoint2VehicleCoords(VehState,Reftraj)
% ��ȫ������ϵת��Ϊ��������ϵ ���ҵ�����
global run_flag;

index_min = -1;
path_Length = length(Reftraj(:,1));
dist_min = 1e10;
local_path_x = [];
local_path_y = [];
%�ҵ����복������Ĳο���
for i=1:1:path_Length
    deltax = Reftraj(i,1) - VehState.X;
    deltay = Reftraj(i,2) - VehState.Y;
    dist = sqrt(power(deltax,2) + power(deltay,2));
    if dist < dist_min
        dist_min = dist;
        index_min = i;
    end
end

%������ֹ����
if (index_min < 0 || (index_min + 3) >  path_Length)
    run_flag = false;
    disp("Index error or reference path termination");
    return;
end

%��������֮���ȫ������ϵתΪ�ֲ�����ϵ ��ƽ������ת
for i=index_min:1:path_Length
    deltax = Reftraj(i,1) - VehState.X;
    deltay = Reftraj(i,2) - VehState.Y;
    next_x = deltax * cos(VehState.phi) + deltay * sin(VehState.phi);
    next_y = deltay * cos(VehState.phi) - deltax * sin(VehState.phi);
    local_path_x = [local_path_x;next_x];
    local_path_y = [local_path_y;next_y];
end
run_flag = true;
    

