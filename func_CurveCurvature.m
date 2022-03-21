function [curvature] = func_CurveCurvature(local_path_x,local_path_y)
%计算曲线曲率信息
if(length(local_path_x) ~= length(local_path_y))
    error("Length error at curvature")
end
    
size_n = length(local_path_x);
point_curvature = zeros(size_n,1);
curvature = zeros(size_n,1);

for i= 2:1:size_n-1
    x1 = local_path_x(i-1);
    x2 = local_path_x(i);
    x3 = local_path_x(i+1);
    y1 = local_path_y(i-1);
    y2 = local_path_y(i);
    y3 = local_path_y(i+1);
    point_curvature(i) = func_PointCurvature(x1,y1,x2,y2,x3,y3);
end
point_curvature(1) = point_curvature(2);
point_curvature(size_n) = point_curvature(size_n-1);

for i=1:1:size_n
    if(i == 1 || i ==size_n)
        curvature(i)  = point_curvature(i);
    else    
       curvature(i) = (point_curvature(i-1)+point_curvature(i)+point_curvature(i+1))/3;
    end
end
end
function [point_curvature] = func_PointCurvature(x1,y1,x2,y2,x3,y3)
    delta_x = x2 - x1;
    delta_y = y2 - y1;
    a = sqrt(power(delta_x,2)+power(delta_y,2));
    
    delta_x = x3 - x2;
    delta_y = y3 - y2;
    b = sqrt(power(delta_x,2)+power(delta_y,2));
    
    delta_x = x1 - x3;
    delta_y = y1 - y3;
    c = sqrt(power(delta_x,2)+power(delta_y,2));
    
    s = (a + b + c)/2;
    A = sqrt(abs(s * (s - a) * (s - b) * (s - c)));
    point_curvature = 4 * A / (a * b * c);
    %2维空间中的叉乘是： A x B = |A||B|Sin(theta)
    % V1(x1, y1) X V2(x2, y2) = x1y2 – y1x2
    %计算两个向量的叉乘，为正逆时针，为负顺时针
    rotate_direction = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);
    if(rotate_direction <0)
        point_curvature = -point_curvature;
    end
end
