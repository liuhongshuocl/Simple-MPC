function [heading] = func_CurveHeading(local_path_x,local_path_y)
%计算曲线参考航向信息
size_n = length(local_path_x);
line_heading = zeros(size_n-1,1);
delta_heading = zeros(size_n,1);
heading = zeros(size_n,1);

if(size_n < 2 || length(local_path_x) ~= length(local_path_y))
    error("Unable to get heading information")
end

for i=1:1:(size_n -1)
    line_heading(i) = atan2(local_path_y(i+1)-local_path_y(i),local_path_x(i+1)-local_path_x(i));
end

for i=2:1:(size_n -1)
    delta_heading(i) = (line_heading(i) - line_heading(i-1))/2;
end
delta_heading(1) = delta_heading(2);
delta_heading(size_n) = delta_heading(size_n-1);

for i = 1:1:size_n - 1
    heading(i) = line_heading(i) - delta_heading(i);
end
heading(size_n) = line_heading(size_n -1) + delta_heading(size_n);
end
    

