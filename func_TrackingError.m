function [e_phi,e_y] = func_TrackingError(local_path_x,local_path_y)
%����������
if(length(local_path_x) ~= length(local_path_y))
    error("Length error at TrackingError")
end
size_n = length(local_path_x);
min_distance = 1e10;
for i=1:1:size_n
    distance = func_Distance(local_path_x(i),local_path_y(i),0,0);
    if(distance < min_distance)
        min_distance = distance;
        min_index = i;
    end
end
    rear_index = max(min_index - 1,1);
    rear_distance = func_Distance(local_path_x(rear_index),local_path_y(rear_index),0,0);
    front_index = min(min_index +1,size_n);
    front_distance = func_Distance(local_path_x(front_index),local_path_y(front_index),0,0);
    %�ҵ�����ĵ�
    if rear_distance < front_distance
        i = min_index - 1;
    else
        i = min_index;
    end
    i = max(i,1);
    l = func_Distance(local_path_x(i),local_path_y(i),local_path_x(i + 1),local_path_y(i + 1));
    vec1 = [local_path_x(i) local_path_y(i)];
    vec2 = [local_path_x(i + 1)-local_path_x(i) local_path_y(i + 1)-local_path_y(i)];
    e_y = (vec1(1) * vec2(2) - vec1(2) * vec2(1)) / l;  %������·���ʱΪ��
    e_phi = 0 - atan2(local_path_y(i + 1)-local_path_y(i),local_path_x(i + 1)-local_path_x(i));  %����·����ʱ�뷽��Ϊ��
    e_phi = func_NormalizeAngle(e_phi);  %��һ��
    e_phi = e_phi * 180 / pi; %����ת�Ƕ�
end

function [distance] = func_Distance(x1,y1,x2,y2)
distance = sqrt(power(x1 - x2, 2) + power(y1 - y2, 2));
end
function [phi] = func_NormalizeAngle(angle)
%ʹ���ȽǱ�����-pi��pi֮��
phi = rem(angle + pi,2 * pi);
if(phi < 0)
    phi = phi + 2*pi;
end
phi = phi - pi;
end

