function [total_s,segment_arc_s,predict_length] = func_CurveLength(local_path_x,local_path_y,s_limit)
% ����ο����ߵ����廡���ͷֶλ���
path_length = length(local_path_x);
s = 0; %�ܳ���
delta_s = 0; %�ֶγ���
total_s = [];
segment_arc_s = [];
for i=1:1:path_length
    if(i ~= 1)
        delta_s = sqrt(power(local_path_x(i)-local_path_x(i-1),2) +power(local_path_y(i)-local_path_y(i-1),2));
    end
    s = s + delta_s;
    total_s = [total_s;s];
    segment_arc_s = [segment_arc_s;delta_s];
    if s > s_limit
        break;
    end
end
    predict_length = length(total_s);
end
        