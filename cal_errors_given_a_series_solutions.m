function [R_error,t_error] = cal_errors_given_a_series_solutions(R_cal,t_cal,R_gt,t_gt)
%正如题目所描述的，给定一系列的求解得到的R和t，和ground truth，寻找最小的R和t的误差
R_error = 1000;
solution_index = 0;
root_num = size(R_cal);
root_num = root_num(3);
for i=1:root_num/2
% 首先求R距离最小的
% 根据R最小的值选择t最小的值，进行运算
temp_error = R_error_metric(R_gt, R_cal(:,:,2*i-1));% 只对于奇数进行选择
if temp_error<R_error
    R_error = temp_error;
    solution_index = i;
end
end
% 对求得的solution对t进行排除
t_error1 = t_error_metric(t_gt, t_cal(:,:,2*solution_index-1));
t_error2 = t_error_metric(t_gt, t_cal(:,:,2*solution_index));
t_error = min(t_error1, t_error2);
end

