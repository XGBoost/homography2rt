function error = t_error_metric(t_gt,t_est)
%度量估计得到的t和真实的t之间角度误差
% 按照pami文章给的误差度量方式
error = acos((t_gt'*t_est)/(norm(t_gt)*norm(t_est)));
error = error*180/pi;
end

