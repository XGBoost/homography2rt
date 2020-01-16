function error = R_error_metric(R_gt,R_est)
%旋转角度的误差度量方式
%参考pami文中的写法
error = acos((trace(R_gt*R_est')-1)/2.0);
error = error*180/pi;
end

