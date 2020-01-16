function [R_total,t_total] = pami1findHomography(points2D_cam1_align,points2D_cam2_align)
%pami1findHomograph Summary of this function goes here
%   根据地面上的两个点，利用怕，pami文章中的case1的情况来进行求解
x1_i = points2D_cam1_align(:, 1);
x1_j = points2D_cam2_align(:, 1);
x2_i = points2D_cam1_align(:, 2);
x2_j = points2D_cam2_align(:, 2);

temp1 = [-x1_j(3)*x1_i(2), -x1_j(3)*x1_i(1), 0, -x1_i(3)*x1_j(3), x1_i(3)*x1_j(2)];
temp2 = [x1_j(3)*x1_i(1), -x1_j(3)*x1_i(2), x1_i(3)*x1_j(3), 0, -x1_i(3)*x1_j(1)];

temp3 = [-x2_j(3)*x2_i(2), -x2_j(3)*x2_i(1), 0, -x2_i(3)*x2_j(3), x2_i(3)*x2_j(2)];
temp4 = [x2_j(3)*x2_i(1), -x2_j(3)*x2_i(2), x2_i(3)*x2_j(3), 0, -x2_i(3)*x2_j(1)];

A = cat(1,temp1, temp2, temp3, temp4);
h = null(A);

h = h / (-sqrt(h(1)^2+h(2)^2));

count = 0;
for i=1:2
    h = -h;
    R = [h(1), -h(2), 0; h(2), h(1), 0; 0, 0, 1];
    t = [h(3),h(4), h(5)-1];
    if count == 0
        count = count+1;
        R_total = R;
        t_total = t;
    else
        count = count+1;
        R_total = cat(3,R_total, R);
        t_total = cat(3,t_total, t);
    end
    end

end

