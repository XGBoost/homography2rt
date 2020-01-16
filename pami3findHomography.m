function [R_total,t_total] = pami3findHomography(points2D_cam1_align,points2D_cam2_align)
% 该算法为pami17文章讨论的算法的第三种情况
% 输入2.5组对应点的信息
% 根据三组对应点求取R和t，解的个数理论上有8个，有的时候有两个虚根，所以有时候可能为4个
x1_i = points2D_cam1_align(:, 1);
x1_j = points2D_cam2_align(:, 1);
x2_i = points2D_cam1_align(:, 2);
x2_j = points2D_cam2_align(:, 2);
x3_i = points2D_cam1_align(:, 3);
x3_j = points2D_cam2_align(:, 3);

a1 = x1_j(3)*x1_i(1);
b1 = x1_j(3)*x1_i(2);
c1 = -x1_i(3)*x1_j(2);
d1 = x1_i(3)*x1_j(1);

temp1 = [0, 0, -a1, -b1, x1_i(1)*x1_j(2), x1_i(2)*x1_j(2)];
temp2 = [a1, b1, 0, 0, -x1_i(1)*x1_j(1), -x1_j(1)*x1_i(2)];

a2 = x2_j(3)*x2_i(1);
b2 = x2_j(3)*x2_i(2);
c2 = -x2_i(3)*x2_j(2);
d2 = x2_i(3)*x2_j(1);

temp3 = [0, 0, -a2, -b2, x2_i(1)*x2_j(2), x2_i(2)*x2_j(2)];
temp4 = [a2, b2, 0, 0, -x2_i(1)*x2_j(1), -x2_j(1)*x2_i(2)];

a3 = x3_j(3)*x3_i(1);
b3 = x3_j(3)*x3_i(2);
c3 = -x3_i(3)*x3_j(2);
d3 = x3_i(3)*x3_j(1);

temp5 = [0, 0, -a3, -b3, x3_i(1)*x3_j(2), x3_i(2)*x3_j(2)];


A = cat(1,temp1, temp2, temp3, temp4, temp5);
b = [c1, d1, c2, d2, c3]';


[U, S, V] = svd(A);
b_ = U'*b;
y = [b_(1)/S(1,1), b_(2)/S(2,2), b_(3)/S(3,3), b_(4)/S(4,4),b_(5)/S(5,5) 0]';
temp11 = V*y;
temp22 = V(:, end);


root =  Quarticsolution(temp11, temp22);


% 用符号运算进行化简
% syms lambda;
% h = temp11+lambda*temp22;
% f =  h(1)^2*h(6)^2-2*h(1)*h(2)*h(5)*h(6)+h(2)^2*h(5)^2+h(3)^2*h(6)^2-2*h(3)*h(4)*h(5)*h(6)+h(4)^2*h(5)^2-h(5)^2-h(6)^2;
% root2 = double(solve(f))
% 


root_size = size(root);

% 以下，理论上有八个解（4×2），但是有时候会有虚根直接排除
count=0;
R_total = eye(3);
t_total = zeros(3,1);
for i =1:root_size(1)
    if isreal(root(i)) == 1
        h = temp11 + root(i)*temp22;
        A*h-b;
        tz = -sqrt(h(5)^2+h(6)^2);
        for j = 1:2
           tz = -tz;
           nx = -h(5)/tz;
           ny = -h(6)/tz;
           tx = nx*(h(4)-h(1))-ny*(h(2)+h(3));
           ty = ny*(h(1)-h(4))-nx*(h(2)+h(3));
           cos_predict = h(1)+nx*tx;
           R = [cos_predict, -sqrt(1-cos_predict^2), 0; 
               sqrt(1-cos_predict^2), cos_predict, 0; 0, 0, 1];
           t_predict = [tx, ty, tz]';
           %tran ./ t_predict
           %disp(cos_predict)
           %disp(t_predict)
           if count==0% 将根
                count = count+1;
                R_total = R;
                t_total = t_predict;
           else
               count = count+1;
               R_total = cat(3,R_total, R);
               t_total = cat(3,t_total,t_predict);
           end
           
       end
    end
end

end

