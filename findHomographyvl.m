function [H] = findHomographyvl(choose_points_cam1_ground,choose_points_cam2_ground,vl1,vl2)
%findHomographyvl Summary of this function goes here
%   根据输入的三个点以及一条灭线，及撒un
p1 = choose_points_cam1_ground';
p2 = choose_points_cam2_ground';
for i = 1:3
    A = [p1(i,1), p1(i,2), 1, 0, 0, 0, -p1(i,1)*p2(i,1),-p1(i,2)*p2(i,1);
    0, 0, 0, p1(i,1), p1(i,2), 1, -p1(i,1)*p2(i,2), -p1(i,2)*p2(i,2)];
    b = [p2(i,1);p2(i,2)];
    if i==1
    temp_A = A;
    temp_b = b;
    else
        temp_A = [temp_A;A];
        temp_b = [temp_b;b];
    end
end
A = [vl2(1), 0, -vl1(1)*vl2(1), vl2(2), 0, -vl1(1)*vl2(2), 1, 0;
     0,vl2(1),-vl1(2)*vl2(1), 0, vl2(2), -vl1(2)*vl2(2), 0, 1];
b = [vl1(1);vl1(2)];
temp_A = [temp_A;A];
temp_b = [temp_b;b];
x = temp_A\temp_b;
H = [x(1),x(2),x(3);x(4),x(5),x(6);x(7),x(8),1];
end

