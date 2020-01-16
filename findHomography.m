function [H] = findHomography(points2D_cam1_homo,points2D_cam2_homo)
% 根据对应点求取homography
%   Detailed explanation goes here
p1 = points2D_cam1_homo';
p2 = points2D_cam2_homo';
for i=1:4
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

x = temp_A\temp_b;
H = [x(1),x(2),x(3);x(4),x(5),x(6);x(7),x(8),1];
end
