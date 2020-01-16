function [condition] = judgedistance(choose_points, threshold)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here   
num = size(choose_points);
num = num(2);% 有多少点的数目
if num==4
    point1 = choose_points(1:2, 1);
    point2 = choose_points(1:2, 2);
    point3 = choose_points(1:2, 3);
    point4 = choose_points(1:2, 4);

    delta12 = norm(point2-point1);
    delta23 = norm(point3-point2);
    delta34 = norm(point4-point3);
    delta41 = norm(point1-point4);

    if (delta12>threshold) & (delta23>threshold) & (delta34>threshold) & (delta41>threshold)
        condition=true;
    else
        condition=false;
    end
elseif num ==3
    point1 = choose_points(1:2, 1);
    point2 = choose_points(1:2, 2);
    point3 = choose_points(1:2, 3);
   
    delta12 = norm(point2-point1);
    delta23 = norm(point3-point2);
    delta31 = norm(point1-point3);
 

    if (delta12>threshold) & (delta23>threshold) & (delta31>threshold) 
        condition=true;
    else
        condition=false;
    end
    
    
end

end

