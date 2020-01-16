% 数值平面上的三个点+数值平面上的两条线的交点

% sideways motion

%%%%%%%%%%%%%pami文章中的设定%%%%%%%%%%%%%
% baseline 0.2, cam1距离scene的average distance为1
% 一共两个plane，一个是地面的plane一个是vertical的plane
% 两个plane都采集200个点
% 焦距为1000个pixel，长度为1（我们的设定），添加noise的时候注意
% field of view为45度,我们这里设定为90度
% 
clear;
clc;
K1 = eye(3);
K2 = eye(3);

% 因为要拍摄到相机的位置，所以朝向不能和垂直方向平面平行
for k = 1:1000
    k
gravity = [0, 1, 0];  % 世界坐标系中的竖直向的方向,也是灭点的方向
angle_cam12world = rand(1,3)*pi/12+pi/12;
%angle_cam12world = [-rand()*pi/12-pi/12, angle_cam12world]
rotm_cam12world = eul2rotm(angle_cam12world);
t_cam12world = [0, 0, 0];

% 三维空间中的点，average distance为5， 这样设定的原因为相机
% 的内参为1,到场景的average distance设置为1不太合适
% 两个相机的baseline为1

z = rand()*2+4;  % 垂直地面的平面的深度为4-6之间
z = [z;z;z];
points3D_world = rand(3, 2)*6+(-3);  % 产生x，y为（-3, 3）之内的点
points3D_world = [points3D_world, z, ones(3,1); gravity, 0];



% 求取点在cam1下的3D坐标
points3D_cam1 = [rotm_cam12world,t_cam12world']*points3D_world';
points3D_cam1 = [points3D_cam1; ones(1,3), 0];
points2D_cam1 = K1*[rotm_cam12world,t_cam12world']*points3D_world';
points2D_cam1_homo = points2D_cam1./points2D_cam1(3, :);

% 以下，将cam1 align到重心的方向
gravity_cam1 = points3D_cam1(1:end-1, end);
theta_x = acos(gravity_cam1(3)/sqrt(gravity_cam1(2)^2+gravity_cam1(3)^2));
theta_y = -sign(gravity_cam1(1))*acos(sqrt(gravity_cam1(3)^2+gravity_cam1(2)^2)/sqrt(gravity_cam1(3)^2+gravity_cam1(2)^2+gravity_cam1(1)^2));
rotm_align_cam1 = eul2rotm([0, theta_y, theta_x]);
% 以下，对于图像1上的点进行变换,方便用pami算法
points2D_cam1_align = K1*rotm_align_cam1*inv(K1)*points2D_cam1_homo;
points2D_cam1_align  = points2D_cam1_align./points2D_cam1_align(3, :);





% 两相机之间的参数
angle_cam22cam1 = rand(1,3)*pi/12+pi/12;
rotm_cam22cam1 = eul2rotm(angle_cam22cam1);
t_cam22cam1 = [0.9, rand(1)+0.1, rand(1)+0.1];  % 使得baseline的长度为20%到场景的深度
% 注意以上为在原世界坐标系下的平移向量
t_cam22cam1 = rotm_cam12world*t_cam22cam1';   %对平移向量进行变换
t_cam22cam1 = t_cam22cam1./norm(t_cam22cam1);



% 求取点在cam2下的3D坐标
points3D_cam2 = [rotm_cam22cam1, t_cam22cam1]*points3D_cam1;
points3D_cam2 = [points3D_cam2; ones(1,3), 0];
points2D_cam2 = K2*[rotm_cam22cam1, t_cam22cam1]*points3D_cam1;
points2D_cam2_homo = points2D_cam2./points2D_cam2(3, :);

% 以下，将cam2 align到重心方向
gravity_cam2 = points3D_cam2(1:end-1, end);
theta_x_cam2 = acos(gravity_cam2(3)/sqrt(gravity_cam2(2)^2+gravity_cam2(3)^2));
theta_y_cam2 = -sign(gravity_cam2(1))*acos(sqrt(gravity_cam2(3)^2+gravity_cam2(2)^2)/sqrt(gravity_cam2(3)^2+gravity_cam2(2)^2+gravity_cam2(1)^2));
rotm_align_cam2 = eul2rotm([0, theta_y_cam2, theta_x_cam2]);
% 以下，对于图像2上的点进行变换，方便用pami算法
points2D_cam2_align = K1*rotm_align_cam2*inv(K1)*points2D_cam2_homo;
points2D_cam2_align  = points2D_cam2_align./points2D_cam2_align(3, :);

[R_pami3, t_pami3] = pami3findHomography(points2D_cam1_align, points2D_cam2_align);

% 对求得的解，进行r的变换，以方便和gt进行比较
root_num = size(R_pami3);
root_num = root_num(3);
for i =1:root_num
    R_temp = inv(rotm_align_cam2)*R_pami3(:,:,i)*rotm_align_cam1;
    t_temp = inv(rotm_align_cam2)*t_pami3(:,:,i);
    %t_cam22cam1./t_temp;
    if i ==1
        R_pami3_full = R_temp;
        t_pami3_full = t_temp;
    else
        R_pami3_full = cat(3, R_pami3_full, R_temp);
        t_pami3_full = cat(3, t_pami3_full, t_temp);
    end
        
end
R_pami3_full;
t_pami3_full;

% 求取homography以及对求取的homography进行分解
H = findHomography(points2D_cam1_homo, points2D_cam2_homo);
[R_4points, t_4points] = decomposeHomography(H,K1);

[R_error_pami,t_error_pami] = cal_errors_given_a_series_solutions(R_pami3_full,t_pami3_full,rotm_cam22cam1,t_cam22cam1);
[R_error_4points,t_error_4points] = cal_errors_given_a_series_solutions(R_4points,t_4points,rotm_cam22cam1,t_cam22cam1);
%rotm_cam22cam1
%t_cam22cam1 ./ t4

%points3D_cam1
%angle_cam12world/pi*180

end





