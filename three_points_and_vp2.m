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
close all;
%rng(666);
debug = true;
%%%%%%%%%%%%%%%%%%%%%%%%实验添加的噪声设定%%%%%%%%%%%%%%
noise = true;
sigma = 0.05;
angle_noise = 0;
iter_number = 1000;
noise_iter = 20;
n = 1000;  %  平面上采样多少个点


%%%%%%%%%%%%%%%%%%%%%%%%%内外参数设定%%%%%%%%%%%%%%%%%%%%
K1 = eye(3);
K2 = eye(3);
gravity = [0, 1, 0];  % 世界坐标系中的竖直向的方向,也是灭点的方向
% 因为要拍摄到相机的位置，所以朝向不能和垂直方向平面平行
% 相机1相对于世界坐标系的坐标
angle_cam12world = rand(1,1)*pi/12+pi/12;
rotm_cam12world = eul2rotm([rand(1,1)*pi/12, -rand(1,1)*pi/12-pi/12, pi/6]);% 第一个角度转30度，保证能够拍摄到灭点的位置
t_cam12world = [0, 0, 0];
% 三维空间中的点，average distance为5， 这样设定的原因为相机
depth= rand()*2+4;  % 垂直地面的平面的深度为4-6之间
points3D_world = rand(n, 2)*6-1;  % 产生x，y为（0, 8）之内的点
points3D_world = [points3D_world, ones(n,1)*depth, ones(n,1); gravity, 0];% 最后一个元素为灭点


%相机2相对于相机1的变换
% 为了保证位置姿势的多样性，先对cam2相对世界坐标的位置进行变换
angle_cam22world = [ 0, 0, rand(1,1)*pi/12+pi/12];
rotm_cam22world = eul2rotm(angle_cam22world);

angle_cam22cam1 = rand(1,3)*pi/12;
rotm_cam22cam1 = eul2rotm(angle_cam22cam1);
%rotm_cam22cam1 = rotm_cam22world*inv(rotm_cam12world);


t_cam22cam1 = [0.9, rand(1)+0.1, rand(1)+0.1];  % 使得baseline的长度为20%到场景的深度
% 注意以上为在原世界坐标系下的平移向量
t_cam22cam1 = rotm_cam12world*t_cam22cam1';   %对平移向量进行变换
t_cam22cam1 = t_cam22cam1./norm(t_cam22cam1);% 两个相机的baseline为1，20% average distance






R1 = zeros(1,20);
t1 = zeros(1,20);
R2 = zeros(1,20);
t2 = zeros(1,20);


for k = 1:20  % 不同的噪声的程度
% 利用不同的variance控制噪声
pixel_noise_cam1 = normrnd(0,sigma*k,2,n)/1000; %之所以除以1000是因为 
pixel_noise_cam2 = normrnd(0,sigma*k,2,n)/1000; %之所以除以1000是因为按照pami，f为1,单位又为pixel 

% 求取点在cam1下的3D坐标
points3D_cam1 = [rotm_cam12world,t_cam12world']*points3D_world';
points3D_cam1 = [points3D_cam1; ones(1,n), 0];
points2D_cam1 = K1*[rotm_cam12world,t_cam12world']*points3D_world';
points2D_cam1_homo = points2D_cam1./points2D_cam1(3, :);

% 如果添加噪声的话
if noise
   points2D_cam1_homo = points2D_cam1_homo+[pixel_noise_cam1,zeros(2,1); zeros(1,n),0];% 灭点不添加噪声取决于noise最后一项
end

% 以下，将cam1 align到重心的方向
gravity_cam1 = points3D_cam1(1:end-1, end);
theta_x = acos(gravity_cam1(3)/sqrt(gravity_cam1(2)^2+gravity_cam1(3)^2));
theta_y = -sign(gravity_cam1(1))*acos(sqrt(gravity_cam1(3)^2+gravity_cam1(2)^2)/sqrt(gravity_cam1(3)^2+gravity_cam1(2)^2+gravity_cam1(1)^2));
rotm_align_cam1 = eul2rotm([0, theta_y, theta_x]);
% 以下，对于图像1上的点进行变换,方便用pami算法
points2D_cam1_align = K1*rotm_align_cam1*inv(K1)*points2D_cam1_homo;
points2D_cam1_align  = points2D_cam1_align./points2D_cam1_align(3, :);



% 求取点在cam2下的3D坐标
points3D_cam2 = [rotm_cam22cam1, t_cam22cam1]*points3D_cam1;
points3D_cam2 = [points3D_cam2; ones(1,n), 0];
points2D_cam2 = K2*[rotm_cam22cam1, t_cam22cam1]*points3D_cam1;
points2D_cam2_homo = points2D_cam2./points2D_cam2(3, :);
if debug
    figure;
    scatter(points2D_cam1_homo(1, :), points2D_cam1_homo(2, :),40,'MarkerEdgeColor',[0 1 0],...
              'MarkerFaceColor',[0 1 0],...
              'LineWidth',1.5);
    % 添加图像区域
    hold on;
    plot([-1, -1, 1, 1, -1], [1, -1, -1, 1, 1]);
    figure;
    scatter(points2D_cam2_homo(1, :), points2D_cam2_homo(2, :),40,'MarkerEdgeColor',[0 0 1],...
              'MarkerFaceColor',[0 0 1],...
              'LineWidth',1.5);
    hold on;
    plot([-1, -1, 1, 1, -1], [1, -1, -1, 1, 1]);
    123
end


if noise
   points2D_cam2_homo = points2D_cam2_homo+[pixel_noise_cam2,zeros(2,1); zeros(1,n),0];% 灭点不添加噪声取决于noise最后一项
end

% 以下，将cam2 align到重心方向
gravity_cam2 = points3D_cam2(1:end-1, end);
theta_x_cam2 = acos(gravity_cam2(3)/sqrt(gravity_cam2(2)^2+gravity_cam2(3)^2));
theta_y_cam2 = -sign(gravity_cam2(1))*acos(sqrt(gravity_cam2(3)^2+gravity_cam2(2)^2)/sqrt(gravity_cam2(3)^2+gravity_cam2(2)^2+gravity_cam2(1)^2));
rotm_align_cam2 = eul2rotm([0, theta_y_cam2, theta_x_cam2]);
% 以下，对于图像2上的点进行变换，方便用pami算法
points2D_cam2_align = K1*rotm_align_cam2*inv(K1)*points2D_cam2_homo;
points2D_cam2_align  = points2D_cam2_align./points2D_cam2_align(3, :);

% 对以上求得的点，用ransac进行去除噪声

R_error_pami_sta = ones(1,iter_number)*100;
t_error_pami_sta = ones(1,iter_number)*100;
R_error_4points_sta = ones(1,iter_number)*100;
t_error_4points_sta = ones(1,iter_number)*100;
        
for iter=1:iter_number
    index= randperm(n,3);
    index2 = [index, n+1];  % 将最后一个灭点取出，用以我们算法计算
    choose_points_cam1 =  points2D_cam1_homo(:,index2);
    choose_points_cam1_align = points2D_cam1_align(:,index);
    choose_points_cam2 =  points2D_cam2_homo(:,index2);
    choose_points_cam2_align = points2D_cam2_align(:,index);
    
    [R_pami3, t_pami3] = pami3findHomography(choose_points_cam1_align, choose_points_cam2_align);

    % 对求得的解，进行r的变换，以方便和gt进行比较
    root_num = size(R_pami3);
    if size(root_num)<3
        %[R_pami3, t_pami3] = pami3findHomography(choose_points_cam1_align, choose_points_cam2_align);
        % 如果根都为虚的话，则结束本次循环
        continue;
    end
    
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


    % 求取homography以及对求取的homography进行分解
    H = findHomography(choose_points_cam1, choose_points_cam2);
    [R_4points, t_4points] = decomposeHomography(H,K1);

    [R_error_pami,t_error_pami] = cal_errors_given_a_series_solutions(R_pami3_full,t_pami3_full,rotm_cam22cam1,t_cam22cam1);
    [R_error_4points,t_error_4points] = cal_errors_given_a_series_solutions(R_4points,t_4points,rotm_cam22cam1,t_cam22cam1);
    
    
    % 将求取的误差进行累计
    R_error_pami_sta(iter) =  R_error_pami;
    t_error_pami_sta(iter) = t_error_pami;
    R_error_4points_sta(iter) = R_error_4points;
    t_error_4points_sta(iter) = t_error_4points;
    
    
end
R_error_pami_sta = sort(R_error_pami_sta);
t_error_pami_sta = sort(t_error_pami_sta);
R_error_4points_sta = sort(R_error_4points_sta);
t_error_4points_sta = sort(t_error_4points_sta);


result_index = round(iter_number / 5);
k;
R1(k) = R_error_pami_sta(result_index)
t1(k) = t_error_pami_sta(result_index)

R2(k) = R_error_4points_sta(result_index)

t2(k) = t_error_4points_sta(result_index)
result = [R1,t1,R2,t2];
save result_small_angle.mat
end
%load '../our_method/result1.mat';
x = 1:1:k;
plot(x, R1,'-*b',x,R2,'-or');
ylabel('rotation error in degree');
xlabel('noise level');
legend('pami17','ours');

figure;
plot(x, t1,'-*b',x,t2,'-or');
ylabel('translation error in degree');
xlabel('noise level');
legend('pami17','ours');
