function [R_4points, t_4points] = decomposeHomography(H,K)
%给定一个homography以及内参矩阵，返回四组对应的解
%    该算法完全按照opencv以及inria提出的算法
%    经过单独debug，大部分情况都是对的，
%    必须要满足一个约束，就是法向量必须相对于第一个相机是朝内的
%    more details please see homo2rt
H = inv(K)*H*K; % 进行normalize
[W, U, V] = svd(H);
H = H / U(2,2);

S = H'*H - eye(3);
M00 = -(S(2,2)*S(3,3)-S(2,3)*S(3,2));
M11 = -(S(1,1)*S(3,3)-S(1,3)*S(3,1));
M22 = -(S(1,1)*S(2,2)-S(1,2)*S(2,1));

rtM00 = sqrt(M00);
rtM11 = sqrt(M11);
rtM22 = sqrt(M22);

M01 = -(S(2,1)*S(3,3)-S(3,1)*S(2,3));
M12 = -(S(1,1)*S(3,2)-S(1,2)*S(3,1));
M02 = -(S(2,1)*S(3,2)-S(2,2)*S(3,1));

e12 = sign(M12);
if M12 == 0
    e12 = 1;
end

e02 = sign(M02);
if M02 == 0
    e02 = 1;
end

e01 = sign(M01);
if M01 == 0
    e01 = 1;
end

nS00 = abs(S(1,1));
nS11 = abs(S(2,2));
nS22 = abs(S(3,3));



% find the max |Sij|
indx =0;
if nS00 < nS11
   indx = 1;
   if nS11<nS22
       indx=2;
   end
else
    if nS00 <nS22
        indx=2;
    end
end


% cal npa and npb
npa = zeros(3, 1);
npb = zeros(3, 1);
switch indx
    case 0
        npa(1) = S(1,1);
        npb(1) = S(1,1);
        npa(2) = S(1,2)+rtM22;
        npb(2) = S(1,2)-rtM22;
        npa(3) = S(1,3)+e12*rtM11;
        npb(3) = S(1,3)-e12*rtM11;
    case 1
        npa(1) = S(1,2)+rtM22;
        npb(1) = S(1,2)-rtM22;
        npa(2) = S(2,2);
        npb(2) = S(2,2);
        npa(3) = S(2,3)-e02*rtM00;
        npb(3) = S(2,3)+e02*rtM00;
    case 2
        npa(1) = S(1,3)+e01*rtM11;
        npb(1) = S(1,3)-e01*rtM11;
        npa(2) = S(2,3)+rtM00;
        npb(2) = S(2,3)-rtM00;
        npa(3) = S(3,3);
        npb(3) = S(3,3);
end

traceS = S(1,1)+S(2,2)+S(3,3);
v = 2.0*sqrt(1+traceS-M00-M11-M22);
ESii = sign(S(indx+1, indx+1));

if ESii == 0
ESii = 1; 
end
r_2 = 2 + traceS + v;
nt_2 = 2 + traceS - v;

r = sqrt(r_2);
n_t = sqrt(nt_2);

na = npa / norm(npa);
nb = npb / norm(npb);

half_nt = 0.5 * n_t;
esii_t_r = ESii * r;

ta_star = half_nt * (esii_t_r * nb - n_t * na);
tb_star = half_nt * (esii_t_r * na - n_t * nb);


% case 1
Ra = H*(eye(3)-(2.0/v)*ta_star*na');
if det(Ra)<0
    Ra = Ra * (-1);
end
ta = Ra * ta_star;
%ta_ratio = ta./t';
na = na;

Ra2 = Ra;
ta2 = -ta;
na2 = -na;

% case2

Rb = H*(eye(3)-(2.0/v)*tb_star*nb');
if det(Rb)<0
    Rb = Rb * (-1);
end
tb = Rb*tb_star;
%tb_ratio = tb./t';
nb = nb;
%R_gt;

Rb2 = Rb;
tb2 = -tb;
nb = -nb;
%{
if (sum(sum(abs(R_gt-Ra))) < 1e-10) & sum(abs(ta-t'))< 1e-10
    Ra;
    ta;
    count=count+1;
else
    if (sum(sum(abs(R_gt-Ra))) < 1e-10) & sum(abs(-ta-t'))< 1e-10
    Ra;
    -ta;
    count = count+1;
    else
        if (sum(sum(abs(R_gt-Rb))) < 1e-10) & sum(abs(tb-t'))< 1e-10
            Rb;
            tb;
            count = count+1;
        else
            if (sum(sum(abs(R_gt-Rb))) < 1e-10) & sum(abs(-tb-t'))< 1e-10
                Rb;
                -tb;
                count = count+1;
            end
        end
    end
end
%}
R1 =  Ra;
t1 =  ta;
R2 =  Ra;
t2 = -ta;
R3 =  Rb;
t3 =  tb;
R4 =  Rb;
t4 = -tb;
R_4points=cat(3, R1, R2, R3, R4);
t_4points=cat(3, t1, t2, t3, t4);
end

