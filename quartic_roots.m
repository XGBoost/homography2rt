%
%                      quartic_roots Version 1.0
%
%           		       Yaguang Yang
%                      
%
%                          January 2014
%
%  Copyright (c) 2014 by Y. Yang
%  All rights reserved.
%
% This function finds analytic solutions for quartic function. It implements 
% the formulas described in [YZ]. The largest roots of a quartic function 
% can be used for Wahba's problem.
%
% Synopsis:
%            [x1,x2,x3,x4,xmax] = quartic_roots(a,b,c,d)
% Input arguments:
%   (a,b,c,d): the coefficient of the quartic equation.
%
% Output arguments:
%   (x1,x2,x3,x4): are four roots of the quartic function
%   xmax: is the maximum real root of the quartic
%
% References:
% [YZ] Yaguang Yang, Zhiqiang Zhou, An analytic solution to Wahba�s problem,
% Aerospace Science and Technology 30 (2013) 46�49.
%
%
%                   Conditions for External Use
%                   ===========================
%
%   1. If modifications are performed on the code, these modifications 
%      shall be  communicated to  the author.  The modified  routines 
%      will  remain the sole  property of the authors.
%   2. Due  acknowledgment  must  be  made of the  use of this code in 
%      research  reports  or  publications.  Whenever such reports are 
%      released for  public  access, a copy should be forwarded to the 
%      authors.
%   3. This code may only be used for research and education, unless 
%      it has been agreed  otherwise with the authors in writing.
% 假设该公式的最高次为1,从而完成对该方程的求解
function [x1,x2,x3,x4,xmax] = quartic_roots(a,b,c,d)


epsilon=10^(-6);

p=a*c-b^2/3-4*d;
q=a*b*c/3-a^2*d-2*b^3/27-c^2+8*b*d/3;
i=sqrt(-1);
w1=double((-1+i*sqrt(3))/2);
w2=double((-1-i*sqrt(3))/2);

y1=(-q/2+sqrt((q/2)^2+(p/3)^3))^(1/3)+(-q/2-sqrt((q/2)^2+(p/3)^3))^(1/3);
y2=w1*(-q/2+sqrt((q/2)^2+(p/3)^3))^(1/3)+w2*(-q/2-sqrt((q/2)^2+(p/3)^3))^(1/3);
y3=w2*(-q/2+sqrt((q/2)^2+(p/3)^3))^(1/3)+w1*(-q/2-sqrt((q/2)^2+(p/3)^3))^(1/3);

x=[];
if imag(y1)==0
    g1=(a+sqrt(a^2-8*b/3+4*y1))/2;
    g2=(a-sqrt(a^2-8*b/3+4*y1))/2;
    h1=(y1+b/3+sqrt((y1+b/3)^2-4*d))/2;
    h2=(y1+b/3-sqrt((y1+b/3)^2-4*d))/2;
    if abs(g1*h2+g2*h1-c)>epsilon
        h=h1; h1=h2; h2=h;
    end
    if abs(g1+g2-a)<epsilon & abs(g1*g2+h1+h2-b)<epsilon ...
	& abs(g1*h2+g2*h1-c)<epsilon & abs(h1*h2-d)<epsilon
    	x1=(-g1+sqrt(g1^2-4*h1))/2;
    	x2=(-g1-sqrt(g1^2-4*h1))/2;
    	x3=(-g2+sqrt(g2^2-4*h2))/2;
    	x4=(-g2-sqrt(g2^2-4*h2))/2;
    	if imag(x1)==0
        	x=[x x1];
    	end
    		if imag(x2)==0
        x=[x x2];
    	end
    	if imag(x3)==0
        	x=[x x3];
    	end
    	if imag(x4)==0
        	x=[x x4];
    	end
    end
    
    xmax = max(x);

elseif imag(y2)==0
    g1=(a+sqrt(a^2-8*b/3+4*y2))/2;
    g2=(a-sqrt(a^2-8*b/3+4*y2))/2;
    h1=(y2+b/3+sqrt((y2+b/3)^2-4*d))/2;
    h2=(y2+b/3-sqrt((y2+b/3)^2-4*d))/2;
    if abs(g1*h2+g2*h1-c)>epsilon
        h=h1; h1=h2; h2=h;
    end
    if abs(g1+g2-a)<epsilon & abs(g1*g2+h1+h2-b)<epsilon ...
	& abs(g1*h2+g2*h1-c)<epsilon & abs(h1*h2-d)<epsilon
    	x1=(-g1+sqrt(g1^2-4*h1))/2;
    	x2=(-g1-sqrt(g1^2-4*h1))/2;
    	x3=(-g2+sqrt(g2^2-4*h2))/2;
    	x4=(-g2-sqrt(g2^2-4*h2))/2;
    	if imag(x1)==0
        x=[x x1];
    	end
    	if imag(x2)==0
        	x=[x x2];
    	end
    	if imag(x3)==0
        	x=[x x3];
    	end
    	if imag(x4)==0
        	x=[x x4];
        end
    end
    
    xmax = max(x);

elseif imag(y3)==0
    g1=(a+sqrt(a^2-8*b/3+4*y3))/2;
    g2=(a-sqrt(a^2-8*b/3+4*y3))/2;
    h1=(y3+b/3+sqrt((y3+b/3)^2-4*d))/2;
    h2=(y3+b/3-sqrt((y3+b/3)^2-4*d))/2;
    if abs(g1*h2+g2*h1-c)>epsilon
        h=h1; h1=h2; h2=h;
    end
    if abs(g1+g2-a)<epsilon & abs(g1*g2+h1+h2-b)<epsilon ...
	& abs(g1*h2+g2*h1-c)<epsilon & abs(h1*h2-d)<epsilon
    	x1=(-g1+sqrt(g1^2-4*h1))/2;
    	x2=(-g1-sqrt(g1^2-4*h1))/2;
    	x3=(-g2+sqrt(g2^2-4*h2))/2;
    	x4=(-g2-sqrt(g2^2-4*h2))/2;
    	if imag(x1)==0
        	x=[x x1];
    	end
    	if imag(x2)==0
        	x=[x x2];
    	end
    	if imag(x3)==0
        	x=[x x3];
    	end
    	if imag(x4)==0
        	x=[x x4];
        end
    end
    
    xmax = max(x);

end






















