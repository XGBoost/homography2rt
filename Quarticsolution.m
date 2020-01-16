function [root] = Quarticsolution(temp11,temp22)
%UNTITLED7 Summary of this function goes here
%   一元四次方程的解或者根
% 用解析解进行求解
a1 = temp11(1);
a2 = temp11(2);
a3 = temp11(3);
a4 = temp11(4);
a5 = temp11(5);
a6 = temp11(6);


b1 = temp22(1);
b2 = temp22(2);
b3 = temp22(3);
b4 = temp22(4);
b5 = temp22(5);
b6 = temp22(6);

term10 = a1^2*a6^2;
term11 = 2*a1^2*a6*b6+2*a1*b1*a6^2;
term12 = a1^2*b6^2+b1^2*a6^2+4*a1*b1*a6*b6;
term13 = 2*b1^2*a6*b6+2*a1*b1*b6^2;
term14 = b1^2*b6^2;

term30 = a2^2*a5^2;
term31 = 2*a2^2*a5*b5+2*a2*b2*a5^2;
term32 = a2^2*b5^2+b2^2*a5^2+4*a2*b2*a5*b5;
term33 = 2*b2^2*a5*b5+2*a2*b2*b5^2;
term34 = b2^2*b5^2;

term40 = a3^2*a6^2;
term41 = 2*a3^2*a6*b6+2*a3*b3*a6^2;
term42 = a3^2*b6^2+b3^2*a6^2+4*a3*b3*a6*b6;
term43 = 2*b3^2*a6*b6+2*a3*b3*b6^2;
term44 = b3^2*b6^2;

term60 = a4^2*a5^2;
term61 = 2*a4^2*a5*b5+2*a4*b4*a5^2;
term62 = a4^2*b5^2+b4^2*a5^2+4*a4*b4*a5*b5;
term63 = 2*b4^2*a5*b5+2*a4*b4*b5^2;
term64 = b4^2*b5^2;


term20 = -2*(a1*a2*a5*a6);
term21 = -2*(a1*a2*a5*b6+a1*a2*b5*a6+a1*b2*a5*a6+b1*a2*a5*a6);
term22 = -2*(a1*a2*b5*b6+a1*b2*a5*b6+a1*b2*b5*a6+b1*a2*a5*b6+b1*a2*b5*a6+b1*b2*a5*a6);
term23 = -2*(a1*b2*b5*b6+b1*a2*b5*b6+b1*b2*a5*b6+b1*b2*b5*a6);
term24 = -2*(b1*b2*b5*b6);

term50 = -2*(a3*a4*a5*a6);
term51 = -2*(a3*a4*a5*b6+a3*a4*b5*a6+a3*b4*a5*a6+b3*a4*a5*a6);
term52 = -2*(a3*a4*b5*b6+a3*b4*a5*b6+a3*b4*b5*a6+b3*a4*a5*b6+b3*a4*b5*a6+b3*b4*a5*a6);
term53 = -2*(a3*b4*b5*b6+b3*a4*b5*b6+b3*b4*a5*b6+b3*b4*b5*a6);
term54 = -2*(b3*b4*b5*b6);

term70 = -1*(a5^2);
term71 = -1*(2*a5*b5);
term72 = -1*(b5^2);
term73 = -1*0;
term74 = -1*0;

term80 = -1*(a6^2);
term81 = -1*(2*a6*b6);
term82 = -1*(b6^2);
term83 = -1*0;
term84 = -1*0;


term0 = term10+term20+term30+term40+term50+term60+term70+term80;
term1 = term11+term21+term31+term41+term51+term61+term71+term81;
term2 = term12+term22+term32+term42+term52+term62+term72+term82;
term3 = term13+term23+term33+term43+term53+term63+term73+term83;
term4 = term14+term24+term34+term44+term54+term64+term74+term84;

p = [term4, term3, term2, term1, term0];
root = roots(p);
end

