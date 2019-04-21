

% function ForwardKinematics()

%theta position taken from simulink model
%outputs 1988X1 table for each t(1-12) or dt(1-12)
th1= t1;
th2= t2;
th3= t3;
th4= t4;
th5= t5;
th6= t6;
th7= t7;
th8= t8;
th9= t9;
th10= t10;
th11= t11;
th12= t12;

d_t1= dt1;
d_t2= dt2;
d_t3= dt3;
d_t4= dt4;
d_t5= dt5;
d_t6= dt6;
d_t7= dt7;
d_t8= dt8;
d_t9= dt9;
d_t10= dt10;
d_t11= dt11;
d_t12= dt12;

L = 10;

% DH table constants
A_1 = 0;     alpha1 = 0;             d1 = L;  
A_2 = 0;     alpha2 = -sym(pi)/2;    d2 = 0;  
A_3 = 0;     alpha3 = sym(pi)/2;     d3 = 0;   
A_4 = 0;     alpha4 = -sym(pi)/2;    d4 = L;   
A_5 = 0;     alpha5 = sym(pi)/2;     d5 = 0;   
A_6 = 0;     alpha6 = -sym(pi)/2;    d6 = L;   
A_7 = 0;     alpha7 = sym(pi)/2;     d7 = 0;   
A_8 = 0;     alpha8 = -sym(pi)/2;    d8 = L;  
A_9 = 0;     alpha9 = sym(pi)/2;     d9 = 0;   
A_10 = 0;    alpha10 = -sym(pi)/2;   d10 = L; 
A_11 = 0;    alpha11 = sym(pi)/2;    d11 = 0;  
A_12 = 0;    alpha12 = -sym(pi)/2;   d12 = L; 
A_13 = 0;    alpha13 = sym(pi)/2;    d13 = 0;  
A_14 = 0;    alpha14 = 0;            d14 = L; 


for i = 1:71:1988
   % trasformations
A1 = dh(A_1, alpha1, d1, 0);
A2 = dh(A_2, alpha2, d2, th1(i));
A3 = dh(A_3, alpha3, d3, th2(i));
A4 = dh(A_4, alpha4, d4, th3(i));
A5 = dh(A_5, alpha5, d5, th4(i));
A6 = dh(A_6, alpha6, d6, th5(i));
A7 = dh(A_7, alpha7, d7, th6(i));
A8 = dh(A_8, alpha8, d8, th7(i));
A9 = dh(A_9, alpha9, d9, th8(i));
A10 = dh(A_10, alpha10, d10, th9(i));
A11 = dh(A_11, alpha11, d11, th10(i));
A12 = dh(A_12, alpha12, d12, th11(i));
A13 = dh(A_13, alpha13, d13, th12(i));
A14 = dh(A_14, alpha14, d14, 0);

% full homogeneuos transfoms
% 
HT_14 = A1*A2*A3*A4*A5*A6*A7*A8*A9*A10*A11*A12*A13*A14; %entire transform

HT_13 = A1*A2*A3*A4*A5*A6*A7*A8*A9*A10*A11*A12*A13;
HT_12 = A1*A2*A3*A4*A5*A6*A7*A8*A9*A10*A11*A12;
HT_11 = A1*A2*A3*A4*A5*A6*A7*A8*A9*A10*A11;
HT_10 = A1*A2*A3*A4*A5*A6*A7*A8*A9*A10;
HT_9 = A1*A2*A3*A4*A5*A6*A7*A8*A9;
HT_8 = A1*A2*A3*A4*A5*A6*A7*A8;
HT_7 = A1*A2*A3*A4*A5*A6*A7;
HT_6 = A1*A2*A3*A4*A5*A6;
HT_5 = A1*A2*A3*A4*A5;
HT_4 = A1*A2*A3*A4;
HT_3 = A1*A2*A3;
HT_2 = A1*A2;
HT_1 = A1;  

% O vectors

O_0 = [0;0;0];                              % always true
O_1 = [HT_1(13) ; HT_1(14) ; HT_1(15)];
O_2 = [HT_2(13) ; HT_2(14) ; HT_2(15)];
O_3 = [HT_3(13) ; HT_3(14) ; HT_3(15)];
O_4 = [HT_4(13) ; HT_4(14) ; HT_4(15)];
O_5 = [HT_5(13) ; HT_5(14) ; HT_5(15)];
O_6 = [HT_6(13) ; HT_6(14) ; HT_6(15)];
O_7 = [HT_7(13) ; HT_7(14) ; HT_7(15)];
O_8 = [HT_8(13) ; HT_8(14) ; HT_8(15)];
O_9 = [HT_9(13) ; HT_9(14) ; HT_9(15)];
O_10 = [HT_10(13) ; HT_10(14) ; HT_10(15)];
O_11 = [HT_11(13) ; HT_11(14) ; HT_11(15)];
O_12 = [HT_12(13) ; HT_12(14) ; HT_12(15)];
O_13 = [HT_13(13) ; HT_13(14) ; HT_13(15)];
O_14 = [HT_14(13) ; HT_14(14) ; HT_14(15)];

%Z vector
                           
Z_0 = [0;0;1];                              % always true   
Z_1 = [HT_1(9) ; HT_1(10) ; HT_1(11)]; 
Z_2 = [HT_2(9) ; HT_2(10) ; HT_2(11)];
Z_3 = [HT_3(9) ; HT_3(10) ; HT_3(11)];
Z_4 = [HT_4(9) ; HT_4(10) ; HT_4(11)]; 
Z_5 = [HT_5(9) ; HT_5(10) ; HT_5(11)];
Z_6 = [HT_6(9) ; HT_6(10) ; HT_6(11)];
Z_7 = [HT_7(9) ; HT_7(10) ; HT_7(11)]; 
Z_8 = [HT_8(9) ; HT_8(10) ; HT_8(11)];
Z_9 = [HT_9(9) ; HT_9(10) ; HT_9(11)];
Z_10 = [HT_10(9) ; HT_10(10) ; HT_10(11)]; 
Z_11 = [HT_11(9) ; HT_11(10) ; HT_11(11)];
Z_12 = [HT_12(9) ; HT_12(10) ; HT_12(11)];
Z_13 = [HT_13(9) ; HT_13(10) ; HT_13(11)]; 
Z_14 = [HT_14(9) ; HT_14(10) ; HT_14(11)];

% O subtraction
O_sub1 = O_14 - O_0;              
O_sub2 = O_14 - O_1;              
O_sub3 = O_14 - O_2;              
O_sub4 = O_14 - O_3;
O_sub5 = O_14 - O_4; 
O_sub6 = O_14 - O_5; 
O_sub7 = O_14 - O_6; 
O_sub8 = O_14 - O_7; 
O_sub9 = O_14 - O_8; 
O_sub10 = O_14 - O_9; 
O_sub11 = O_14 - O_10; 
O_sub12 = O_14 - O_11; 
O_sub13 = O_14 - O_12; 
O_sub14 = O_14 - O_13; 

% Jacobian structure
% J cross product
J_top1 = cross(Z_0,O_sub1);
J_top2 = cross(Z_1,O_sub2);
J_top3 = cross(Z_2,O_sub3);
J_top4 = cross(Z_3,O_sub4);
J_top5 = cross(Z_4,O_sub5);
J_top6 = cross(Z_5,O_sub6);
J_top7 = cross(Z_6,O_sub7);
J_top8 = cross(Z_7,O_sub8);
J_top9 = cross(Z_8,O_sub9);
J_top10 = cross(Z_9,O_sub10);
J_top11 = cross(Z_10,O_sub11);
J_top12 = cross(Z_11,O_sub12);
J_top13 = cross(Z_12,O_sub13);
J_top14 = cross(Z_13,O_sub14);

% jacobian individual colmns
J_colmn1 = [J_top1;Z_0];  
J_colmn2 = [J_top2;Z_1];
J_colmn3 = [J_top3;Z_2];
J_colmn4 = [J_top4;Z_3];
J_colmn5 = [J_top5;Z_4];
J_colmn6 = [J_top6;Z_5];
J_colmn7 = [J_top7;Z_6];
J_colmn8 = [J_top8;Z_7];
J_colmn9 = [J_top9;Z_8];
J_colmn10 = [J_top10;Z_9];
J_colmn11 = [J_top11;Z_10];
J_colmn12 = [J_top12;Z_11];
J_colmn13 = [J_top13;Z_12];
J_colmn14 = [J_top14;Z_13];

%final velocity equations of motion

J = simplify([J_colmn1 J_colmn2 J_colmn3 J_colmn4 J_colmn5 J_colmn6 J_colmn7 ...
    J_colmn8 J_colmn9 J_colmn10 J_colmn11 J_colmn12 J_colmn13 J_colmn14]);

dot_q = [0; dt1(i); dt2(i); dt3(i); dt4(i); dt5(i); dt6(i); ...
    dt7(i); dt8(i); dt9(i); dt10(i); dt11(i); dt12(i); 0];

%end effector velocity
vel = simplify(J*dot_q)
end


% 
% % J_11 = [J_top1 J_top2 J_top3];
% % J1_11 = [J_top1(1) J_top2(1) ; J_top1(2) J_top2(2)];
% % J2_11 = [J_top2(1) J_top3(1) ; J_top2(2) J_top3(2)];
% % J_det = simplify(det(J2_11));
% 
% % end

function [dhMatrix] = dh(a,alpha,d,theta)

dhMatrix = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta) ;
    sin(theta) cos(theta)*cos(alpha) -sin(alpha)*cos(theta) a*sin(theta); 
    0 sin(alpha) cos(alpha) d ; 
    0 0 0 1];
end

% function [rotx] = Rx(theta)
% rotx = [1 0 0 0 ; 0 cos(theta) -sin(theta) 0; 0 sin(theta) cos(theta) 0 ; 0 0 0 1];
% end
% 
% function [roty] = Ry(theta)
% roty = [ cos(theta) 0 sin(theta) 0 ; 0 1 0 0; -sin(theta) 0 cos(theta) 0 ; 0 0 0 1];
% end
% 
% function [rotz] = Rz(theta)
% rotz = [ cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];
% end
% 
% function [transfx] = Tx(disp)
% transfx = [ 1 0 0 disp ; 0 1 0 0 ; 0 0 1 0 ; 0 0 0 1];
% end
% 
% function [transfy] = Ty(disp)
% transfy = [ 1 0 0 0 ; 0 1 0 disp ; 0 0 1 0 ; 0 0 0 1];
% end
% 
% function [transfz] = Tz(disp)
% transfz = [ 1 0 0 0 ; 0 1 0 0 ; 0 0 1 disp ; 0 0 0 1];
% end

