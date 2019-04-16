% dhparams = [ 0 0 L 0;
%             0 -pi/2 0 0;
%            0 pi/2 0 0;
%            0 -pi/2 L 0;
%            0 pi/2 0 0;
%            0 -pi/2 L 0;
%            0 pi/2 0 0;
%            0 -pi/2 L 0;
%            0 pi/2 0 0;
%            0 -pi/2 L 0;
%            0 pi/2 0 0;
%            0 -pi/2 L 0;
%            0 pi/2 0 0;
%            0 0 L 0;]

function ForwardKinematics()
t1= sym('t1');
t2= sym('t2');
t3= sym('t3');
t4= sym('t4');
t5= sym('t5');
t6= sym('t6');
t7= sym('t7');
t8= sym('t8');
t9= sym('t9');
t10= sym('t10');
t11= sym('t11');
t12= sym('t12');
t13= sym('t13');
t14= sym('t14');

L = sym('L');

% DH table
A_1 = 0;     alpha1 = 0;             d1 = L;   theta1 = 0;
A_2 = 0;     alpha2 = -sym(pi)/2;    d2 = 0;   theta2 = t1;
A_3 = 0;     alpha3 = sym(pi)/2;     d3 = 0;   theta3 = t2;
A_4 = 0;     alpha4 = -sym(pi)/2;    d4 = L;   theta4 = t3;
A_5 = 0;     alpha5 = sym(pi)/2;     d5 = 0;   theta5 = t4;
A_6 = 0;     alpha6 = -sym(pi)/2;    d6 = L;   theta6 = t5;
A_7 = 0;     alpha7 = sym(pi)/2;     d7 = 0;   theta7 = t6;
A_8 = 0;     alpha8 = -sym(pi)/2;    d8 = L;   theta8 = t7;
A_9 = 0;     alpha9 = sym(pi)/2;     d9 = 0;   theta9 = t8;
A_10 = 0;    alpha10 = -sym(pi)/2;   d10 = L;  theta10 = t9;
A_11 = 0;    alpha11 = sym(pi)/2;    d11 = 0;  theta11 = t10;
A_12 = 0;    alpha12 = -sym(pi)/2;   d12 = L;  theta12 = t11;
A_13 = 0;    alpha13 = sym(pi)/2;    d13 = 0;  theta13 = t12;
A_14 = 0;    alpha14 = 0;            d14 = L;  theta14 = t13;

% Link Matrices
A1 = dh(A_1, alpha1, d1, theta1);
A2 = dh(A_2, alpha2, d2, theta2);
A3 = dh(A_3, alpha3, d3, theta3);
A4 = dh(A_4, alpha4, d4, theta4);
A5 = dh(A_5, alpha5, d5, theta5);
A6 = dh(A_6, alpha6, d6, theta6);
A7 = dh(A_7, alpha7, d7, theta7);
A8 = dh(A_8, alpha8, d8, theta8);
A9 = dh(A_9, alpha9, d9, theta9);
A10 = dh(A_10, alpha10, d10, theta10);
A11 = dh(A_11, alpha11, d11, theta11);
A12 = dh(A_12, alpha12, d12, theta12);
A13 = dh(A_13, alpha13, d13, theta13);
A14 = dh(A_14, alpha14, d14, theta14);

%---------------------------------------------------------
%homogeneuos transfom

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

% %------------------------------------------------------
% % O vector

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

% %----------------------------------------------------------
% % Z vector
                           
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

% %------------------------------------------------------------

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

% %-----------------------------------------------------------------
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

% J individual full columns
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
% 
J_Part1 = [J_colmn1 J_colmn2 J_colmn3 J_colmn4 J_colmn5 J_colmn6 J_colmn7] 
J_Part2 = [J_colmn8 J_colmn9 J_colmn10 J_colmn11 J_colmn12 J_colmn13 J_colmn14]
%         
%----------------------------------------------------------------
% singularity started but not completed

% J_11 = [J_top1 J_top2 J_top3];
% J1_11 = [J_top1(1) J_top2(1) ; J_top1(2) J_top2(2)];
% J2_11 = [J_top2(1) J_top3(1) ; J_top2(2) J_top3(2)];
% J_det = simplify(det(J2_11));

end

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

