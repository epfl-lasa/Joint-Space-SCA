% Produced by CVXGEN, 2020-07-15 07:48:00 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: cvxsolve.m.
% Description: Solution file, via cvx, for use with sample.m.
function [vars, status] = cvxsolve(params, settings)
if isfield(params, 'J_1')
  J_1 = params.J_1;
elseif isfield(params, 'J')
  J_1 = params.J{1};
else
  error 'could not find J_1'
end
if isfield(params, 'J_2')
  J_2 = params.J_2;
elseif isfield(params, 'J')
  J_2 = params.J{2};
else
  error 'could not find J_2'
end
if isfield(params, 'J_3')
  J_3 = params.J_3;
elseif isfield(params, 'J')
  J_3 = params.J{3};
else
  error 'could not find J_3'
end
if isfield(params, 'J_4')
  J_4 = params.J_4;
elseif isfield(params, 'J')
  J_4 = params.J{4};
else
  error 'could not find J_4'
end
if isfield(params, 'J_5')
  J_5 = params.J_5;
elseif isfield(params, 'J')
  J_5 = params.J{5};
else
  error 'could not find J_5'
end
if isfield(params, 'J_6')
  J_6 = params.J_6;
elseif isfield(params, 'J')
  J_6 = params.J{6};
else
  error 'could not find J_6'
end
if isfield(params, 'J_7')
  J_7 = params.J_7;
elseif isfield(params, 'J')
  J_7 = params.J{7};
else
  error 'could not find J_7'
end
if isfield(params, 'J_8')
  J_8 = params.J_8;
elseif isfield(params, 'J')
  J_8 = params.J{8};
else
  error 'could not find J_8'
end
if isfield(params, 'J_9')
  J_9 = params.J_9;
elseif isfield(params, 'J')
  J_9 = params.J{9};
else
  error 'could not find J_9'
end
if isfield(params, 'J_10')
  J_10 = params.J_10;
elseif isfield(params, 'J')
  J_10 = params.J{10};
else
  error 'could not find J_10'
end
if isfield(params, 'J_11')
  J_11 = params.J_11;
elseif isfield(params, 'J')
  J_11 = params.J{11};
else
  error 'could not find J_11'
end
if isfield(params, 'J_12')
  J_12 = params.J_12;
elseif isfield(params, 'J')
  J_12 = params.J{12};
else
  error 'could not find J_12'
end
if isfield(params, 'J_13')
  J_13 = params.J_13;
elseif isfield(params, 'J')
  J_13 = params.J{13};
else
  error 'could not find J_13'
end
if isfield(params, 'J_14')
  J_14 = params.J_14;
elseif isfield(params, 'J')
  J_14 = params.J{14};
else
  error 'could not find J_14'
end
if isfield(params, 'J_15')
  J_15 = params.J_15;
elseif isfield(params, 'J')
  J_15 = params.J{15};
else
  error 'could not find J_15'
end
if isfield(params, 'J_16')
  J_16 = params.J_16;
elseif isfield(params, 'J')
  J_16 = params.J{16};
else
  error 'could not find J_16'
end
if isfield(params, 'J_17')
  J_17 = params.J_17;
elseif isfield(params, 'J')
  J_17 = params.J{17};
else
  error 'could not find J_17'
end
if isfield(params, 'J_18')
  J_18 = params.J_18;
elseif isfield(params, 'J')
  J_18 = params.J{18};
else
  error 'could not find J_18'
end
if isfield(params, 'J_19')
  J_19 = params.J_19;
elseif isfield(params, 'J')
  J_19 = params.J{19};
else
  error 'could not find J_19'
end
if isfield(params, 'J_20')
  J_20 = params.J_20;
elseif isfield(params, 'J')
  J_20 = params.J{20};
else
  error 'could not find J_20'
end
if isfield(params, 'J_21')
  J_21 = params.J_21;
elseif isfield(params, 'J')
  J_21 = params.J{21};
else
  error 'could not find J_21'
end
if isfield(params, 'J_22')
  J_22 = params.J_22;
elseif isfield(params, 'J')
  J_22 = params.J{22};
else
  error 'could not find J_22'
end
if isfield(params, 'J_23')
  J_23 = params.J_23;
elseif isfield(params, 'J')
  J_23 = params.J{23};
else
  error 'could not find J_23'
end
if isfield(params, 'J_24')
  J_24 = params.J_24;
elseif isfield(params, 'J')
  J_24 = params.J{24};
else
  error 'could not find J_24'
end
if isfield(params, 'J_25')
  J_25 = params.J_25;
elseif isfield(params, 'J')
  J_25 = params.J{25};
else
  error 'could not find J_25'
end
if isfield(params, 'J_26')
  J_26 = params.J_26;
elseif isfield(params, 'J')
  J_26 = params.J{26};
else
  error 'could not find J_26'
end
if isfield(params, 'J_27')
  J_27 = params.J_27;
elseif isfield(params, 'J')
  J_27 = params.J{27};
else
  error 'could not find J_27'
end
if isfield(params, 'J_28')
  J_28 = params.J_28;
elseif isfield(params, 'J')
  J_28 = params.J{28};
else
  error 'could not find J_28'
end
if isfield(params, 'J_29')
  J_29 = params.J_29;
elseif isfield(params, 'J')
  J_29 = params.J{29};
else
  error 'could not find J_29'
end
if isfield(params, 'J_30')
  J_30 = params.J_30;
elseif isfield(params, 'J')
  J_30 = params.J{30};
else
  error 'could not find J_30'
end
if isfield(params, 'J_31')
  J_31 = params.J_31;
elseif isfield(params, 'J')
  J_31 = params.J{31};
else
  error 'could not find J_31'
end
if isfield(params, 'J_32')
  J_32 = params.J_32;
elseif isfield(params, 'J')
  J_32 = params.J{32};
else
  error 'could not find J_32'
end
if isfield(params, 'J_33')
  J_33 = params.J_33;
elseif isfield(params, 'J')
  J_33 = params.J{33};
else
  error 'could not find J_33'
end
if isfield(params, 'J_34')
  J_34 = params.J_34;
elseif isfield(params, 'J')
  J_34 = params.J{34};
else
  error 'could not find J_34'
end
if isfield(params, 'J_35')
  J_35 = params.J_35;
elseif isfield(params, 'J')
  J_35 = params.J{35};
else
  error 'could not find J_35'
end
if isfield(params, 'J_36')
  J_36 = params.J_36;
elseif isfield(params, 'J')
  J_36 = params.J{36};
else
  error 'could not find J_36'
end
if isfield(params, 'J_37')
  J_37 = params.J_37;
elseif isfield(params, 'J')
  J_37 = params.J{37};
else
  error 'could not find J_37'
end
if isfield(params, 'J_38')
  J_38 = params.J_38;
elseif isfield(params, 'J')
  J_38 = params.J{38};
else
  error 'could not find J_38'
end
damping = params.damping;
dx = params.dx;
hard = params.hard;
qlow = params.qlow;
qmax = params.qmax;
qmin = params.qmin;
qref = params.qref;
qup = params.qup;
slack = params.slack;
svm_val_1 = params.svm_val_1;
svm_val_2 = params.svm_val_2;
svm_val_3 = params.svm_val_3;
svm_val_4 = params.svm_val_4;
svm_val_5 = params.svm_val_5;
svm_val_6 = params.svm_val_6;
svm_val_7 = params.svm_val_7;
svm_val_8 = params.svm_val_8;
svm_val_9 = params.svm_val_9;
svm_val_10 = params.svm_val_10;
svm_vec_1 = params.svm_vec_1;
svm_vec_2 = params.svm_vec_2;
svm_vec_3 = params.svm_vec_3;
svm_vec_4 = params.svm_vec_4;
svm_vec_5 = params.svm_vec_5;
svm_vec_6 = params.svm_vec_6;
svm_vec_7 = params.svm_vec_7;
svm_vec_8 = params.svm_vec_8;
svm_vec_9 = params.svm_vec_9;
svm_vec_10 = params.svm_vec_10;
cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable dq(38, 1);
  variable delta(36, 1);

  minimize(damping'*square(dq + qref) + slack'*square(delta));
  subject to
    J_1(1)*dq(1) + J_2(1)*dq(2) + J_3(1)*dq(3) + J_4(1)*dq(4) + J_5(1)*dq(5) + J_6(1)*dq(6) + J_7(1)*dq(7) + J_8(1)*dq(8) + J_9(1)*dq(9) + J_10(1)*dq(10) + J_11(1)*dq(11) + J_12(1)*dq(12) + J_13(1)*dq(13) + J_14(1)*dq(14) + J_15(1)*dq(15) + J_16(1)*dq(16) + J_17(1)*dq(17) + J_18(1)*dq(18) + J_19(1)*dq(19) + J_20(1)*dq(20) + J_21(1)*dq(21) + J_22(1)*dq(22) + J_23(1)*dq(23) + J_24(1)*dq(24) + J_25(1)*dq(25) + J_26(1)*dq(26) + J_27(1)*dq(27) + J_28(1)*dq(28) + J_29(1)*dq(29) + J_30(1)*dq(30) + J_31(1)*dq(31) + J_32(1)*dq(32) + J_33(1)*dq(33) + J_34(1)*dq(34) + J_35(1)*dq(35) + J_36(1)*dq(36) + J_37(1)*dq(37) + J_38(1)*dq(38) == dx(1) + (1 - hard(1))*delta(1);
    J_1(2)*dq(1) + J_2(2)*dq(2) + J_3(2)*dq(3) + J_4(2)*dq(4) + J_5(2)*dq(5) + J_6(2)*dq(6) + J_7(2)*dq(7) + J_8(2)*dq(8) + J_9(2)*dq(9) + J_10(2)*dq(10) + J_11(2)*dq(11) + J_12(2)*dq(12) + J_13(2)*dq(13) + J_14(2)*dq(14) + J_15(2)*dq(15) + J_16(2)*dq(16) + J_17(2)*dq(17) + J_18(2)*dq(18) + J_19(2)*dq(19) + J_20(2)*dq(20) + J_21(2)*dq(21) + J_22(2)*dq(22) + J_23(2)*dq(23) + J_24(2)*dq(24) + J_25(2)*dq(25) + J_26(2)*dq(26) + J_27(2)*dq(27) + J_28(2)*dq(28) + J_29(2)*dq(29) + J_30(2)*dq(30) + J_31(2)*dq(31) + J_32(2)*dq(32) + J_33(2)*dq(33) + J_34(2)*dq(34) + J_35(2)*dq(35) + J_36(2)*dq(36) + J_37(2)*dq(37) + J_38(2)*dq(38) == dx(2) + (1 - hard(2))*delta(2);
    J_1(3)*dq(1) + J_2(3)*dq(2) + J_3(3)*dq(3) + J_4(3)*dq(4) + J_5(3)*dq(5) + J_6(3)*dq(6) + J_7(3)*dq(7) + J_8(3)*dq(8) + J_9(3)*dq(9) + J_10(3)*dq(10) + J_11(3)*dq(11) + J_12(3)*dq(12) + J_13(3)*dq(13) + J_14(3)*dq(14) + J_15(3)*dq(15) + J_16(3)*dq(16) + J_17(3)*dq(17) + J_18(3)*dq(18) + J_19(3)*dq(19) + J_20(3)*dq(20) + J_21(3)*dq(21) + J_22(3)*dq(22) + J_23(3)*dq(23) + J_24(3)*dq(24) + J_25(3)*dq(25) + J_26(3)*dq(26) + J_27(3)*dq(27) + J_28(3)*dq(28) + J_29(3)*dq(29) + J_30(3)*dq(30) + J_31(3)*dq(31) + J_32(3)*dq(32) + J_33(3)*dq(33) + J_34(3)*dq(34) + J_35(3)*dq(35) + J_36(3)*dq(36) + J_37(3)*dq(37) + J_38(3)*dq(38) == dx(3) + (1 - hard(3))*delta(3);
    J_4(4)*dq(4) + J_5(4)*dq(5) + J_6(4)*dq(6) == dx(4) + (1 - hard(4))*delta(4);
    J_4(5)*dq(4) + J_5(5)*dq(5) + J_6(5)*dq(6) == dx(5) + (1 - hard(5))*delta(5);
    J_4(6)*dq(4) + J_5(6)*dq(5) + J_6(6)*dq(6) == dx(6) + (1 - hard(6))*delta(6);
    J_1(7)*dq(1) + J_2(7)*dq(2) + J_3(7)*dq(3) + J_4(7)*dq(4) + J_5(7)*dq(5) + J_6(7)*dq(6) + J_13(7)*dq(13) + J_14(7)*dq(14) + J_15(7)*dq(15) + J_16(7)*dq(16) + J_17(7)*dq(17) + J_18(7)*dq(18) == dx(7) + (1 - hard(7))*delta(7);
    J_1(8)*dq(1) + J_2(8)*dq(2) + J_3(8)*dq(3) + J_4(8)*dq(4) + J_5(8)*dq(5) + J_6(8)*dq(6) + J_13(8)*dq(13) + J_14(8)*dq(14) + J_15(8)*dq(15) + J_16(8)*dq(16) + J_17(8)*dq(17) + J_18(8)*dq(18) == dx(8) + (1 - hard(8))*delta(8);
    J_1(9)*dq(1) + J_2(9)*dq(2) + J_3(9)*dq(3) + J_4(9)*dq(4) + J_5(9)*dq(5) + J_6(9)*dq(6) + J_13(9)*dq(13) + J_14(9)*dq(14) + J_15(9)*dq(15) + J_16(9)*dq(16) + J_17(9)*dq(17) + J_18(9)*dq(18) == dx(9) + (1 - hard(9))*delta(9);
    J_4(10)*dq(4) + J_5(10)*dq(5) + J_6(10)*dq(6) + J_13(10)*dq(13) + J_14(10)*dq(14) + J_15(10)*dq(15) + J_16(10)*dq(16) + J_17(10)*dq(17) + J_18(10)*dq(18) == dx(10) + (1 - hard(10))*delta(10);
    J_4(11)*dq(4) + J_5(11)*dq(5) + J_6(11)*dq(6) + J_13(11)*dq(13) + J_14(11)*dq(14) + J_15(11)*dq(15) + J_16(11)*dq(16) + J_17(11)*dq(17) + J_18(11)*dq(18) == dx(11) + (1 - hard(11))*delta(11);
    J_4(12)*dq(4) + J_5(12)*dq(5) + J_6(12)*dq(6) + J_13(12)*dq(13) + J_14(12)*dq(14) + J_15(12)*dq(15) + J_16(12)*dq(16) + J_17(12)*dq(17) + J_18(12)*dq(18) == dx(12) + (1 - hard(12))*delta(12);
    J_1(13)*dq(1) + J_2(13)*dq(2) + J_3(13)*dq(3) + J_4(13)*dq(4) + J_5(13)*dq(5) + J_6(13)*dq(6) + J_19(13)*dq(19) + J_20(13)*dq(20) + J_21(13)*dq(21) + J_22(13)*dq(22) + J_23(13)*dq(23) + J_24(13)*dq(24) == dx(13) + (1 - hard(13))*delta(13);
    J_1(14)*dq(1) + J_2(14)*dq(2) + J_3(14)*dq(3) + J_4(14)*dq(4) + J_5(14)*dq(5) + J_6(14)*dq(6) + J_19(14)*dq(19) + J_20(14)*dq(20) + J_21(14)*dq(21) + J_22(14)*dq(22) + J_23(14)*dq(23) + J_24(14)*dq(24) == dx(14) + (1 - hard(14))*delta(14);
    J_1(15)*dq(1) + J_2(15)*dq(2) + J_3(15)*dq(3) + J_4(15)*dq(4) + J_5(15)*dq(5) + J_6(15)*dq(6) + J_19(15)*dq(19) + J_20(15)*dq(20) + J_21(15)*dq(21) + J_22(15)*dq(22) + J_23(15)*dq(23) + J_24(15)*dq(24) == dx(15) + (1 - hard(15))*delta(15);
    J_4(16)*dq(4) + J_5(16)*dq(5) + J_6(16)*dq(6) + J_19(16)*dq(19) + J_20(16)*dq(20) + J_21(16)*dq(21) + J_22(16)*dq(22) + J_23(16)*dq(23) + J_24(16)*dq(24) == dx(16) + (1 - hard(16))*delta(16);
    J_4(17)*dq(4) + J_5(17)*dq(5) + J_6(17)*dq(6) + J_19(17)*dq(19) + J_20(17)*dq(20) + J_21(17)*dq(21) + J_22(17)*dq(22) + J_23(17)*dq(23) + J_24(17)*dq(24) == dx(17) + (1 - hard(17))*delta(17);
    J_4(18)*dq(4) + J_5(18)*dq(5) + J_6(18)*dq(6) + J_19(18)*dq(19) + J_20(18)*dq(20) + J_21(18)*dq(21) + J_22(18)*dq(22) + J_23(18)*dq(23) + J_24(18)*dq(24) == dx(18) + (1 - hard(18))*delta(18);
    J_1(19)*dq(1) + J_2(19)*dq(2) + J_3(19)*dq(3) + J_4(19)*dq(4) + J_5(19)*dq(5) + J_6(19)*dq(6) + J_7(19)*dq(7) + J_8(19)*dq(8) + J_9(19)*dq(9) + J_25(19)*dq(25) + J_26(19)*dq(26) + J_27(19)*dq(27) + J_28(19)*dq(28) + J_29(19)*dq(29) + J_30(19)*dq(30) + J_31(19)*dq(31) == dx(19) + (1 - hard(19))*delta(19);
    J_1(20)*dq(1) + J_2(20)*dq(2) + J_3(20)*dq(3) + J_4(20)*dq(4) + J_5(20)*dq(5) + J_6(20)*dq(6) + J_7(20)*dq(7) + J_8(20)*dq(8) + J_9(20)*dq(9) + J_25(20)*dq(25) + J_26(20)*dq(26) + J_27(20)*dq(27) + J_28(20)*dq(28) + J_29(20)*dq(29) + J_30(20)*dq(30) + J_31(20)*dq(31) == dx(20) + (1 - hard(20))*delta(20);
    J_1(21)*dq(1) + J_2(21)*dq(2) + J_3(21)*dq(3) + J_4(21)*dq(4) + J_5(21)*dq(5) + J_6(21)*dq(6) + J_7(21)*dq(7) + J_8(21)*dq(8) + J_9(21)*dq(9) + J_25(21)*dq(25) + J_26(21)*dq(26) + J_27(21)*dq(27) + J_28(21)*dq(28) + J_29(21)*dq(29) + J_30(21)*dq(30) + J_31(21)*dq(31) == dx(21) + (1 - hard(21))*delta(21);
    J_4(22)*dq(4) + J_5(22)*dq(5) + J_6(22)*dq(6) + J_7(22)*dq(7) + J_8(22)*dq(8) + J_9(22)*dq(9) + J_25(22)*dq(25) + J_26(22)*dq(26) + J_27(22)*dq(27) + J_28(22)*dq(28) + J_29(22)*dq(29) + J_30(22)*dq(30) + J_31(22)*dq(31) == dx(22) + (1 - hard(22))*delta(22);
    J_4(23)*dq(4) + J_5(23)*dq(5) + J_6(23)*dq(6) + J_7(23)*dq(7) + J_8(23)*dq(8) + J_9(23)*dq(9) + J_25(23)*dq(25) + J_26(23)*dq(26) + J_27(23)*dq(27) + J_28(23)*dq(28) + J_29(23)*dq(29) + J_30(23)*dq(30) + J_31(23)*dq(31) == dx(23) + (1 - hard(23))*delta(23);
    J_4(24)*dq(4) + J_5(24)*dq(5) + J_6(24)*dq(6) + J_7(24)*dq(7) + J_8(24)*dq(8) + J_9(24)*dq(9) + J_25(24)*dq(25) + J_26(24)*dq(26) + J_27(24)*dq(27) + J_28(24)*dq(28) + J_29(24)*dq(29) + J_30(24)*dq(30) + J_31(24)*dq(31) == dx(24) + (1 - hard(24))*delta(24);
    J_1(25)*dq(1) + J_2(25)*dq(2) + J_3(25)*dq(3) + J_4(25)*dq(4) + J_5(25)*dq(5) + J_6(25)*dq(6) + J_7(25)*dq(7) + J_8(25)*dq(8) + J_9(25)*dq(9) + J_32(25)*dq(32) + J_33(25)*dq(33) + J_34(25)*dq(34) + J_35(25)*dq(35) + J_36(25)*dq(36) + J_37(25)*dq(37) + J_38(25)*dq(38) == dx(25) + (1 - hard(25))*delta(25);
    J_1(26)*dq(1) + J_2(26)*dq(2) + J_3(26)*dq(3) + J_4(26)*dq(4) + J_5(26)*dq(5) + J_6(26)*dq(6) + J_7(26)*dq(7) + J_8(26)*dq(8) + J_9(26)*dq(9) + J_32(26)*dq(32) + J_33(26)*dq(33) + J_34(26)*dq(34) + J_35(26)*dq(35) + J_36(26)*dq(36) + J_37(26)*dq(37) + J_38(26)*dq(38) == dx(26) + (1 - hard(26))*delta(26);
    J_1(27)*dq(1) + J_2(27)*dq(2) + J_3(27)*dq(3) + J_4(27)*dq(4) + J_5(27)*dq(5) + J_6(27)*dq(6) + J_7(27)*dq(7) + J_8(27)*dq(8) + J_9(27)*dq(9) + J_32(27)*dq(32) + J_33(27)*dq(33) + J_34(27)*dq(34) + J_35(27)*dq(35) + J_36(27)*dq(36) + J_37(27)*dq(37) + J_38(27)*dq(38) == dx(27) + (1 - hard(27))*delta(27);
    J_4(28)*dq(4) + J_5(28)*dq(5) + J_6(28)*dq(6) + J_7(28)*dq(7) + J_8(28)*dq(8) + J_9(28)*dq(9) + J_32(28)*dq(32) + J_33(28)*dq(33) + J_34(28)*dq(34) + J_35(28)*dq(35) + J_36(28)*dq(36) + J_37(28)*dq(37) + J_38(28)*dq(38) == dx(28) + (1 - hard(28))*delta(28);
    J_4(29)*dq(4) + J_5(29)*dq(5) + J_6(29)*dq(6) + J_7(29)*dq(7) + J_8(29)*dq(8) + J_9(29)*dq(9) + J_32(29)*dq(32) + J_33(29)*dq(33) + J_34(29)*dq(34) + J_35(29)*dq(35) + J_36(29)*dq(36) + J_37(29)*dq(37) + J_38(29)*dq(38) == dx(29) + (1 - hard(29))*delta(29);
    J_4(30)*dq(4) + J_5(30)*dq(5) + J_6(30)*dq(6) + J_7(30)*dq(7) + J_8(30)*dq(8) + J_9(30)*dq(9) + J_32(30)*dq(32) + J_33(30)*dq(33) + J_34(30)*dq(34) + J_35(30)*dq(35) + J_36(30)*dq(36) + J_37(30)*dq(37) + J_38(30)*dq(38) == dx(30) + (1 - hard(30))*delta(30);
    J_4(31)*dq(4) + J_5(31)*dq(5) + J_6(31)*dq(6) + J_7(31)*dq(7) + J_8(31)*dq(8) + J_9(31)*dq(9) == dx(31) + (1 - hard(31))*delta(31);
    J_4(32)*dq(4) + J_5(32)*dq(5) + J_6(32)*dq(6) + J_7(32)*dq(7) + J_8(32)*dq(8) + J_9(32)*dq(9) == dx(32) + (1 - hard(32))*delta(32);
    J_4(33)*dq(4) + J_5(33)*dq(5) + J_6(33)*dq(6) + J_7(33)*dq(7) + J_8(33)*dq(8) + J_9(33)*dq(9) == dx(33) + (1 - hard(33))*delta(33);
    J_4(34)*dq(4) + J_5(34)*dq(5) + J_6(34)*dq(6) + J_7(34)*dq(7) + J_8(34)*dq(8) + J_9(34)*dq(9) + J_10(34)*dq(10) + J_11(34)*dq(11) + J_12(34)*dq(12) == dx(34) + (1 - hard(34))*delta(34);
    J_4(35)*dq(4) + J_5(35)*dq(5) + J_6(35)*dq(6) + J_7(35)*dq(7) + J_8(35)*dq(8) + J_9(35)*dq(9) + J_10(35)*dq(10) + J_11(35)*dq(11) + J_12(35)*dq(12) == dx(35) + (1 - hard(35))*delta(35);
    J_4(36)*dq(4) + J_5(36)*dq(5) + J_6(36)*dq(6) + J_7(36)*dq(7) + J_8(36)*dq(8) + J_9(36)*dq(9) + J_10(36)*dq(10) + J_11(36)*dq(11) + J_12(36)*dq(12) == dx(36) + (1 - hard(36))*delta(36);
    qlow(1) <= dq(7);
    qlow(2) <= dq(8);
    qlow(3) <= dq(9);
    qlow(4) <= dq(10);
    qlow(5) <= dq(11);
    qlow(6) <= dq(12);
    qlow(7) <= dq(13);
    qlow(8) <= dq(14);
    qlow(9) <= dq(15);
    qlow(10) <= dq(16);
    qlow(11) <= dq(17);
    qlow(12) <= dq(18);
    qlow(13) <= dq(19);
    qlow(14) <= dq(20);
    qlow(15) <= dq(21);
    qlow(16) <= dq(22);
    qlow(17) <= dq(23);
    qlow(18) <= dq(24);
    qlow(19) <= dq(25);
    qlow(20) <= dq(26);
    qlow(21) <= dq(27);
    qlow(22) <= dq(28);
    qlow(23) <= dq(29);
    qlow(24) <= dq(30);
    qlow(25) <= dq(31);
    qlow(26) <= dq(32);
    qlow(27) <= dq(33);
    qlow(28) <= dq(34);
    qlow(29) <= dq(35);
    qlow(30) <= dq(36);
    qlow(31) <= dq(37);
    qlow(32) <= dq(38);
    dq(7) <= qup(1);
    dq(8) <= qup(2);
    dq(9) <= qup(3);
    dq(10) <= qup(4);
    dq(11) <= qup(5);
    dq(12) <= qup(6);
    dq(13) <= qup(7);
    dq(14) <= qup(8);
    dq(15) <= qup(9);
    dq(16) <= qup(10);
    dq(17) <= qup(11);
    dq(18) <= qup(12);
    dq(19) <= qup(13);
    dq(20) <= qup(14);
    dq(21) <= qup(15);
    dq(22) <= qup(16);
    dq(23) <= qup(17);
    dq(24) <= qup(18);
    dq(25) <= qup(19);
    dq(26) <= qup(20);
    dq(27) <= qup(21);
    dq(28) <= qup(22);
    dq(29) <= qup(23);
    dq(30) <= qup(24);
    dq(31) <= qup(25);
    dq(32) <= qup(26);
    dq(33) <= qup(27);
    dq(34) <= qup(28);
    dq(35) <= qup(29);
    dq(36) <= qup(30);
    dq(37) <= qup(31);
    dq(38) <= qup(32);
    svm_vec_1(1)*(1/qmax(19) - qmin(19))*dq(25) + svm_vec_1(2)*(1/qmax(20) - qmin(20))*dq(26) + svm_vec_1(3)*(1/qmax(21) - qmin(21))*dq(27) + svm_vec_1(4)*(1/qmax(22) - qmin(22))*dq(28) + svm_vec_1(5)*(1/qmax(23) - qmin(23))*dq(29) + svm_vec_1(6)*(1/qmax(24) - qmin(24))*dq(30) + svm_vec_1(7)*(1/qmax(25) - qmin(25))*dq(31) + svm_vec_1(8)*(1/qmax(26) - qmin(26))*dq(32) + svm_vec_1(9)*(1/qmax(27) - qmin(27))*dq(33) + svm_vec_1(10)*(1/qmax(28) - qmin(28))*dq(34) + svm_vec_1(11)*(1/qmax(29) - qmin(29))*dq(35) + svm_vec_1(12)*(1/qmax(30) - qmin(30))*dq(36) + svm_vec_1(13)*(1/qmax(31) - qmin(31))*dq(37) + svm_vec_1(14)*(1/qmax(32) - qmin(32))*dq(38) <= svm_val_1;
    svm_vec_2(1)*(1/qmax(1) - qmin(1))*dq(7) + svm_vec_2(2)*(1/qmax(2) - qmin(2))*dq(8) + svm_vec_2(3)*(1/qmax(3) - qmin(3))*dq(9) + svm_vec_2(4)*(1/qmax(7) - qmin(7))*dq(13) + svm_vec_2(5)*(1/qmax(8) - qmin(8))*dq(14) + svm_vec_2(6)*(1/qmax(9) - qmin(9))*dq(15) + svm_vec_2(7)*(1/qmax(10) - qmin(10))*dq(16) + svm_vec_2(8)*(1/qmax(11) - qmin(11))*dq(17) + svm_vec_2(9)*(1/qmax(12) - qmin(12))*dq(18) + svm_vec_2(10)*(1/qmax(19) - qmin(19))*dq(25) + svm_vec_2(11)*(1/qmax(20) - qmin(20))*dq(26) + svm_vec_2(12)*(1/qmax(21) - qmin(21))*dq(27) + svm_vec_2(13)*(1/qmax(22) - qmin(22))*dq(28) + svm_vec_2(14)*(1/qmax(23) - qmin(23))*dq(29) + svm_vec_2(15)*(1/qmax(24) - qmin(24))*dq(30) + svm_vec_2(16)*(1/qmax(25) - qmin(25))*dq(31) <= svm_val_2;
    svm_vec_3(1)*(1/qmax(1) - qmin(1))*dq(7) + svm_vec_3(2)*(1/qmax(2) - qmin(2))*dq(8) + svm_vec_3(3)*(1/qmax(3) - qmin(3))*dq(9) + svm_vec_3(4)*(1/qmax(13) - qmin(13))*dq(19) + svm_vec_3(5)*(1/qmax(14) - qmin(14))*dq(20) + svm_vec_3(6)*(1/qmax(15) - qmin(15))*dq(21) + svm_vec_3(7)*(1/qmax(16) - qmin(16))*dq(22) + svm_vec_3(8)*(1/qmax(17) - qmin(17))*dq(23) + svm_vec_3(9)*(1/qmax(18) - qmin(18))*dq(24) + svm_vec_3(10)*(1/qmax(19) - qmin(19))*dq(25) + svm_vec_3(11)*(1/qmax(20) - qmin(20))*dq(26) + svm_vec_3(12)*(1/qmax(21) - qmin(21))*dq(27) + svm_vec_3(13)*(1/qmax(22) - qmin(22))*dq(28) + svm_vec_3(14)*(1/qmax(23) - qmin(23))*dq(29) + svm_vec_3(15)*(1/qmax(24) - qmin(24))*dq(30) + svm_vec_3(16)*(1/qmax(25) - qmin(25))*dq(31) <= svm_val_3;
    svm_vec_4(1)*(1/qmax(1) - qmin(1))*dq(7) + svm_vec_4(2)*(1/qmax(2) - qmin(2))*dq(8) + svm_vec_4(3)*(1/qmax(3) - qmin(3))*dq(9) + svm_vec_4(4)*(1/qmax(19) - qmin(19))*dq(25) + svm_vec_4(5)*(1/qmax(20) - qmin(20))*dq(26) + svm_vec_4(6)*(1/qmax(21) - qmin(21))*dq(27) + svm_vec_4(7)*(1/qmax(22) - qmin(22))*dq(28) + svm_vec_4(8)*(1/qmax(23) - qmin(23))*dq(29) + svm_vec_4(9)*(1/qmax(24) - qmin(24))*dq(30) + svm_vec_4(10)*(1/qmax(25) - qmin(25))*dq(31) <= svm_val_4;
    svm_vec_5(1)*(1/qmax(1) - qmin(1))*dq(7) + svm_vec_5(2)*(1/qmax(2) - qmin(2))*dq(8) + svm_vec_5(3)*(1/qmax(3) - qmin(3))*dq(9) + svm_vec_5(4)*(1/qmax(7) - qmin(7))*dq(13) + svm_vec_5(5)*(1/qmax(8) - qmin(8))*dq(14) + svm_vec_5(6)*(1/qmax(9) - qmin(9))*dq(15) + svm_vec_5(7)*(1/qmax(10) - qmin(10))*dq(16) + svm_vec_5(8)*(1/qmax(11) - qmin(11))*dq(17) + svm_vec_5(9)*(1/qmax(12) - qmin(12))*dq(18) + svm_vec_5(10)*(1/qmax(26) - qmin(26))*dq(32) + svm_vec_5(11)*(1/qmax(27) - qmin(27))*dq(33) + svm_vec_5(12)*(1/qmax(28) - qmin(28))*dq(34) + svm_vec_5(13)*(1/qmax(29) - qmin(29))*dq(35) + svm_vec_5(14)*(1/qmax(30) - qmin(30))*dq(36) + svm_vec_5(15)*(1/qmax(31) - qmin(31))*dq(37) + svm_vec_5(16)*(1/qmax(32) - qmin(32))*dq(38) <= svm_val_5;
    svm_vec_6(1)*(1/qmax(1) - qmin(1))*dq(7) + svm_vec_6(2)*(1/qmax(2) - qmin(2))*dq(8) + svm_vec_6(3)*(1/qmax(3) - qmin(3))*dq(9) + svm_vec_6(4)*(1/qmax(13) - qmin(13))*dq(19) + svm_vec_6(5)*(1/qmax(14) - qmin(14))*dq(20) + svm_vec_6(6)*(1/qmax(15) - qmin(15))*dq(21) + svm_vec_6(7)*(1/qmax(16) - qmin(16))*dq(22) + svm_vec_6(8)*(1/qmax(17) - qmin(17))*dq(23) + svm_vec_6(9)*(1/qmax(18) - qmin(18))*dq(24) + svm_vec_6(10)*(1/qmax(26) - qmin(26))*dq(32) + svm_vec_6(11)*(1/qmax(27) - qmin(27))*dq(33) + svm_vec_6(12)*(1/qmax(28) - qmin(28))*dq(34) + svm_vec_6(13)*(1/qmax(29) - qmin(29))*dq(35) + svm_vec_6(14)*(1/qmax(30) - qmin(30))*dq(36) + svm_vec_6(15)*(1/qmax(31) - qmin(31))*dq(37) + svm_vec_6(16)*(1/qmax(32) - qmin(32))*dq(38) <= svm_val_6;
    svm_vec_7(1)*(1/qmax(1) - qmin(1))*dq(7) + svm_vec_7(2)*(1/qmax(2) - qmin(2))*dq(8) + svm_vec_7(3)*(1/qmax(3) - qmin(3))*dq(9) + svm_vec_7(4)*(1/qmax(26) - qmin(26))*dq(32) + svm_vec_7(5)*(1/qmax(27) - qmin(27))*dq(33) + svm_vec_7(6)*(1/qmax(28) - qmin(28))*dq(34) + svm_vec_7(7)*(1/qmax(29) - qmin(29))*dq(35) + svm_vec_7(8)*(1/qmax(30) - qmin(30))*dq(36) + svm_vec_7(9)*(1/qmax(31) - qmin(31))*dq(37) + svm_vec_7(10)*(1/qmax(32) - qmin(32))*dq(38) <= svm_val_7;
    svm_vec_8(1)*(1/qmax(7) - qmin(7))*dq(13) + svm_vec_8(2)*(1/qmax(8) - qmin(8))*dq(14) + svm_vec_8(3)*(1/qmax(9) - qmin(9))*dq(15) + svm_vec_8(4)*(1/qmax(10) - qmin(10))*dq(16) + svm_vec_8(5)*(1/qmax(11) - qmin(11))*dq(17) + svm_vec_8(6)*(1/qmax(12) - qmin(12))*dq(18) + svm_vec_8(7)*(1/qmax(13) - qmin(13))*dq(19) + svm_vec_8(8)*(1/qmax(14) - qmin(14))*dq(20) + svm_vec_8(9)*(1/qmax(15) - qmin(15))*dq(21) + svm_vec_8(10)*(1/qmax(16) - qmin(16))*dq(22) + svm_vec_8(11)*(1/qmax(17) - qmin(17))*dq(23) + svm_vec_8(12)*(1/qmax(18) - qmin(18))*dq(24) <= svm_val_8;
    svm_vec_9(1)*(1/qmax(1) - qmin(1))*dq(7) + svm_vec_9(2)*(1/qmax(2) - qmin(2))*dq(8) + svm_vec_9(3)*(1/qmax(3) - qmin(3))*dq(9) + svm_vec_9(4)*(1/qmax(7) - qmin(7))*dq(13) + svm_vec_9(5)*(1/qmax(8) - qmin(8))*dq(14) + svm_vec_9(6)*(1/qmax(9) - qmin(9))*dq(15) + svm_vec_9(7)*(1/qmax(10) - qmin(10))*dq(16) + svm_vec_9(8)*(1/qmax(11) - qmin(11))*dq(17) + svm_vec_9(9)*(1/qmax(12) - qmin(12))*dq(18) <= svm_val_9;
    svm_vec_10(1)*(1/qmax(1) - qmin(1))*dq(7) + svm_vec_10(2)*(1/qmax(2) - qmin(2))*dq(8) + svm_vec_10(3)*(1/qmax(3) - qmin(3))*dq(9) + svm_vec_10(4)*(1/qmax(13) - qmin(13))*dq(19) + svm_vec_10(5)*(1/qmax(14) - qmin(14))*dq(20) + svm_vec_10(6)*(1/qmax(15) - qmin(15))*dq(21) + svm_vec_10(7)*(1/qmax(16) - qmin(16))*dq(22) + svm_vec_10(8)*(1/qmax(17) - qmin(17))*dq(23) + svm_vec_10(9)*(1/qmax(18) - qmin(18))*dq(24) <= svm_val_10;
cvx_end
vars.delta = delta;
vars.dq = dq;
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
