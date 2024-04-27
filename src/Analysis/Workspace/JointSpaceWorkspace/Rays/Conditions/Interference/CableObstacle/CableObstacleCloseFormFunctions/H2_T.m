function out1 = H2_T(in1,in2,in3)
%H2_T
%    OUT1 = H2_T(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    14-Oct-2020 11:03:12

A1 = in2(1,:);
A2 = in2(2,:);
A3 = in2(3,:);
b11 = in1(1);
b12 = in1(4);
b21 = in1(2);
b22 = in1(5);
b31 = in1(3);
b32 = in1(6);
c1 = in3(1,:);
c2 = in3(2,:);
c3 = in3(3,:);
c4 = in3(4,:);
c5 = in3(5,:);
c6 = in3(6,:);
c7 = in3(7,:);
c8 = in3(8,:);
c9 = in3(9,:);
c10 = in3(10,:);
c11 = in3(11,:);
c12 = in3(12,:);
c13 = in3(13,:);
c14 = in3(14,:);
c15 = in3(15,:);
c16 = in3(16,:);
c17 = in3(17,:);
c18 = in3(18,:);
c19 = in3(19,:);
c20 = in3(20,:);
c21 = in3(21,:);
c22 = in3(22,:);
c23 = in3(23,:);
c24 = in3(24,:);
c25 = in3(25,:);
c26 = in3(26,:);
c27 = in3(27,:);
t2 = A1.^2;
t3 = A2.^2;
t4 = A3.^2;
t5 = b11.^2;
t6 = b21.^2;
t7 = b31.^2;
t8 = -b12;
t9 = -b22;
t10 = -b32;
t11 = A1+t8;
t12 = A2+t9;
t13 = A3+t10;
t14 = t11.^2;
t15 = t12.^2;
t16 = t13.^2;
out1 = [c22.*t5+c23.*t6+c24.*t7+A1.*c13.*t5.*3.0+A2.*c14.*t6.*3.0+A2.*c16.*t5+A1.*c18.*t6+A3.*c15.*t7.*3.0+A3.*c17.*t5+A1.*c20.*t7+A3.*c19.*t6+A2.*c21.*t7+b11.*b21.*c25+b11.*b31.*c26+b21.*b31.*c27+c1.*t2.*t5.*6.0+c2.*t3.*t6.*6.0+c3.*t4.*t7.*6.0+c10.*t2.*t6+c10.*t3.*t5+c11.*t2.*t7+c11.*t4.*t5+c12.*t3.*t7+c12.*t4.*t6+A1.*A2.*c4.*t5.*3.0+A1.*A3.*c5.*t5.*3.0+A1.*A2.*c6.*t6.*3.0+A2.*A3.*c7.*t6.*3.0+A1.*A3.*c8.*t7.*3.0+A2.*A3.*c9.*t7.*3.0+A1.*b11.*b21.*c16.*2.0+A2.*b11.*b21.*c18.*2.0+A1.*b11.*b31.*c17.*2.0+A3.*b11.*b31.*c20.*2.0+A2.*b21.*b31.*c19.*2.0+A3.*b21.*b31.*c21.*2.0+b11.*b21.*c4.*t2.*3.0+b11.*b21.*c6.*t3.*3.0+b11.*b31.*c5.*t2.*3.0+b11.*b31.*c8.*t4.*3.0+b21.*b31.*c7.*t3.*3.0+b21.*b31.*c9.*t4.*3.0+A1.*A2.*b11.*b21.*c10.*4.0+A1.*A3.*b11.*b31.*c11.*4.0+A2.*A3.*b21.*b31.*c12.*4.0,b11.*c22.*t11.*-2.0-b11.*c25.*t12-b11.*c26.*t13-b21.*c23.*t12.*2.0-b21.*c25.*t11-b21.*c27.*t13-b31.*c24.*t13.*2.0-b31.*c26.*t11-b31.*c27.*t12-b11.*c1.*t2.*t11.*1.2e+1-b11.*c4.*t2.*t12.*3.0-b11.*c5.*t2.*t13.*3.0-b11.*c6.*t3.*t12.*3.0-b11.*c10.*t3.*t11.*2.0-b11.*c8.*t4.*t13.*3.0-b11.*c11.*t4.*t11.*2.0-b21.*c2.*t3.*t12.*1.2e+1-b21.*c4.*t2.*t11.*3.0-b21.*c6.*t3.*t11.*3.0-b21.*c7.*t3.*t13.*3.0-b21.*c10.*t2.*t12.*2.0-b21.*c9.*t4.*t13.*3.0-b21.*c12.*t4.*t12.*2.0-b31.*c5.*t2.*t11.*3.0-b31.*c3.*t4.*t13.*1.2e+1-b31.*c7.*t3.*t12.*3.0-b31.*c8.*t4.*t11.*3.0-b31.*c9.*t4.*t12.*3.0-b31.*c11.*t2.*t13.*2.0-b31.*c12.*t3.*t13.*2.0-A1.*b11.*c13.*t11.*6.0-A1.*b11.*c16.*t12.*2.0-A2.*b11.*c16.*t11.*2.0-A1.*b11.*c17.*t13.*2.0-A3.*b11.*c17.*t11.*2.0-A2.*b11.*c18.*t12.*2.0-A3.*b11.*c20.*t13.*2.0-A1.*b21.*c16.*t11.*2.0-A2.*b21.*c14.*t12.*6.0-A1.*b21.*c18.*t12.*2.0-A2.*b21.*c18.*t11.*2.0-A2.*b21.*c19.*t13.*2.0-A3.*b21.*c19.*t12.*2.0-A3.*b21.*c21.*t13.*2.0-A1.*b31.*c17.*t11.*2.0-A3.*b31.*c15.*t13.*6.0-A2.*b31.*c19.*t12.*2.0-A1.*b31.*c20.*t13.*2.0-A3.*b31.*c20.*t11.*2.0-A2.*b31.*c21.*t13.*2.0-A3.*b31.*c21.*t12.*2.0-A1.*A2.*b11.*c4.*t11.*6.0-A1.*A3.*b11.*c5.*t11.*6.0-A1.*A2.*b11.*c10.*t12.*4.0-A1.*A3.*b11.*c11.*t13.*4.0-A1.*A2.*b21.*c6.*t12.*6.0-A1.*A2.*b21.*c10.*t11.*4.0-A2.*A3.*b21.*c7.*t12.*6.0-A2.*A3.*b21.*c12.*t13.*4.0-A1.*A3.*b31.*c8.*t13.*6.0-A1.*A3.*b31.*c11.*t11.*4.0-A2.*A3.*b31.*c9.*t13.*6.0-A2.*A3.*b31.*c12.*t12.*4.0,c22.*t14+c23.*t15+c24.*t16+A1.*c13.*t14.*3.0+A2.*c14.*t15.*3.0+A2.*c16.*t14+A1.*c18.*t15+A3.*c15.*t16.*3.0+A3.*c17.*t14+A1.*c20.*t16+A3.*c19.*t15+A2.*c21.*t16+c1.*t2.*t14.*6.0+c2.*t3.*t15.*6.0+c3.*t4.*t16.*6.0+c10.*t2.*t15+c10.*t3.*t14+c11.*t2.*t16+c11.*t4.*t14+c12.*t3.*t16+c12.*t4.*t15+c25.*t11.*t12+c26.*t11.*t13+c27.*t12.*t13+c4.*t2.*t11.*t12.*3.0+c5.*t2.*t11.*t13.*3.0+c6.*t3.*t11.*t12.*3.0+c7.*t3.*t12.*t13.*3.0+c8.*t4.*t11.*t13.*3.0+c9.*t4.*t12.*t13.*3.0+A1.*A2.*c4.*t14.*3.0+A1.*A3.*c5.*t14.*3.0+A1.*A2.*c6.*t15.*3.0+A2.*A3.*c7.*t15.*3.0+A1.*A3.*c8.*t16.*3.0+A2.*A3.*c9.*t16.*3.0+A1.*c16.*t11.*t12.*2.0+A1.*c17.*t11.*t13.*2.0+A2.*c18.*t11.*t12.*2.0+A2.*c19.*t12.*t13.*2.0+A3.*c20.*t11.*t13.*2.0+A3.*c21.*t12.*t13.*2.0+A1.*A2.*c10.*t11.*t12.*4.0+A1.*A3.*c11.*t11.*t13.*4.0+A2.*A3.*c12.*t12.*t13.*4.0];