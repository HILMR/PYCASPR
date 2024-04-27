function t_c = F_G_t_coeffs(in1,in2)
%F_G_T_COEFFS
%    T_C = F_G_T_COEFFS(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Dec-2020 17:11:19

c1 = in2(1,:);
c2 = in2(2,:);
c3 = in2(3,:);
c4 = in2(4,:);
c5 = in2(5,:);
c6 = in2(6,:);
c7 = in2(7,:);
c8 = in2(8,:);
c9 = in2(9,:);
c10 = in2(10,:);
c11 = in2(11,:);
c12 = in2(12,:);
c13 = in2(13,:);
c14 = in2(14,:);
c15 = in2(15,:);
c16 = in2(16,:);
c17 = in2(17,:);
c18 = in2(18,:);
c19 = in2(19,:);
c20 = in2(20,:);
c21 = in2(21,:);
c22 = in2(22,:);
c23 = in2(23,:);
c24 = in2(24,:);
c25 = in2(25,:);
c26 = in2(26,:);
c27 = in2(27,:);
c28 = in2(28,:);
c29 = in2(29,:);
c30 = in2(30,:);
c31 = in2(31,:);
e11 = in1(1);
e12 = in1(4);
e21 = in1(2);
e22 = in1(5);
e31 = in1(3);
e32 = in1(6);
t2 = e11-e12;
t3 = t2.^2;
t4 = e21-e22;
t5 = t4.^2;
t6 = e31-e32;
t7 = t6.^2;
t8 = e12.^2;
t9 = e22.^2;
t10 = e32.^2;
t_c = [c1.*t3.^2+c2.*t5.^2+c3.*t7.^2+c10.*t3.*t5+c11.*t3.*t7+c12.*t5.*t7+c4.*t2.*t3.*t4+c5.*t2.*t3.*t6+c6.*t2.*t4.*t5+c7.*t4.*t5.*t6+c8.*t2.*t6.*t7+c9.*t4.*t6.*t7,c13.*t2.*t3+c14.*t4.*t5+c16.*t3.*t4+c18.*t2.*t5+c17.*t3.*t6+c15.*t6.*t7+c20.*t2.*t7+c19.*t5.*t6+c21.*t4.*t7+c1.*e12.*t2.*t3.*4.0+c4.*e12.*t3.*t4.*3.0+c5.*e12.*t3.*t6.*3.0+c6.*e12.*t4.*t5+c10.*e12.*t2.*t5.*2.0+c4.*e22.*t2.*t3+c11.*e12.*t2.*t7.*2.0+c2.*e22.*t4.*t5.*4.0+c8.*e12.*t6.*t7+c6.*e22.*t2.*t5.*3.0+c10.*e22.*t3.*t4.*2.0+c7.*e22.*t5.*t6.*3.0+c5.*e32.*t2.*t3+c9.*e22.*t6.*t7+c12.*e22.*t4.*t7.*2.0+c3.*e32.*t6.*t7.*4.0+c7.*e32.*t4.*t5+c8.*e32.*t2.*t7.*3.0+c9.*e32.*t4.*t7.*3.0+c11.*e32.*t3.*t6.*2.0+c12.*e32.*t5.*t6.*2.0,c22.*t3+c23.*t5+c24.*t7+c13.*e12.*t3.*3.0+c18.*e12.*t5+c20.*e12.*t7+c14.*e22.*t5.*3.0+c16.*e22.*t3+c21.*e22.*t7+c17.*e32.*t3+c15.*e32.*t7.*3.0+c19.*e32.*t5+c1.*t3.*t8.*6.0+c2.*t5.*t9.*6.0+c3.*t7.*t10.*6.0+c10.*t3.*t9+c10.*t5.*t8+c11.*t3.*t10+c11.*t7.*t8+c12.*t5.*t10+c12.*t7.*t9+c25.*t2.*t4+c26.*t2.*t6+c27.*t4.*t6+c4.*e12.*e22.*t3.*3.0+c6.*e12.*e22.*t5.*3.0+c5.*e12.*e32.*t3.*3.0+c8.*e12.*e32.*t7.*3.0+c7.*e22.*e32.*t5.*3.0+c9.*e22.*e32.*t7.*3.0+c16.*e12.*t2.*t4.*2.0+c17.*e12.*t2.*t6.*2.0+c18.*e22.*t2.*t4.*2.0+c19.*e22.*t4.*t6.*2.0+c20.*e32.*t2.*t6.*2.0+c21.*e32.*t4.*t6.*2.0+c4.*t2.*t4.*t8.*3.0+c5.*t2.*t6.*t8.*3.0+c6.*t2.*t4.*t9.*3.0+c7.*t4.*t6.*t9.*3.0+c8.*t2.*t6.*t10.*3.0+c9.*t4.*t6.*t10.*3.0+c10.*e12.*e22.*t2.*t4.*4.0+c11.*e12.*e32.*t2.*t6.*4.0+c12.*e22.*e32.*t4.*t6.*4.0,c28.*t2+c29.*t4+c30.*t6+c22.*e12.*t2.*2.0+c25.*e12.*t4+c26.*e12.*t6+c23.*e22.*t4.*2.0+c25.*e22.*t2+c27.*e22.*t6+c26.*e32.*t2+c24.*e32.*t6.*2.0+c27.*e32.*t4+c13.*t2.*t8.*3.0+c14.*t4.*t9.*3.0+c16.*t4.*t8+c18.*t2.*t9+c15.*t6.*t10.*3.0+c17.*t6.*t8+c20.*t2.*t10+c19.*t6.*t9+c21.*t4.*t10+c16.*e12.*e22.*t2.*2.0+c18.*e12.*e22.*t4.*2.0+c17.*e12.*e32.*t2.*2.0+c20.*e12.*e32.*t6.*2.0+c19.*e22.*e32.*t4.*2.0+c21.*e22.*e32.*t6.*2.0+c1.*e12.*t2.*t8.*4.0+c4.*e12.*t4.*t8+c5.*e12.*t6.*t8+c6.*e12.*t4.*t9.*3.0+c10.*e12.*t2.*t9.*2.0+c11.*e12.*t2.*t10.*2.0+c4.*e22.*t2.*t8.*3.0+c8.*e12.*t6.*t10.*3.0+c2.*e22.*t4.*t9.*4.0+c6.*e22.*t2.*t9+c7.*e22.*t6.*t9+c10.*e22.*t4.*t8.*2.0+c5.*e32.*t2.*t8.*3.0+c9.*e22.*t6.*t10.*3.0+c12.*e22.*t4.*t10.*2.0+c3.*e32.*t6.*t10.*4.0+c7.*e32.*t4.*t9.*3.0+c8.*e32.*t2.*t10+c9.*e32.*t4.*t10+c11.*e32.*t6.*t8.*2.0+c12.*e32.*t6.*t9.*2.0,c31+c28.*e12+c29.*e22+c30.*e32+c22.*t8+c23.*t9+c24.*t10+c1.*t8.^2+c2.*t9.^2+c3.*t10.^2+c25.*e12.*e22+c26.*e12.*e32+c27.*e22.*e32+c13.*e12.*t8+c18.*e12.*t9+c20.*e12.*t10+c14.*e22.*t9+c16.*e22.*t8+c21.*e22.*t10+c15.*e32.*t10+c17.*e32.*t8+c19.*e32.*t9+c10.*t8.*t9+c11.*t8.*t10+c12.*t9.*t10+c4.*e12.*e22.*t8+c6.*e12.*e22.*t9+c5.*e12.*e32.*t8+c8.*e12.*e32.*t10+c7.*e22.*e32.*t9+c9.*e22.*e32.*t10];