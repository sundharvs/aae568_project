function H_sym = H_fcn(bx,by,bz,q1,q2,q3,q4,rx,ry,rz)
%H_fcn
%    H_sym = H_fcn(BX,BY,BZ,Q1,Q2,Q3,Q4,RX,RY,RZ)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    19-Apr-2024 20:05:36

t2 = q1.^2;
t3 = q2.^2;
t4 = q3.^2;
t5 = q4.^2;
t6 = q4.^3;
t7 = q1.*q2.*2.0;
t8 = q1.*q3.*2.0;
t9 = q1.*q4.*2.0;
t10 = q2.*q3.*2.0;
t11 = q2.*q4.*2.0;
t12 = q3.*q4.*2.0;
t13 = -rx;
t14 = -ry;
t15 = -rz;
t16 = -t10;
t17 = -t11;
t18 = -t12;
t19 = -t3;
t20 = -t4;
t21 = -t5;
t22 = -t6;
t23 = bx+t13;
t24 = by+t14;
t25 = bz+t15;
t26 = t7+t12;
t27 = t8+t11;
t28 = t9+t10;
t29 = t7+t18;
t30 = t8+t17;
t31 = t9+t16;
t32 = t23.*t27;
t33 = t24.*t28;
t34 = t25.*t26;
t38 = t2+t5+t19+t20;
t39 = t2+t3+t20+t21;
t40 = t2+t4+t19+t22;
t35 = t23.*t31;
t36 = t24.*t29;
t37 = t25.*t30;
t44 = t23.*t39;
t45 = t24.*t40;
t46 = t25.*t38;
t41 = -t35;
t42 = -t36;
t43 = -t37;
t47 = t33+t43+t44;
t48 = t32+t42+t46;
t49 = t34+t41+t45;
t50 = t47.^2;
t51 = t48.^2;
t52 = t49.^2;
t53 = 1.0./t47;
t54 = 1.0./t50;
t55 = t50+t51+t52;
t56 = 1.0./sqrt(t55);
mt1 = [t56.*(t27.*t48.*2.0-t31.*t49.*2.0+t39.*t47.*2.0).*(-1.0./2.0),t31.*t53.*(-3.33507935e+2)-t39.*t49.*t54.*3.33507935e+2,t27.*t53.*3.34712952e+2-t39.*t48.*t54.*3.34712952e+2,0.0,0.0,0.0,0.0,0.0,0.0,t56.*(t28.*t47.*2.0-t29.*t48.*2.0+t40.*t49.*2.0).*(-1.0./2.0)];
mt2 = [t40.*t53.*3.33507935e+2-t28.*t49.*t54.*3.33507935e+2,t29.*t53.*(-3.34712952e+2)-t28.*t48.*t54.*3.34712952e+2,0.0,0.0,0.0,0.0,0.0,0.0,t56.*(t26.*t49.*2.0-t30.*t47.*2.0+t38.*t48.*2.0).*(-1.0./2.0)];
mt3 = [t26.*t53.*3.33507935e+2+t30.*t49.*t54.*3.33507935e+2,t38.*t53.*3.34712952e+2+t30.*t48.*t54.*3.34712952e+2,0.0,0.0,0.0,0.0,0.0,0.0];
H_sym = reshape([mt1,mt2,mt3],3,9);
end
