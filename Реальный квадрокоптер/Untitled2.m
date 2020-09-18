clear;
clc
syms k l b Ix L Iy delta_Fy delta_Mx delta_My delta_Mz Iz
A = [k , k, k, k;
    k*L, k*L, -k*L,-k*L;
    k*b, -k*b, k*b, -k*b;
    k*L, - k*L, -k*L, k*L];
B=[delta_Fy;delta_Mx;delta_My;delta_Mz];
X = linsolve(A, B);
simplify(X)