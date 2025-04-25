clear
clc
B1 = [0 0 1 0 0.033 0]';
B2 = [0 -1 0 -0.5076 0 0]';
B3 = [0 -1 0 -0.3526 0 0]';
B4 = [0 -1 0 -0.2176 0 0]';
B5 = [0 0 1 0 0 0]';
Blist = [B1 B2 B3 B4 B5];
Tsb = [1 0 0 0;
         0 1 0 0;
         0 0 1 0.0963;
         0 0 0 1];
Tb0 = [1 0 0 0.1662
        0 1 0 0
        0 0 1 0.0026
        0 0 0 1];
T0e = [1 0 0 0.033
        0 1 0 0
        0 0 1 0.6546
        0 0 0 1];
l = 0.47/2;
r = 0.0475;
w = 0.3/2;
F = r/4.*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w)
    1 1 1 1
    -1 1 -1 1];
F6 = zeros(6,4); 
F6(3:5,1:4) = F;
%%
cfg = [0 0 0 0 0 0.2 -1.6 0];

Xd = [0 0 1 0.5
    0 1 0 0
    -1 0 0 0.5
    0 0 0 1];
Xd_n = [0 0 1 0.6
    0 1 0 0
    -1 0 0 0.3
    0 0 0 1];
X = [0.17 0 0.985 0.387
    0 1 0 0
    -0.985 0 0.17 0.570
    0 0 0 1];
dt = 0.01;

Kp = 0*eye(6);
Ki = 0*eye(6);

[V,Xerr_k] = Feedbackcontrol(X, Xd, Xd_n, Kp, Ki, dt)
thetalist = cfg(4:8); 
Jb_arm = JacobianBody(Blist,thetalist);
T_0e = FKinBody(T0e,Blist,thetalist')

Jb_base = Adjoint(TransInv(T_0e)*TransInv(Tb0))*F6;
Je = [Jb_base Jb_arm];
u_control = pinv(Je,1e-4)*V;

function [V,Xerr_k] = Feedbackcontrol(X, Xd, Xd_n, Kp, Ki, dt)
%X = Tse
%Xd = Tse,d
%Xd_n = Tse,d,next

%V: ee twist in frame e
%Feed forward
Xerr_i = 0;
Vd = 1/dt*MatrixLog6(TransInv(Xd)*Xd_n);
Vd = se3ToVec(Vd); %to vector
%Proportional
Xerr_k = MatrixLog6(TransInv(X)*Xd);
Xerr_k = se3ToVec(Xerr_k); %to vector
%integral
Xerr_i = Xerr_i + Xerr_k *dt;

V = Adjoint(TransInv(X)*Xd)*Vd ...
    + Kp * Xerr_k...
    + Ki * Xerr_i;

end