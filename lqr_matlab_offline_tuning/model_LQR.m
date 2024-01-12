function [K, L_sum] = model_LQR(xc, yc, xp, yp, Ipin)

% xc=-5.6843e-14;
% yc=288.1721;
disp(yc)
% xp=-2.6382e-14;
% yp=114.6396;
% Ipin=0.0299;

global fIsRmCapOriModel;
global ml1 ml2 ml3 ml4;

syms theta(t) x(t) phi(t) T(t) Tp(t) N(t) P(t) Nm(t) Pm(t)
syms theta_dot x_dot phi_dot x_dot_dot theta_dot_dot phi_dot_dot

%进行参数的赋值——都转化成国际单位制
L_sum = sqrt(xc ^ 2 + yc ^ 2);
L_now = sqrt((xc - xp)^2 + (yc - yp)^2);
Lm_now = L_sum - L_now;
L = L_now * 0.001;
Lm = Lm_now * 0.001;

if fIsRmCapOriModel == 1
    R = 0.0675; % drive wheel radius
    mw = 1 * 2; % drive wheel mass
    Iw = 0.0003271 * 2; % rotor inertia
    M = 5; % body mass
    Im = 0.78714333 * 0.062665311 * 5/12;
else
    R = 0.050; % drive wheel radius
    mw = 3 * 2; % drive wheel mass
    Iw = 0.00375805 * 2;
    M = 12; % body mass
    Im = 0.78714333 * 0.062665311;
end

l = 0;
mp = ml1 + ml2 + ml3 + ml4; % pendulum mass
Ip = Ipin;
g = 9.81;
% subs(A_final, ...
%     [R, L, Lm, l, mw, mp, M, Iw, Ip, Im], ...
%     [0.0675, L_now, Lm_now, 0,  0.406, 0.808, 15.245, 0.000508589, Ipin, 0.062665311]);%R=50mm;
% subs(B_final, ...
%     [R, L, Lm, l, mw, mp, M, Iw, Ip, Im], ...
%     [0.0675, L_now, Lm_now, 0,  0.406, 0.808, 15.245, 0.000508589, Ipin, 0.062665311]);%R=50mm;

%列出替代方程
Nm(t) = M*diff(diff(x+(L+Lm)*sin(theta)-l*sin(phi),t) , t);
Pm(t)= M*g + M*diff(diff(x+(L+Lm)*sin(theta)-l*cos(phi),t), t);
N(t) = Nm(t) + mp*diff(diff(x + L*sin(theta), t), t);
P(t) = Pm(t) + mp*g + mp*diff(diff(L*cos(theta), t), t);
%简化，用字母替代
Nm(t) = (subs(Nm(t), ...
    [diff(x, t, t), diff(theta, t, t), diff(phi,t,t)], ...
    [x_dot_dot, theta_dot_dot, phi_dot_dot]));
Nm(t) = vpa(subs(Nm(t), ...
    [diff(x, t), diff(theta, t), diff(phi, t)], ...
    [x_dot theta_dot, phi_dot]));
Pm(t) = subs(Pm(t), ...
    [diff(x, t, t), diff(theta, t, t), diff(phi,t,t)], ...
    [x_dot_dot, theta_dot_dot, phi_dot_dot]);
Pm(t) = vpa(subs(Pm(t), ...
    [diff(x, t), diff(theta, t), diff(phi, t)], ...
    [x_dot theta_dot, phi_dot]));
N(t) = subs(N(t), ...
    [diff(x, t, t), diff(theta, t, t), diff(phi,t,t)], ...
    [x_dot_dot, theta_dot_dot, phi_dot_dot]);
N(t) = vpa(subs(N(t), ...
    [diff(x, t), diff(theta, t), diff(phi, t)], ...
    [x_dot theta_dot, phi_dot]));
P(t) = subs(P(t), ...
    [diff(x, t, t), diff(theta, t, t), diff(phi,t,t)], ...
    [x_dot_dot, theta_dot_dot, phi_dot_dot]);
P(t) = vpa(subs(P(t), ...
    [diff(x, t), diff(theta, t), diff(phi, t)], ...
    [x_dot theta_dot, phi_dot]));

eqns = [x_dot_dot == (T-N(t)*R)/(Iw/R + mw*R);
    Ip*theta_dot_dot == (P(t)*L+Pm(t)*Lm) * sin(theta)-(N(t)*L+ Nm(t)*Lm) * cos(theta)- T(t) + Tp(t);
    Im*phi_dot_dot == Tp(t)+Nm*l*cos(phi)+Pm*l*sin(phi)];
vars = [x_dot_dot theta_dot_dot phi_dot_dot];

[x_dot_dot, theta_dot_dot, phi_dot_dot] = solve(eqns, vars);
%对平衡点进行线性化
X = [theta(t); theta_dot; x(t); x_dot; phi(t); phi_dot];
u = [T(t);Tp(t)];
X_dot = [theta_dot; theta_dot_dot; x_dot; x_dot_dot; phi_dot; phi_dot_dot];
A = jacobian(X_dot, X);
B = jacobian(X_dot, u);
A = subs(A, [theta(t), theta_dot, x_dot, phi(t), phi_dot, T(t), Tp(t)], zeros(1,7));%代入平衡点处的值, X = [0 0 x 0 0 0] u = [0 0]
B = subs(B, [theta(t), theta_dot, x_dot, phi(t), phi_dot, T(t), Tp(t)], zeros(1,7));
A = double(A);
B = double(B);
size(A,1);
%算可控矩阵的秩，如果等于A的阶数，则系统可控
if(rank(ctrb(A, B)) == size(A, 1))
    disp('系统可控')
else
    disp('系统不可控')
end

C = eye(6);
D = zeros(6,2);
v_Q = [100 100 100 10 5000 1];
% v_Q = [100 100 100 10 1e-8 1e-8]; % parameters for in-air state
Q = diag(v_Q);
v_R = [1 0.25];
% v_R = [1e8 1]; % parameters for in-air state
R = diag(v_R);
sys = ss(A, B, C, D);
K = lqr(sys, Q, R); %得到反馈增益矩阵

% Test out the closed-loop system
% closed_sys = ss(A-B*K,B,C,D);
% step(closed_sys);

end
