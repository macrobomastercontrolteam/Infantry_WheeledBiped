data = load('data2.dat');

t = data(:,1);
TorqueL = data(:,2);
TorqueR = data(:,3);
velocity_now = data(:, 4);
velocity_set = data(:, 5);
theta0 = data(:,6);
theta0_dot = data(:, 7);
pitch = data(:, 8);
pitch_dot = data(:, 9);
roll = data(:, 10);
roll_set = data(:, 11);
yaw_dot = data(:, 12);

R_angle4 = data(:, 13);

L0_L = data(:, 14);
L0_R = data(:, 15);
theta0_L = data(:, 16);
theta0_R = data(:, 17);
L0_L_set = data(:, 18);
L0_R_set = data(:, 19);

TorqueLL_now = data(:,20);
TorqueRL_now = data(:,21);
TorqueLR_now = data(:,22);
TorqueRR_now = data(:,23);

TorqueLL_set = data(:,24);
TorqueRL_set = data(:,25);
TorqueLR_set = data(:,26);
TorqueRR_set = data(:,27);

isJumpInTheAir = data(:,28);

LF_set = data(:,29);
RF_set = data(:,30);

WheelSpeedL = data(:,31);
WheelSpeedR = data(:,32);

SupportF_L = data(:,33);
SupportF_R = data(:,34);

figure;

subplot(3,3,1);
stairs([t,t], [TorqueL,TorqueR]);
legend("Drive L" ,"Drive R",'Location','southwest');
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

%%%%%%%%%% set comparison %%%%%%%%%%
subplot(3,3,2);
stairs([t t t t], [TorqueLL_set, TorqueRL_set, TorqueLR_set, TorqueRR_set]);
legend("Joint LL set" ,"Joint RL set", "Joint LR set" ,"Joint RR set",'Location','southwest');
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

subplot(3,3,3);
stairs([t t t t], [TorqueLL_now, TorqueRL_now, TorqueLR_now, TorqueRR_now]);
legend("Joint LL" ,"Joint RL", "Joint LR" ,"Joint RR",'Location','southwest');
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

% subplot(3,3,2);
% stairs(t, TorqueLL_now, t, TorqueRL_now, t, TorqueLL_set, t, TorqueRL_set);
% legend("Joint LL" ,"Joint RL", "Joint LL set" ,"Joint RL set",'Location','southwest');
% xlabel("t(s)");
% ylabel("Torque(Nm)");
% grid on;

% subplot(3,3,3);
% stairs(t, TorqueLR_now, t, TorqueRR_now, t, TorqueLL_set, t, TorqueRR_set);
% legend("Joint LR" ,"Joint RR", "Joint LR set" ,"Joint RR set",'Location','southwest');
% xlabel("t(s)");
% ylabel("Torque(Nm)");
% grid on;

%%%%%%%%%% Now vs Set %%%%%%%%%%
% subplot(3,3,2);
% stairs(t, TorqueLL_now, t, TorqueLL_set);
% legend("Joint LL", "Joint LL set", 'Location','southwest');
% xlabel("t(s)");
% ylabel("Torque(Nm)");
% grid on;

% subplot(3,3,3);
% stairs(t, TorqueRL_now, t, TorqueRL_set);
% legend("Joint RL" ,"Joint RL set",'Location','southwest');
% xlabel("t(s)");
% ylabel("Torque(Nm)");
% grid on;

subplot(3,3,4);
stairs(t, isJumpInTheAir);
legend("isJumpInTheAir" ,'Location','southwest');
xlabel("t(s)");
ylabel("Boolean");
grid on;

% subplot(3,3,4);
% ylim([0 2]);
% stairs(t, velocity_now, 'b', t, velocity_set, 'r');
% legend('v_{now}', 'v_{set}','Location','southwest');
% xlabel("t(s)");
% ylabel("Torque(Nm)");
% grid on;

subplot(3,3,5);
stairs([t t t], [theta0_L, theta0_R, pitch]);
legend("theta0_L", "theta0_R", "pitch",'Location','southwest');
xlabel("t(s)");
ylabel("angle(rad)");
grid on;

subplot(3,3,6);
stairs([t t], [LF_set, RF_set]);
legend("Left support F set","Right support F set",'Location','southwest');
xlabel("t(s)");
ylabel("Support Force(N)");
grid on;

% subplot(3,3,6);
% stairs(t, pitch);
% legend("pitch",'Location','southwest');
% xlabel("t(s)");
% ylabel("pitch(rad)");
% grid on;

% subplot(3,3,7);
% stairs(t, roll, t, roll_set);
% legend("roll", "roll_{set}",'Location','southwest');
% xlabel("t(s)");
% ylabel("roll(rad)");
% grid on;

subplot(3,3,7);
stairs([t t], [SupportF_L, SupportF_R]);
legend("SupportF_L", "SupportF_R",'Location','southwest');
xlabel("t(s)");
ylabel("Force(N)");
grid on;

subplot(3,3,8);
stairs([t t], [WheelSpeedL, WheelSpeedR]);
legend("WheelSpeedL", "WheelSpeedR",'Location','southwest');
xlabel("t(s)");
ylabel("speed(m/s)");
grid on;

% subplot(3,3,8);
% stairs(t, R_angle4);
% legend("R_angle4",'Location','southwest');
% xlabel("t(s)");
% ylabel("angle(rad)");
% grid on;

subplot(3,3,9);
stairs([t t t t], [L0_L*1000, L0_L_set*1000, L0_R*1000, L0_R_set*1000]);
legend("L0 L","L0 Lset", "L0 R","L0 Rset",'Location','southwest');
xlabel("t(s)");
ylabel("L0(mm)");
grid on;

% subplot(3,3,10);
% stairs(t, L0_L*1000, t, L0_L_set*1000);
% legend("L0 L","L0 Lset",'Location','southwest');
% xlabel("t(s)");
% ylabel("L0(mm)");
% grid on;
%
% subplot(3,3,11);
% stairs(t, L0_R*1000, t, L0_R_set*1000);
% legend("L0 R","L0 Rset",'Location','southwest');
% xlabel("t(s)");
% ylabel("L0(mm)");
% grid on;