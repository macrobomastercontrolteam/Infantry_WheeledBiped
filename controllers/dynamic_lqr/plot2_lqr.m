data = load('data2.dat');

t = data(:,1);
TWheel_set_L = data(:,2);
TWheel_set_R = data(:,3);
velocity_set = data(:, 4);
angle0_sim = data(:,5);
angle0_dot = data(:, 6);
pitch = data(:, 7);
pitch_dot = data(:, 8);
roll = data(:, 9);
roll_set = data(:, 10);
yaw_dot = data(:, 11);

L0_L = data(:, 12);
L0_R = data(:, 13);
angle0_L = data(:, 14);
angle0_R = data(:, 15);
L0_L_set = data(:, 16);
L0_R_set = data(:, 17);

TorqueLL_now = data(:,18);
TorqueRL_now = data(:,19);
TorqueLR_now = data(:,20);
TorqueRR_now = data(:,21);

TorqueLL_set = data(:,22);
TorqueRL_set = data(:,23);
TorqueLR_set = data(:,24);
TorqueRR_set = data(:,25);

isJumpInTheAir = data(:,26);

LF_set = data(:,27);
RF_set = data(:,28);

WheelSpeedL = data(:,29);
WheelSpeedR = data(:,30);

SupportF_L = data(:,31);
SupportF_R = data(:,32);

WheelDis_L = data(:,33);
WheelDis_R = data(:,34);

Tp_set_L = data(:,35);
Tp_set_R = data(:,36);

WheelDisSet_sim = data(:,37);

figure;

subplot(3,3,1);
stairs([t,t], [TWheel_set_L, TWheel_set_R]);
legend("Drive L" ,"Drive R",'Location','southwest');
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

subplot(3,3,2);
stairs([t,t,t,t], [TorqueLL_set, TorqueRL_set, TorqueLR_set, TorqueRR_set]);
legend("Joint LL set" ,"Joint RL set", "Joint LR set" ,"Joint RR set",'Location','southwest');
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

% subplot(3,3,3);
% stairs([t,t,t,t,t], [angle0_L, angle0_R, angle0_sim, angle0_dot, pitch]);
% legend("angle0_L", "angle0_R", "angle0 sim", "angle0 dot", "pitch", 'Location','southwest');
% xlabel("t(s)");
% ylabel("angle(rad)");
% grid on;

subplot(3,3,3);
stairs([t,t,t], [angle0_sim, angle0_dot, pitch]);
legend("angle0 sim", "angle0 dot", "pitch", 'Location','southwest');
xlabel("t(s)");
ylabel("angle(rad)");
grid on;

subplot(3,3,4);
stairs([t,t], [SupportF_L, SupportF_R]);
legend("SupportF_L", "SupportF_R",'Location','southwest');
xlabel("t(s)");
ylabel("Force(N)");
grid on;

subplot(3,3,5);
stairs([t,t,t], [WheelSpeedL, WheelSpeedR, velocity_set]);
legend("WheelSpeedL", "WheelSpeedR", "velocity_set",'Location','southwest');
xlabel("t(s)");
ylabel("speed(m/s)");
grid on;

subplot(3,3,6);
stairs([t,t,t], [WheelDisSet_sim, WheelDis_L, WheelDis_R]);
legend("DisSet sim","Dis L","Dis R",'Location','southwest');
xlabel("t(s)");
ylabel("dis(m)");
grid on;

subplot(3,3,7);
stairs([t,t,t,t], [L0_L*1000, L0_L_set*1000, L0_R*1000, L0_R_set*1000]);
legend("L0 L","L0 Lset", "L0 R","L0 Rset",'Location','southwest');
xlabel("t(s)");
ylabel("L0(mm)");
grid on;
