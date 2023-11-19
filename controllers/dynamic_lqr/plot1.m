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
yaw_dset = data(:, 13);
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

figure;

subplot(3,3,1);
plot(t, TorqueL, t, TorqueR);
legend("Drive Torque Left" ,"Drive Torque Right",'Location','southwest');
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

%%%%%%%%%% set comparison %%%%%%%%%%
subplot(3,3,2);
plot(t, TorqueLL_set, t, TorqueRL_set, t, TorqueLR_set, t, TorqueRR_set);
legend("Left Joint Torque Left set" ,"Right Joint Torque Left set", "Left Joint Torque right set" ,"Right Joint Torque right set",'Location','southwest');
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

subplot(3,3,3);
plot(t, TorqueLL_now, t, TorqueRL_now, t, TorqueLR_now, t, TorqueRR_now);
legend("Left Joint Torque Left" ,"Right Joint Torque Left", "Left Joint Torque Right" ,"Right Joint Torque Right",'Location','southwest');
xlabel("t(s)");
ylabel("Torque(Nm)");
grid on;

% subplot(3,3,2);
% plot(t, TorqueLL_now, t, TorqueRL_now, t, TorqueLL_set, t, TorqueRL_set);
% legend("Left Joint Torque Left" ,"Right Joint Torque Left", "Left Joint Torque Left set" ,"Right Joint Torque Left set",'Location','southwest');
% xlabel("t(s)");
% ylabel("Torque(Nm)");
% grid on;

% subplot(3,3,3);
% plot(t, TorqueLR_now, t, TorqueRR_now, t, TorqueLL_set, t, TorqueRR_set);
% legend("Left Joint Torque Right" ,"Right Joint Torque Right", "Left Joint Torque Right set" ,"Right Joint Torque Right set",'Location','southwest');
% xlabel("t(s)");
% ylabel("Torque(Nm)");
% grid on;

%%%%%%%%%% Now vs Set %%%%%%%%%%
% subplot(3,3,2);
% plot(t, TorqueLL_now, t, TorqueLL_set);
% legend("Left Joint Torque Left", "Left Joint Torque Left set", 'Location','southwest');
% xlabel("t(s)");
% ylabel("Torque(Nm)");
% grid on;

% subplot(3,3,3);
% plot(t, TorqueRL_now, t, TorqueRL_set);
% legend("Right Joint Torque Left" ,"Right Joint Torque Left set",'Location','southwest');
% xlabel("t(s)");
% ylabel("Torque(Nm)");
% grid on;

subplot(3,3,4);
plot(t, isJumpInTheAir);
legend("isJumpInTheAir" ,'Location','southwest');
xlabel("t(s)");
ylabel("Boolean");
grid on;

% subplot(3,3,4);
% ylim([0 2]);
% plot(t, velocity_now, 'b', t, velocity_set, 'r');
% legend('v_{now}', 'v_{set}','Location','southwest');
% xlabel("t(s)");
% ylabel("Torque(Nm)");
% grid on;

subplot(3,3,5);
plot(t, theta0_L, t, theta0_R);
legend("theta0_L", "theta0_R",'Location','southwest');
xlabel("t(s)");
ylabel("theta0(rad)");
grid on;

subplot(3,3,6);
plot(t, LF_set, t, RF_set);
legend("Left support F set","Right support F set",'Location','southwest');
xlabel("t(s)");
ylabel("Support Force(N)");
grid on;

% subplot(3,3,6);
% plot(t, pitch);
% legend("pitch",'Location','southwest');
% xlabel("t(s)");
% ylabel("pitch(rad)");
% grid on;

subplot(3,3,7);
plot(t, roll, t, roll_set);
legend("roll", "roll_{set}",'Location','southwest');
xlabel("t(s)");
ylabel("roll(rad)");
grid on;

subplot(3,3,8);
plot(t, WheelSpeedL, t, WheelSpeedR);
legend("WheelSpeedL", "WheelSpeedR",'Location','southwest');
xlabel("t(s)");
ylabel("speed(m/s)");
grid on;

% subplot(3,3,8);
% plot(t, yaw_dot, t, yaw_dset);
% legend("yaw_{dot}", "yaw_{dset}",'Location','southwest');
% xlabel("t(s)");
% ylabel("yaw dot(rad/s)");
% grid on;

subplot(3,3,9);
plot(t, L0_L*1000, t, L0_L_set*1000, t, L0_R*1000, t, L0_R_set*1000);
legend("L0 L","L0 Lset", "L0 R","L0 Rset",'Location','southwest');
xlabel("t(s)");
ylabel("L0(mm)");
grid on;

% subplot(3,3,10);
% plot(t, L0_L*1000, t, L0_L_set*1000);
% legend("L0 L","L0 Lset",'Location','southwest');
% xlabel("t(s)");
% ylabel("L0(mm)");
% grid on;
%
% subplot(3,3,11);
% plot(t, L0_R*1000, t, L0_R_set*1000);
% legend("L0 R","L0 Rset",'Location','southwest');
% xlabel("t(s)");
% ylabel("L0(mm)");
% grid on;
