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

SupportF_L = data(:,33);
SupportF_R = data(:,34);

figure;

subplot(3,3,1);
plot(t, theta0_dot);
legend("theta0_dot",'Location','southwest');
xlabel("t(s)");
ylabel("angular speed(rad/s)");
grid on;

subplot(3,3,3);
plot(t, isJumpInTheAir);
legend("isJumpInTheAir" ,'Location','southwest');
xlabel("t(s)");
ylabel("Boolean");
grid on;

subplot(3,3,5);
plot(t, theta0_L, t, theta0_R, t, pitch);
legend("theta0_L", "theta0_R", "pitch",'Location','southwest');
xlabel("t(s)");
ylabel("angle(rad)");
grid on;
