#include "biped.h"
#include "stdio.h"
#include "string.h"
#include <webots/robot.h>

#define JUMP_CHARGE_WAIT_MS 100.0f
#define JUMP_CHARGE_TIMEOUT_MS 1000.0f
#define JUMP_LAUNCH_TIMEOUT_MS 1000.0f
#define JUMP_SUPPORT_FORCE_IGNORE_TIMEOUT_MS 300.0f
#define JUMP_SHRINK_TIMEOUT_MS 1000.0f

#define LEG_L0_MIN_THRESHOLD (LEG_L0_MIN + 0.03f)
#define LEG_L0_MAX_THRESHOLD (LEG_L0_MAX - 0.04f)

#define LQR_ADJUST_COEFF 1.0f

biped_t biped;

// Jump parameters
const fp32 JumpTorqueDecayTau = 0.01f;

void command_motor(void);
void biped_limitSetpoint(void);
void biped_jumpManager(void);
void biped_status_update(void);

void biped_init(void)
{
	biped.time_step_s = CHASSIS_CONTROL_TIME_S;

	LegClass_t_Init(&biped.leg_L);
	LegClass_t_Init(&biped.leg_R);
	LegClass_t_Init(&biped.leg_simplified);

	biped.fBipedEnable = 0;

	// biped.yaw_dot_last = 0;
	biped.yaw.last = 0;
	// @TODO
	// biped.balance_angle = -0.0064f; // keep facing downwards
	biped.balance_angle = 0.0f; // keep facing downwards
	biped.jumpState = JUMP_IDLE;
	biped.brakeState = BRAKE_IDLE;
	biped.isJumpInTheAir = 0;

	biped.roll.set = 0;
	biped.yaw.set = 0;
	// biped.velocity.set = 0;
	// biped.velocity.now = 0;
	// biped.acc_up_max = 1.0;
	// biped.acc_down_max = 1.0;

	biped.fBipedEnable = 0;
	biped.HipTorque_MaxLimit = 0;
	biped.DriveTorque_MaxLimit = 0;

	// PID for angular velocity
	// fragile, must keep Ki = 0
	const fp32 turn_pid_param[3] = {3, 0.001f, 0.1f}; // for 10ms loop time
	PID_init(&biped.turn_pid, PID_POSITION, turn_pid_param, 3, 1, 0.9f, &rad_err_handler);

	// fragile, must keep Ki very small
	const fp32 split_pid_param[3] = {100, 0, 10}; // for 5ms loop time
	PID_init(&biped.split_pid, PID_POSITION, split_pid_param, HIP_TORQUE_MAX, HIP_TORQUE_MAX / 2.0f, 0.9f, &filter_err_handler);

	const fp32 roll_pid_param[3] = {1000, 0, 10};
	PID_init(&biped.roll_pid, PID_POSITION, roll_pid_param, 25, 0, 0.9f, &filter_err_handler);

	const fp32 invPendulumInAir_pid_param[3] = {200, 10, 10};
	PID_init(&biped.invPendulumInAir_pid, PID_POSITION, invPendulumInAir_pid_param, HIP_TORQUE_MAX * 2.0f, HIP_TORQUE_MAX * 2.0f, 0.9f, &filter_err_handler);

	const fp32 wheelBrakeInAir_pid_param[3] = {7, 1, 0};
	PID_init(&biped.wheelBrakeInAirL_pid, PID_POSITION, wheelBrakeInAir_pid_param, DRIVE_TORQUE_MAX, DRIVE_TORQUE_MAX, 0.9f, &filter_err_handler);
	PID_init(&biped.wheelBrakeInAirR_pid, PID_POSITION, wheelBrakeInAir_pid_param, DRIVE_TORQUE_MAX, DRIVE_TORQUE_MAX, 0.9f, &filter_err_handler);

#if (MODEL_ORIG_RM_CAP == 0)
	const fp32 K_coeff_init[12][4] = {
		{-228.312861923588, 294.032841402069, -161.859980835154, -27.9124607725149},
		{-311.553508160858, 488.738545499858, -288.194998275156, 69.8532732024956},
		{-25.0754494709052, 30.5969720475325, -27.3011613956695, -7.41107450495795},
		{-93.0949035697954, 139.631825126946, -79.3257068867355, 19.0350639940176},
		{-33.1309413829780, 40.3900983299054, -17.2457434417403, -7.36223473018761},
		{-106.980642246657, 153.627662686586, -82.3368141566720, 17.7880546399025},
		{-18.1356341071563, 20.8763808035375, -9.34978742039602, -10.0834547221280},
		{-145.995128360192, 202.696600747869, -104.633505889388, 21.7346468775176},
		{-378.233687962070, 543.155810337850, -291.104598164008, 62.8902703006772},
		{468.542266417460, -571.202248474433, 243.891642697369, 104.117722046717},
		{-13.4938391590358, 19.1993789061047, -10.4541636935974, 2.47975499342254},
		{13.6341049836551, -16.7207310260419, 7.21326317048111, 3.11249772591918}};
#elif ((MODEL_ORIG_RM_CAP == 1) || (MODEL_ORIG_RM_CAP == 2))
	const fp32 K_coeff_init[12][4] = {
		{-67.5535479847655, 124.646483120571, -98.4756037854575, -29.0334310447502},
		{178.221313840797, -67.1211042652315, -81.7832322332885, 51.4018806865341},
		{25.1486489385685, -20.5828911647525, -8.23254533101615, -9.28219514778163},
		{40.9211217574687, -10.3569754613559, -24.4605565670383, 14.5066570570501},
		{-13.2057269871359, 21.4116803768985, -12.0297886549121, -7.56675873258061},
		{12.2672396312499, 21.9473645455584, -35.5594295926848, 13.9271852550812},
		{5.75766110963911, -2.52120495161917, -2.02574233316584, -10.5341475151121},
		{-3.15918339675547, 47.0630030589857, -50.2008824277757, 17.3533957999215},
		{43.3712415847315, 77.5956515656025, -125.721569025392, 49.2400356865271},
		{186.757182268181, -302.80688804881, 170.126902765673, 107.010128218062},
		{-2.20079649909642, 5.53533895833806, -4.57580709932171, 1.55561219584514},
		{4.66676997580658, -7.42632648445587, 4.13675939662882, 2.29204214490184}};
#endif

	memcpy(biped.K_coeff, K_coeff_init, sizeof(biped.K_coeff));

	for (uint8_t row = 0; row < 12; row++)
	{
		for (uint8_t col = 0; col < 4; col++)
		{
			// // higher gain for dis
			// if ((row == 2 * 2 + 0) || (row == 2 * 2 + 1))
			// {
			// 	biped.K_coeff[row][col] *= 0.25f;
			// }
			// else
			{
				biped.K_coeff[row][col] *= LQR_ADJUST_COEFF;
			}
		}
	}

	// const fp32 K_coeff_inAir_init[12][4] = {
	// 	{0, 0, 0, 0},
	// 	{-75.1602, 117.0037, 191.5325, 17.9267},
	// 	{0, 0, 0, 0},
	// 	{24.8338, 21.9035, -4.0933, 22.7593},
	// 	{0, 0, 0, 0},
	// 	{0, 0, 0, 0},
	// 	{0, 0, 0, 0},
	// 	{0, 0, 0, 0},
	// 	{0, 0, 0, 0},
	// 	{0, 0, 0, 0},
	// 	{0, 0, 0, 0},
	// 	{0, 0, 0, 0}};
	// memcpy(biped.K_coeff_inAir, K_coeff_inAir_init, sizeof(biped.K_coeff_inAir));

	// biped.Tp_final_estimator.size = TP_FINAL_ESTIMATOR_SIZE;
	// biped.Tp_final_estimator.ring = biped.Tp_final_estimator_buffer;
	// moving_average_calc(0, &(biped.Tp_final_estimator), MOVING_AVERAGE_RESET);
}

void biped_status_update(void)
{
	// biped.roll.dot = Deadzone(biped.roll.dot, 0.1f);
	biped.roll.last = biped.roll.now;

	// biped.pitch.dot = Deadzone(biped.pitch.dot, 0.0f);
	biped.pitch.now -= biped.balance_angle; // the angle relative to the balanced pitch

	// biped.yaw.dot = Deadzone(biped.yaw.dot, 0.08f);
	// biped.yaw.ddot = (biped.yaw.dot - biped.yaw_dot_last) / (biped.time_step_s);
	// biped.yaw_dot_last = biped.yaw.dot;
	if (biped.yaw.last == 0)
		biped.yaw.set = biped.yaw.now;
	biped.yaw.last = biped.yaw.now;

	biped.leg_L.dis.now = biped.motor_measure[CHASSIS_ID_DRIVE_LEFT].multi_position * DRIVE_WHEEL_RADIUS;
	biped.leg_R.dis.now = biped.motor_measure[CHASSIS_ID_DRIVE_RIGHT].multi_position * DRIVE_WHEEL_RADIUS;
	biped.leg_simplified.dis.now = (biped.leg_L.dis.now + biped.leg_R.dis.now) / 2.0f;

	biped.leg_L.dis.dot = (biped.leg_L.dis.now - biped.leg_L.dis.last) / biped.time_step_s;
	biped.leg_R.dis.dot = (biped.leg_R.dis.now - biped.leg_R.dis.last) / biped.time_step_s;
	biped.leg_simplified.dis.dot = (biped.leg_simplified.dis.now - biped.leg_simplified.dis.last) / biped.time_step_s;
	// biped.leg_simplified.dis.dot = first_order_filter((biped.leg_simplified.dis.now - biped.leg_simplified.dis.last) / biped.time_step_s, biped.leg_simplified.dis.dot, 0.9f);

	biped.leg_L.dis.last = biped.leg_L.dis.now;
	biped.leg_R.dis.last = biped.leg_R.dis.now;
	biped.leg_simplified.dis.last = biped.leg_simplified.dis.now;

	LegClass_t_ForwardKinematics(&biped.leg_L, biped.pitch.now);
	biped.leg_L.angle0.dot = (biped.leg_L.angle0.now - biped.leg_L.angle0.last) / biped.time_step_s;

	LegClass_t_ForwardKinematics(&biped.leg_R, biped.pitch.now);
	biped.leg_R.angle0.dot = (biped.leg_R.angle0.now - biped.leg_R.angle0.last) / biped.time_step_s;

	biped.leg_simplified.angle1 = (biped.leg_L.angle1 + biped.leg_R.angle1) / 2.0f;
	biped.leg_simplified.angle4 = (biped.leg_L.angle4 + biped.leg_R.angle4) / 2.0f;
	LegClass_t_ForwardKinematics(&biped.leg_simplified, biped.pitch.now);
	biped.leg_simplified.angle0.dot = (biped.leg_simplified.angle0.now - biped.leg_simplified.angle0.last) / biped.time_step_s;
	// biped.leg_simplified.angle0.dot = first_order_filter((biped.leg_simplified.angle0.now - biped.leg_simplified.angle0.last) / biped.time_step_s, biped.leg_simplified.angle0.dot, 0.9f);

	biped.leg_L.L0.dot = (biped.leg_L.L0.now - biped.leg_L.L0.last) / biped.time_step_s;
	biped.leg_L.L0.last = biped.leg_L.L0.now;
	biped.leg_R.L0.dot = (biped.leg_R.L0.now - biped.leg_R.L0.last) / biped.time_step_s;
	biped.leg_R.L0.last = biped.leg_R.L0.now;
}

void inv_pendulum_ctrl(void)
{
	if (biped.isJumpInTheAir)
	{
		biped.leg_L.TWheel_set = PID_calc(&biped.wheelBrakeInAirL_pid, biped.leg_L.dis.dot, 0, biped.time_step_s);
		biped.leg_R.TWheel_set = PID_calc(&biped.wheelBrakeInAirR_pid, biped.leg_R.dis.dot, 0, biped.time_step_s);

		// @TODO: predict best simLeg angle before landing based on initial velocity
		fp32 out_invPendulumInAir = PID_calc(&biped.invPendulumInAir_pid, biped.leg_simplified.angle0.now, 0, biped.time_step_s);
		biped.leg_L.Tp_set = -out_invPendulumInAir;
		biped.leg_R.Tp_set = -out_invPendulumInAir;
	}
	else
	{
		// Choose the K_coeff based upon if the robot is jump in the air
		// fp32(*biped.K_coeff)[4];
		// K_coeff_now = (biped.isJumpInTheAir) ? biped.K_coeff_inAir : biped.K_coeff;

		// Generating K matrix based on current leg lengths
		for (uint8_t col = 0; col < 6; col++)
		{
			for (uint8_t row = 0; row < 2; row++)
			{
				uint8_t num = col * 2 + row;
				biped.leg_simplified.K[row][col] = biped.K_coeff[num][0] * pow(biped.leg_simplified.L0.now, 3) +
				                                   biped.K_coeff[num][1] * pow(biped.leg_simplified.L0.now, 2) +
				                                   biped.K_coeff[num][2] * biped.leg_simplified.L0.now +
				                                   biped.K_coeff[num][3];
			}
		}

		memset(biped.leg_simplified.Xd, 0, sizeof(biped.leg_simplified.Xd));
		biped.leg_simplified.Xd[2][0] = biped.leg_simplified.dis.set;

		biped.leg_simplified.X[0][0] = biped.leg_simplified.angle0.now;
		biped.leg_simplified.X[1][0] = biped.leg_simplified.angle0.dot;
		biped.leg_simplified.X[2][0] = biped.leg_simplified.dis.now;
		biped.leg_simplified.X[3][0] = biped.leg_simplified.dis.dot;
		biped.leg_simplified.X[4][0] = biped.pitch.now;
		biped.leg_simplified.X[5][0] = biped.pitch.dot;

		fp32 matrix_Xd_minus_X[6][1];
		for (uint8_t row = 0; row < 6; row++)
		{
			matrix_Xd_minus_X[row][0] = biped.leg_simplified.Xd[row][0] - biped.leg_simplified.X[row][0];
		}

		// u = K(Xd-X);
		fp32 Matrix_u[2][1];
		matrixMultiplication(2, 6, 6, 1, biped.leg_simplified.K, matrix_Xd_minus_X, Matrix_u);
		biped.leg_simplified.TWheel_set = Matrix_u[0][0];
		biped.leg_simplified.Tp_set = Matrix_u[1][0];

		// reduction = 22.5% at 0.8
		// const fp32 TWheel_brakezone_threshold = 3.6f;
		// const fp32 TWheel_brakezone_order = 1.0f;
		// reduction = 25% at 0.8
		// const fp32 TWheel_brakezone_threshold = 3.2f;
		// const fp32 TWheel_brakezone_order = 1.0f;
		// reduction = 20% at 0.8
		// const fp32 TWheel_brakezone_threshold = 4.0f;
		// const fp32 TWheel_brakezone_order = 1.0f;
		// biped.leg_simplified.TWheel_set = brakezone(biped.leg_simplified.TWheel_set, TWheel_brakezone_threshold, TWheel_brakezone_order);

		// output
		// if (biped.jumpState == JUMP_LAUNCH)
		// {
		// 	// Maintain wheel speed during JUMP_LAUNCH, to avoid singularity in LQR when wheel leaves ground
		// 	biped.leg_L.TWheel_set = 0;
		// 	biped.leg_R.TWheel_set = 0;
		// }
		// else
		{
			biped.leg_L.TWheel_set = biped.leg_simplified.TWheel_set / 2.0f;
			biped.leg_R.TWheel_set = biped.leg_simplified.TWheel_set / 2.0f;
		}
		biped.leg_L.Tp_set = -biped.leg_simplified.Tp_set / 2.0f;
		biped.leg_R.Tp_set = -biped.leg_simplified.Tp_set / 2.0f;
	}
}

void torque_ctrl()
{
	// Support force offset
	if (biped.jumpState == JUMP_LAUNCH)
	{
		// Force is directly set in JUMP_LAUNCH state
	}
	else if ((biped.isJumpInTheAir) || (biped.jumpState == JUMP_CHARGE))
	{
		biped.leg_L.F_set = 0;
		biped.leg_R.F_set = 0;
	}
	else
	{
		biped.leg_L.F_set = -UNLOADED_ROBOT_HALF_WEIGHT;
		biped.leg_R.F_set = -UNLOADED_ROBOT_HALF_WEIGHT;
	}

	/****************** Hip joint torque adjustment by PID ******************/
	fp32 out_spilt = PID_calc(&biped.split_pid, biped.leg_L.angle0.now - biped.leg_R.angle0.now, 0, biped.time_step_s);
	biped.leg_L.Tp_set -= out_spilt; // sign is determined based on simulation
	biped.leg_R.Tp_set += out_spilt;

	if (biped.isJumpInTheAir == 0)
	{
		fp32 out_roll = PID_calc(&biped.roll_pid, biped.roll.now, biped.roll.set, biped.time_step_s);
		biped.leg_L.F_set -= out_roll;
		biped.leg_R.F_set += out_roll;
	}

	// leg length adjustment
	if (biped.jumpState == JUMP_LAUNCH)
	{
		// No leg length PID control in JUMP_LAUNCH state
	}
	else
	{
		pid_type_def *legL_pid_mode;
		pid_type_def *legR_pid_mode;
		// if (biped.jumpState == JUMP_CHARGE)
		// {
		// 	legL_pid_mode = &(biped.leg_L.supportFCharge_pid);
		// 	legR_pid_mode = &(biped.leg_R.supportFCharge_pid);
		// }
		if (biped.isJumpInTheAir)
		{
			// Slightly maintain leg length in JUMP_CHARGE state to avoid structure collision
			legL_pid_mode = &(biped.leg_L.supportFInAir_pid);
			legR_pid_mode = &(biped.leg_R.supportFInAir_pid);
		}
		else
		{
			legL_pid_mode = &(biped.leg_L.supportF_pid);
			legR_pid_mode = &(biped.leg_R.supportF_pid);
		}
		fp32 out_L = PID_calc(legL_pid_mode, biped.leg_L.L0.now, biped.leg_L.L0.set, biped.time_step_s);
		fp32 out_R = PID_calc(legR_pid_mode, biped.leg_R.L0.now, biped.leg_R.L0.set, biped.time_step_s);
		biped.leg_L.F_set -= out_L;
		biped.leg_R.F_set -= out_R;
	}

	if ((biped.jumpState != JUMP_LAUNCH) && (biped.isJumpInTheAir == 0))
	{
		fp32 out_turn = PID_calc(&biped.turn_pid, biped.yaw.now, biped.yaw.set, biped.time_step_s);
		biped.leg_L.TWheel_set -= out_turn;
		biped.leg_R.TWheel_set += out_turn;
		// counteract torque by friction
		// Note: Tp_set and Twheel_set have opposite directions, and they cancel out each other when effecting the moment of inertia of the simplified leg
		// biped.leg_L.Tp_set += out_turn;
		// biped.leg_R.Tp_set -= out_turn;
	}

	// Virtual Model Control (VMC)
	fp32 Torque_L[2][1];
	fp32 Torque_R[2][1];
	LegClass_t_VMC(&biped.leg_L, biped.leg_L.F_set, biped.leg_L.Tp_set, Torque_L);
	biped.leg_L.TL_set = -Torque_L[0][0];
	biped.leg_L.TR_set = Torque_L[1][0];

	LegClass_t_VMC(&biped.leg_R, biped.leg_R.F_set, biped.leg_R.Tp_set, Torque_R);
	biped.leg_R.TL_set = -Torque_R[0][0];
	biped.leg_R.TR_set = Torque_R[1][0];

	// maintain torque scales after saturation
	fp32 Tp_max = MAX(MAX(MAX(fabs(biped.leg_L.TL_set), fabs(biped.leg_L.TR_set)), fabs(biped.leg_R.TL_set)), fabs(biped.leg_R.TR_set));
	if (Tp_max > biped.HipTorque_MaxLimit)
	{
		fp32 Tp_normalizer = biped.HipTorque_MaxLimit / Tp_max;
		biped.leg_L.TL_set *= Tp_normalizer;
		biped.leg_L.TR_set *= Tp_normalizer;
		biped.leg_R.TL_set *= Tp_normalizer;
		biped.leg_R.TR_set *= Tp_normalizer;
	}
	fp32 TWheel_max = MAX(fabs(biped.leg_L.TWheel_set), fabs(biped.leg_R.TWheel_set));
	if (TWheel_max > biped.DriveTorque_MaxLimit)
	{
		fp32 TWheel_normalizer = biped.DriveTorque_MaxLimit / TWheel_max;
		biped.leg_L.TWheel_set *= TWheel_normalizer;
		biped.leg_R.TWheel_set *= TWheel_normalizer;
	}

	// fp32 Tp_max = MAX(MAX(MAX(fabs(biped.leg_L.TL_set), fabs(biped.leg_L.TR_set)), fabs(biped.leg_R.TL_set)), fabs(biped.leg_R.TR_set));
	// fp32 TWheel_max = MAX(fabs(biped.leg_L.TWheel_set), fabs(biped.leg_R.TWheel_set));
	// if ((Tp_max > biped.HipTorque_MaxLimit) || (TWheel_max > biped.DriveTorque_MaxLimit))
	// {
	// 	fp32 Tp_normalizer = biped.HipTorque_MaxLimit / Tp_max;
	// 	fp32 TWheel_normalizer = biped.DriveTorque_MaxLimit / TWheel_max;
	// 	fp32 final_normalizer = MIN(Tp_normalizer, TWheel_normalizer);
	// 	biped.leg_L.TL_set *= final_normalizer;
	// 	biped.leg_L.TR_set *= final_normalizer;
	// 	biped.leg_R.TL_set *= final_normalizer;
	// 	biped.leg_R.TR_set *= final_normalizer;

	// 	biped.leg_L.TWheel_set *= final_normalizer;
	// 	biped.leg_R.TWheel_set *= final_normalizer;
	// }
}

void biped_limitSetpoint(void)
{
	biped.leg_L.L0.set = fp32_constrain(biped.leg_L.L0.set, LEG_L0_MIN, LEG_L0_MAX);
	biped.leg_R.L0.set = fp32_constrain(biped.leg_R.L0.set, LEG_L0_MIN, LEG_L0_MAX);

	// biped.velocity.set = limitVelocity(fp32_constrain(biped.velocity.set, -2.5f, 2.5f),biped.leg_simplified.L0.now);
	biped.roll.set = fp32_constrain(biped.roll.set, -0.4f, 0.4f);
}

/**
 * @brief: Manage jump state
 * @author: 2024 Mac Capstone Team
 * @Date: 2024-01-01
 */
void biped_jumpManager(void)
{
	static uint32_t state_entry_time_ms = 0;
	static jumpState_e last_jumpState = JUMP_IDLE;
	switch (biped.jumpState)
	{
		case JUMP_IDLE:
		{
			// safety guard
			biped.isJumpInTheAir = 0;
			break;
		}
		case JUMP_CHARGE_INIT:
		{
			// Raise the legs to initiate the jump
			if (biped.isJumpInTheAir == 0)
			{
				// Part of force to support robot body is removed but the part to maintain leg length is kept to avoid structure collision
				biped.leg_L.L0.set = LEG_L0_MIN;
				biped.leg_R.L0.set = LEG_L0_MIN;

				// @TODO: jump with non-zero roll
				biped.roll.set = 0;

				biped.jumpState = JUMP_CHARGE;
			}
			else
			{
				biped.leg_L.L0.set = LEG_L0_MID;
				biped.leg_R.L0.set = LEG_L0_MID;
				biped.jumpState = JUMP_IDLE;
			}
			break;
		}
		case JUMP_CHARGE:
		{
			if ((fabs(biped.roll.now) < 0.1f) &&
			    (fabs(biped.pitch.now - biped.balance_angle) < 0.1f) &&
			    (fabs(biped.pitch.dot) < 0.1f) &&
			    (MIN(biped.leg_L.L0.now, biped.leg_R.L0.now) <= LEG_L0_MIN_THRESHOLD) &&
			    (biped.time_ms - state_entry_time_ms > JUMP_CHARGE_WAIT_MS))
			{
				PID_clear(&biped.leg_L.supportFCharge_pid);
				PID_clear(&biped.leg_R.supportFCharge_pid);
				biped.HipTorque_MaxLimit = HIP_TORQUE_BURST_MAX;
				biped.jumpState = JUMP_LAUNCH;
			}
			else if (biped.time_ms - state_entry_time_ms > JUMP_CHARGE_TIMEOUT_MS)
			{
				biped.leg_L.L0.set = LEG_L0_MID;
				biped.leg_R.L0.set = LEG_L0_MID;
				biped.jumpState = JUMP_IDLE;
			}
			break;
		}
		case JUMP_LAUNCH:
		{
			if ((MAX(biped.leg_L.L0.now, biped.leg_R.L0.now) >= LEG_L0_MAX_THRESHOLD) ||
			    (biped.time_ms - state_entry_time_ms > JUMP_LAUNCH_TIMEOUT_MS))
			{
				biped.isJumpInTheAir = 1;

				// Back to PID control
				biped.leg_L.L0.set = LEG_L0_MID;
				biped.leg_R.L0.set = LEG_L0_MID;
				// Avoid integral windup
				PID_clear(&biped.leg_L.supportFInAir_pid);
				PID_clear(&biped.leg_R.supportFInAir_pid);
				biped.jumpState = JUMP_SHRINK;
			}
			else
			{
				// @TODO: Direct decaying torque control
				fp32 initSupportF = -300.0f; // arbitrary large value
				fp32 decaySupportF = initSupportF * (1.0f - exp((MAX(biped.leg_L.L0.now, biped.leg_R.L0.now) - LEG_L0_MAX_THRESHOLD) / JumpTorqueDecayTau));
				biped.leg_L.F_set = decaySupportF;
				biped.leg_R.F_set = decaySupportF;
			}
			break;
		}
		case JUMP_SHRINK:
		{
			// Shrink until landing
			fp32 SupportF_L[2][1];
			fp32 SupportF_R[2][1];
			LegClass_t_Inv_VMC(&biped.leg_L, -biped.motor_measure[CHASSIS_ID_HIP_BL].torque, biped.motor_measure[CHASSIS_ID_HIP_FL].torque, SupportF_L);
			LegClass_t_Inv_VMC(&biped.leg_R, -biped.motor_measure[CHASSIS_ID_HIP_BR].torque, biped.motor_measure[CHASSIS_ID_HIP_FR].torque, SupportF_R);

			// if (biped.time_ms - state_entry_time_ms < 900)
			// {
			// 	printf("leg_L_angle1_dot: %f, leg_R_angle1_dot: %f, SupportF: %f, biped.leg_simplified.angle0.dot = %f, biped.time_ms = %d\n", leg_L_angle1_dot, leg_R_angle1_dot, SupportF_L[0][0] + SupportF_R[0][0], biped.leg_simplified.angle0.dot, biped.time_ms);
			// }

			// JUMP_SUPPORT_FORCE_IGNORE_TIMEOUT_MS avoids detected the impact when hip collide with the hinge stop
			if (biped.time_ms - state_entry_time_ms > JUMP_SUPPORT_FORCE_IGNORE_TIMEOUT_MS)
			{
				// to avoid singularity when leg is very long, must restart support mode after some time being landed
				uint8_t fLanded = 0;
				// fLanded |= (SupportF_L[0][0] + SupportF_R[0][0] >= UNLOADED_ROBOT_WEIGHT * 3.0f);
				fLanded |= (biped.leg_simplified.angle0.dot >= 1.5f); // best judgement of landing according to testing
				// fLanded |= (biped.leg_L.F_set + biped.leg_R.F_set > UNLOADED_ROBOT_WEIGHT);
				if (fLanded)
				{
					biped.isJumpInTheAir = 0;
					PID_clear(&biped.leg_L.supportF_pid);
					PID_clear(&biped.leg_R.supportF_pid);
					PID_clear(&biped.turn_pid);
					biped.jumpState = JUMP_IDLE;
					biped.HipTorque_MaxLimit = HIP_TORQUE_MAX;
				}
				else if (biped.time_ms - state_entry_time_ms > JUMP_SHRINK_TIMEOUT_MS)
				{
					biped.isJumpInTheAir = 0;
					biped.jumpState = JUMP_IDLE;
					if (biped.fBipedEnable)
					{
						printf("Emergency Brake Engaged at biped.time_ms = %d!\n", biped.time_ms);
						biped.fBipedEnable = 0;
					}
				}
			}
			break;
		}
		default:
		{
			// should not reach here
			biped.fBipedEnable = 0;
			biped.isJumpInTheAir = 0;
			break;
		}
	}

	if (biped.jumpState != last_jumpState)
	{
		const char jumpStateName[6][20] = {"JUMP_IDLE", "JUMP_CHARGE_INIT", "JUMP_CHARGE", "JUMP_LAUNCH", "JUMP_SHRINK"};
		printf("Jump state is: %s, biped.isJumpInTheAir: %d, biped.time_ms: %d\n", jumpStateName[biped.jumpState], biped.isJumpInTheAir, biped.time_ms);
		last_jumpState = biped.jumpState;
		state_entry_time_ms = biped.time_ms;
	}
}

uint8_t biped_jumpStart(void)
{
	if ((biped.jumpState == JUMP_IDLE) && (biped.isJumpInTheAir == 0))
	{
		biped.jumpState = JUMP_CHARGE_INIT;
		return 1;
	}
	else
	{
		return 0;
	}
}

fp32 limitVelocity(fp32 speed_set, fp32 L0)
{
	fp32 speed_max = -30.0f * L0 + 10.7f;
	if (speed_max < 0)
	{
		speed_max = 0;
	}
	return (speed_set > speed_max) ? speed_max : speed_set;
}

void biped_task(void)
{
	biped_status_update();

	biped_jumpManager();

	biped_limitSetpoint();

	inv_pendulum_ctrl();
	torque_ctrl();

	// Emergency Brake
	if (biped.fBipedEnable)
	{
		if (fabs(biped.pitch.now) >= 80.0f / 180.0f * PI) // || toe_is_error(DBUS_TOE))
		{
			// Angle is too dangerous, stop control
			printf("Emergency Brake Engaged at biped.time_ms = %d!\n", biped.time_ms);
			biped.fBipedEnable = 0;
		}
	}

#if !DISABLE_STEER_MOTOR_POWER
	if (biped.fBipedEnable == 0)
#endif
	{
		biped.HipTorque_MaxLimit = 0;
	}

#if !DISABLE_DRIVE_MOTOR_POWER
	if (biped.fBipedEnable == 0)
#endif
	{
		biped.DriveTorque_MaxLimit = 0;
	}

	biped.leg_L.TL_set = fp32_constrain(biped.leg_L.TL_set, -biped.HipTorque_MaxLimit, biped.HipTorque_MaxLimit);
	biped.leg_L.TR_set = fp32_constrain(biped.leg_L.TR_set, -biped.HipTorque_MaxLimit, biped.HipTorque_MaxLimit);
	biped.leg_R.TL_set = fp32_constrain(biped.leg_R.TL_set, -biped.HipTorque_MaxLimit, biped.HipTorque_MaxLimit);
	biped.leg_R.TR_set = fp32_constrain(biped.leg_R.TR_set, -biped.HipTorque_MaxLimit, biped.HipTorque_MaxLimit);
	biped.leg_L.TWheel_set = fp32_constrain(biped.leg_L.TWheel_set, -biped.DriveTorque_MaxLimit, biped.DriveTorque_MaxLimit);
	biped.leg_R.TWheel_set = fp32_constrain(biped.leg_R.TWheel_set, -biped.DriveTorque_MaxLimit, biped.DriveTorque_MaxLimit);
}