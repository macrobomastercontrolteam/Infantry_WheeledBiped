#ifndef _BIPED_H
#define _BIPED_H

#include "Leg.h"
#include "pid.h"
#include "user_lib.h"
#include <math.h>

#define DRIVE_WHEEL_RADIUS 0.0675f

typedef enum
{
	JUMP_IDLE = 0,
	JUMP_CHARGE_INIT = 1,
	JUMP_CHARGE = 2,
	JUMP_LAUNCH = 3,
	JUMP_SHRINK = 4,
} jumpState_e;

typedef enum
{
	BRAKE_IDLE = 0,
	BRAKE_ENABLE = 1,
} brakeState_e;

// indices of biped.motor_measure
typedef enum
{
	CHASSIS_ID_HIP_FR = 0, // right front
	CHASSIS_ID_HIP_FL = 1, // left front
	CHASSIS_ID_HIP_BL = 2, // left back
	CHASSIS_ID_HIP_BR = 3, // right back
	CHASSIS_ID_DRIVE_RIGHT = 4,
	CHASSIS_ID_DRIVE_LEFT = 5,
	CHASSIS_ID_YAW = 6,
	CHASSIS_ID_PIT = 7,
	CHASSIS_ID_TRIGGER = 8,
	CHASSIS_ID_LAST = CHASSIS_ID_TRIGGER + 1,
} chassis_motor_ID_e;

typedef union
{
	uint8_t temperature;
	struct
	{
		uint16_t ecd;
		int16_t speed_rpm;
		int16_t given_current;
		int16_t last_ecd;
	};
	struct
	{
		fp32 multi_position; // rad
		fp32 position;       // rad
		fp32 velocity;       // rad/s
		fp32 torque;         // Nm
	};
} motor_measure_t;

typedef struct
{
	fp32 time_step_s; // second
	uint32_t time_ms; // millisecond

	pid_type_def turn_pid;
	pid_type_def split_pid;
	pid_type_def roll_pid;
	pid_type_def invPendulumInAir_pid;
	pid_type_def wheelBrakeInAirL_pid;
	pid_type_def wheelBrakeInAirR_pid;

	fp32 K_coeff[12][4];
	fp32 K_coeff_inAir[12][4];

	motor_measure_t motor_measure[CHASSIS_ID_LAST];

	// fp32 acc_up_max, acc_down_max, acc_now;
	fp32 balance_angle;
	// fp32 yaw_dot_last;
	variable_status_t velocity, yaw, pitch, roll;
	fp32 accel_x, accel_y, accel_z;

	LegClass_t leg_L, leg_R, leg_simplified;
	uint8_t isJumpInTheAir;
	uint8_t fBipedEnable;
	fp32 HipTorque_MaxLimit;
	fp32 DriveTorque_MaxLimit;
	jumpState_e jumpState;
	brakeState_e brakeState;
} biped_t;

extern biped_t biped;

void biped_init(void);
void biped_task(void);
void inv_pendulum_ctrl(void);
void torque_ctrl(void);
uint8_t biped_jumpStart(void);
fp32 limitVelocity(fp32 speed_set, fp32 L0);

#endif /* _BIPED_H */
