/***
 * @CreatedTime   2022-04-23 17:02:54
 * @LastEditors   Undefined
 * @LastEditTime  2022-04-23 19:58:16
 * @FilePath      \bishe\bishe.cpp
 */

#include <webots/robot.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include "Leg.h"
#include "biped.h"
#include "pid.h"
#include "user_lib.h"
#include <math.h>
#include <stdio.h>

const int time_step_ms = CHASSIS_CONTROL_TIME_MS; // millisecond
fp32 time_s;                                      // second
fp32 sampling_time;                               // the time_s to start detection, in seconds
int starttime;

WbDeviceTag camera;
WbDeviceTag accel;
WbDeviceTag gyro;
WbDeviceTag imu;
WbDeviceTag gps;
WbDeviceTag encoder_wheelL, encoder_wheelR, encoder_BL, encoder_BR, encoder_FL, encoder_FR;
WbDeviceTag BL_legmotor, BR_legmotor, FL_legmotor, FR_legmotor, L_Wheelmotor, R_Wheelmotor;
WbDeviceTag mkeyboard;

uint8_t stop_flag;
uint8_t sampling_flag;

void Wait(int ms);
void MyRobot_init(void);
void command_motor(void);
void MyRobot_run(void);

int main(int argc, char **argv)
{
	wb_robot_init();

	FILE *fptr = fopen("data2.dat", "w");
	if (fptr == NULL)
	{
		printf("Error! Cannot open file!\n");
		wb_robot_cleanup();
		return 0;
	}

	MyRobot_init();
	biped_init();
	while (wb_robot_step(time_step_ms) != -1)
	{
		time_s = wb_robot_get_time();
		biped.time_ms = time_s * 1000.0f;
		MyRobot_run();
		if (time_s > 0.75)
		{
			// temperory debug
			fp32 SupportF_L[2][1];
			fp32 SupportF_R[2][1];
			LegClass_t_Inv_VMC(&biped.leg_L, -biped.motor_measure[CHASSIS_ID_HIP_BL].torque, biped.motor_measure[CHASSIS_ID_HIP_FL].torque, SupportF_L);
			LegClass_t_Inv_VMC(&biped.leg_R, -biped.motor_measure[CHASSIS_ID_HIP_BR].torque, biped.motor_measure[CHASSIS_ID_HIP_FR].torque, SupportF_R);
			fp32 SupportF_L_temp = SupportF_L[0][0];
			fp32 SupportF_R_temp = SupportF_R[0][0];

			fprintf(fptr, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %f %f %f\n",
			        time_s,
			        biped.leg_L.TWheel_set,
			        biped.leg_R.TWheel_set,
			        biped.leg_simplified.dis.dot, 
			        biped.leg_simplified.angle0.now,
			        biped.leg_simplified.angle0.dot,
			        biped.pitch.now,
			        biped.pitch.dot,
			        biped.roll.now,
			        biped.roll.set,
			        biped.yaw.dot,
			        biped.leg_L.L0.now,
			        biped.leg_R.L0.now,
			        biped.leg_L.angle0.now,
			        biped.leg_R.angle0.now,
			        biped.leg_L.L0.set,
			        biped.leg_R.L0.set,
			        biped.motor_measure[CHASSIS_ID_HIP_BL].torque,
			        biped.motor_measure[CHASSIS_ID_HIP_BR].torque,
			        biped.motor_measure[CHASSIS_ID_HIP_FL].torque,
			        biped.motor_measure[CHASSIS_ID_HIP_FR].torque,
			        biped.leg_L.TL_set,
			        biped.leg_R.TL_set,
			        biped.leg_L.TR_set,
			        biped.leg_R.TR_set,
			        biped.isJumpInTheAir,
			        biped.leg_L.F_set,
			        biped.leg_R.F_set,
			        biped.leg_L.dis.dot,
			        biped.leg_R.dis.dot,
			        SupportF_L_temp,
			        SupportF_R_temp,
			        biped.leg_L.dis.now,
			        biped.leg_R.dis.now,
			        biped.leg_L.Tp_set,
			        biped.leg_R.Tp_set,
			        biped.leg_simplified.dis.set);
		}
	}

	fclose(fptr);
	wb_robot_cleanup();
	return 0;
}

void MyRobot_init(void)
{
	/* Webots settings */
	wb_keyboard_enable(time_step_ms);

	camera = wb_robot_get_device("camera");
	wb_camera_enable(camera, time_step_ms);

	accel = wb_robot_get_device("accel");
	wb_accelerometer_enable(accel, time_step_ms);

	gyro = wb_robot_get_device("gyro");
	wb_gyro_enable(gyro, time_step_ms);

	imu = wb_robot_get_device("imu");
	wb_inertial_unit_enable(imu, time_step_ms);

	gps = wb_robot_get_device("gps");
	wb_gps_enable(gps, time_step_ms);

	encoder_wheelL = wb_robot_get_device("encoder_wheelL");
	wb_position_sensor_enable(encoder_wheelL, time_step_ms);
	encoder_wheelR = wb_robot_get_device("encoder_wheelR");
	wb_position_sensor_enable(encoder_wheelR, time_step_ms);

	encoder_BL = wb_robot_get_device("encoder_BL");
	wb_position_sensor_enable(encoder_BL, time_step_ms);
	encoder_BR = wb_robot_get_device("encoder_BR");
	wb_position_sensor_enable(encoder_BR, time_step_ms);
	encoder_FL = wb_robot_get_device("encoder_FL");
	wb_position_sensor_enable(encoder_FL, time_step_ms);
	encoder_FR = wb_robot_get_device("encoder_FR");
	wb_position_sensor_enable(encoder_FR, time_step_ms);

	L_Wheelmotor = wb_robot_get_device("L_Motor");
	R_Wheelmotor = wb_robot_get_device("R_Motor");
	wb_motor_set_velocity(L_Wheelmotor, 0);
	wb_motor_set_velocity(R_Wheelmotor, 0);
	wb_motor_set_position(L_Wheelmotor, INFINITY);
	wb_motor_set_position(R_Wheelmotor, INFINITY);

	BL_legmotor = wb_robot_get_device("BL_Motor");
	FL_legmotor = wb_robot_get_device("FL_Motor");
	BR_legmotor = wb_robot_get_device("BR_Motor");
	FR_legmotor = wb_robot_get_device("FR_Motor");
	wb_motor_set_position(BL_legmotor, 0);
	wb_motor_set_position(FL_legmotor, 0);
	wb_motor_set_position(BR_legmotor, 0);
	wb_motor_set_position(FR_legmotor, 0);

	wb_motor_enable_torque_feedback(BL_legmotor, time_step_ms);
	wb_motor_enable_torque_feedback(BR_legmotor, time_step_ms);
	wb_motor_enable_torque_feedback(FL_legmotor, time_step_ms);
	wb_motor_enable_torque_feedback(FR_legmotor, time_step_ms);
	wb_motor_enable_torque_feedback(L_Wheelmotor, time_step_ms);
	wb_motor_enable_torque_feedback(R_Wheelmotor, time_step_ms);
}

// /**
//  * @brief: Millisecond Delay
//  * @author: Dandelion
//  * @Date: 2023-03-27 16:35:24
//  * @param {int} ms
//  * @return {*}
//  */
// void Wait(int ms)
// {
// 	fp32 start_time = wb_robot_get_time();
// 	fp32 s = ms / 1000.0;
// 	while (s + start_time >= wb_robot_get_time())
// 		MyStep();
// }

void command_motor(void)
{
	if ((biped.leg_L.TL_set != biped.leg_L.TL_set) || (biped.leg_L.TR_set != biped.leg_L.TR_set) || (biped.leg_R.TL_set != biped.leg_R.TL_set) || (biped.leg_R.TR_set != biped.leg_R.TR_set))
	{
		printf("NaN detected in command_motor\n");
	}

	wb_motor_set_torque(BL_legmotor, biped.leg_L.TL_set);
	wb_motor_set_torque(FL_legmotor, biped.leg_L.TR_set);
	wb_motor_set_torque(BR_legmotor, biped.leg_R.TL_set);
	wb_motor_set_torque(FR_legmotor, biped.leg_R.TR_set);
	// wb_motor_set_position(BL_legmotor, -PI / 3);
	// wb_motor_set_position(FL_legmotor, -PI / 3);
	// wb_motor_set_position(BR_legmotor, -PI / 3);
	// wb_motor_set_position(FR_legmotor, -PI / 3);

	wb_motor_set_torque(L_Wheelmotor, biped.leg_L.TWheel_set);
	wb_motor_set_torque(R_Wheelmotor, biped.leg_R.TWheel_set);
}

void MyRobot_run(void)
{
	const double *imu_rpy_values = wb_inertial_unit_get_roll_pitch_yaw(imu);
	const double *gyro_values = wb_gyro_get_values(gyro);
	const double *accel_values = wb_accelerometer_get_values(accel);

	// fp32 robot_x = gps->getValues()[0];
	// biped.velocity.now = wb_gps_get_speed(gps);
	// Roll Pitch Yaw angles are assigned in an inconventional order for IMU, where XYZ corresponds to RYP
	biped.pitch.now = imu_rpy_values[1];
	// biped.pitch.dot = first_order_filter(gyro_values[2], biped.pitch.dot, 0.9f);
	biped.pitch.dot = gyro_values[2];
	biped.roll.now = imu_rpy_values[0];
	biped.roll.dot = gyro_values[0];
	biped.yaw.now = imu_rpy_values[2];
	biped.yaw.dot = gyro_values[1];

	biped.accel_x = accel_values[0];
	biped.accel_y = accel_values[1];
	biped.accel_z = accel_values[2];
	// @TODO: remove gravity

	biped.motor_measure[CHASSIS_ID_DRIVE_LEFT].multi_position = wb_position_sensor_get_value(encoder_wheelL);
	biped.motor_measure[CHASSIS_ID_DRIVE_RIGHT].multi_position = wb_position_sensor_get_value(encoder_wheelR);

	biped.motor_measure[CHASSIS_ID_HIP_BL].torque = wb_motor_get_torque_feedback(BL_legmotor);
	biped.motor_measure[CHASSIS_ID_HIP_FL].torque = wb_motor_get_torque_feedback(FL_legmotor);
	biped.motor_measure[CHASSIS_ID_HIP_BR].torque = wb_motor_get_torque_feedback(BR_legmotor);
	biped.motor_measure[CHASSIS_ID_HIP_FR].torque = wb_motor_get_torque_feedback(FR_legmotor);

	biped.motor_measure[CHASSIS_ID_DRIVE_LEFT].torque = wb_motor_get_torque_feedback(L_Wheelmotor);
	biped.motor_measure[CHASSIS_ID_DRIVE_RIGHT].torque = wb_motor_get_torque_feedback(R_Wheelmotor);

	biped.leg_L.angle1 = 2.0 / 3.0 * PI - wb_position_sensor_get_value(encoder_FL);
	biped.leg_L.angle4 = 1.0 / 3.0 * PI + wb_position_sensor_get_value(encoder_BL);
	biped.leg_R.angle1 = 2.0 / 3.0 * PI - wb_position_sensor_get_value(encoder_FR);
	biped.leg_R.angle4 = 1.0 / 3.0 * PI + wb_position_sensor_get_value(encoder_BR);

	int key = wb_keyboard_get_key();
	static int last_key;
	while (key > 0)
	{
		switch (key)
		{
			case 'X':
			{
				// disable robot control
				biped.fBipedEnable = 0;
				biped.isJumpInTheAir = 0;
				biped.jumpState = JUMP_IDLE;

				biped.HipTorque_MaxLimit = 0.0f;
				biped.DriveTorque_MaxLimit = 0.0f;
				// biped.leg_L.fResetMultiAngleOffset = 1;
				// biped.leg_R.fResetMultiAngleOffset = 1;
				biped.leg_simplified.dis.now = 0;
				biped.leg_simplified.dis.set = biped.leg_simplified.dis.now;
				break;
			}
			case 'C':
			{
				// enable robot control
				biped.fBipedEnable = 1;
				biped.isJumpInTheAir = 0;
				biped.jumpState = JUMP_IDLE;

				biped.roll.last = biped.roll.now;

				biped.yaw.set = biped.yaw.now;
				biped.yaw.last = biped.yaw.now;

				biped.pitch.set = 0.0f;
				// @TODO: biped.balance_angle
				biped.pitch.last = biped.pitch.now;

				// biped.velocity.set = 0;
				// biped.velocity.last = biped.velocity.now;

				biped.leg_L.dis.now = 0;
				biped.leg_R.dis.now = 0;
				biped.leg_simplified.dis.now = 0;
				biped.leg_simplified.dis.set = biped.leg_simplified.dis.now;

				biped.leg_L.angle0.last = biped.leg_L.angle0.now;
				biped.leg_R.angle0.last = biped.leg_R.angle0.now;

				biped.leg_L.L0.set = 0.25f;
				biped.leg_R.L0.set = 0.25f;
				biped.leg_L.L0.last = biped.leg_L.L0.now;
				biped.leg_R.L0.last = biped.leg_R.L0.now;

				biped.leg_L.dis.last = biped.leg_L.dis.now;
				biped.leg_R.dis.last = biped.leg_R.dis.now;

				// biped.leg_L.fResetMultiAngleOffset = 1;
				// biped.leg_R.fResetMultiAngleOffset = 1;

				biped.HipTorque_MaxLimit = HIP_TORQUE_MAX;
				biped.DriveTorque_MaxLimit = DRIVE_TORQUE_MAX;
				break;
			}
			case WB_KEYBOARD_UP:
				biped.leg_simplified.dis.set += 0.02f;
				break;
			case WB_KEYBOARD_DOWN:
				biped.leg_simplified.dis.set -= 0.02f;
				break;
			case WB_KEYBOARD_RIGHT:
				if (ABS(biped.leg_simplified.dis.dot) > 0.1)
					biped.yaw.set -= 0.01f;
				else
					biped.yaw.set -= 0.02f;
				break;
			case WB_KEYBOARD_LEFT:
				if (ABS(biped.leg_simplified.dis.dot) > 0.1)
					biped.yaw.set += 0.01f;
				else
					biped.yaw.set += 0.02f;
				break;
			case WB_KEYBOARD_END:
				biped.leg_simplified.dis.set = biped.leg_simplified.dis.now;
				biped.yaw.set = 0;
				break;
			case 'W':
				biped.leg_L.L0.set += 0.0002f;
				biped.leg_R.L0.set += 0.0002f;
				break;
			case 'S':
				biped.leg_L.L0.set -= 0.0002f;
				biped.leg_R.L0.set -= 0.0002f;
				break;
			case 'A':
				biped.roll.set -= 0.002f;
				break;
			case 'D':
				biped.roll.set += 0.002f;
				break;
			case 'O':
				sampling_flag = 1;
				sampling_time = time_s;
				break;
			case 'J': // 'J' key to trigger the jump
				biped_jumpStart();
				break;
			case ' ':
				if (last_key != key)
					stop_flag = 1;
				break;
		}
		last_key = key;
		key = wb_keyboard_get_key();
	}

	biped_task();
	command_motor();
}
