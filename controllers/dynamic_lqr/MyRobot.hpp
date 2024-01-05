/***
 * @CreatedTime   2022-04-23 19:57:04
 * @LastEditors   Undefined
 * @LastEditTime  2022-04-25 15:29:11
 * @FilePath      \bishe\Robot.h
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen>
#include "MyDefine.h"
#include "Pid.h"
#include "Leg.h"

using namespace Eigen;
using namespace webots;
using namespace std;

class MyRobot : public Robot
{
private:
    /* data */
    u8 time_step;        // millisecond
    float time;          // second
    float sampling_time; 
    float yaw_get;
    int starttime;
    float balance_angle;

    Camera *camera;
    Gyro *gyro;
    InertialUnit *imu;
    GPS *gps;
    PositionSensor *encoder_wheelL, *encoder_wheelR, *encoder_BL, *encoder_BR, *encoder_FL, *encoder_FR;
    Motor *BL_legmotor, *BR_legmotor, *FL_legmotor, *FR_legmotor, *L_Wheelmotor, *R_Wheelmotor;
    Keyboard *mkeyboard;

    PID_Controller turn_pid;
    PID_Controller split_pid;
    PID_Controller roll_pid;
    PID_Controller invPendulumInAir_pid;
    PID_Controller wheelBrakeInAirL_pid;
    PID_Controller wheelBrakeInAirR_pid;
    Matrix<float, 12, 4> K_coeff;
    Matrix<float, 12, 4> K_coeff_inAir;

    DataStructure initialLegPosition_L; // get inital position
    DataStructure initialLegPosition_R;
    float initialBalanceAngle;
    enum eJumpState
    {
        JUMP_INIT = 0,
        JUMP_CHARGE = 1,
        JUMP_LAUNCH = 2,
        JUMP_SHRINK = 3,
        JUMP_IDLE = 4,
        JUMP_RESUME = 5,
    } jumpState;

    float acc_up_max, acc_down_max, acc_now;

    u8 stop_flag;

public:
    MyRobot();
    ~MyRobot();

    u8 sampling_flag;

    LegClass leg_L, leg_R, leg_simplified;
    DataStructure velocity, yaw, pitch, roll;

    // Jump parameters
    // @TODO: detection of landing by combination of sensors: motor torque, altitude, etc.
    bool isJumpInTheAir;
    const float JumpTorqueDecayTau = 0.01;

    const float MotorTorque_Clearance = 0.5;
    float HipTorque_MaxLimit = HIP_TORQUE_MAX - MotorTorque_Clearance;
    float DriveTorque_MaxLimit = DRIVE_TORQUE_MAX - MotorTorque_Clearance;

    const float Torque_landing_threshold = 0.1; // landing detection torque threshold
    const float LegL0_Min = 0.2;
    const float LegL0_Max = 0.35;
    const float LegL0_Mid = (LegL0_Min+LegL0_Max)/2;
    const float LegL0_Clearance = 0.02;
    const float LegL0_Min_Threshold = LegL0_Min + LegL0_Clearance;
    const float LegL0_Max_Threshold = LegL0_Max - LegL0_Clearance;

    static MyRobot *get()
    {
        static MyRobot robot;
        return &robot;
    }

    void command_motor(void);
    void inv_pendulum_ctrl(LegClass *leg_sim, LegClass *leg_L, LegClass *leg_R,
                           DataStructure pitch,
                           float dt, float v_set);
    void torque_ctrl(LegClass *leg_sim, LegClass *leg_L, LegClass *leg_R,
                     DataStructure pitch, DataStructure roll, DataStructure yaw,
                     float dt, float v_set);
    void status_update(LegClass *leg_sim, LegClass *leg_L, LegClass *leg_R,
                       DataStructure pitch, DataStructure roll, DataStructure yaw,
                       float dt, float v_set);
    void MyStep();
    void Wait(int ms);
    void run();
    void jumpManager();
    float limitVelocity(float speed_set, float L0);
    double getVelNow() { return velocity.now; };
    double getVelSet() { return velocity.set; };
    double getWheelLeftTorque() { return L_Wheelmotor->getTorqueFeedback(); };
    double getWheelRightTorque() { return R_Wheelmotor->getTorqueFeedback(); };
};

#endif
