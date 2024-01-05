/*
 * @Description: Leg Types
 * @Version: 2.0
 * @Author: Dandelion
 * @Date: 2023-03-24 17:20:36
 * @LastEditTime: 2023-04-18 18:28:10
 * @FilePath: \webots_sim\controllers\dynamic_lqr\Leg.h
 */
#ifndef _LEG_H
#define _LEG_H

#include <Eigen>
#include "MyDefine.h"
#include "Pid.h"
using namespace Eigen;

typedef struct DATA
{
    /* data */
    float now;
    float last;
    float set;
    float set_dot;
    float dot;  // derivative
    float ddot; // second derivative
} DataStructure;

class LegClass
{
private:
    float l1, l2, l3, l4, l5; // Unit mm

public:
    // data
    float angle1, angle2, angle3, angle4; // radians
    DataStructure angle0;
    DataStructure L0;
    DataStructure dis;
    // Coordinates
    float xa, ya;
    float xb, yb;
    float xc, yc;
    float xd, yd;
    float xe, ye;
    // Force and Torque
    float TL_now, TR_now, TWheel_now; 
    float TL_set, TR_set, TWheel_set;
    float F_set;  
    float Tp_set; 

    Matrix<float, 2, 6> K;     
    Matrix<float, 6, 1> X, Xd; //[theta, theta_dot, x, x_dot, phi, phi_dot]
    PID_Controller supportF_pid;
    PID_Controller supportFInAir_pid; // PID when in air

    LegClass();
    void ForwardKinematics(const float angle1, const float angle4, const float pitch);
    void InvKinematics(const float xc, const float yc);
    Matrix<float, 2, 1> VMC(const float F, const float Tp);
    Matrix<float, 2, 1> Inv_VMC(const float TL, const float TR);
};

#endif
