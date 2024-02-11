/*
 * @Description: Leg Variables Definition
 * @Version: 2.0
 * @Author: Dandelion
 * @Date: 2023-03-24 17:19:53
 * @LastEditTime: 2023-07-28 23:23:01
 * @FilePath: \webots_sim\controllers\dynamic_lqr\Leg.c
 */
#include "Leg.h"
#include "math.h"
#include "string.h"
#include "user_lib.h"

// Unit mm
#if (MODEL_ORIG_RM_CAP == 0)
const fp32 legLength1 = 0.180f;
const fp32 legLength2 = 0.200f;
const fp32 legLength3 = 0.200f;
const fp32 legLength4 = 0.180f;
const fp32 legLength5 = 0.120f;
#elif ((MODEL_ORIG_RM_CAP == 1) || (MODEL_ORIG_RM_CAP == 2))
// Unit mm
const fp32 legLength1 = 0.150f;
const fp32 legLength2 = 0.250f;
const fp32 legLength3 = 0.250f;
const fp32 legLength4 = 0.150f;
const fp32 legLength5 = 0.108f;
#endif

void LegClass_t_Init(LegClass_t *leg)
{
	leg->F_set = 0;
	leg->dis.now = 0;
	leg->dis.last = 0;
	leg->dis.dot = 0;
	leg->dis.last = 0;

	leg->xc = 0;
#if (MODEL_ORIG_RM_CAP == 0)
	leg->yc = 0.28817;
#elif ((MODEL_ORIG_RM_CAP == 1) || (MODEL_ORIG_RM_CAP == 2))
	leg->yc = 0.344051;
#endif
	leg->angle0.now = 0;
	leg->angle1 = PI / 3 * 2;
	leg->angle4 = PI / 3;
	LegClass_t_ForwardKinematics(leg, 0);
	leg->L0.set = 0.260; // Initial value
	leg->L0.last = leg->L0.set;

	memset(leg->X, 0, sizeof(leg->X));

#if (MODEL_ORIG_RM_CAP == 0)
	const fp32 K_init[2][6] = {{-55.6021, -13.3377, -9.7707, -11.4781, 15.0562, 0.7386},
	                            {19.9343, 5.5433, 4.2585, 4.9211, 138.1783, 4.1289}};
#elif ((MODEL_ORIG_RM_CAP == 1) || (MODEL_ORIG_RM_CAP == 2))
	const fp32 K_init[2][6] = {{-50.9107, -13.5268, -9.7089, -11.2951, 16.9369, 0.5469},
	                            {22.5774, 6.5316, 4.7905, 5.5240, 137.3046, 3.0263}};
#endif
	memcpy(leg->K, K_init, sizeof(leg->X));

	// avoid saturation here to maintain scale in motor torques
	// Kd must be big to maintain height!
	const static fp32 supportF_pid_param[3] = {1000, 20, 500};
	PID_init(&(leg->supportF_pid), PID_POSITION, supportF_pid_param, 99999, 10, 0.9f, &filter_err_handler);

	const fp32 supportFInAir_pid_param[3] = {500, 10, 250};
	PID_init(&(leg->supportFInAir_pid), PID_POSITION, supportFInAir_pid_param, 99999, 10, 0.9f, &filter_err_handler);

// 	const fp32 supportFCharge_pid_param[3] = {1000, 20, 500};
// 	PID_init(&(leg->supportFCharge_pid), PID_POSITION, supportFCharge_pid_param, 99999, 10, 0.9f, &filter_err_handler);
}

void LegClass_t_InvKinematics(LegClass_t *leg, const fp32 xc, const fp32 yc)
{
	leg->xc = xc;
	leg->yc = yc;

	fp32 m, n, b, x1, y1;
	fp32 A, B, C;

	A = 2 * legLength1 * yc;
	B = 2 * legLength1 * (xc + legLength5 / 2);
	C = legLength2 * legLength2 - legLength1 * legLength1 - xc * xc - yc * yc - legLength5 * legLength5 / 4 + xc * legLength5;
	leg->angle1 = 2 * atan2f((A + sqrt(A * A + B * B - C * C)), (B - C));

	m = legLength1 * cos(leg->angle1);
	n = legLength1 * sin(leg->angle1);
	b = 0;
	x1 = ((xc - m) * cos(b) - (yc - n) * sin(b)) + m;
	y1 = ((xc - m) * sin(b) + (yc - n) * cos(b)) + n;
	A = 2 * y1 * legLength4;
	B = 2 * legLength4 * (x1 - legLength5 / 2);
	C = legLength3 * legLength3 + legLength5 * x1 - legLength4 * legLength4 - legLength5 * legLength5 / 4 - x1 * x1 - y1 * y1;
	leg->angle4 = 2 * atan2f((A - sqrt(A * A + B * B - C * C)), (B - C));
}

void LegClass_t_ForwardKinematics(LegClass_t *leg, const fp32 pitch)
{
	leg->xb = legLength1 * cos(leg->angle1) - legLength5 / 2;
	leg->yb = legLength1 * sin(leg->angle1);
	leg->xd = legLength5 / 2 + legLength4 * cos(leg->angle4);
	leg->yd = legLength4 * sin(leg->angle4);
	fp32 lbd = sqrt(pow((leg->xd - leg->xb), 2) + pow((leg->yd - leg->yb), 2));
	fp32 A0 = 2 * legLength2 * (leg->xd - leg->xb);
	fp32 B0 = 2 * legLength2 * (leg->yd - leg->yb);
	fp32 C0 = pow(legLength2, 2) + pow(lbd, 2) - pow(legLength3, 2);
	fp32 D0 = pow(legLength3, 2) + pow(lbd, 2) - pow(legLength2, 2);
	leg->angle2 = 2 * atan2f((B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))), (A0 + C0));
	leg->angle3 = PI - 2 * atan2f((B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(D0, 2))), (A0 + D0));
	leg->xc = leg->xb + legLength2 * cos(leg->angle2);
	leg->yc = leg->yb + legLength2 * sin(leg->angle2);

	leg->L0.now = sqrt(pow(leg->xc, 2) + pow(leg->yc, 2));
	// Multiply by the rotation matrix of pitch
	fp32 matrix_R[2][2];
	fp32 cor_XY[2][1];
	fp32 cor_XY_then[2][1];
	matrix_R[0][0] = cos(pitch);
	matrix_R[0][1] = -sin(pitch);
	matrix_R[1][0] = sin(pitch);
	matrix_R[1][1] = cos(pitch);
	cor_XY[0][0] = leg->xc;
	cor_XY[1][0] = leg->yc;
	matrixMultiplication(2, 2, 2, 1, matrix_R, cor_XY, cor_XY_then);
	leg->angle0.last = leg->angle0.now;
	// zero angle corresponds to center position
	leg->angle0.now = atan2f(cor_XY_then[0][0], cor_XY_then[1][0]);
}

void LegClass_t_VMC(LegClass_t *leg, const fp32 F, const fp32 Tp, fp32 ActualF[2][1])
{
	fp32 Trans[2][2];
	fp32 VirtualF[2][1];

	Trans[0][0] = legLength1 * cos(leg->angle0.now + leg->angle3) * sin(leg->angle1 - leg->angle2) / sin(leg->angle2 - leg->angle3);
	Trans[0][1] = legLength1 * sin(leg->angle0.now + leg->angle3) * sin(leg->angle1 - leg->angle2) / (leg->L0.now * sin(leg->angle2 - leg->angle3));
	Trans[1][0] = legLength4 * cos(leg->angle0.now + leg->angle2) * sin(leg->angle3 - leg->angle4) / sin(leg->angle2 - leg->angle3);
	Trans[1][1] = legLength4 * sin(leg->angle0.now + leg->angle2) * sin(leg->angle3 - leg->angle4) / (leg->L0.now * sin(leg->angle2 - leg->angle3));

	VirtualF[0][0] = F;
	VirtualF[1][0] = Tp;

	matrixMultiplication(2, 2, 2, 1, Trans, VirtualF, ActualF);
}

void LegClass_t_Inv_VMC(LegClass_t *leg, const fp32 TL, const fp32 TR, fp32 VirtualF[2][1])
{
	fp32 TransInv[2][2];
	fp32 ActualF[2][1];

	TransInv[0][0] = sin(leg->angle0.now + leg->angle2) / (legLength1 * sin(leg->angle1 - leg->angle2));
	TransInv[0][1] = -sin(leg->angle0.now + leg->angle3) / (legLength4 * sin(leg->angle3 - leg->angle4));
	TransInv[1][0] = -(leg->L0.now * cos(leg->angle0.now + leg->angle2)) / (legLength1 * sin(leg->angle1 - leg->angle2));
	TransInv[1][1] = (leg->L0.now * cos(leg->angle0.now + leg->angle3)) / (legLength4 * sin(leg->angle3 - leg->angle4));

	ActualF[0][0] = TL;
	ActualF[1][0] = TR;

	matrixMultiplication(2, 2, 2, 1, TransInv, ActualF, VirtualF);
}
