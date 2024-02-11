#ifndef USER_LIB_H
#define USER_LIB_H

#include "struct_typedef.h"
#include <math.h>

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;

typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;

typedef const int32_t sc32; /*!< Read Only */
typedef const int16_t sc16; /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef const uint32_t uc32; /*!< Read Only */
typedef const uint16_t uc16; /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

// switches between original, RM, capstone models
#define MODEL_ORIG_RM_CAP 1

#define PI 3.141593f
#define PI_half 1.570796f
#define G_gravity 9.81f

#define MOVING_AVERAGE_RESET 1
#define MOVING_AVERAGE_CALC 0

#define DISABLE_DRIVE_MOTOR_POWER 0
#define DISABLE_STEER_MOTOR_POWER 0
#define DISABLE_YAW_MOTOR_POWER 1
#define DISABLE_PITCH_MOTOR_POWER 1
#define DISABLE_SHOOT_MOTOR_POWER 1

#define STATIC_ASSERT(predicate) _impl_CASSERT_LINE(predicate, __LINE__, __FILE__)
#define _impl_PASTE(a, b) a##b
#define _impl_CASSERT_LINE(predicate, line, file) \
	typedef char _impl_PASTE(assertion_failed_##file##_, line)[2 * !!(predicate)-1];

/**
  * @brief          remote control dealline solve,because the value of rocker is not zero in middle place,
  * @param          input:the raw channel value 
  * @param          output: the processed channel value
  * @param          deadline
  */
#define deadband_limit(input, output, deadline)            \
    {                                                      \
        if ((input) > (deadline) || (input) < -(deadline)) \
        {                                                  \
            (output) = (input);                            \
        }                                                  \
        else                                               \
        {                                                  \
            (output) = 0;                                  \
        }                                                  \
    }

#define MULTIPLY_MATRIX_WRAPPER(m1, m2, result, fvalid)                                  \
	do                                                                                   \
	{                                                                                    \
		uint8_t m1_rows = sizeof(m1) / sizeof(m1[0]);                                    \
		uint8_t m1_cols = sizeof(m1[0]) / sizeof(m1[0][0]);                              \
		uint8_t m2_rows = sizeof(m2) / sizeof(m2[0]);                                    \
		uint8_t m2_cols = sizeof(m2[0]) / sizeof(m2[0][0]);                              \
		uint8_t result_rows = sizeof(result) / sizeof(result[0]);                        \
		uint8_t result_cols = sizeof(result[0]) / sizeof(result[0][0]);                  \
		fvalid = 0;                                                                      \
		if ((result_rows = m1_rows) && (result_cols = m2_cols))                          \
		{                                                                                \
			fvalid = matrixMultiplication(m1_rows, m1_cols, m2_rows, m2_cols, m1, m2, result); \
		}                                                                                \
	} while (0)

/* some simple math function */
#define ABS(x) ((x) > 0 ? (x) : -(x))                                                         // 绝对值；括号x的括号不能丢
#define SIGN(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))                                            // 获取数据正负号
#define SQUARE(x) ((x) * (x))                                                                 // 平方
#define HYPOT(x, y) (sqrt((x) * (x) + (y) * (y)))                                             // 斜边
#define BELONG(x, min, max) ((x) >= (MIN(min, max)) && (x) <= (MAX(min, max)) ? 1 : 0) // if x in [min, max] return 1
#define MAX(x, y) (((x) >= (y)) ? (x) : (y))                                                  // 两个数的最大值
#define MIN(x, y) (((x) >= (y)) ? (y) : (x))                                                  // 两个数的最小值
#define rad2deg(X) ((X) / PI * 180.0f)                                                         // 角度转换                                                   // 角度转换
#define deg2rad(X) ((X) / 180.0f * PI)

/* 两点间距离 */
#define CountDistance(x_from, y_from, x_to, y_to) \
    (fp32)(sqrt((x_to - x_from) * (x_to - x_from) + (y_to - y_from) * (y_to - y_from)))
/* 数量积 */
#define InnerProduct(vector1, vector2) \
    (fp32)(vector1.x * vector2.x + vector1.y * vector2.y)
#define Deadzone(input, threshold) ((fabs(input) < threshold) ? 0 : input)
//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#define CHASSIS_CONTROL_TIME_MS 10.0f
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)

// Hardware Properties
#define MOTOR_TORQUE_CLEARANCE 0.2f
#define HIP_TORQUE_BURST_MAX (20.0f - MOTOR_TORQUE_CLEARANCE)
#define HIP_TORQUE_MAX 8.0f
#define DRIVE_TORQUE_MAX 3.0f
#define UNLOADED_ROBOT_MASS 5.0f                                          // unit is kg
#define UNLOADED_ROBOT_WEIGHT (UNLOADED_ROBOT_MASS * G_gravity) // unit is N
#define UNLOADED_ROBOT_HALF_WEIGHT (UNLOADED_ROBOT_MASS / 2.0f * G_gravity) // unit is N

// The least standing requirement
// #define HIP_TORQUE_MAX 3.5f

typedef struct
{
    uint8_t size;
    uint8_t cursor;
    fp32 *ring;
    fp32 sum;
} moving_average_type_t;

uint8_t matrixMultiplication(const uint8_t m1_rows, const uint8_t m1_cols, const uint8_t m2_rows, const uint8_t m2_cols, const fp32 m1[m1_rows][m1_cols], const fp32 m2[m2_rows][m2_cols], fp32 result[m1_rows][m2_cols]);
fp32 moving_average_calc(fp32 input, moving_average_type_t* moving_average_type, uint8_t fInit);
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
fp32 first_order_filter(fp32 input, fp32 prev_output, fp32 coeff);
fp32 second_order_filter(fp32 input, fp32 output_prev1, fp32 output_prev2, fp32 coeff1, fp32 coeff2);
fp32 brakezone(fp32 input, fp32 threshold, fp32 order);
fp32 brakezone_symmetric(fp32 input, fp32 threshold, fp32 order);

#endif /* USER_LIB_H */
