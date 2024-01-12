#include "user_lib.h"
#include "math.h"

/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

/**
  * @brief          Moving average
  * @author         2022 MacFalcons
  * @param[in]      input
  * @param[in]      handler
  * @retval         average
  */
fp32 moving_average_calc(fp32 input, moving_average_type_t* moving_average_type, uint8_t fInit)
{
    fp32 output;
    if (fInit == MOVING_AVERAGE_RESET)
    {
        moving_average_type->sum = input * moving_average_type->size;
        for (uint8_t i = 0; i < (moving_average_type->size); i++)
        {
            moving_average_type->ring[i] = input;
        }
        moving_average_type->cursor = 0;
        output = input;
    }
    else
    {
        // history[cursor] is the current oldest history in the ring
        moving_average_type->sum = moving_average_type->sum - moving_average_type->ring[moving_average_type->cursor] + input;
        moving_average_type->ring[moving_average_type->cursor] = input;
        moving_average_type->cursor = (moving_average_type->cursor + 1) % moving_average_type->size;
        output = (moving_average_type->sum) / ((fp32)(moving_average_type->size));
    }
    return output;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//循环限幅函数
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//角度格式化为-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

uint8_t matrixMultiplication(const uint8_t m1_rows, const uint8_t m1_cols, const uint8_t m2_rows, const uint8_t m2_cols, const float m1[m1_rows][m1_cols], const float m2[m2_rows][m2_cols], float result[m1_rows][m2_cols])
{
    uint8_t fvalid;
    fvalid = (m1_cols == m2_rows);
    fvalid &= ((m1 != result) && (m2 != result));
    if (fvalid)
    {
        // printf("Resultant Matrix is:\n");
        for (uint8_t i = 0; i < m1_rows; i++)
        {
            for (uint8_t j = 0; j < m2_cols; j++)
            {
                result[i][j] = 0;

                for (uint8_t k = 0; k < m2_rows; k++)
                {
                    result[i][j] += m1[i][k] * m2[k][j];
                }
                // printf("%f\t", result[i][j]);
            }
            // printf("\n");
        }
    }
    return fvalid;
}
