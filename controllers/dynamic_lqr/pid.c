/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include <stddef.h>
#include "user_lib.h"

void LimitMax(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout, fp32 filter_coeff, fp32 (*err_handler)(fp32 set, fp32 ref, fp32 err[3], fp32 filter_coeff))
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->err_handler = err_handler;
    pid->filter_coeff = filter_coeff;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set, fp32 dt)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = pid->err_handler(set, ref, pid->error, pid->filter_coeff);
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];

        pid->Iout += pid->Ki * pid->error[0] * dt;
        // pid->Iout = Deadzone(pid->Iout, 0.1f);
        LimitMax(&pid->Iout, pid->max_iout);

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]) / dt;
        // Coefficient 0.75-0.8 is the best for CHASSIS_CONTROL_TIME_MS = 10
        // for CHASSIS_CONTROL_TIME_MS = 5, don't use filter here
        pid->Dbuf[0] = first_order_filter(pid->Dbuf[0], pid->Dbuf[1], 0.8f);
        pid->Dout = pid->Kd * pid->Dbuf[0];

        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(&pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);

        pid->Iout = pid->Ki * pid->error[0] * dt;
        // pid->Iout = Deadzone(pid->Iout, 0.1f);
        LimitMax(&pid->Iout, pid->max_iout);

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]) / dt;
        pid->Dout = pid->Kd * pid->Dbuf[0];

        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(&pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

fp32 raw_err_handler(fp32 set, fp32 ref, fp32 err[3], fp32 filter_coeff)
{
  return set - ref;
}

fp32 rad_err_handler(fp32 set, fp32 ref, fp32 err[3], fp32 filter_coeff)
{
  return rad_format(set - ref);
}

fp32 filter_err_handler(fp32 set, fp32 ref, fp32 err[3], fp32 filter_coeff)
{
  fp32 new_err = set - ref;
  fp32 output = first_order_filter(new_err, err[1], filter_coeff);
  return output;
}

fp32 filter_rad_err_handler(fp32 set, fp32 ref, fp32 err[3], fp32 filter_coeff)
{
  fp32 new_err = rad_format(set - ref);
  fp32 output = first_order_filter(new_err, err[1], filter_coeff);
  return output;
}