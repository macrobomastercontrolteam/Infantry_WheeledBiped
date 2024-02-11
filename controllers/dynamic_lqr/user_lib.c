#include "math.h"
#include "user_lib.h"

fp32 first_order_filter(fp32 input, fp32 output_prev, fp32 coeff)
{
	if (coeff > 1)
	{
		return NAN;
	}
	fp32 output = (1 - coeff) * output_prev + coeff * input;
	return output;
}

fp32 second_order_filter(fp32 input, fp32 output_prev1, fp32 output_prev2, fp32 coeff1, fp32 coeff2)
{
	if (coeff1 + coeff2 >= 1)
	{
		return NAN;
	}
	fp32 output = coeff1 * input + coeff2 * output_prev1 + (1 - coeff1 - coeff2) * output_prev2;
	return output;
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

fp32 brakezone(fp32 input, fp32 threshold, fp32 order)
{
	// tuning techniques:
	// adjustment step: threshold, then order
	// threshold: affects oscillation amplitude and curvature around 0
	fp32 input_abs = fabs(input);
	if ((threshold <= 0) || (order <= 0))
	{
		// should not reach here
		// disabled case
		return input;
	}
	else if (input_abs >= threshold)
	{
		return input;
	}
	else
	{
		return input * pow(input_abs / threshold, order);
	}
}

fp32 brakezone_symmetric(fp32 input, fp32 threshold, fp32 order)
{
	fp32 output = input;
	if ((threshold <= 0) || (order <= 0))
	{
		// should not reach here
		// disabled case
	}
	else
	{
		fp32 input_abs = fabs(input);
		if (input_abs < threshold)
		{
			output = input * pow(input_abs / threshold, order);
		}
		else
		{
			fp32 twice_threshold = 2.0f * threshold;
			if (input_abs <= twice_threshold)
			{
				fp32 x_abs_minus_2s = input_abs - twice_threshold;
				output = SIGN(input) * (x_abs_minus_2s * pow(fabs(x_abs_minus_2s) / threshold, order) + twice_threshold);
			}
			else
			{
				// bypass
			}
		}
	}
	return output;
}
