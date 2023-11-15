/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of?
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.? See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date              Author           Modification
  * V1.0.0      January-15-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */




#ifndef __pid_H__
#define __pid_H__

//#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
enum
{
    LAST  = 0,
    NOW   = 1,
};

/**
  * @brief     PID 结构体
  */
typedef struct
{
    /* p、i、d参数 */
    float p;
    float i;
    float d;

    /* 目标值、反馈值、误差值 */
    float set;
    float get;
    float err[2];

    /* p、i、d各项计算出的输出 */
    float pout;
    float iout;
    float dout;

    /* pid公式计算出的总输出 */
    float out;

    /* pid公式计算出的总输出超出量 */
    float excess;

    /* pid最大输出限制  */
    float max_output;

    /* pid积分输出项限幅 */
    float integral_limit;

} pid_t;

/**
  * @brief     PID 初始化函数
  * @param[in] pid: PID 结构体
  * @param[in] max_out: 最大输出
  * @param[in] intergral_limit: 积分限幅
  * @param[in] kp/ki/kd: 具体 PID 参数
  */
extern void pid_init(pid_t *pid, float max_out, float intergral_limit, \
              float kp, float ki, float kd);

/**
  * @brief     PID 参数复位函数
  * @param[in] pid: PID 结构体
  * @param[in] kp/ki/kd: 具体 PID 参数
  */
extern void pid_reset(pid_t *pid, float kp, float ki, float kd);

/**
  * @brief     PID 计算函数，使用位置式 PID 计算
  * @param[in] pid: PID 结构体
  * @param[in] get: 反馈数据
  * @param[in] set: 目标数据
  * @retval    PID 计算输出
  */
extern float pid_calc(pid_t *pid, float get, float set);
extern float pid_calc_balance(pid_t *pid, float get, float set,float gyro_y);
extern float pid_anti_windup_calc(pid_t *pid, float get, float set);
extern float pid_loop_calc(pid_t *pid,float get,float set,float max_value,float min_value);
float pid_calc_KI_Separation(pid_t* pid,float get,float set,float err_threshold);
float pid_reset_i_calc(pid_t *pid, float get, float set, float iout_threshold);


#endif
