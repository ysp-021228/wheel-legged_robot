//
// Created by xhuanc on 2021/11/14.
//

#ifndef DEMO1_AHRS_H
#define DEMO1_AHRS_H


#include "AHRS_MiddleWare.h"



#define sampleFreq	1000.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

 /**
  * @brief          根据加速度的数据，磁力计的数据进行四元数初始化
  * @param[in]      需要初始化的四元数数组
  * @param[in]      用于初始化的加速度计,(x,y,z)不为空 单位 m/s2
  * @param[in]      用于初始化的磁力计计,(x,y,z)不为空 单位 uT
  * @retval         返回空
  */
extern void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3]);

/**
  * @brief          根据陀螺仪的数据，加速度的数据，磁力计的数据进行四元数更新
  * @param[in]      需要更新的四元数数组
  * @param[in]      更新定时时间，固定定时调用，例如1000Hz，传入的数据为0.001f,
  * @param[in]      用于更新的陀螺仪数据,数组顺序(x,y,z) 单位 rad
  * @param[in]      用于初始化的加速度数据,数组顺序(x,y,z) 单位 m/s2
  * @param[in]      用于初始化的磁力计数据,数组顺序(x,y,z) 单位 uT
  * @retval         1:更新成功, 0:更新失败
  */
extern bool_t AHRS_update(fp32 quat[4],  fp32 timing_time,  fp32 gyro[3],  fp32 accel[3],  fp32 mag[3]);

/**
  * @brief          根据四元数大小计算对应的欧拉角偏航yaw
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的偏航角yaw 单位 rad
  */
extern fp32 get_yaw(const fp32 quat[4]);

/**
  * @brief          根据四元数大小计算对应的欧拉角俯仰角 pitch
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的俯仰角 pitch 单位 rad
  */
extern fp32 get_pitch(const fp32 quat[4]);
/**
  * @brief          根据四元数大小计算对应的欧拉角横滚角 roll
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的横滚角 roll 单位 rad
  */
extern fp32 get_roll(const fp32 quat[4]);

/**
  * @brief          根据四元数大小计算对应的欧拉角yaw，pitch，roll
  * @param[in]      四元数数组，不为NULL
  * @param[in]      返回的偏航角yaw 单位 rad
  * @param[in]      返回的俯仰角pitch  单位 rad
  * @param[in]      返回的横滚角roll 单位 rad
  */
extern void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
/**
  * @brief          返回当前的重力加速度
  * @param[in]      空
  * @retval         返回重力加速度 单位 m/s2
  */
extern fp32 get_carrier_gravity(void);


extern void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) ;
extern void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az) ;
#endif

