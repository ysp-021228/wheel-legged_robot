//
// Created by xhuanc on 2021/10/10.
//

#ifndef DEMO1_CHASSIS_H
#define DEMO1_CHASSIS_H

/*include*/
#include "struct_typedef.h"
#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "user_lib.h"

/*define*/
#define resume(handle)         if(osThreadGetState(handle)==osThreadSuspended)\
                                {                                               \
                                osThreadResume(handle);                          \
                                }                                                \

//任务开始空闲一段时间

#define CHASSIS_TASK_INIT_TIME 357

#define CHASSIS_X_CHANNEL 1

#define CHASSIS_Z_CHANNEL 2

#define CHASSIS_MODE_CHANNEL 0

//控制量系数 根据最大速度和遥控器通知区间比例来算

#define CHASSIS_VX_RC_SEN 0.006f

#define CHASSIS_VY_RC_SEN 0.005f

#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_RC_DEADLINE 10

//底盘运动过程最大前进速度 mm/s
#define NORMAL_MAX_CHASSIS_SPEED_X 2500.0f

//底盘运动过程最大平移速度  mm/s
#define NORMAL_MAX_CHASSIS_SPEED_Y 1500.0f

#define CHASSIS_WZ_SET_SCALE 0.1f

//功率控制
#define CHASSIS_POWER_LIMIT_NO_REF 40
#define CHASSIS_CURRENT_LIMIT_TOTAL 50000
#define CHASSIS_CURRENT_LIMIT_40W 25000
#define CHASSIS_POWER_BUFF 60
//#define CHASSIS_POWER_LIMIT_DEBUG

#define CONTINUES_HURT_JUDGE() (HAL_GetTick()-start_hurt_time>1000)

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 200.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f




/**********************  平衡底盘  *************************/


//#define CHASSIS_PITCH_ANGLE_PID_KP   0.4f
//#define CHASSIS_PITCH_ANGLE_PID_KI   0.02f
//#define CHASSIS_PITCH_ANGLE_PID_KD   4.0f
//#define CHASSIS_PITCH_ANGLE_MAX_OUT  2000.f
//#define CHASSIS_PITCH_ANGLE_MAX_IOUT 130.f
//
//
//#define CHASSIS_SPEED_PID_KP         1.2f
//#define CHASSIS_SPEED_PID_KI         0.25f
//#define CHASSIS_SPEED_PID_KD         0.0f
//#define CHASSIS_SPEED_PID_MAX_OUT    20.0f
//#define CHASSIS_SPEED_PID_MAX_IOUT   5.0f

#define CHASSIS_PITCH_ANGLE_PID_KP   0.02f
#define CHASSIS_PITCH_ANGLE_PID_KI   0.003f
#define CHASSIS_PITCH_ANGLE_PID_KD   0.25f
#define CHASSIS_PITCH_ANGLE_MAX_OUT  10.f
#define CHASSIS_PITCH_ANGLE_MAX_IOUT 2.f

//
//#define CHASSIS_SPEED_PID_KP         0.7f
//#define CHASSIS_SPEED_PID_KI         0.1f
//#define CHASSIS_SPEED_PID_KD         0.08f
//#define CHASSIS_SPEED_PID_MAX_OUT    30.0f
//#define CHASSIS_SPEED_PID_MAX_IOUT   5.0f




#define CHASSIS_SPEED_PID_KP         0.180f
#define CHASSIS_SPEED_PID_KI         0.0080f
#define CHASSIS_SPEED_PID_KD         0.0f
#define CHASSIS_SPEED_PID_MAX_OUT    18.0f
#define CHASSIS_SPEED_PID_MAX_IOUT   1.0f

#define CHASSIS_BACK_SPEED_PID_KP         0.0001f
#define CHASSIS_BACK_SPEED_PID_KI         0.0f
#define CHASSIS_BACK_SPEED_PID_KD         0.0f
#define CHASSIS_BACK_SPEED_PID_MAX_OUT    2.0f
#define CHASSIS_BACK_SPEED_PID_MAX_IOUT   5.0f

#define CHASSIS_BACK_PITCH_ANGLE_PID_KP   0.05f
#define CHASSIS_BACK_PITCH_ANGLE_PID_KI   0.00f
#define CHASSIS_BACK_PITCH_ANGLE_PID_KD   0.00f
#define CHASSIS_BACK_PITCH_ANGLE_MAX_OUT  1000.f
#define CHASSIS_BACK_PITCH_ANGLE_MAX_IOUT 130.f

#define CHASSIS_BREAK_PID_KP         0.05f
#define CHASSIS_BREAK_PID_KI         0.001f
#define CHASSIS_BREAK_PID_KD         0.0f
#define CHASSIS_BREAK_PID_MAX_OUT    50.0f
#define CHASSIS_BREAK_PID_MAX_IOUT   10.0f

//#define CHASSIS_3508_PID_KP     1.5f
//#define CHASSIS_3508_PID_KI     0.08f
//#define CHASSIS_3508_PID_KD     0.0f
//#define CHASSIS_3508_PID_MAX_OUT 2000.0f
//#define CHASSIS_3508_PID_MAX_IOUT 300.0f

#define CHASSIS_3508_PID_KP     2.3f
#define CHASSIS_3508_PID_KI     0.1f
#define CHASSIS_3508_PID_KD     0.0f
#define CHASSIS_3508_PID_MAX_OUT 2000.0f
#define CHASSIS_3508_PID_MAX_IOUT 300.0f

#define CHASSIS_BACK_ANGLE_PID_KP   0.08f
#define CHASSIS_BACK_ANGLE_PID_KI   0.045f
#define CHASSIS_BACK_ANGLE_PID_KD   0.05f
#define CHASSIS_BACK_ANGLE_MAX_OUT  30.f
#define CHASSIS_BACK_ANGLE_MAX_IOUT 5.f

#define CHASSIS_PERIOD 15 // 单位为ms 底盘任务运行周期
#define MAX_CHASSIS_VX_SPEED 12.f
#define MAX_CHASSIS_VW_SPEED 2500.f//150
#define MAX_CHASSIS_TILTABLE_ANGLE 25
#define MIN_CHASSIS_TILTABLE_ANGLE -25

#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_VW 0.01f    //MAX_CHASSIS_VR_SPEED / RC_MAX_VALUE

#define FAST_MAX_SPEED 3000.f //快速模式下底盘的最大速度
#define NORMAL_MAX_SPEED MAX_CHASSIS_VX_SPEED //通常模式下底盘的最大速度
#define SLOW_MAX_SPEED 500.f //慢速模式下底盘的最大速度
#define SPEED_CHANGE 15.f //普通模式下最大速度（2500 mm/s）?期望加到最大速度的时间（1000ms)*底盘任务的运行周期(CHASSIS_PERIOD 4ms)
#define ROTATION_SPEED_CHANGE 10.f
#define GYRO_TO_RPM 94.2477796076937f //  °/min

//底盘机械信息 mm
#define M3508_MAX_RPM 710
#define BALANCE_WHEEL_R 0.11 //平衡兵轮子半径
#define BALANCE_TRACK 863.938f //平衡兵轮子周长
#define BALANCE_REDUCTION_RATIO 1.0f //平衡兵电机减速比
#define BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED  (PI*BALANCE_WHEEL_R/180.0f) //平衡兵电机rpm转为轮子的转速m/s
//枚举 结构体

#define push_angle_kp   0.2f
#define push_angle_ki   0.00f
#define push_angle_kd   0.0f
#define push_angle_MAX_OUT  15000.0f //5300.0f
#define push_angle_MAX_IOUT 1600.0f // 16000.0f

#define push_speed_kp   1.5f
#define push_speed_ki   0.0f
#define push_speed_kd   0.0f
#define push_speed_MAX_OUT  15000.0f //9500.0f
#define push_speed_MAX_IOUT 1600.0f //16000.0f

typedef enum {
  CHASSIS_RELAX, //底盘失能 注意底盘失能和底盘刹车的区别
  CHASSIS_BACK,
  CHASSIS_ONLY,
  CHASSIS_BLOCK,//底盘刹车
  CHASSIS_FOLLOW_GIMBAL,
  CHASSIS_SPIN,
  CHASSIS_INDEPENDENT_CONTROL,
  CHASSIS_OFF_GROUND,
} chassis_mode_e;

typedef enum {
  RF = 1,
  LF = 0,
  LB,
  RB
} chassis_motor_index_e;

struct Chassis{
  chassis_mode_e mode;

  chassis_mode_e last_mode;
  motor_3508_t motor_chassis[2];
  motor_6020_t motor_steer[4];

  motor_6020_t motor_push;
  motor_6020_t motor_left;

  fp32 absolute_angle_get;
  fp32 absolute_angle_set;
  fp32 yaw_relative_angle_get;
  fp32 balanced_angle;
  fp32 gyro_set;

  pid_t chassis_vw_pid;
  pid_t angle_p;
  pid_t chassis_speed_p;
  pid_t chassis_break_p;
  pid_t chassis_back_speed_p;
  pid_t chassis_back_angle_p;

  fp32 vx;
  fp32 vy;
  fp32 vw;
  fp32 run;

  fp32 vx_pc;
  fp32 vy_pc;
  fp32 vw_pc;

  bool_t chassis_is_back;

};

//变量

//函数声明
extern void chassis_task(void const *pvParameters);

//
#endif //DEMO1_CHASSIS_H

