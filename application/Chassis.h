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

/**********************  平衡底盘  *************************/

#define CHASSIS_PERIOD 15 // 单位为ms 底盘任务运行周期
#define MAX_CHASSIS_VX_SPEED 12.f

#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_VW 0.01f    //MAX_CHASSIS_VR_SPEED / RC_MAX_VALUE

//底盘机械信息 mm
#define M3508_MAX_RPM 710
#define BALANCE_WHEEL_R 0.11 //平衡兵轮子半径
#define BALANCE_TRACK 863.938f //平衡兵轮子周长
#define BALANCE_REDUCTION_RATIO 1.0f //平衡兵电机减速比
#define BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED  (PI*BALANCE_WHEEL_R/180.0f) //平衡兵电机rpm转为轮子的转速m/s

//枚举 结构体
enum ChassisMode {
  CHASSIS_RELAX, //底盘失能 注意底盘失能和底盘刹车的区别
  CHASSIS_BACK,
  CHASSIS_ONLY,
  CHASSIS_BLOCK,//底盘刹车
  CHASSIS_FOLLOW_GIMBAL,
  CHASSIS_SPIN,
  CHASSIS_INDEPENDENT_CONTROL,
  CHASSIS_OFF_GROUND,
};

enum ChassisMotorIndex {
  RF = 1,
  LF = 0,
  LB,
  RB
};

struct IMUSetPoint {
  fp32 pitch;
  fp32 yaw;
  fp32 roll;
};

struct IMUReference {
  fp32 pitch;
  fp32 yaw;
  fp32 roll;
};

struct Chassis {
  enum ChassisMode mode;
  enum ChassisMode last_mode;
  motor_3508_t motor_chassis[2];

  struct IMUSetPoint imu_set_point;
  struct IMUReference imu_reference;

  pid_t chassis_vw_pid;

  fp32 vx;
  fp32 vy;
  fp32 vw;
};

//函数声明
extern void chassis_task(void const *pvParameters);
struct Chassis get_chassis();

#endif //DEMO1_CHASSIS_H

