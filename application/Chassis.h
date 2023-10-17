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
  CHASSIS_DISABLE,
  CHASSIS_ENABLE,
  CHASSIS_OFF_GROUND,
  CHASSIS_MODE_NUM,
};

enum ChassisMotorIndex {
  RF = 1,
  LF = 0,
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

struct MoveSpeedSetPoint {
  fp32 vx;
  fp32 vw;
};

struct StateVariable{
  fp32 theta;
  fp32 theta_dot;
  fp32 x;
  fp32 x_dot;
  fp32 phi;
  fp32 phi_dot;

  fp32 theta_ddot;
};

/*******************************************************************************
 *                            ForwardKinematics                                *
 *******************************************************************************/
struct FKL0 {
  fp32 L0;
  fp32 L0_last;
  fp32 L0_dot;
  fp32 L0_ddot;
};

struct FKPhi {
  fp32 phi0;
  fp32 phi1;
  fp32 phi2;
  fp32 phi3;
  fp32 phi4;
};

struct FKPointCoordinates {
  fp32 a_x, a_y;
  fp32 b_x, b_y;
  fp32 c_x, c_y;
  fp32 d_x, d_y;
  fp32 e_x, e_y;
};

struct ForwardKinematics {
  struct FKL0 fk_L0;
  struct FKPhi fk_phi;
  struct FKPointCoordinates fk_point_coordinates;
};

/*******************************************************************************
 *                           Inverse Kinematics                                *
 *******************************************************************************/
struct InverseKinematics {
  fp32 Jocbian[2][2];
  fp32 Jocbian_Inv[2][2];

  fp32 R[2][2];
  fp32 M[2][2];
  fp32 F[2][1];
  fp32 T[2][1];

  fp32 F_feedback[2][1];
  fp32 T_feedback[2][1];

  fp32 Fn;
};

/*******************************************************************************
 *                                  Chassis                                    *
 *******************************************************************************/
struct Chassis {
  enum ChassisMode mode;
  enum ChassisMode last_mode;

  struct StateVariable state_variable_leg_L;
  struct StateVariable state_variable_leg_R;

  struct Motor3508 motor_chassis[2];

  struct IMUSetPoint imu_set_point;
  struct IMUReference imu_reference;

  struct ForwardKinematics forward_kinematics_leg_L;
  struct InverseKinematics inverse_kinematics_leg_L;

  struct ForwardKinematics forward_kinematics_leg_R;
  struct InverseKinematics inverse_kinematics_leg_R;

  pid_t chassis_vw_pid;

  struct MoveSpeedSetPoint move_speed_set_point;

  fp32 mileage;
};

/*******************************************************************************
 *                                  Funtion                                    *
 *******************************************************************************/
extern void chassis_task(void const *pvParameters);
struct Chassis get_chassis();

#endif //DEMO1_CHASSIS_H

