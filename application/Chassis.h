#ifndef DEMO1_CHASSIS_H
#define DEMO1_CHASSIS_H

/*include*/
#include "struct_typedef.h"
#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "user_lib.h"
#include "CyberGear.h"

/*define*/
#define resume(handle)         if(osThreadGetState(handle)==osThreadSuspended)\
                                {                                               \
                                osThreadResume(handle);                          \
                                }                                                \

#define CHASSIS_TASK_INIT_TIME 50

#define CHASSIS_X_CHANNEL 1

#define CHASSIS_Z_CHANNEL 2

#define LF_MOTOR_ID (0x66)
#define LB_MOTOR_ID (0x67)
#define RB_MOTOR_ID (0x68)
#define RF_MOTOR_ID (0x69)

/**********************  平衡底盘  *************************/

#define CHASSIS_PERIOD 10 // 单位为ms 底盘任务运行周期
#define MAX_CHASSIS_VX_SPEED 1.f

#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_VW 0.01f    //MAX_CHASSIS_VR_SPEED / RC_MAX_VALUE

/*******************************************************************************
 *                         Robot physical parameters                           *
 *******************************************************************************/
#define M3508_MAX_RPM 710
#define BALANCE_WHEEL_R 0.05 //平衡兵轮子半径m
#define BALANCE_REDUCTION_RATIO (1.0f/19) //平衡兵电机减速比
#define BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED  ((2 * PI  * BALANCE_WHEEL_R*BALANCE_REDUCTION_RATIO) / 60.0f) //平衡兵电机rpm转为轮子的转速m/s

#define L1 0.11f
#define L2 0.18f
#define L3 0.18f
#define L4 0.11f
#define L5 0.09f

#define CHASSIS_ROTATION_RADIUS 0.1577f
#define BODY_WEIGHT 2.839f
#define WHEEL_WEIGHT 0.183f

#define MECHANICAL_LEG_LIMIT_ANGLE 0.10472f

#define GRAVITY_A 9.8f

/*******************************************************************************
 *                                PID parameters                               *
 *******************************************************************************/
#define CHASSIS_LEG_LO_PID_P 500
#define CHASSIS_LEG_L0_PID_I 0
#define CHASSIS_LEG_L0_PID_D 100
#define CHASSIS_LEG_L0_PID_IOUT_LIMIT 2
#define CHASSIS_LEG_L0_PID_OUT_LIMIT 100

#define CHASSIS_VW_PID_P -1000
#define CHASSIS_VW_PID_I 0
#define CHASSIS_VW_PID_D 0
#define CHASSIS_VW_PID_IOUT_LIMIT 0
#define CHASSIS_VW_PID_OUT_LIMIT 2

//枚举 结构体
enum ChassisMode {
  CHASSIS_DISABLE,
  CHASSIS_UNENABLED_LEG,
  CHASSIS_ENABLED_LEG,
  CHASSIS_OFF_GROUND,
  CHASSIS_MODE_NUM,
};

enum LegIndex {
  R = 1,
  L = 0,
};

struct IMUSetPoint {
  fp32 pitch;
  fp32 yaw;
  fp32 roll;
};

struct IMUReference {
  //Triaxial Angle
  fp32 pitch_angle;
  fp32 yaw_angle;
  fp32 roll_angle;

  //Triaxial angular velocity
  fp32 pitch_gyro;
  fp32 yaw_gyro;
  fp32 roll_gyro;

  //Triaxial acceleration
  fp32 ax;
  fp32 ay;
  fp32 az;

  //The triaxial acceleration removes the gravitational acceleration
  fp32 ax_filtered;
  fp32 ay_filtered;
  fp32 az_filtered;

  fp32 robot_az;
};

struct ChassisMoveSpeedSetPoint {
  fp32 vx;
  fp32 vw;
};

struct ChassisMoveSpeedReference {
  fp32 vx;
  fp32 vw;
};

struct StateVariable {
  fp32 theta;
  fp32 theta_last;
  fp32 theta_dot;
  fp32 theta_dot_last;
  fp32 theta_ddot;
  fp32 x;
  fp32 x_dot;
  fp32 phi;
  fp32 phi_dot;

};

/*******************************************************************************
 *                            ForwardKinematics                                *
 *******************************************************************************/
struct FKL0 {
  fp32 L0;
  fp32 L0_last;
  fp32 L0_dot;
  fp32 L0_dot_last;
  fp32 L0_ddot;
};

struct FKPhi {//The phi Angle in the five-link
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

struct VMC {
  struct ForwardKinematics forward_kinematics;
  union {
    fp32 array[2][1];
    struct {
      fp32 w1_fdb;
      fp32 w4_fdb;
    } E;
  } W_fdb;

  union {
    fp32 array[2][1];
    struct {
      fp32 w0_fdb;
      fp32 vy_fdb;
    } E;
  } V_fdb;

  union {
    fp32 array[2][1];
    struct {
      fp32 T1_fdb;
      fp32 T4_fdb;
    } E;
  } T1_T4_fdb;

  union {
    fp32 array[2][1];
    struct {
      fp32 T1_set_point;
      fp32 T4_set_point;
    } E;
  } T1_T4_set_point;

  union {
    fp32 array[2][1];
    struct {
      fp32 Tp_fdb;
      fp32 Fy_fdb;
    } E;
  } Fxy_fdb;

  union {
    fp32 array[2][2];
    struct {
      fp32 Tp_set_point;
      fp32 Fy_set_point;
    } E;
  } Fxy_set_point;

  union {
    fp32 array[2][2];
    struct {
      fp32 x1_1;
      fp32 x1_2;
      fp32 x2_1;
      fp32 x2_2;
    } E;
  } J_w_to_v;

  union {
    fp32 array[2][2];
    struct {
      fp32 x1_1;
      fp32 x1_2;
      fp32 x2_1;
      fp32 x2_2;
    } E;
  } J_F_to_T;

  union {
    fp32 array[2][2];
    struct {
      fp32 x1_1;
      fp32 x1_2;
      fp32 x2_1;
      fp32 x2_2;
    } E;
  } J_T_to_F;
};

/*******************************************************************************
 *                                   Robot                                     *
 *******************************************************************************/
struct Wheel {
  fp32 speed;
  fp32 mileage;
  fp32 torque;
  struct IMUReference imu_reference;
  struct Motor3508 motor_3508;
};

struct Leg {
  enum LegIndex leg_index;
  struct StateVariable state_variable_reference;
  struct StateVariable state_variable_set_point;
  struct StateVariable state_variable_error;
  struct Wheel wheel;
  struct CyberGearData cyber_gear_data[3];
  struct VMC vmc;
  pid_t pid;
  fp32 L0_set_point;
  fp32 Fn;
};

struct Chassis {
  enum ChassisMode mode;
  enum ChassisMode last_mode;

  struct IMUSetPoint imu_set_point;
  struct IMUReference imu_reference;

  struct Leg leg_L;
  struct Leg leg_R;

  pid_t chassis_vw_pid;

  struct ChassisMoveSpeedSetPoint chassis_move_speed_set_point;
  struct ChassisMoveSpeedReference chassis_move_speed_reference;

  fp32 mileage;
};

/*******************************************************************************
 *                                  Funtion                                    *
 *******************************************************************************/
extern void chassis_task(void const *pvParameters);
struct Chassis get_chassis();

#endif //DEMO1_CHASSIS_H

