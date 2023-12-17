#ifndef DEMO1_CHASSIS_H
#define DEMO1_CHASSIS_H

/*include*/
#include "struct_typedef.h"
#include "IO/can_receive.h"
#include "Controller/pid.h"
#include "IO/remote.h"
#include "user_lib.h"
#include "CyberGear.h"

/*define*/
#define resume(handle)         if(osThreadGetState(handle)==osThreadSuspended)\
                                {                                               \
                                osThreadResume(handle);                          \
                                }                                                \

/*******************************************************************************
 *                                 Task Time                                   *
 *******************************************************************************/
#define CHASSIS_TASK_INIT_TIME 50
#define CHASSIS_PERIOD 5 // 单位为ms 底盘任务运行周期
#define MILLISECOND_TO_SECOND 0.001f

/*******************************************************************************
 *                                  Motor ID                                   *
 *******************************************************************************/
#define LF_MOTOR_ID (0x66)
#define LB_MOTOR_ID (0x67)
#define RB_MOTOR_ID (0x68)
#define RF_MOTOR_ID (0x69)

/*******************************************************************************
 *                                    Limit                                    *
 *******************************************************************************/
#define MAX_CHASSIS_VX_SPEED 1.5f
#define MAX_CHASSIS_VW_TORQUE 0.5f
#define MAX_CHASSIS_YAW_INCREMENT 0.02f
#define MIN_L0 0.08f
#define MAX_L0 0.25f
#define DEFAULT_L0 0.15f
#define MAX_PITCH 0.35f
#define MIN_PITCH -0.35f
#define MAX_ROLL 0.17f
#define MIN_ROLL -0.17f
#define MAX_WHEEL_TORQUE 5.f
#define MIN_WHEEL_TORQUE -5.f
#define MAX_JOINT_TORQUE 7.f
#define MIN_JOINT_TORQUE -7.f

#define WHEEL_THETA_LIMIT      1.5f
#define WHEEL_THETA_DOT_LIMIT  0.5f
#define WHEEL_X_LIMIT          0.f
#define WHEEL_X_DOT_LIMIT      1.0f
#define WHEEL_PHI_LIMIT        1.0f
#define WHEEL_PHI_DOT_LIMIT    0.5f

#define JOINT_THETA_LIMIT      15.5f
#define JOINT_THETA_DOT_LIMIT  20.5f
#define JOINT_X_LIMIT          0.f
#define JOINT_X_DOT_LIMIT      10.0f
#define JOINT_PHI_LIMIT        10.5f
#define JOINT_PHI_DOT_LIMIT    10.0f

/*******************************************************************************
 *                                Remote control                               *
 *******************************************************************************/
#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Z_CHANNEL 2
#define CHASSIS_PIT_CHANNEL 3
#define CHASSIS_ROLL_CHANNEL 0

#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_VW  (MAX_CHASSIS_VW_TORQUE/660)
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)
#define RC_TO_PITCH ((MAX_PITCH-MIN_PITCH)/660)
#define RC_TO_ROLL ((MAX_ROLL-MIN_ROLL)/660)
#define RC_TO_L0  ((MAX_L0-MIN_L0)/1320)

/*******************************************************************************
 *                         Robot physical parameters                           *
 *******************************************************************************/
#define M3508_MAX_RPM 710
#define BALANCE_WHEEL_R 0.05f //平衡兵轮子半径m
#define BALANCE_REDUCTION_RATIO (1.0f/19) //平衡兵电机减速比
#define BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED  ((2 * PI  * BALANCE_WHEEL_R*BALANCE_REDUCTION_RATIO) / 60.0f) //平衡兵电机rpm转为轮子的转速m/s

#define L1 0.11f
#define L2 0.18f
#define L3 0.18f
#define L4 0.11f
#define L5 0.09f

#define CHASSIS_ROTATION_RADIUS 0.1577f
#define BODY_WEIGHT 3.046f
#define WHEEL_WEIGHT 0.183f
#define MECHANICAL_LEG_LIMIT_ANGLE 0.10472f

#define GRAVITY_A 9.8f

/*******************************************************************************
 *                                PID parameters                               *
 *******************************************************************************/
#define CHASSIS_LEG_LO_PID_P 250
#define CHASSIS_LEG_L0_PID_I 0.2
#define CHASSIS_LEG_L0_PID_D 5000
#define CHASSIS_LEG_L0_PID_IOUT_LIMIT 3
#define CHASSIS_LEG_L0_PID_OUT_LIMIT 100000

#define CHASSIS_OFFGROUND_LEG_LO_PID_P 250
#define CHASSIS_OFFGROUND_LEG_L0_PID_I 0
#define CHASSIS_OFFGROUND_LEG_L0_PID_D 00000
#define CHASSIS_OFFGROUND_LEG_L0_PID_IOUT_LIMIT 3
#define CHASSIS_OFFGROUND_LEG_L0_PID_OUT_LIMIT 10000

#define CHASSIS_VW_PID_P -3
#define CHASSIS_VW_PID_I -0.01
#define CHASSIS_VW_PID_D -200
#define CHASSIS_VW_PID_IOUT_LIMIT 0.2
#define CHASSIS_VW_PID_OUT_LIMIT 0.5

//#define CHASSIS_VW_PID_P 0.0
//#define CHASSIS_VW_PID_I 0
//#define CHASSIS_VW_PID_D -0
//#define CHASSIS_VW_PID_IOUT_LIMIT 0.2
//#define CHASSIS_VW_PID_OUT_LIMIT 0.5

#define CHASSIS_ROLL_PID_P 0.2
#define CHASSIS_ROLL_PID_I 0.0015
#define CHASSIS_ROLL_PID_D 2
#define CHASSIS_ROLL_PID_IOUT_LIMIT 0.05
#define CHASSIS_ROLL_PID_OUT_LIMIT 0.2

#define CHASSIS_LEG_COORDINATION_PID_P 2
#define CHASSIS_LEG_COORDINATION_PID_I 0.0
#define CHASSIS_LEG_COORDINATION_PID_D 0
#define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 2
#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 5

enum ChassisMode {
  CHASSIS_INIT,
  CHASSIS_DISABLE,
  CHASSIS_ENABLED_LEG,
  CHASSIS_JUMP,
  CHASSIS_MODE_NUM,
};

enum JumpState{
  NOT_READY,
  READY,
  STRETCHING,
  SHRINKING,
  STRETCHING_AGAIN,
  FALLING,
  LANDING,
};

enum LegIndex {
  R = 1,
  L = 0,
};

struct LegFlag{
  bool_t OFF_GROUND_FLAG;
  bool_t IMPACT_FLAG;
};

struct JumpFlag{
  bool_t jump_completed;
  bool_t offground;
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
  fp32 x_dot_last;
  fp32 x_ddot;
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
  struct LegFlag leg_flag;
  struct StateVariable state_variable_reference;
  struct StateVariable state_variable_set_point;
  struct StateVariable state_variable_error;
  struct StateVariable state_variable_wheel_out;
  struct StateVariable state_variable_joint_out;
  struct Wheel wheel;
  struct CyberGearData cyber_gear_data[3];
  struct VMC vmc;
  pid_t ground_pid;
  pid_t offground_pid;
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
  pid_t chassis_roll_pid;
  pid_t chassis_leg_coordination_pid;

  struct ChassisMoveSpeedSetPoint chassis_move_speed_set_point;
  struct ChassisMoveSpeedReference chassis_move_speed_reference;

  fp32 mileage;
  fp32 L0_delta;
  fp32 phi_0_error;
  fp32 steer_compensatory_torque;

  struct JumpFlag jump_flag;
  enum JumpState jump_state;
  bool_t  init_flag;
};

/*******************************************************************************
 *                                  Funtion                                    *
 *******************************************************************************/
extern void chassis_task(void const *pvParameters);
struct Chassis get_chassis();

#endif //DEMO1_CHASSIS_H

