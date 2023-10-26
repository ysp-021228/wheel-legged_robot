/*include*/
#include <stdlib.h>

#include "Chassis.h"
#include "cmsis_os.h"
#include "remote.h"
#include "can_receive.h"
#include "user_lib.h"
#include "ramp.h"
#include "arm_math.h"
#include "Detection.h"
#include "Atti.h"
#include "CyberGear.h"

struct Chassis chassis;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static void chassis_init(struct Chassis *chassis);
static void chassis_ctrl_info_get();
static void chassis_relax_handle();
static void chassis_enabled_leg_handle();
static void chassis_unable_leg_handle();
static void chassis_wheel_cal(fp32 vx, fp32 vw);
static void chassis_imu_info_update();
static void chassis_relax_judge();
void chassis_device_offline_handle();
static void chassis_off_ground_detection();
static void chassis_info_update();
static void chassis_motor_info_update();
static void leg_state_variable_get(struct Leg *leg);
fp32 cal_leg_theta(fp32 phi0);
static void chassis_forward_kinematics();
static void chassis_inverse_kinematics();
static void chassis_motor_cmd_send();
static void chassis_K_matrix_fitting(fp32 L0, fp32 K[6], const fp32 KL[6][4]);

fp32 unable_leg_K[6] = {0, 0, 0, 0, 0, 0};
fp32 wheel_K[6] = {0, 0, 0, 0, 0, 0};
fp32 joint_K[6] = {0, 0, 0, 0, 0, 0};
fp32 wheel_fitting_factor[6][4] = {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
};
fp32 joint_fitting_factor[6][4] = {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
};

void chassis_task(void const *pvParameters) {

  vTaskDelay(CHASSIS_TASK_INIT_TIME);

  chassis_init(&chassis);

  while (1) {
    chassis_info_update();

    chassis_ctrl_info_get();

    chassis_relax_judge();

    chassis_device_offline_handle();

    switch (chassis.mode) {
      case CHASSIS_ENABLED_LEG: {
        chassis_enabled_leg_handle();
        cyber_gear_control_mode(&cybergears_1[LF_MOTOR_ID], -0.5, 0, 0, 0, 0);//lb
        cyber_gear_control_mode(&cybergears_1[LB_MOTOR_ID], 0.5, 0, 0, 0, 0);
        cyber_gear_control_mode(&cybergears_1[RF_MOTOR_ID], 0.5, 0, 0, 0, 0);
        cyber_gear_control_mode(&cybergears_1[RB_MOTOR_ID], -0.5, 0, 0, 0, 0);//rf
      }
        break;

      case CHASSIS_UNENABLED_LEG: {
        chassis_unable_leg_handle();
        cyber_gear_control_mode(&cybergears_1[LF_MOTOR_ID], 0.9, 0, 0, 0, 0);//lb
        cyber_gear_control_mode(&cybergears_1[LB_MOTOR_ID], -0.9, 0, 0, 0, 0);
        cyber_gear_control_mode(&cybergears_1[RF_MOTOR_ID], -0.9, 0, 0, 0, 0);
        cyber_gear_control_mode(&cybergears_1[RB_MOTOR_ID], 0.9, 0, 0, 0, 0);//rf
      }
        break;

      case CHASSIS_OFF_GROUND: {

      }
        break;

      case CHASSIS_DISABLE: {
        cyber_gear_control_mode(&cybergears_1[LF_MOTOR_ID], 0, 0, 0, 0, 0);//lb
        cyber_gear_control_mode(&cybergears_1[LB_MOTOR_ID], 0, 0, 0, 0, 0);
        cyber_gear_control_mode(&cybergears_1[RF_MOTOR_ID], 0, 0, 0, 0, 0);
        cyber_gear_control_mode(&cybergears_1[RB_MOTOR_ID], 0, 0, 0, 0, 0);//rf
        chassis_relax_handle();
        chassis_init(&chassis);
      }
        break;
    }

//    chassis_motor_cmd_send();

    vTaskDelay(CHASSIS_PERIOD);
  }
}

static void chassis_info_update() {
  chassis_imu_info_update();
  chassis_motor_info_update();
  chassis.mileage =
      (chassis.leg_L.wheel.mileage + chassis.leg_R.wheel.mileage) / 2;//The state variable x should use this value
  if (chassis.chassis_move_speed_set_point.vx != 0) {
    chassis.mileage = 0;
  }
}

static void chassis_motor_info_update() {
  chassis.leg_L.wheel.speed = motor_3508_measure[0].speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  chassis.leg_R.wheel.speed = -motor_3508_measure[1].speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  chassis.leg_L.wheel.mileage = chassis.leg_L.wheel.mileage + CHASSIS_PERIOD * 0.001 * (chassis.leg_L.wheel.speed);
  chassis.leg_R.wheel.mileage = chassis.leg_R.wheel.mileage + CHASSIS_PERIOD * 0.001 * (chassis.leg_R.wheel.speed);
  //todo 关节电机相关状态更新,正运动学需要的量phi1 phi4
  chassis.leg_L.cyber_gear_data[0].angle = cybergears_1[LF_MOTOR_ID].angle;
  chassis.leg_L.cyber_gear_data[1].angle = cybergears_1[LB_MOTOR_ID].angle;
  chassis.leg_R.cyber_gear_data[0].angle = cybergears_1[RB_MOTOR_ID].angle;
  chassis.leg_R.cyber_gear_data[1].angle = cybergears_1[RF_MOTOR_ID].angle;
}

static void leg_state_variable_get(struct Leg *leg) {
  if (leg == NULL) {
    return;
  }
  leg->state_variable.theta_last = leg->state_variable.theta;
  leg->state_variable.theta = cal_leg_theta(leg->forward_kinematics.fk_phi.phi0);
  leg->state_variable.theta_dot_last = leg->state_variable.theta_dot;
  leg->state_variable.theta_dot =
      (leg->state_variable.theta - leg->state_variable.theta_last) / (CHASSIS_PERIOD * 0.001f);
  leg->state_variable.theta_ddot =
      (leg->state_variable.theta_dot - leg->state_variable.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);
  leg->state_variable.x = 0;
  leg->state_variable.x_dot = leg->wheel.speed;
  leg->state_variable.phi = chassis.imu_reference.pitch_angle;
  leg->state_variable.phi_dot = chassis.imu_reference.pitch_gyro;
  //todo 对位移处理，提高刹车性能
}

fp32 cal_leg_theta(fp32 phi0) {
  fp32 theta = 0, alpha = 0;//alpha is the Angle at which the virtual joint motor is turned

  alpha = PI / 2 - phi0;

  if (alpha * chassis.imu_reference.pitch_angle < 0) {
    theta = ABS(alpha) + ABS(chassis.imu_reference.pitch_angle);
    if ((alpha > 0) && (chassis.imu_reference.pitch_angle < 0)) {
    } else {
      theta *= -1;
    }
  } else {
    theta = ABS(alpha) - ABS(chassis.imu_reference.pitch_angle);
    if ((alpha < 0) && (chassis.imu_reference.pitch_angle < 0)) {
      theta *= -1;
    } else {
    }
  }
  return theta;
}

static void chassis_ctrl_info_get() {
  chassis.chassis_move_speed_set_point.vx = (float) (get_rc_ctrl().rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX;
  chassis.chassis_move_speed_set_point.vw = (float) (get_rc_ctrl().rc.ch[CHASSIS_Z_CHANNEL]) * RC_TO_VW;

  if (switch_is_down(get_rc_ctrl().rc.s[RC_s_L]) && switch_is_down(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis.last_mode = chassis.mode;
    chassis.mode = CHASSIS_DISABLE;
  } else if (switch_is_mid(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis.last_mode = chassis.mode;
    chassis.mode = CHASSIS_UNENABLED_LEG;
  } else if (switch_is_up(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis.last_mode = chassis.mode;
    chassis.mode = CHASSIS_ENABLED_LEG;
  }
}

static void chassis_init(struct Chassis *chassis) {

  if (chassis == NULL)
    return;

  chassis->mode = chassis->last_mode = CHASSIS_DISABLE;
  chassis->leg_L.wheel.motor_3508.motor_measure = motor_3508_measure;
  chassis->leg_R.wheel.motor_3508.motor_measure = motor_3508_measure + 1;

  cyber_gear_init(hcan1, LF_MOTOR_ID, &cybergears_1[LF_MOTOR_ID]);
  cyber_gear_init(hcan1, LB_MOTOR_ID, &cybergears_1[LB_MOTOR_ID]);
  cyber_gear_init(hcan1, RB_MOTOR_ID, &cybergears_1[RB_MOTOR_ID]);
  cyber_gear_init(hcan1, RF_MOTOR_ID, &cybergears_1[RF_MOTOR_ID]);

  cyber_gear_mode(&cybergears_1[LF_MOTOR_ID], 0);
  cyber_gear_mode(&cybergears_1[LB_MOTOR_ID], 0);
  cyber_gear_mode(&cybergears_1[RB_MOTOR_ID], 0);
  cyber_gear_mode(&cybergears_1[RF_MOTOR_ID], 0);

  cyber_gear_enable(&cybergears_1[LF_MOTOR_ID]);
  cyber_gear_enable(&cybergears_1[LB_MOTOR_ID]);
  cyber_gear_enable(&cybergears_1[RB_MOTOR_ID]);
  cyber_gear_enable(&cybergears_1[RF_MOTOR_ID]);

}

static void chassis_relax_handle() {
  chassis.chassis_move_speed_set_point.vx = 0;
  chassis.chassis_move_speed_set_point.vw = 0;

  chassis.mileage = 0;
}

static void chassis_enabled_leg_handle() {
  chassis_forward_kinematics();

  chassis_K_matrix_fitting(chassis.leg_L.forward_kinematics.fk_L0.L0, wheel_K, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_L.forward_kinematics.fk_L0.L0, joint_K, joint_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.forward_kinematics.fk_L0.L0, wheel_K, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.forward_kinematics.fk_L0.L0, joint_K, joint_fitting_factor);

  leg_state_variable_get(&chassis.leg_L);
  leg_state_variable_get(&chassis.leg_R);
  //todo Cal_Leg_Motors_Torque_tar

}

static void chassis_unable_leg_handle() {
  //todo: 不使用腿模式的LQR算法编写
//  chassis.leg_L.wheel.torque=(  unable_leg_K[0]*(chassis.mileage)+
//      unable_leg_K[1]*(chassis.chassis_move_speed_reference.vx-chassis.chassis_move_speed_set_point.vx)+
//      unable_leg_K[2]*()
//      )
}

void chassis_device_offline_handle() {
  if (detect_list[DETECT_REMOTE].status == OFFLINE) {
    chassis.mode = CHASSIS_DISABLE;
  }
}

static void chassis_imu_info_update() {
  chassis.imu_reference.pitch_angle = *(get_ins_angle() + 1);
  chassis.imu_reference.yaw_angle = -*(get_ins_angle() + 0);
  chassis.imu_reference.roll_angle = *(get_ins_angle() + 2);

  chassis.imu_reference.pitch_gyro = *(get_ins_gyro() + 1);
  chassis.imu_reference.yaw_gyro = -*(get_ins_gyro() + 2);
  chassis.imu_reference.roll_gyro = *(get_ins_gyro() + 0);
  //todo 三轴重力加速度对应数组确定
  chassis.imu_reference.ax = *(get_ins_accel() + 0);
  chassis.imu_reference.ay = *(get_ins_accel() + 1);
  chassis.imu_reference.az = *(get_ins_accel() + 2);

  chassis.imu_reference.ax_filtered = chassis.imu_reference.ax - GRAVITY_A * sin(chassis.imu_reference.pitch_angle);
  chassis.imu_reference.ay_filtered = chassis.imu_reference.ay
      - GRAVITY_A * cos(chassis.imu_reference.pitch_angle) * sin(chassis.imu_reference.roll_angle);
  chassis.imu_reference.az_filtered = chassis.imu_reference.az
      - GRAVITY_A * cos(chassis.imu_reference.pitch_angle) * cos(chassis.imu_reference.roll_angle);
  chassis.imu_reference.robot_az = chassis.imu_reference.ax_filtered * sin(chassis.imu_reference.pitch_angle)
      + chassis.imu_reference.ay_filtered * sin(-chassis.imu_reference.roll_angle)
          * cos(chassis.imu_reference.pitch_angle)
      + chassis.imu_reference.az_filtered * cos(chassis.imu_reference.pitch_angle)
          * cos(chassis.imu_reference.roll_angle);
}

static void chassis_forward_kinematics() {
  //leg_L
  chassis.leg_L.forward_kinematics.fk_point_coordinates.b_x = cos(chassis.leg_L.forward_kinematics.fk_phi.phi1) * L1;
  chassis.leg_L.forward_kinematics.fk_point_coordinates.b_y = sin(chassis.leg_L.forward_kinematics.fk_phi.phi1) * L1;
  chassis.leg_L.forward_kinematics.fk_point_coordinates.d_x = cos(chassis.leg_L.forward_kinematics.fk_phi.phi4) * L4
      + L5;
  chassis.leg_L.forward_kinematics.fk_point_coordinates.d_y = sin(chassis.leg_L.forward_kinematics.fk_phi.phi4) * L4;

  fp32 L_A0 = (chassis.leg_L.forward_kinematics.fk_point_coordinates.d_x
      - chassis.leg_L.forward_kinematics.fk_point_coordinates.b_x) * 2.f * L2;
  fp32 L_B0 = (chassis.leg_L.forward_kinematics.fk_point_coordinates.d_y
      - chassis.leg_L.forward_kinematics.fk_point_coordinates.b_y) * 2.f * L2;
  fp32 L_BD_sq = (chassis.leg_L.forward_kinematics.fk_point_coordinates.d_x
      - chassis.leg_L.forward_kinematics.fk_point_coordinates.b_x)
      * (chassis.leg_L.forward_kinematics.fk_point_coordinates.d_x
          - chassis.leg_L.forward_kinematics.fk_point_coordinates.b_x)
      + (chassis.leg_L.forward_kinematics.fk_point_coordinates.d_y
          - chassis.leg_L.forward_kinematics.fk_point_coordinates.b_y)
          * (chassis.leg_L.forward_kinematics.fk_point_coordinates.d_y
              - chassis.leg_L.forward_kinematics.fk_point_coordinates.b_y);
  fp32 L_C0 = L2 * L2 + L_BD_sq - L3 * L3;

  fp32 temp = L_A0 * L_A0 + L_B0 * L_B0 - L_C0 * L_C0;
  fp32 y = L_B0 + sqrt(ABS(temp));
  fp32 x = L_A0 + L_C0;
  chassis.leg_L.forward_kinematics.fk_phi.phi2 = 2.f * atan2(y, x);

  chassis.leg_L.forward_kinematics.fk_point_coordinates.c_x =
      L1 * cos(chassis.leg_L.forward_kinematics.fk_phi.phi1) + L2 * cos(chassis.leg_L.forward_kinematics.fk_phi.phi2);
  chassis.leg_L.forward_kinematics.fk_point_coordinates.c_y =
      L1 * sin(chassis.leg_L.forward_kinematics.fk_phi.phi1) + L2 * sin(chassis.leg_L.forward_kinematics.fk_phi.phi2);
  y = chassis.leg_L.forward_kinematics.fk_point_coordinates.c_y
      - chassis.leg_L.forward_kinematics.fk_point_coordinates.d_y;
  x = chassis.leg_L.forward_kinematics.fk_point_coordinates.c_x
      - chassis.leg_L.forward_kinematics.fk_point_coordinates.d_x;
  chassis.leg_L.forward_kinematics.fk_phi.phi3 = atan2(y, x);

  temp = (chassis.leg_L.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f)
      * (chassis.leg_L.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f)
      + chassis.leg_L.forward_kinematics.fk_point_coordinates.c_y
          * chassis.leg_L.forward_kinematics.fk_point_coordinates.c_y;
  chassis.leg_L.forward_kinematics.fk_L0.L0_last = chassis.leg_L.forward_kinematics.fk_L0.L0;
  chassis.leg_L.forward_kinematics.fk_L0.L0 = sqrt(ABS(temp));
  chassis.leg_L.forward_kinematics.fk_L0.L0_dot_last = chassis.leg_L.forward_kinematics.fk_L0.L0_dot;
  chassis.leg_L.forward_kinematics.fk_L0.L0_dot =
      (chassis.leg_L.forward_kinematics.fk_L0.L0 - chassis.leg_L.forward_kinematics.fk_L0.L0_last)
          / (CHASSIS_PERIOD * 0.001);
  chassis.leg_L.forward_kinematics.fk_L0.L0_ddot =
      (chassis.leg_L.forward_kinematics.fk_L0.L0_dot - chassis.leg_L.forward_kinematics.fk_L0.L0_dot_last)
          / (CHASSIS_PERIOD * 0.001);
  y = chassis.leg_L.forward_kinematics.fk_point_coordinates.c_y;
  x = chassis.leg_L.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f;
  chassis.leg_L.forward_kinematics.fk_phi.phi0 = atan2(y, x);


  //leg_R
  chassis.leg_R.forward_kinematics.fk_point_coordinates.b_x = cos(chassis.leg_R.forward_kinematics.fk_phi.phi1) * L1;
  chassis.leg_R.forward_kinematics.fk_point_coordinates.b_y = sin(chassis.leg_R.forward_kinematics.fk_phi.phi1) * L1;
  chassis.leg_R.forward_kinematics.fk_point_coordinates.d_x = cos(chassis.leg_R.forward_kinematics.fk_phi.phi4) * L4
      + L5;
  chassis.leg_R.forward_kinematics.fk_point_coordinates.d_y = sin(chassis.leg_R.forward_kinematics.fk_phi.phi4) * L4;

  fp32 R_A0 = (chassis.leg_R.forward_kinematics.fk_point_coordinates.d_x
      - chassis.leg_R.forward_kinematics.fk_point_coordinates.b_x) * 2.f * L2;
  fp32 R_B0 = (chassis.leg_R.forward_kinematics.fk_point_coordinates.d_y
      - chassis.leg_R.forward_kinematics.fk_point_coordinates.b_y) * 2.f * L2;
  fp32 R_BD_sq = (chassis.leg_R.forward_kinematics.fk_point_coordinates.d_x
      - chassis.leg_R.forward_kinematics.fk_point_coordinates.b_x)
      * (chassis.leg_R.forward_kinematics.fk_point_coordinates.d_x
          - chassis.leg_R.forward_kinematics.fk_point_coordinates.b_x)
      + (chassis.leg_R.forward_kinematics.fk_point_coordinates.d_y
          - chassis.leg_R.forward_kinematics.fk_point_coordinates.b_y)
          * (chassis.leg_R.forward_kinematics.fk_point_coordinates.d_y
              - chassis.leg_R.forward_kinematics.fk_point_coordinates.b_y);
  fp32 R_C0 = L2 * L2 + R_BD_sq - L3 * L3;

  temp = R_A0 * R_A0 + R_B0 * R_B0 - R_C0 * R_C0;
  y = R_B0 + sqrt(ABS(temp));
  x = R_A0 + R_C0;
  chassis.leg_R.forward_kinematics.fk_phi.phi2 = 2.f * atan2(y, x);

  chassis.leg_R.forward_kinematics.fk_point_coordinates.c_x =
      L1 * cos(chassis.leg_R.forward_kinematics.fk_phi.phi1) + L2 * cos(chassis.leg_R.forward_kinematics.fk_phi.phi2);
  chassis.leg_R.forward_kinematics.fk_point_coordinates.c_y =
      L1 * sin(chassis.leg_R.forward_kinematics.fk_phi.phi1) + L2 * sin(chassis.leg_R.forward_kinematics.fk_phi.phi2);
  y = chassis.leg_R.forward_kinematics.fk_point_coordinates.c_y
      - chassis.leg_R.forward_kinematics.fk_point_coordinates.d_y;
  x = chassis.leg_R.forward_kinematics.fk_point_coordinates.c_x
      - chassis.leg_R.forward_kinematics.fk_point_coordinates.d_x;
  chassis.leg_R.forward_kinematics.fk_phi.phi3 = atan2(y, x);

  temp = (chassis.leg_R.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f)
      * (chassis.leg_R.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f)
      + chassis.leg_R.forward_kinematics.fk_point_coordinates.c_y
          * chassis.leg_R.forward_kinematics.fk_point_coordinates.c_y;
  chassis.leg_R.forward_kinematics.fk_L0.L0_last = chassis.leg_R.forward_kinematics.fk_L0.L0;
  chassis.leg_R.forward_kinematics.fk_L0.L0 = sqrt(ABS(temp));
  chassis.leg_R.forward_kinematics.fk_L0.L0_dot_last = chassis.leg_R.forward_kinematics.fk_L0.L0_dot;
  chassis.leg_R.forward_kinematics.fk_L0.L0_dot =
      (chassis.leg_R.forward_kinematics.fk_L0.L0 - chassis.leg_R.forward_kinematics.fk_L0.L0_last)
          / (CHASSIS_PERIOD * 0.001);
  chassis.leg_R.forward_kinematics.fk_L0.L0_ddot =
      (chassis.leg_R.forward_kinematics.fk_L0.L0_dot - chassis.leg_R.forward_kinematics.fk_L0.L0_dot_last)
          / (CHASSIS_PERIOD * 0.001);
  y = chassis.leg_R.forward_kinematics.fk_point_coordinates.c_y;
  x = chassis.leg_R.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f;
  chassis.leg_R.forward_kinematics.fk_phi.phi0 = atan2(y, x);
}

static void chassis_inverse_kinematics() {

}

static void chassis_K_matrix_fitting(fp32 L0, fp32 K[6], const fp32 KL[6][4]) {
  for (int i = 0; i < 6; i++) {
    K[i] = KL[i][0] * pow(L0, 3) + KL[i][1] * pow(L0, 2) + KL[i][2] * pow(L0, 1) + KL[i][3] * pow(L0, 0);
  }
}

static void chassis_relax_judge() {
  if (ABS(chassis.imu_reference.pitch_angle) > 32) {
    chassis.mode = CHASSIS_DISABLE;
  }
}

static void chassis_off_ground_detection() {
  //todo 跳跃离地和 提起来切换失能模式
}

static void chassis_motor_cmd_send() {
  CAN_cmd_motor(CAN_1,
                CAN_MOTOR_0x1FF_ID,
                chassis.leg_L.wheel.motor_3508.give_current,
                chassis.leg_R.wheel.motor_3508.give_current,
                0,
                0);

//  cyber_gear_control_mode(&cybergears_1[LF_MOTOR_ID], chassis.leg_L.cyber_gear_data[0].torque, 0, 0, 0, 0);
//  cyber_gear_control_mode(&cybergears_1[LB_MOTOR_ID], chassis.leg_L.cyber_gear_data[1].torque, 0, 0, 0, 0);
//  cyber_gear_control_mode(&cybergears_1[RF_MOTOR_ID], chassis.leg_R.cyber_gear_data[0].torque, 0, 0, 0, 0);
//  cyber_gear_control_mode(&cybergears_1[RB_MOTOR_ID], chassis.leg_R.cyber_gear_data[1].torque, 0, 0, 0, 0);

  cyber_gear_control_mode(&cybergears_1[LF_MOTOR_ID], 0, 0, 0, 0, 0);//lb
  cyber_gear_control_mode(&cybergears_1[LB_MOTOR_ID], 0, 0, 0, 0, 0);
  cyber_gear_control_mode(&cybergears_1[RF_MOTOR_ID], 0, 0, 0, 0, 0);
  cyber_gear_control_mode(&cybergears_1[RB_MOTOR_ID], 0, 0, 0, 0, 0);//rf

}

struct Chassis get_chassis() {
  return chassis;
}

//遥控离线刹车原地不动
//电机离线直接全部失能

