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

struct Chassis chassis;

fp32 motor_LF_speed, motor_RF_speed, chassis_speed;

static void chassis_init(struct Chassis *chassis);
static void chassis_set_mode(struct Chassis *chassis);
static void chassis_ctrl_info_get();
static void chassis_relax_handle();
static void chassis_wheel_cal(fp32 vx, fp32 vw);
static void chassis_angle_update();
static void chassis_relax_judge();
void chassis_device_offline_handle();
static void chassis_off_ground_detection();
static void chassis_info_update();

void chassis_task(void const *pvParameters) {

  vTaskDelay(CHASSIS_TASK_INIT_TIME);

  chassis_init(&chassis);

  while (1) {

    chassis_info_update();

    chassis_ctrl_info_get();

    chassis_set_mode(&chassis);

    chassis_relax_judge();

    switch (chassis.mode) {
      case CHASSIS_ENABLE: {

      }
        break;

      case CHASSIS_OFF_GROUND: {
      }
        break;

      case CHASSIS_DISABLE: {
        chassis_relax_handle();
        chassis_init(&chassis);
      }
        break;
    }

    vTaskDelay(CHASSIS_PERIOD);
  }
}

static void chassis_info_update() {
  chassis_angle_update();
  chassis.mileage = chassis.mileage + 15 * 0.001 * (motor_RF_speed + motor_LF_speed) / 2;
  if (chassis.move_speed_set_point.vx != 0) {
    chassis.mileage = 0;
  }
}

static void chassis_ctrl_info_get() {
  chassis.move_speed_set_point.vx = (float) (get_rc_ctrl().rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX;
  chassis.move_speed_set_point.vw = (float) (get_rc_ctrl().rc.ch[CHASSIS_Z_CHANNEL]) * RC_TO_VW;
}

static void chassis_init(struct Chassis *chassis) {

  if (chassis == NULL)
    return;

  chassis->mode = chassis->last_mode = CHASSIS_DISABLE;

}

static void chassis_set_mode(struct Chassis *chassis) {

  if (chassis == NULL)
    return;

  if (switch_is_down(get_rc_ctrl().rc.s[RC_s_L]) && switch_is_down(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis->last_mode = chassis->mode;
    chassis->mode = CHASSIS_DISABLE;
  } else if (switch_is_mid(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis->last_mode = chassis->mode;
    chassis->mode = CHASSIS_ENABLE;
  } else if (switch_is_up(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis->last_mode = chassis->mode;
    chassis->mode = CHASSIS_ENABLE;
  }
}

static void chassis_relax_handle() {
  chassis.move_speed_set_point.vx = 0;
  chassis.move_speed_set_point.vw = 0;

  chassis.mileage = 0;
}

static void chassis_wheel_cal(fp32 vx, fp32 vw) {
  //以下两行注释为2023赛季平衡兵轮子转速线速度计算函数
  motor_LF_speed = chassis.motor_chassis[LF].motor_measure->speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  motor_RF_speed = -chassis.motor_chassis[RF].motor_measure->speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;

}

static void chassis_spin_handle() {
  //以下两行注释为2023赛季平衡兵轮子转速线速度计算函数
  motor_LF_speed = chassis.motor_chassis[LF].motor_measure->speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  motor_RF_speed = -chassis.motor_chassis[RF].motor_measure->speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
}

void chassis_device_offline_handle() {
  if (detect_list[DETECT_REMOTE].status == OFFLINE) {
    chassis.mode = CHASSIS_DISABLE;
  }
}

static void chassis_angle_update() {
  chassis.imu_reference.pitch = *(get_ins_angle() + 1) * MOTOR_RAD_TO_ANGLE;
  chassis.imu_reference.yaw = -*(get_ins_angle() + 0) * MOTOR_RAD_TO_ANGLE;
  chassis.imu_reference.roll = *(get_ins_angle() + 2) * MOTOR_RAD_TO_ANGLE;
}

static void chassis_relax_judge() {
  if (ABS(chassis.imu_reference.pitch) > 32) {
    chassis.mode = CHASSIS_DISABLE;
  }
}

struct Chassis get_chassis() {
  return chassis;
}

//遥控离线刹车原地不动
//电机离线直接全部失能

