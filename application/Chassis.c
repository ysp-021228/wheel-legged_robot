/*include*/
#include <stdlib.h>

#include "Chassis.h"
#include "cmsis_os.h"
#include "remote.h"
#include "can_receive.h"
#include "user_lib.h"
#include "ramp.h"
#include "key_board.h"
#include "Gimbal.h"
#include "arm_math.h"
#include "Referee.h"
#include "Detection.h"
#include "launcher.h"
#include "Atti.h"

ramp_function_source_t chassis_3508_ramp[4];
struct Chassis chassis;
extern gimbal_t gimbal;
extern launcher_t launcher;
extern key_board_t KeyBoard;
uint32_t start_hurt_time;
fp32 mileage;
fp32 motor_LF_speed, motor_RF_speed, chassis_speed;

static void chassis_init(struct Chassis *chassis);
static void chassis_set_mode(struct Chassis *chassis);
static void chassis_ctrl_info_get();
static void chassis_relax_handle();
static void chassis_wheel_cal(fp32 vx, fp32 vw);
static void chassis_wheel_loop_cal();
static void chassis_spin_handle();
static void chassis_only_handle();
static void chassis_angle_update();
static void chassis_relax_judge();
void chassis_device_offline_handle();
static void chassis_off_ground_detection();

void chassis_task(void const *pvParameters) {

  vTaskDelay(CHASSIS_TASK_INIT_TIME);

  chassis_init(&chassis);

  while (1) {

    chassis_angle_update();

    chassis_ctrl_info_get();

    chassis_set_mode(&chassis);

    // chassis_relax_judge();

    switch (chassis.mode) {

      case CHASSIS_SPIN: {
        chassis_spin_handle();
        CAN_cmd_balance_signal_motor(CAN_1, 0x141, chassis.motor_chassis[0].give_current);
        CAN_cmd_balance_signal_motor(CAN_1, 0x142, chassis.motor_chassis[1].give_current);
      }
        break;

      case CHASSIS_OFF_GROUND: {
      }

      case CHASSIS_RELAX: {
        chassis_relax_handle();
        chassis_init(&chassis);
      }
        break;
    }
    if (chassis.vx != 0) {
      mileage = 0;
    }
    mileage = mileage + 15 * 0.001 * (motor_RF_speed + motor_LF_speed) / 2;
    vTaskDelay(CHASSIS_PERIOD);
  }
}

static void chassis_ctrl_info_get() {
  chassis.vx = (float) (get_rc_ctrl().rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX;
  chassis.vw = (float) (get_rc_ctrl().rc.ch[CHASSIS_Z_CHANNEL]) * RC_TO_VW;
}

static void chassis_init(struct Chassis *chassis) {

  if (chassis == NULL)
    return;

  chassis->mode = chassis->last_mode = CHASSIS_RELAX;

  ramp_init(&chassis_3508_ramp[LF], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
  ramp_init(&chassis_3508_ramp[RF], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);

  chassis->balanced_angle = 3;
}

static void chassis_set_mode(struct Chassis *chassis) {

  if (chassis == NULL)
    return;

  if (switch_is_down(get_rc_ctrl().rc.s[RC_s_L]) && switch_is_down(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis->last_mode = chassis->mode;
    chassis->mode = CHASSIS_RELAX;
  }else if (switch_is_mid(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis->last_mode = chassis->mode;
    chassis->mode = CHASSIS_FOLLOW_GIMBAL;
  } else if (switch_is_up(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis->last_mode = chassis->mode;
    chassis->mode = CHASSIS_SPIN;
  }

  if (KeyBoard.E.click_flag == 1)//
  {
    chassis->mode = CHASSIS_SPIN;
  }
}

static void chassis_relax_handle() {

  chassis.vx = 0;
  chassis.vw = 0;

  mileage = 0;
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
    chassis.mode = CHASSIS_RELAX;
  }
}
static void chassis_angle_update() {
  chassis.absolute_angle_get = -*(get_ins_angle()+2) * MOTOR_RAD_TO_ANGLE;
}

static void chassis_relax_judge() {
  if (ABS(chassis.absolute_angle_get) > 32) {
    chassis.mode = CHASSIS_RELAX;
  }
}

struct Chassis get_chassis(){
  return chassis;
}

