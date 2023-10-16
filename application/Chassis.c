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
#include "bsp_laser.h"

extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
ramp_function_source_t chassis_angle_get_ramp;
ramp_function_source_t chassis_vy_ramp;
ramp_function_source_t chassis_3508_ramp[4];
first_order_filter_type_t chassis_speed_set_f;
first_order_filter_type_t chassis_speed_get_f;
first_order_filter_type_t chassis_angle_set_f;
first_order_filter_type_t chassis_angle_get_f;
chassis_t chassis;
extern gimbal_t gimbal;
extern launcher_t launcher;
extern key_board_t KeyBoard;
uint8_t chassis_right_avertence_flag;
uint8_t chassis_left_avertence_flag;
uint8_t hurt_flag;
uint32_t start_hurt_time;
fp32 period_speed;
fp32 num_speed;
fp32 period_angle;
fp32 num_angle;
Vector_msg chassis_vector_msg;

fp32 chassis_real_speed;

fp32 mileage;
fp32 motor_LF_speed, motor_RF_speed, chassis_speed;

static void chassis_init(chassis_t *chassis);

static void chassis_set_mode(chassis_t *chassis);

static void chassis_ctrl_info_get();

static void chassis_vector_info_get();

static void chassis_pc_ctrl();

static void chassis_relax_handle();

static void chassis_back_handle();

static void chassis_wheel_cal(fp32 vx, fp32 vw);

static void chassis_wheel_loop_cal();

static void chassis_follow_gimbal_handle();

static void chassis_spin_handle();

static void chassis_only_handle();

static void chassis_angle_update();

static void chassis_relax_judge();

static void chassis_power_limit();

static void chassis_Vector_Download();

void chassis_device_offline_handle();

static void chassis_push_control();

static void chassis_Vector_Upload();

static void chassis_off_ground_detection();

void chassis_task(void const *pvParameters) {

  vTaskDelay(CHASSIS_TASK_INIT_TIME);

  chassis_init(&chassis);

  while (1) {
    CAN_cmd_balance_signal_motor(CAN_1, 0x142, 0);

    chassis_angle_update();


//        chassis_ctrl_info_get();

//        update_pc_info();

//        chassis_set_mode(&chassis);

    // chassis_relax_judge();
//        chassis.vw=0;//调试用

//        chassis_device_offline_handle();
    switch (chassis.mode) {
      case CHASSIS_BACK: {
//                chassis_back_handle();
//                chassis_relax_handle();
      }
        break;

      case CHASSIS_ONLY: {
//                chassis_wheel_cal(chassis.vx,  chassis.vw);
        chassis_only_handle();
//                chassis_power_limit();
//                CAN_speed_cmd_motor(CAN_1,0x141,chassis.motor_chassis[LF].speed*100);
//                CAN_speed_cmd_motor(CAN_1,0x142,chassis.motor_chassis[RF].speed*100);

//                chassis_relax_handle();
      }
        break;

      case CHASSIS_FOLLOW_GIMBAL: {
        chassis_wheel_cal(chassis.vx, chassis.vw);
        //chassis_power_limit();
//                CAN_cmd_balance_signal_motor(CAN_1,0x141,chassis.motor_chassis[0].give_current);
        CAN_cmd_balance_signal_motor(CAN_1, 0x142, chassis.motor_chassis[1].give_current);
//                CAN_cmd_balance_signal_motor(CAN_1,0x141,0);
//                CAN_cmd_balance_signal_motor(CAN_1,0x142,0);
//                chassis_relax_handle();
      }
        break;

      case CHASSIS_SPIN: {
        chassis_spin_handle();
        //chassis_power_limit();
        CAN_cmd_balance_signal_motor(CAN_1, 0x141, chassis.motor_chassis[0].give_current);
        CAN_cmd_balance_signal_motor(CAN_1, 0x142, chassis.motor_chassis[1].give_current);
//                chassis_relax_handle();
      }
        break;

      case CHASSIS_OFF_GROUND: {
        chassis_real_speed = 0;
        mileage = 0;
        CAN_cmd_balance_signal_motor(CAN_1, 0x141, 0);
        CAN_cmd_balance_signal_motor(CAN_1, 0x142, 0);
      }

      case CHASSIS_RELAX:chassis_relax_handle();
        chassis_init(&chassis);
        break;
    }
    if (ABS(chassis.absolute_angle_get) > 30) {
      chassis_relax_handle();
    }
    if (chassis.vx != 0) {
      mileage = 0;
    }
    mileage = mileage + 15 * 0.001 * (motor_RF_speed + motor_LF_speed) / 2;
    vTaskDelay(CHASSIS_PERIOD);
  }

}

static void chassis_ctrl_info_get() {

  chassis_pc_ctrl();
  chassis.vx = (float) (get_rc_ctrl().rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX
      + chassis.vx_pc;

  chassis.vw = (float) (get_rc_ctrl().rc.ch[CHASSIS_Z_CHANNEL]) * RC_TO_VW
      + chassis.vw_pc;

}

static void chassis_init(chassis_t *chassis) {

  if (chassis == NULL)
    return;

  uint8_t i = 0;

  pid_init(&chassis->chassis_vw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT,
           CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD);

  for (i = 0; i < 2; i++) {
    chassis->motor_chassis[i].motor_measure = motor_3508_measure + i;
    pid_init(&chassis->motor_chassis[i].speed_p,
             CHASSIS_3508_PID_MAX_OUT,
             CHASSIS_3508_PID_MAX_IOUT,
             CHASSIS_3508_PID_KP,
             CHASSIS_3508_PID_KI,
             CHASSIS_3508_PID_KD);
  }
  chassis->motor_push.motor_measure = &motor_3508_measure[3];//0x205
  chassis->motor_left.motor_measure = &motor_pitch_measure;//0x206

  pid_init(&chassis->motor_push.speed_p, push_speed_MAX_OUT, push_speed_MAX_IOUT, push_speed_kp,
           push_speed_ki, push_speed_kd);

  pid_init(&chassis->motor_push.angle_p, push_angle_MAX_OUT, push_angle_MAX_IOUT, push_angle_kp,
           push_angle_ki, push_angle_kd);

  pid_init(&chassis->motor_left.speed_p, push_speed_MAX_OUT, push_speed_MAX_IOUT, push_speed_kp,
           push_speed_ki, push_speed_kd);

  pid_init(&chassis->motor_left.angle_p, push_angle_MAX_OUT, push_angle_MAX_IOUT, push_angle_kp,
           push_angle_ki, push_angle_kd);

  chassis->motor_push.motor_measure->offset_ecd = 8091;
  chassis->motor_left.motor_measure->offset_ecd = 7821;

  pid_init(&chassis->chassis_speed_p,
           CHASSIS_SPEED_PID_MAX_OUT,
           CHASSIS_SPEED_PID_MAX_IOUT,
           CHASSIS_SPEED_PID_KP,
           CHASSIS_SPEED_PID_KI,
           CHASSIS_SPEED_PID_KD);

  pid_init(&chassis->angle_p,
           CHASSIS_PITCH_ANGLE_MAX_OUT,
           CHASSIS_PITCH_ANGLE_MAX_IOUT,
           CHASSIS_PITCH_ANGLE_PID_KP,
           CHASSIS_PITCH_ANGLE_PID_KI,
           CHASSIS_PITCH_ANGLE_PID_KD);

  pid_init(&chassis->chassis_back_speed_p,
           CHASSIS_BACK_SPEED_PID_MAX_OUT,
           CHASSIS_BACK_SPEED_PID_MAX_IOUT,
           CHASSIS_BACK_SPEED_PID_KP,
           CHASSIS_BACK_SPEED_PID_KI,
           CHASSIS_BACK_SPEED_PID_KD);

  pid_init(&chassis->chassis_back_angle_p,
           CHASSIS_BACK_PITCH_ANGLE_MAX_OUT,
           CHASSIS_BACK_PITCH_ANGLE_MAX_IOUT,
           CHASSIS_BACK_PITCH_ANGLE_PID_KP,
           CHASSIS_BACK_PITCH_ANGLE_PID_KI,
           CHASSIS_BACK_PITCH_ANGLE_PID_KD);

  chassis->mode = chassis->last_mode = CHASSIS_RELAX;

  period_angle = 1;
  num_angle = 1;
  period_speed = 1;
  num_speed = 1;
//    ramp_init(&chassis_angle_get_ramp,0.0001f,MAX_CHASSIS_TILTABLE_ANGLE,-MIN_CHASSIS_TILTABLE_ANGLE);
//    ramp_init(&chassis_vy_ramp,0.0001f,MAX_CHASSIS_VX_SPEED,-MAX_CHASSIS_VX_SPEED);
  ramp_init(&chassis_3508_ramp[LF], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
  ramp_init(&chassis_3508_ramp[RF], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
  //  first_order_filter_init(&chassis_speed_set_f,period_speed,num_speed);
//    first_order_filter_init(&chassis_speed_get_f,period_speed,num_speed);
//    //first_order_filter_init(&chassis_angle_set_f,period_angle,num_angle);
//    first_order_filter_init(&chassis_angle_get_f,period_angle,num_angle);

  chassis->balanced_angle = 3;
}

static void chassis_set_mode(chassis_t *chassis) {

  if (chassis == NULL)
    return;
  if (switch_is_down(get_rc_ctrl().rc.s[RC_s_L]) && switch_is_down(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis->last_mode = chassis->mode;
    chassis->mode = CHASSIS_RELAX;
  } else if (switch_is_down(get_rc_ctrl().rc.s[RC_s_R]) && !switch_is_down(get_rc_ctrl().rc.s[RC_s_L])) {
    chassis->last_mode = chassis->mode;
    if (chassis->last_mode == CHASSIS_RELAX) {
      chassis->mode = CHASSIS_BACK;
      chassis->chassis_is_back = 0;
    } else if (chassis->mode == CHASSIS_BACK
        && chassis->chassis_is_back == 1
        ) {
      chassis->mode = CHASSIS_ONLY;
    }

  } else if (switch_is_mid(get_rc_ctrl().rc.s[RC_s_R])) {
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
  //UI????---??????
  ui_robot_status.chassis_mode = chassis->mode;
}

static void chassis_pc_ctrl() {

  if (KeyBoard.W.status == KEY_PRESS)//
  {
    chassis.vx_pc += SPEED_CHANGE;//
  } else if (KeyBoard.S.status == KEY_PRESS) {
    chassis.vx_pc -= SPEED_CHANGE;
  } else {
    chassis.vx_pc = 0;
  }

  if (KeyBoard.A.status == KEY_PRESS)//
  {
    chassis.vy_pc -= SPEED_CHANGE;
  } else if (KeyBoard.D.status == KEY_PRESS) {
    chassis.vy_pc += SPEED_CHANGE;
  } else {
    chassis.vy_pc = 0;
  }

  if (chassis.mode == CHASSIS_INDEPENDENT_CONTROL) //
  {
    if (KeyBoard.Q.status == KEY_PRESS)//
    {
      chassis.vw_pc -= ROTATION_SPEED_CHANGE;
    } else if (KeyBoard.E.status == KEY_PRESS) {
      chassis.vw_pc += ROTATION_SPEED_CHANGE;
    } else {
      chassis.vw_pc = 0;
    }
  }

  if (KeyBoard.SHIFT.status == KEY_PRESS) {
    chassis.speed_mode = FAST;
  } else if (KeyBoard.CTRL.status == KEY_PRESS) {
    chassis.speed_mode = SLOW;
  } else {
    chassis.speed_mode = NORMAL;
  }

}

static void chassis_relax_handle() {

  chassis_real_speed = 0;
  chassis.vx = 0;
  chassis.vw = 0;
  CAN_cmd_balance_signal_motor(CAN_1, 0x141, 0);
  CAN_cmd_balance_signal_motor(CAN_1, 0x142, 0);
  mileage = 0;
}

fp32 motor_vx;
fp32 break_vx;
fp32 wheel_rpm[2];
static void chassis_back_handle() {
  if (chassis.absolute_angle_get <= 7 && chassis.absolute_angle_get >= -7
      && (ABS(motor_LF_speed + motor_RF_speed) / 2.0f) < 6) {
    chassis.chassis_is_back = true;
  }
  if (chassis.chassis_is_back == false) {
//        chassis_speed=0;
    //以下两行注释为2023赛季平衡兵轮子转速线速度计算函数
    motor_LF_speed = chassis.motor_chassis[LF].motor_measure->speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
    motor_RF_speed = -chassis.motor_chassis[RF].motor_measure->speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
    //底盘速度环

    first_order_filter_cali(&chassis_speed_get_f, (fp32) (motor_LF_speed + motor_RF_speed) / 2.0f);

    chassis.absolute_angle_set = pid_calc(&chassis.chassis_back_speed_p,
                                          ((motor_LF_speed + motor_RF_speed) / 2.0f),
                                          chassis.vx);

    //底盘角度环

    first_order_filter_cali(&chassis_angle_get_f, chassis.absolute_angle_get);
//        motor_vx= pid_calc(&chassis.angle_p,chassis.absolute_angle_get,chassis.balanced_angle+chassis.absolute_angle_set);
    motor_vx = pid_calc(&chassis.chassis_back_angle_p, chassis.absolute_angle_get, 0);

    //里程计计算


    //以下两行为2023赛季平衡兵电机转速线速度计算函数
    wheel_rpm[0] = ((-motor_vx) * 60) / (2 * PI * 0.11);
    wheel_rpm[1] = ((motor_vx) * 60) / (2 * PI * 0.11);

    chassis.motor_chassis[LF].rpm_set = wheel_rpm[0];
    chassis.motor_chassis[RF].rpm_set = wheel_rpm[1];
    chassis.motor_chassis[LF].speed = chassis.motor_chassis[LF].rpm_set * 360 / 60;
    chassis.motor_chassis[RF].speed = chassis.motor_chassis[RF].rpm_set * 360 / 60;
  }
}

fp32 LQR_vx_out[2];
fp32 K1[6] = {-10.000000, -18.026502, -52.274290, -15.375695, 10.000000, 2.345114};
//fp32 K2[6]={ -2.236068,-4.224023,-16.132258,-4.137404,-2.236068,-2.260908};
static void chassis_wheel_cal(fp32 vx, fp32 vw) {

  //以下两行注释为2023赛季平衡兵轮子转速线速度计算函数
  motor_LF_speed = chassis.motor_chassis[LF].motor_measure->speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  motor_RF_speed = -chassis.motor_chassis[RF].motor_measure->speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  chassis_real_speed = (motor_LF_speed + motor_RF_speed) / 2.0f;

  LQR_vx_out[0] = K1[1] * (chassis_real_speed - chassis.vx);
  LQR_vx_out[1] = K1[1] * (chassis_real_speed - chassis.vx);
  VAL_LIMIT(LQR_vx_out[0], -30, 30);
  VAL_LIMIT(LQR_vx_out[1], -30, 30);

  chassis.motor_chassis[0].torque = -(K1[0] * (mileage) +
      LQR_vx_out[0] +
      K1[2] * (-INS_angle[2] - 0.04) +
      K1[3] * (-INS_gyro[0]) +
      K1[4] * (chassis.yaw_relative_angle_get - 0.06) +
//                                       K1[4]*(-INS_angle[0]) +-
      K1[5] * (-INS_gyro[2]));

  chassis.motor_chassis[1].torque = -(K1[0] * (mileage) +
      LQR_vx_out[1] +
      K1[2] * (-INS_angle[2] - 0.04) +
      K1[3] * (-INS_gyro[0]) -
      K1[4] * (chassis.yaw_relative_angle_get - 0.06) -
//                                       K1[4]*(-INS_angle[0]) -
      K1[5] * (-INS_gyro[2]));

  chassis.motor_chassis[0].give_current = (chassis.motor_chassis[0].torque / 0.32) * (2000 / 32);
  chassis.motor_chassis[1].give_current = -(chassis.motor_chassis[1].torque / 0.32) * (2000 / 32);
  VAL_LIMIT(chassis.motor_chassis[0].give_current, -2000, 2000);
  VAL_LIMIT(chassis.motor_chassis[1].give_current, -2000, 2000);
}

static void chassis_wheel_loop_cal() {

  chassis.motor_chassis[LF].give_current = pid_calc(&chassis.motor_chassis[LF].speed_p,
                                                    chassis.motor_chassis[LF].motor_measure->speed_rpm,
                                                    chassis.motor_chassis[LF].rpm_set);

  chassis.motor_chassis[RF].give_current = pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                    chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                    chassis.motor_chassis[RF].rpm_set);

}

static void chassis_only_handle() {
  wheel_rpm[0] = ((-chassis.vx * 0.1 + 0.1 * chassis.vw) * 60) / (2 * PI * 0.11);
  wheel_rpm[1] = ((chassis.vx * 0.1 + 0.1 * chassis.vw) * 60) / (2 * PI * 0.11);

  chassis.motor_chassis[LF].rpm_set = wheel_rpm[0];
  chassis.motor_chassis[RF].rpm_set = wheel_rpm[1];
  chassis.motor_chassis[LF].speed = chassis.motor_chassis[LF].rpm_set * 360 / 60;
  chassis.motor_chassis[RF].speed = chassis.motor_chassis[RF].rpm_set * 360 / 60;
}

static void chassis_follow_gimbal_handle() {

  chassis.vw = pid_calc(&chassis.chassis_vw_pid, -gimbal.yaw.relative_angle_get, 0);
  VAL_LIMIT(chassis.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);

}

static void chassis_aslant_follow_gimbal_handle() {
  KeyBoard.E.click_flag = 0;
  laser_on();
  launcher.fire_mode = Fire_ON;
  if (Referee.RobotHurt.hurt_type == 5 && hurt_flag == 0) {
    start_hurt_time = HAL_GetTick();
    hurt_flag = 1;
  }
  if (chassis_right_avertence_flag == 1) {
    chassis.vw = pid_calc(&chassis.chassis_vw_pid, -gimbal.yaw.relative_angle_get, 90);
  } else if (chassis_left_avertence_flag == 1) {
    chassis.vw = pid_calc(&chassis.chassis_vw_pid, -gimbal.yaw.relative_angle_get, -90);

  }
}

static void chassis_spin_handle() {
  //以下两行注释为2023赛季平衡兵轮子转速线速度计算函数
  motor_LF_speed = chassis.motor_chassis[LF].motor_measure->speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  motor_RF_speed = -chassis.motor_chassis[RF].motor_measure->speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  chassis_real_speed = (motor_LF_speed + motor_RF_speed) / 2.0f;

  LQR_vx_out[0] = K1[1] * (chassis_real_speed - chassis.vx);
  LQR_vx_out[1] = K1[1] * (chassis_real_speed - chassis.vx);
  VAL_LIMIT(LQR_vx_out[0], -30, 30);
  VAL_LIMIT(LQR_vx_out[1], -30, 30);

  chassis.motor_chassis[0].torque = -(K1[0] * (mileage) +
      LQR_vx_out[0] +
      K1[2] * (-INS_angle[2] - 0.04) +
      K1[3] * (-INS_gyro[0]) +
//                                       K1[4]*(chassis.yaw_relative_angle_get-0.06) +
      //                                       K1[4]*(-INS_angle[0]) +-
      K1[5] * (0.1));

  chassis.motor_chassis[1].torque = -(K1[0] * (mileage) +
      LQR_vx_out[1] +
      K1[2] * (-INS_angle[2] - 0.04) +
      K1[3] * (-INS_gyro[0]) -
//                                       K1[4]*(chassis.yaw_relative_angle_get-0.06) -
      //                                       K1[4]*(-INS_angle[0]) -
      K1[5] * (0.1));

  chassis.motor_chassis[0].give_current = (chassis.motor_chassis[0].torque / 0.32) * (2000 / 32);
  chassis.motor_chassis[1].give_current = -(chassis.motor_chassis[1].torque / 0.32) * (2000 / 32);
  VAL_LIMIT(chassis.motor_chassis[0].give_current, -2000, 2000);
  VAL_LIMIT(chassis.motor_chassis[1].give_current, -2000, 2000);
}

static void chassis_indepent_control_handle() {
  KeyBoard.E.click_flag = 0;
  laser_on();
  launcher.fire_mode = Fire_ON;

}

void chassis_device_offline_handle() {
  if (detect_list[DETECT_REMOTE].status == OFFLINE) {
    chassis.mode = CHASSIS_RELAX;
  }
}
int16_t give_current_limit[4];
int16_t debug_current[4];
int16_t debug_give_current[4];
first_order_filter_type_t give_limit = {
    .frame_period=20,
    .num=1,
    .out=0,
    .input=0
};
void chassis_power_limit() {
  chassis.chassis_power_limit.total_current = 0;
  chassis.chassis_power_limit.total_current_limit = 0;
  fp32 power_buffer = chassis.chassis_power_limit.power_buff;
  fp32 limit_k;
  if (detect_list[DETECT_REFEREE].status != ONLINE) {
    chassis.chassis_power_limit.total_current_limit = CHASSIS_CURRENT_LIMIT_40W;
  } else {
    chassis.chassis_power_limit.power_buff = Referee.PowerHeatData.chassis_power_buffer > CHASSIS_POWER_BUFF ?
                                             CHASSIS_POWER_BUFF : Referee.PowerHeatData.chassis_power_buffer;
    chassis.chassis_power_limit.limit_k = chassis.chassis_power_limit.power_buff / CHASSIS_POWER_BUFF;

    if (chassis.chassis_power_limit.power_buff < 20) {
      chassis.chassis_power_limit.limit_k = chassis.chassis_power_limit.limit_k * chassis.chassis_power_limit.limit_k
          * chassis.chassis_power_limit.limit_k;
    } else {
      chassis.chassis_power_limit.limit_k = chassis.chassis_power_limit.limit_k * chassis.chassis_power_limit.limit_k;
    }
//        power_buffer=Referee.PowerHeatData.chassis_power_buffer>CHASSIS_POWER_BUFF?
//                                               CHASSIS_POWER_BUFF:Referee.PowerHeatData.chassis_power_buffer;
//        if(power_buffer>52&&power_buffer<=60)
//            limit_k=chassis.chassis_power_limit.power_buff/CHASSIS_POWER_BUFF;
//        else if(power_buffer>30)
//            limit_k=limit_k*limit_k;
//        else
//            limit_k=limit_k*limit_k*limit_k;
//        for(uint8_t i=0;i<4;i++)
//            chassis.chassis_power_limit.total_current+=abs(chassis.motor_chassis[i].give_current);
//        for(uint8_t i=0;i<4;i++)
//            give_current_limit[i]=(int16_t)(chassis.motor_chassis[i].give_current*limit_k);

    chassis.chassis_power_limit.total_current_limit = chassis.chassis_power_limit.limit_k * CHASSIS_CURRENT_LIMIT_TOTAL;
  }
  for (uint8_t i = 0; i < 4; i++)
    chassis.chassis_power_limit.total_current += abs(chassis.motor_chassis[i].give_current);

  for (uint8_t i = 0; i < 4; i++)
    give_current_limit[i] = chassis.motor_chassis[i].give_current;

  //
  for (uint8_t i = 0; i < 4; i++)
    debug_current[i] = chassis.motor_chassis[i].give_current;

  if (chassis.chassis_power_limit.total_current > chassis.chassis_power_limit.total_current_limit) {
    for (uint8_t i = 0; i < 4; i++)
      give_current_limit[i] =
          (int16_t) (chassis.motor_chassis[i].give_current * chassis.chassis_power_limit.total_current_limit
              / chassis.chassis_power_limit.total_current);
    //give_current_limit[i]=(int16_t)(chassis.motor_chassis[i].give_current*chassis.chassis_power_limit.limit_k);
  }
  for (uint8_t i = 0; i < 4; i++)
    chassis.motor_chassis[i].give_current = debug_give_current[i];

}

static void chassis_angle_update() {
  chassis.absolute_angle_get = -INS_angle[2] * MOTOR_RAD_TO_ANGLE;
}

static void chassis_relax_judge() {
  if (ABS(chassis.absolute_angle_get) > 32) {
    chassis.mode = CHASSIS_RELAX;
  }
}

