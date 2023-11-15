/*include*/
#include <stdlib.h>

#include "Subsystem/Chassis.h"
#include "cmsis_os.h"
#include "IO/remote.h"
#include "IO/can_receive.h"
#include "user_lib.h"
#include "arm_math.h"
#include "Subsystem/Detection.h"
#include "Device/Atti.h"
#include "CyberGear.h"
#include "chassis_algorithm.h"

struct Chassis chassis;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

fp32 wheel_K_L[6] = {0, 0, 0, 0, 0, 0};
fp32 joint_K_L[6] = {0, 0, 0, 0, 0, 0};

fp32 wheel_K_R[6] = {0, 0, 0, 0, 0, 0};
fp32 joint_K_R[6] = {0, 0, 0, 0, 0, 0};

fp32 wheel_fitting_factor[6][4] = {
    {-1646.924405, 213.914414, -48.404761, -0.591737},
    {132.643205, -38.349908, -0.852857, -0.172735},

    {1108.147318, -170.632047, 5.865122, -0.498328},
    {2769.004713, -414.553082, 16.788342, -0.930631},

    {120.632950, -148.900646, -3.510030, 5.506320},
    {-151.748316, 17.059403, -1.704834, 0.406821}
};
fp32 joint_fitting_factor[6][4] = {
    {47907.722195, -9892.085410, 788.070850, 9.756630},
    {-564.792576, -81.727188, 18.329920, 4.059584},

    {6837.153119, -1212.766774, 43.244902, 7.985787},
    {-11512.091221, 1386.097877, -106.456406, 15.819744},

    {-33315.577466, 5602.029788, 2.956268, 28.679460},
    {-1209.772314, 217.708476, 7.609868, 0.813373}
};
/*******************************************************************************
 *                                    Init                                     *
 *******************************************************************************/
static void chassis_init(struct Chassis *chassis) {
  if (chassis == NULL)
    return;

  chassis->mode = chassis->last_mode = CHASSIS_DISABLE;
  chassis->leg_L.wheel.motor_3508.motor_measure = motor_3508_measure;
  chassis->leg_R.wheel.motor_3508.motor_measure = motor_3508_measure + 1;

  chassis->leg_L.leg_index = L;
  chassis->leg_R.leg_index = R;

  cyber_gear_init(hcan2, LF_MOTOR_ID, &cybergears_2[LF_MOTOR_ID]);
  cyber_gear_init(hcan2, LB_MOTOR_ID, &cybergears_2[LB_MOTOR_ID]);
  cyber_gear_init(hcan2, RB_MOTOR_ID, &cybergears_2[RB_MOTOR_ID]);
  cyber_gear_init(hcan2, RF_MOTOR_ID, &cybergears_2[RF_MOTOR_ID]);

  mi_motor_disable(&cybergears_2[LF_MOTOR_ID]);
  osDelay(1);
  mi_motor_disable(&cybergears_2[LB_MOTOR_ID]);
  osDelay(1);
  mi_motor_disable(&cybergears_2[RB_MOTOR_ID]);
  osDelay(1);
  mi_motor_disable(&cybergears_2[RF_MOTOR_ID]);
  osDelay(100);

  mi_motor_set_zero(&cybergears_2[LF_MOTOR_ID]);
  osDelay(1);
  mi_motor_set_zero(&cybergears_2[LB_MOTOR_ID]);
  osDelay(1);
  mi_motor_set_zero(&cybergears_2[RB_MOTOR_ID]);
  osDelay(1);
  mi_motor_set_zero(&cybergears_2[RF_MOTOR_ID]);
  osDelay(1);

  cyber_gear_enable(&cybergears_2[LF_MOTOR_ID]);
  osDelay(1);
  cyber_gear_enable(&cybergears_2[LB_MOTOR_ID]);
  osDelay(1);
  cyber_gear_enable(&cybergears_2[RB_MOTOR_ID]);
  osDelay(1);
  cyber_gear_enable(&cybergears_2[RF_MOTOR_ID]);
  osDelay(1);

  pid_init(&chassis->leg_L.pid,
           CHASSIS_LEG_L0_PID_OUT_LIMIT,
           CHASSIS_LEG_L0_PID_IOUT_LIMIT,
           CHASSIS_LEG_LO_PID_P,
           CHASSIS_LEG_L0_PID_I,
           CHASSIS_LEG_L0_PID_D);

  pid_init(&chassis->leg_R.pid,
           CHASSIS_LEG_L0_PID_OUT_LIMIT,
           CHASSIS_LEG_L0_PID_IOUT_LIMIT,
           CHASSIS_LEG_LO_PID_P,
           CHASSIS_LEG_L0_PID_I,
           CHASSIS_LEG_L0_PID_D);

  pid_init(&chassis->chassis_vw_pid,
           CHASSIS_VW_PID_OUT_LIMIT,
           CHASSIS_VW_PID_IOUT_LIMIT,
           CHASSIS_VW_PID_P,
           CHASSIS_VW_PID_I,
           CHASSIS_VW_PID_D);

  pid_init(&chassis->chassis_roll_pid,
           CHASSIS_ROLL_PID_OUT_LIMIT,
           CHASSIS_ROLL_PID_IOUT_LIMIT,
           CHASSIS_ROLL_PID_P,
           CHASSIS_ROLL_PID_I,
           CHASSIS_ROLL_PID_D);

}

/*******************************************************************************
 *                              Getter & Setter                                *
 *******************************************************************************/
static void chassis_imu_info_update() {
  chassis.imu_reference.pitch_angle = *(get_ins_angle() + 1);
  chassis.imu_reference.yaw_angle = -*(get_ins_angle() + 0);
  chassis.imu_reference.roll_angle = *(get_ins_angle() + 2);

  chassis.imu_reference.pitch_gyro = *(get_ins_gyro() + 1);
  chassis.imu_reference.yaw_gyro = -*(get_ins_gyro() + 2);
  chassis.imu_reference.roll_gyro = *(get_ins_gyro() + 0);

  chassis.imu_reference.ax = *(get_ins_accel() + 0);
  chassis.imu_reference.ay = *(get_ins_accel() + 1);
  chassis.imu_reference.az = *(get_ins_accel() + 2);

  chassis.imu_reference.ax_filtered = chassis.imu_reference.ax - GRAVITY_A * sinf(chassis.imu_reference.pitch_angle);
  chassis.imu_reference.ay_filtered = chassis.imu_reference.ay
      - GRAVITY_A * cosf(chassis.imu_reference.pitch_angle) * sinf(chassis.imu_reference.roll_angle);
  chassis.imu_reference.az_filtered = chassis.imu_reference.az
      - GRAVITY_A * cosf(chassis.imu_reference.pitch_angle) * cosf(chassis.imu_reference.roll_angle);
  chassis.imu_reference.robot_az = chassis.imu_reference.ax_filtered * sinf(chassis.imu_reference.pitch_angle)
      + chassis.imu_reference.ay_filtered * sinf(-chassis.imu_reference.roll_angle)
          * cosf(chassis.imu_reference.pitch_angle)
      + chassis.imu_reference.az_filtered * cosf(chassis.imu_reference.pitch_angle)
          * cosf(chassis.imu_reference.roll_angle);
}

static void chassis_motor_info_update() {
  chassis.leg_L.wheel.speed = -motor_3508_measure[0].speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  chassis.leg_R.wheel.speed = -motor_3508_measure[1].speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  chassis.leg_L.wheel.mileage = chassis.leg_L.wheel.mileage + CHASSIS_PERIOD * MILLISECOND_TO_SECOND * (chassis.leg_L.wheel.speed);
  chassis.leg_R.wheel.mileage = chassis.leg_R.wheel.mileage + CHASSIS_PERIOD * MILLISECOND_TO_SECOND * (chassis.leg_R.wheel.speed);

  chassis.leg_L.cyber_gear_data[0].angle = cybergears_2[LB_MOTOR_ID].angle;
  chassis.leg_L.cyber_gear_data[1].angle = cybergears_2[LF_MOTOR_ID].angle;
  chassis.leg_R.cyber_gear_data[0].angle = cybergears_2[RB_MOTOR_ID].angle;
  chassis.leg_R.cyber_gear_data[1].angle = cybergears_2[RF_MOTOR_ID].angle;

  chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1 =
      PI - (chassis.leg_L.cyber_gear_data[0].angle - MECHANICAL_LEG_LIMIT_ANGLE);
  chassis.leg_L.vmc.forward_kinematics.fk_phi.phi4 =
      -chassis.leg_L.cyber_gear_data[1].angle - MECHANICAL_LEG_LIMIT_ANGLE;
  chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1 =
      PI - (-chassis.leg_R.cyber_gear_data[0].angle - MECHANICAL_LEG_LIMIT_ANGLE);
  chassis.leg_R.vmc.forward_kinematics.fk_phi.phi4 =
      chassis.leg_R.cyber_gear_data[1].angle - MECHANICAL_LEG_LIMIT_ANGLE;
}

static void chassis_info_update() {
  chassis_imu_info_update();
  chassis_motor_info_update();
  chassis.mileage =
      (-chassis.leg_L.wheel.mileage + chassis.leg_R.wheel.mileage) / 2;//The state variable x should use this value
  if (chassis.chassis_move_speed_set_point.vx != 0 || get_rc_ctrl().rc.ch[CHASSIS_Z_CHANNEL] != 0) {
    chassis.mileage = 0;
    chassis.leg_L.wheel.mileage = 0;
    chassis.leg_R.wheel.mileage = 0;
  }
}

static void leg_state_variable_reference_get(struct Leg *leg) {
  if (leg == NULL) {
    return;
  }
  leg->state_variable_reference.theta_last = leg->state_variable_reference.theta;

  if (leg->leg_index == L) {
    leg->state_variable_reference.theta =
        cal_leg_theta(leg->vmc.forward_kinematics.fk_phi.phi0, -chassis.imu_reference.pitch_angle);
  } else if (leg->leg_index == R) {
    leg->state_variable_reference.theta =
        cal_leg_theta(leg->vmc.forward_kinematics.fk_phi.phi0, -chassis.imu_reference.pitch_angle);
  }

  leg->state_variable_reference.theta_dot_last = leg->state_variable_reference.theta_dot;
  leg->state_variable_reference.theta_dot =
      (leg->state_variable_reference.theta - leg->state_variable_reference.theta_last) / (CHASSIS_PERIOD * MILLISECOND_TO_SECOND);
  leg->state_variable_reference.theta_ddot =
      (leg->state_variable_reference.theta_dot - leg->state_variable_reference.theta_dot_last)
          / (CHASSIS_PERIOD * MILLISECOND_TO_SECOND);

  if (leg->leg_index == L) {
    leg->state_variable_reference.x = -leg->wheel.mileage;
    leg->state_variable_reference.x_dot = -leg->wheel.speed;
  } else if (leg->leg_index == R) {
    leg->state_variable_reference.x = leg->wheel.mileage;
    leg->state_variable_reference.x_dot = leg->wheel.speed;
  }

  if (leg->leg_index == L) {
    leg->state_variable_reference.phi = -chassis.imu_reference.pitch_angle;
    leg->state_variable_reference.phi_dot = -chassis.imu_reference.pitch_gyro;
  } else if (leg->leg_index == R) {
    leg->state_variable_reference.phi = -chassis.imu_reference.pitch_angle;
    leg->state_variable_reference.phi_dot = -chassis.imu_reference.pitch_gyro;
  }
  //todo 对位移处理，提高刹车性能
}

static void leg_state_variable_set_point_set(struct Leg *leg, fp32 vx) {
  if (leg == NULL) {
    return;
  }

  leg->state_variable_set_point.x = 0;
  leg->state_variable_set_point.x_dot = vx;
  leg->state_variable_set_point.theta = 0;
  leg->state_variable_set_point.theta_dot = 0;
  leg->state_variable_set_point.phi = chassis.imu_set_point.pitch;
  leg->state_variable_set_point.phi_dot = 0;
}

static void leg_state_variable_error_get(struct Leg *leg) {
  if (leg == NULL) {
    return;
  }

  leg->state_variable_error.x = leg->state_variable_reference.x - leg->state_variable_set_point.x;
  leg->state_variable_error.x_dot = leg->state_variable_reference.x_dot - leg->state_variable_set_point.x_dot;
  leg->state_variable_error.theta = leg->state_variable_reference.theta - leg->state_variable_set_point.theta;
  leg->state_variable_error.theta_dot =
      leg->state_variable_reference.theta_dot - leg->state_variable_set_point.theta_dot;
  leg->state_variable_error.phi = leg->state_variable_reference.phi - leg->state_variable_set_point.phi;
  leg->state_variable_error.phi_dot = leg->state_variable_reference.phi_dot - leg->state_variable_set_point.phi_dot;
}

static void wheel_motors_torque_set_point_cal(struct Leg *leg) {
  if (leg == NULL) {
    return;
  }
  if (leg->leg_index == L) {
    leg->wheel.torque = 0;
    leg->wheel.torque += leg->state_variable_error.theta * wheel_K_L[0];//
    leg->wheel.torque += leg->state_variable_error.theta_dot * wheel_K_L[1];//
    leg->wheel.torque += leg->state_variable_error.x * wheel_K_L[2];
    leg->wheel.torque += leg->state_variable_error.x_dot * wheel_K_L[3];
    leg->wheel.torque += leg->state_variable_error.phi * wheel_K_L[4];//
    leg->wheel.torque += leg->state_variable_error.phi_dot * wheel_K_L[5];
  } else if (leg->leg_index == R) {
    leg->wheel.torque = 0;
    leg->wheel.torque += leg->state_variable_error.theta * wheel_K_R[0];//
    leg->wheel.torque += leg->state_variable_error.theta_dot * wheel_K_R[1];//
    leg->wheel.torque += leg->state_variable_error.x * wheel_K_R[2];
    leg->wheel.torque += leg->state_variable_error.x_dot * wheel_K_R[3];
    leg->wheel.torque += leg->state_variable_error.phi * wheel_K_R[4];//
    leg->wheel.torque += leg->state_variable_error.phi_dot * wheel_K_R[5];
  }
  if (leg->leg_index == R) {
    leg->wheel.torque *= -1;
  }

  leg->wheel.torque += chassis.chassis_move_speed_set_point.vw;

  VAL_LIMIT(leg->wheel.torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);

//  if (leg->wheel.torque > 0) {
//    leg->wheel.torque += 0.05f;
//  } else {
//    leg->wheel.torque -= 0.05f;
//  }
}

static void joint_motors_torque_set_point_cal() {
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point = 0;
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point = 0;
//R
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_error.theta * joint_K_R[0];//
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_error.theta_dot * joint_K_R[1];//
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_error.x * joint_K_R[2];//
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_error.x_dot * joint_K_R[3];//
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_error.phi * joint_K_R[4];//
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_error.phi_dot * joint_K_R[5];//
//L
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_error.theta * joint_K_L[0];//
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_error.theta_dot * joint_K_L[1];//
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_error.x * joint_K_L[2];//
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_error.x_dot * joint_K_L[3];//
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_error.phi * joint_K_L[4];//
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_error.phi_dot * joint_K_L[5];//

  pid_calc(&chassis.leg_L.pid, chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, chassis.leg_L.L0_set_point);
  chassis.leg_L.vmc.Fxy_set_point.E.Fy_set_point = chassis.leg_L.pid.out + BODY_WEIGHT * GRAVITY_A * 0.5;

  pid_calc(&chassis.leg_R.pid, chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, chassis.leg_R.L0_set_point);
  chassis.leg_R.vmc.Fxy_set_point.E.Fy_set_point = chassis.leg_R.pid.out + BODY_WEIGHT * GRAVITY_A * 0.5;

  VMC_positive_dynamics(&chassis.leg_R.vmc);
  VMC_positive_dynamics(&chassis.leg_L.vmc);

  chassis.leg_L.cyber_gear_data[2].torque = chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point;
  chassis.leg_L.cyber_gear_data[0].torque = chassis.leg_L.vmc.T1_T4_set_point.E.T1_set_point;//F
  chassis.leg_L.cyber_gear_data[1].torque = chassis.leg_L.vmc.T1_T4_set_point.E.T4_set_point;//B

  chassis.leg_R.cyber_gear_data[2].torque = chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point;
  chassis.leg_R.cyber_gear_data[0].torque = chassis.leg_R.vmc.T1_T4_set_point.E.T1_set_point;//F
  chassis.leg_R.cyber_gear_data[1].torque = chassis.leg_R.vmc.T1_T4_set_point.E.T4_set_point;//B

  VAL_LIMIT(chassis.leg_R.cyber_gear_data[0].torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
  VAL_LIMIT(chassis.leg_R.cyber_gear_data[1].torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
  VAL_LIMIT(chassis.leg_L.cyber_gear_data[0].torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
  VAL_LIMIT(chassis.leg_L.cyber_gear_data[1].torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
}

static void chassis_motors_torque_set_point_cal(struct Leg *leg) {
  joint_motors_torque_set_point_cal();
  wheel_motors_torque_set_point_cal(leg);
}

static void vmc_inverse_solution(struct Leg *leg) {
  if (leg == NULL) {
    return;
  }
  Vmc_Negative_Kinematics(&leg->vmc, leg->cyber_gear_data[0].speed, leg->cyber_gear_data[1].speed);
  Vmc_Negative_Dynamics(&leg->vmc, leg->cyber_gear_data[0].torque, leg->cyber_gear_data[1].torque);
}

static void chassis_ctrl_info_get() {
  chassis.chassis_move_speed_set_point.vx = (float) (get_rc_ctrl().rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX;
  chassis.chassis_move_speed_set_point.vw = (float) (get_rc_ctrl().rc.ch[CHASSIS_Z_CHANNEL]) * -RC_TO_VW;
  chassis.imu_set_point.pitch = (float) (get_rc_ctrl().rc.ch[CHASSIS_PIT_CHANNEL]) * RC_TO_PITCH;
  chassis.leg_L.L0_set_point = -(float) (get_rc_ctrl().rc.ch[4]) * RC_TO_L0 + DEFAULT_L0;
  chassis.leg_R.L0_set_point = -(float) (get_rc_ctrl().rc.ch[4]) * RC_TO_L0 + DEFAULT_L0;

  if (chassis.chassis_move_speed_set_point.vw == 0) {
    chassis.chassis_move_speed_set_point.vw =
        pid_loop_calc(&chassis.chassis_vw_pid, chassis.imu_reference.yaw_angle, chassis.imu_set_point.yaw, PI, -PI);
  } else {
    chassis.imu_set_point.yaw = chassis.imu_reference.yaw_angle;
  }

  chassis.L0_delta = pid_calc(&chassis.chassis_roll_pid, chassis.imu_reference.roll_angle, 0.f);
  chassis.leg_L.L0_set_point -= chassis.L0_delta;
  chassis.leg_R.L0_set_point += chassis.L0_delta;

  VAL_LIMIT(chassis.leg_L.L0_set_point, MIN_L0, MAX_L0);
  VAL_LIMIT(chassis.leg_R.L0_set_point, MIN_L0, MAX_L0);

  if (switch_is_down(get_rc_ctrl().rc.s[RC_s_L]) && switch_is_down(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis.last_mode = chassis.mode;
    chassis.mode = CHASSIS_DISABLE;
  } else if (switch_is_mid(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis.last_mode = chassis.mode;
    chassis.mode = CHASSIS_ENABLED_LEG;
  } else if (switch_is_up(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis.last_mode = chassis.mode;
    chassis.mode = CHASSIS_ENABLED_LEG;
  }
}

void chassis_device_offline_handle() {
  if (detect_list[DETECT_REMOTE].status == OFFLINE) {
    chassis.mode = CHASSIS_DISABLE;
  }
}

struct Chassis get_chassis() {
  return chassis;
}

static void chassis_forward_kinematics() {
  //leg_L
  chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_x =
      cosf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1) * L1;
  chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_y =
      sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1) * L1;
  chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_x =
      cosf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi4) * L4
          + L5;
  chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_y =
      sinf(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi4) * L4;

  fp32 L_A0 = (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_x
      - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_x) * 2.f * L2;
  fp32 L_B0 = (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_y
      - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_y) * 2.f * L2;
  fp32 L_BD_sq = (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_x
      - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_x)
      * (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_x
          - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_x)
      + (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_y
          - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_y)
          * (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_y
              - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.b_y);
  fp32 L_C0 = L2 * L2 + L_BD_sq - L3 * L3;

  fp32 temp = L_A0 * L_A0 + L_B0 * L_B0 - L_C0 * L_C0;
  fp32 y = L_B0 + sqrtf(ABS(temp));
  fp32 x = L_A0 + L_C0;
  chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

  chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_x =
      L1 * cos(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1)
          + L2 * cos(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2);
  chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_y =
      L1 * sin(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1)
          + L2 * sin(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi2);
  y = chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_y
      - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_y;
  x = chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_x
      - chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.d_x;
  chassis.leg_L.vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

  temp = (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f)
      * (chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f)
      + chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_y
          * chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_y;
  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_last = chassis.leg_L.vmc.forward_kinematics.fk_L0.L0;
  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));
  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot_last = chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot;
  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot =
      (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 - chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_last)
          / (CHASSIS_PERIOD * MILLISECOND_TO_SECOND);
  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_ddot =
      (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot - chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot_last)
          / (CHASSIS_PERIOD * MILLISECOND_TO_SECOND);
  y = chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_y;
  x = chassis.leg_L.vmc.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f;
  chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);


  //leg_R
  chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_x =
      cosf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1) * L1;
  chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_y =
      sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1) * L1;
  chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_x =
      cosf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi4) * L4
          + L5;
  chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_y =
      sinf(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi4) * L4;

  fp32 R_A0 = (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_x
      - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_x) * 2.f * L2;
  fp32 R_B0 = (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_y
      - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_y) * 2.f * L2;
  fp32 R_BD_sq = (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_x
      - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_x)
      * (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_x
          - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_x)
      + (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_y
          - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_y)
          * (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_y
              - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.b_y);
  fp32 R_C0 = L2 * L2 + R_BD_sq - L3 * L3;

  temp = R_A0 * R_A0 + R_B0 * R_B0 - R_C0 * R_C0;
  y = R_B0 + sqrtf(ABS(temp));
  x = R_A0 + R_C0;
  chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

  chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_x =
      L1 * cos(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1)
          + L2 * cos(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2);
  chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_y =
      L1 * sin(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1)
          + L2 * sin(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi2);
  y = chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_y
      - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_y;
  x = chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_x
      - chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.d_x;
  chassis.leg_R.vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

  temp = (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f)
      * (chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f)
      + chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_y
          * chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_y;
  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_last = chassis.leg_R.vmc.forward_kinematics.fk_L0.L0;
  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));
  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot_last = chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot;
  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot =
      (chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 - chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_last)
          / (CHASSIS_PERIOD * MILLISECOND_TO_SECOND);
  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_ddot =
      (chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot - chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot_last)
          / (CHASSIS_PERIOD * MILLISECOND_TO_SECOND);
  y = chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_y;
  x = chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f;
  chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);
}

static void chassis_relax_judge() {
  if (ABS(chassis.imu_reference.pitch_angle) > 32) {
    chassis.mode = CHASSIS_DISABLE;
  }
}

static void chassis_off_ground_handle(struct Leg *leg) {
  if (leg->leg_index == L) {
    for (int i = 0; i < 6; i++) {
      wheel_K_L[i] = 0;
    }
    for (int i = 2; i < 6; i++) {
      joint_K_L[i] = 0;
    }
  } else if (leg->leg_index == R) {
    for (int i = 0; i < 6; i++) {
      wheel_K_R[i] = 0;
    }
    for (int i = 2; i < 6; i++) {
      joint_K_R[i] = 0;
    }
  }
}

static void chassis_off_ground_detection(struct Leg *leg) {
  if (leg->Fn <= 10) {
    chassis_off_ground_handle(leg);
  } else {
  }
}

static void chassis_motor_cmd_send() {

  CAN_cmd_motor(CAN_1,
                CAN_MOTOR_0x200_ID,
                chassis.leg_L.wheel.motor_3508.give_current,
                chassis.leg_R.wheel.motor_3508.give_current,
                0,
                0);

//  CAN_cmd_motor(CAN_1,
//                CAN_MOTOR_0x200_ID,
//                0,
//                0,
//                0,
//                0);

//  cyber_gear_control_mode(&cybergears_2[LF_MOTOR_ID], 0, 0, 0, 0, 0);
//  cyber_gear_control_mode(&cybergears_2[LB_MOTOR_ID], 0, 0, 0, 0, 0);
//  osDelay(2);
//  cyber_gear_control_mode(&cybergears_2[RB_MOTOR_ID], 0, 0, 0, 0, 0);
//  cyber_gear_control_mode(&cybergears_2[RF_MOTOR_ID], 0, 0, 0, 0, 0);

  cyber_gear_control_mode(&cybergears_2[LF_MOTOR_ID], chassis.leg_L.cyber_gear_data[0].torque, 0, 0, 0, 0);
  cyber_gear_control_mode(&cybergears_2[LB_MOTOR_ID], chassis.leg_L.cyber_gear_data[1].torque, 0, 0, 0, 0);
  osDelay(2);
  cyber_gear_control_mode(&cybergears_2[RB_MOTOR_ID], -chassis.leg_R.cyber_gear_data[1].torque, 0, 0, 0, 0);
  cyber_gear_control_mode(&cybergears_2[RF_MOTOR_ID], -chassis.leg_R.cyber_gear_data[0].torque, 0, 0, 0, 0);
  osDelay(2);

}

/*******************************************************************************
 *                                  Subtask                                   *
 *******************************************************************************/
static void chassis_enabled_leg_handle() {
  chassis_forward_kinematics();

  chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * 0.2f, wheel_K_L, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * 0.2f, joint_K_L, joint_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * 0.2f, wheel_K_R, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * 0.2f, joint_K_R, joint_fitting_factor);

  chassis_off_ground_detection(&chassis.leg_L);
  chassis_off_ground_detection(&chassis.leg_R);

  leg_state_variable_reference_get(&chassis.leg_L);
  leg_state_variable_reference_get(&chassis.leg_R);

  leg_state_variable_set_point_set(&chassis.leg_L, chassis.chassis_move_speed_set_point.vx);
  leg_state_variable_set_point_set(&chassis.leg_R, chassis.chassis_move_speed_set_point.vx);

  leg_state_variable_error_get(&chassis.leg_L);
  leg_state_variable_error_get(&chassis.leg_R);

  chassis_motors_torque_set_point_cal(&chassis.leg_L);
  chassis_motors_torque_set_point_cal(&chassis.leg_R);

  chassis.leg_L.wheel.motor_3508.give_current = -chassis.leg_L.wheel.torque * MOTOR_3508_TORQUE_TO_DATA;
  chassis.leg_R.wheel.motor_3508.give_current = -chassis.leg_R.wheel.torque * MOTOR_3508_TORQUE_TO_DATA;

  vmc_inverse_solution(&chassis.leg_L);
  vmc_inverse_solution(&chassis.leg_R);

  leg_fn_cal(&chassis.leg_L, chassis.imu_reference.robot_az);
  leg_fn_cal(&chassis.leg_R, chassis.imu_reference.robot_az);
}

static void chassis_relax_handle() {
  chassis.chassis_move_speed_set_point.vx = 0;
  chassis.chassis_move_speed_set_point.vw = 0;

  chassis.mileage = 0;
  chassis.leg_L.wheel.mileage = 0;
  chassis.leg_R.wheel.mileage = 0;

  chassis.leg_R.cyber_gear_data[0].torque = 0;
  chassis.leg_R.cyber_gear_data[1].torque = 0;
  chassis.leg_L.cyber_gear_data[0].torque = 0;
  chassis.leg_L.cyber_gear_data[1].torque = 0;

  chassis.leg_L.wheel.motor_3508.give_current = 0;
  chassis.leg_R.wheel.motor_3508.give_current = 0;

  chassis.leg_L.L0_set_point = DEFAULT_L0;
  chassis.leg_R.L0_set_point = DEFAULT_L0;

  chassis.imu_set_point.yaw = chassis.imu_reference.yaw_angle;
}

/*******************************************************************************
 *                                Control Loop                                 *
 *******************************************************************************/
void chassis_task(void const *pvParameters) {

  vTaskDelay(CHASSIS_TASK_INIT_TIME);

  chassis_init(&chassis);

  TickType_t last_wake_time = xTaskGetTickCount();
  while (1) {
    chassis_info_update();

    chassis_ctrl_info_get();

//    chassis_relax_judge();

    chassis_device_offline_handle();

    switch (chassis.mode) {
      case CHASSIS_ENABLED_LEG: {
        chassis_enabled_leg_handle();
      }
        break;

      case CHASSIS_DISABLE: {
        chassis_relax_handle();
      }
        break;
    }

    chassis_motor_cmd_send();

    vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
  }
}

//todo
// roll automatic stabilization
// Turning and tilting body