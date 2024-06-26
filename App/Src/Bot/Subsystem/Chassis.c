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
#include "bsp_buzzer.h"
#define DUBUG 0

struct Chassis chassis;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
uint8_t rc_sw_R_last;

fp32 init_wheel_K_L[6] = {-3.462473, -0.275274, -0.240808, -0.290170, 0.240408, 0.050661};
fp32 init_wheel_K_R[6] = {-3.462473, -0.275274, -0.240808, -0.290170, 0.240408, 0.050661};
fp32 init_joint_K_L[6] = {0.700426, 0.038837, 0.050144, 0.055549, 0.805145, 0.176428};
fp32 init_joint_K_R[6] = {0.700426, 0.038837, 0.050144, 0.055549, 0.805145, 0.176428};

fp32 wheel_K_L[6] = {0, 0, 0, 0, 0, 0};
fp32 joint_K_L[6] = {0, 0, 0, 0, 0, 0};

fp32 wheel_K_R[6] = {0, 0, 0, 0, 0, 0};
fp32 joint_K_R[6] = {0, 0, 0, 0, 0, 0};

fp32 wheel_fitting_factor[6][4] = {
    {-1011.973162, 205.891794, -51.990130, -0.816575},
    {295.819732, -49.757780, -1.156901, -0.277667},

    {1325.935878, -187.488799, 4.987205, -0.668164},
    {3373.101743, -480.871683, 20.463444, -1.015208},

    {1024.651547, -314.622321, -8.563741, 6.561355},
    {-66.639272, -3.930818, -2.022788, 0.637302}
};
fp32 joint_fitting_factor[6][4] = {
    {39108.037835, -7801.811178, 506.614735, 8.438491},
    {455.503233, -227.278755, 7.148530, 3.767256},

    {7213.337382, -1226.907642, 34.055066, 6.457887},
    {-15935.813908, 2232.077974, -156.473365, 10.634943},

    {-32334.636763, 4317.031008, 104.682186, 31.191821},
    {-1969.728767, 248.478883, 17.458928, 2.255648}
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
  osDelay(10);
  cyber_gear_enable(&cybergears_2[LB_MOTOR_ID]);
  osDelay(10);
  cyber_gear_enable(&cybergears_2[RB_MOTOR_ID]);
  osDelay(10);
  cyber_gear_enable(&cybergears_2[RF_MOTOR_ID]);
  osDelay(10);

  pid_init(&chassis->leg_L.ground_pid,
           CHASSIS_LEG_L0_PID_OUT_LIMIT,
           CHASSIS_LEG_L0_PID_IOUT_LIMIT,
           CHASSIS_LEG_LO_PID_P,
           CHASSIS_LEG_L0_PID_I,
           CHASSIS_LEG_L0_PID_D);

  pid_init(&chassis->leg_R.ground_pid,
           CHASSIS_LEG_L0_PID_OUT_LIMIT,
           CHASSIS_LEG_L0_PID_IOUT_LIMIT,
           CHASSIS_LEG_LO_PID_P,
           CHASSIS_LEG_L0_PID_I,
           CHASSIS_LEG_L0_PID_D);

  pid_init(&chassis->leg_R.offground_pid,
           CHASSIS_OFFGROUND_LEG_L0_PID_OUT_LIMIT,
           CHASSIS_OFFGROUND_LEG_L0_PID_IOUT_LIMIT,
           CHASSIS_OFFGROUND_LEG_LO_PID_P,
           CHASSIS_OFFGROUND_LEG_L0_PID_I,
           CHASSIS_OFFGROUND_LEG_L0_PID_D);

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

  pid_init(&chassis->chassis_leg_coordination_pid,
           CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
           CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
           CHASSIS_LEG_COORDINATION_PID_P,
           CHASSIS_LEG_COORDINATION_PID_I,
           CHASSIS_LEG_COORDINATION_PID_D);

  chassis->leg_L.leg_flag.IMPACT_FLAG = 0;
  chassis->leg_R.leg_flag.IMPACT_FLAG = 0;

  chassis->jump_flag.jump_completed = 1;
  chassis->jump_flag.offground = 1;
  chassis->init_flag = false;
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
      - GRAVITY_A * cosf(chassis.imu_reference.pitch_angle) *
          sinf(chassis.imu_reference.roll_angle);
  chassis.imu_reference.az_filtered = chassis.imu_reference.az
      - GRAVITY_A * cosf(chassis.imu_reference.pitch_angle) *
          cosf(chassis.imu_reference.roll_angle);
  chassis.imu_reference.robot_az = chassis.imu_reference.ax_filtered * sinf(chassis.imu_reference.pitch_angle)
      + chassis.imu_reference.ay_filtered * sinf(-chassis.imu_reference.roll_angle)
          * cosf(chassis.imu_reference.pitch_angle)
      + chassis.imu_reference.az_filtered * cosf(chassis.imu_reference.pitch_angle)
          * cosf(chassis.imu_reference.roll_angle);
}

static void chassis_motor_info_update() {
  chassis.leg_L.wheel.speed = -motor_3508_measure[0].speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  chassis.leg_R.wheel.speed = -motor_3508_measure[1].speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  chassis.leg_L.wheel.torque_reference = -motor_3508_measure[0].given_current / MOTOR_3508_TORQUE_TO_DATA;
  chassis.leg_R.wheel.torque_reference = -motor_3508_measure[1].given_current / MOTOR_3508_TORQUE_TO_DATA;
  chassis.leg_L.wheel.mileage =
      chassis.leg_L.wheel.mileage + CHASSIS_PERIOD * MILLISECOND_TO_SECOND * (chassis.leg_L.wheel.speed);
  chassis.leg_R.wheel.mileage =
      chassis.leg_R.wheel.mileage + CHASSIS_PERIOD * MILLISECOND_TO_SECOND * (chassis.leg_R.wheel.speed);

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
  chassis.chassis_move_speed_reference.vx =
      ((-chassis.leg_L.wheel.speed) + (chassis.leg_R.wheel.speed)) / 2;
  chassis.mileage =
      (-chassis.leg_L.wheel.mileage + chassis.leg_R.wheel.mileage) /
          2;//The state variable x should use this value
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
      (leg->state_variable_reference.theta - leg->state_variable_reference.theta_last)
          / (CHASSIS_PERIOD * MILLISECOND_TO_SECOND);
  leg->state_variable_reference.theta_ddot =
      (leg->state_variable_reference.theta_dot - leg->state_variable_reference.theta_dot_last)
          / (CHASSIS_PERIOD * MILLISECOND_TO_SECOND);

  if (leg->leg_index == L) {
    leg->state_variable_reference.x = -leg->wheel.mileage;
    leg->state_variable_reference.x_dot_last = leg->state_variable_reference.x_dot;
    leg->state_variable_reference.x_dot = -leg->wheel.speed;
    leg->state_variable_reference.x_ddot =
        (leg->state_variable_reference.x_dot - leg->state_variable_reference.x_dot_last)
            / (CHASSIS_PERIOD * MILLISECOND_TO_SECOND);
  } else if (leg->leg_index == R) {
    leg->state_variable_reference.x = leg->wheel.mileage;
    leg->state_variable_reference.x_dot_last = leg->state_variable_reference.x_dot;
    leg->state_variable_reference.x_dot = leg->wheel.speed;
    leg->state_variable_reference.x_ddot =
        (leg->state_variable_reference.x_dot - leg->state_variable_reference.x_dot_last)
            / (CHASSIS_PERIOD * MILLISECOND_TO_SECOND);
  }

  if (leg->leg_index == L) {
    leg->state_variable_reference.phi = -chassis.imu_reference.pitch_angle;
    leg->state_variable_reference.phi_dot = -chassis.imu_reference.pitch_gyro;
  } else if (leg->leg_index == R) {
    leg->state_variable_reference.phi = -chassis.imu_reference.pitch_angle;
    leg->state_variable_reference.phi_dot = -chassis.imu_reference.pitch_gyro;
  }
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
//  VAL_LIMIT(leg->state_variable_error.x, -0.1, 0.1);
  leg->state_variable_error.x_dot = leg->state_variable_reference.x_dot - leg->state_variable_set_point.x_dot;
  leg->state_variable_error.theta = leg->state_variable_reference.theta - leg->state_variable_set_point.theta;
  leg->state_variable_error.theta_dot =
      leg->state_variable_reference.theta_dot - leg->state_variable_set_point.theta_dot;
  leg->state_variable_error.phi = leg->state_variable_reference.phi - leg->state_variable_set_point.phi;
  leg->state_variable_error.phi_dot = leg->state_variable_reference.phi_dot - leg->state_variable_set_point.phi_dot;
}

static void leg_state_variable_out_get(struct Leg *leg) {
  if (leg == NULL) {
    return;
  }
  if (chassis.mode == CHASSIS_INIT) {
    if (leg->leg_index == L) {
      leg->state_variable_wheel_out.theta = leg->state_variable_error.theta * init_wheel_K_L[0];
      leg->state_variable_wheel_out.theta_dot = leg->state_variable_error.theta_dot * init_wheel_K_L[1];
      leg->state_variable_wheel_out.x = leg->state_variable_error.x * init_wheel_K_L[2];
      leg->state_variable_wheel_out.x_dot = leg->state_variable_error.x_dot * init_wheel_K_L[3];
      leg->state_variable_wheel_out.phi = leg->state_variable_error.phi * init_wheel_K_L[4];
      leg->state_variable_wheel_out.phi_dot = leg->state_variable_error.phi_dot * init_wheel_K_L[5];

      leg->state_variable_joint_out.theta = leg->state_variable_error.theta * init_joint_K_L[0];
      leg->state_variable_joint_out.theta_dot = leg->state_variable_error.theta_dot * init_joint_K_L[1];
      leg->state_variable_joint_out.x = leg->state_variable_error.x * init_joint_K_L[2];
      leg->state_variable_joint_out.x_dot = leg->state_variable_error.x_dot * init_joint_K_L[3];
      leg->state_variable_joint_out.phi = leg->state_variable_error.phi * init_joint_K_L[4];
      leg->state_variable_joint_out.phi_dot = leg->state_variable_error.phi_dot * init_joint_K_L[5];
    } else if (leg->leg_index == R) {
      leg->state_variable_wheel_out.theta = leg->state_variable_error.theta * init_wheel_K_R[0];
      leg->state_variable_wheel_out.theta_dot = leg->state_variable_error.theta_dot * init_wheel_K_R[1];
      leg->state_variable_wheel_out.x = leg->state_variable_error.x * init_wheel_K_R[2];
      leg->state_variable_wheel_out.x_dot = leg->state_variable_error.x_dot * init_wheel_K_R[3];
      leg->state_variable_wheel_out.phi = leg->state_variable_error.phi * init_wheel_K_R[4];
      leg->state_variable_wheel_out.phi_dot = leg->state_variable_error.phi_dot * init_wheel_K_R[5];

      leg->state_variable_joint_out.theta = leg->state_variable_error.theta * init_joint_K_R[0];
      leg->state_variable_joint_out.theta_dot = leg->state_variable_error.theta_dot * init_joint_K_R[1];
      leg->state_variable_joint_out.x = leg->state_variable_error.x * init_joint_K_R[2];
      leg->state_variable_joint_out.x_dot = leg->state_variable_error.x_dot * init_joint_K_R[3];
      leg->state_variable_joint_out.phi = leg->state_variable_error.phi * init_joint_K_R[4];
      leg->state_variable_joint_out.phi_dot = leg->state_variable_error.phi_dot * init_joint_K_R[5];
    }
  } else {
    if (leg->leg_index == L) {
      leg->state_variable_wheel_out.theta = leg->state_variable_error.theta * wheel_K_L[0];
      leg->state_variable_wheel_out.theta_dot = leg->state_variable_error.theta_dot * wheel_K_L[1];
      leg->state_variable_wheel_out.x = leg->state_variable_error.x * wheel_K_L[2];
      leg->state_variable_wheel_out.x_dot = leg->state_variable_error.x_dot * wheel_K_L[3];
      leg->state_variable_wheel_out.phi = leg->state_variable_error.phi * wheel_K_L[4];
      leg->state_variable_wheel_out.phi_dot = leg->state_variable_error.phi_dot * wheel_K_L[5];

      leg->state_variable_joint_out.theta = leg->state_variable_error.theta * joint_K_L[0];
      leg->state_variable_joint_out.theta_dot = leg->state_variable_error.theta_dot * joint_K_L[1];
      leg->state_variable_joint_out.x = leg->state_variable_error.x * joint_K_L[2];
      leg->state_variable_joint_out.x_dot = leg->state_variable_error.x_dot * joint_K_L[3];
      leg->state_variable_joint_out.phi = leg->state_variable_error.phi * joint_K_L[4];
      leg->state_variable_joint_out.phi_dot = leg->state_variable_error.phi_dot * joint_K_L[5];
    } else if (leg->leg_index == R) {
      leg->state_variable_wheel_out.theta = leg->state_variable_error.theta * wheel_K_R[0];
      leg->state_variable_wheel_out.theta_dot = leg->state_variable_error.theta_dot * wheel_K_R[1];
      leg->state_variable_wheel_out.x = leg->state_variable_error.x * wheel_K_R[2];
      leg->state_variable_wheel_out.x_dot = leg->state_variable_error.x_dot * wheel_K_R[3];
      leg->state_variable_wheel_out.phi = leg->state_variable_error.phi * wheel_K_R[4];
      leg->state_variable_wheel_out.phi_dot = leg->state_variable_error.phi_dot * wheel_K_R[5];

      leg->state_variable_joint_out.theta = leg->state_variable_error.theta * joint_K_R[0];
      leg->state_variable_joint_out.theta_dot = leg->state_variable_error.theta_dot * joint_K_R[1];
      leg->state_variable_joint_out.x = leg->state_variable_error.x * joint_K_R[2];
      leg->state_variable_joint_out.x_dot = leg->state_variable_error.x_dot * joint_K_R[3];
      leg->state_variable_joint_out.phi = leg->state_variable_error.phi * joint_K_R[4];
      leg->state_variable_joint_out.phi_dot = leg->state_variable_error.phi_dot * joint_K_R[5];
    }
  }

//  //wheels
//  VAL_LIMIT(leg->state_variable_wheel_out.theta, -WHEEL_THETA_LIMIT, WHEEL_THETA_LIMIT);
//  VAL_LIMIT(leg->state_variable_wheel_out.theta_dot, -WHEEL_THETA_DOT_LIMIT, WHEEL_THETA_DOT_LIMIT);
////  VAL_LIMIT(leg->state_variable_wheel_out.x, -WHEEL_X_LIMIT, WHEEL_X_LIMIT);
//  VAL_LIMIT(leg->state_variable_wheel_out.x_dot, -WHEEL_X_DOT_LIMIT, WHEEL_X_DOT_LIMIT);
//  VAL_LIMIT(leg->state_variable_wheel_out.phi, -WHEEL_PHI_LIMIT, WHEEL_PHI_LIMIT);
//  VAL_LIMIT(leg->state_variable_wheel_out.phi_dot, -WHEEL_PHI_DOT_LIMIT, WHEEL_PHI_DOT_LIMIT);
//
//  //joints
//  VAL_LIMIT(leg->state_variable_joint_out.theta, -JOINT_THETA_LIMIT, JOINT_THETA_LIMIT);
//  VAL_LIMIT(leg->state_variable_joint_out.theta_dot, -JOINT_THETA_DOT_LIMIT, JOINT_THETA_DOT_LIMIT);
////  VAL_LIMIT(leg->state_variable_joint_out.x, -JOINT_X_LIMIT, JOINT_X_LIMIT);
//  VAL_LIMIT(leg->state_variable_joint_out.x_dot, -JOINT_X_DOT_LIMIT, JOINT_X_DOT_LIMIT);
//  VAL_LIMIT(leg->state_variable_joint_out.phi, -JOINT_PHI_LIMIT, JOINT_PHI_LIMIT);
//  VAL_LIMIT(leg->state_variable_joint_out.phi_dot, -JOINT_PHI_DOT_LIMIT, JOINT_PHI_DOT_LIMIT);
}

static void wheel_motors_torque_set_point_cal(struct Leg *leg) {
  if (leg == NULL) {
    return;
  }
  if (leg->leg_index == L) {
    leg->wheel.torque = 0;
    leg->wheel.torque += leg->state_variable_wheel_out.theta;//
    leg->wheel.torque += leg->state_variable_wheel_out.theta_dot;//
    leg->wheel.torque += leg->state_variable_wheel_out.x;
    leg->wheel.torque += leg->state_variable_wheel_out.x_dot;
    leg->wheel.torque += leg->state_variable_wheel_out.phi;//
    leg->wheel.torque += leg->state_variable_wheel_out.phi_dot;
  } else if (leg->leg_index == R) {
    leg->wheel.torque = 0;
    leg->wheel.torque += leg->state_variable_wheel_out.theta;
    leg->wheel.torque += leg->state_variable_wheel_out.theta_dot;
    leg->wheel.torque += leg->state_variable_wheel_out.x;
    leg->wheel.torque += leg->state_variable_wheel_out.x_dot;
    leg->wheel.torque += leg->state_variable_wheel_out.phi;
    leg->wheel.torque += leg->state_variable_wheel_out.phi_dot;
  }
  if (leg->leg_index == R) {
    leg->wheel.torque *= -1;
  }

  leg->wheel.torque += chassis.chassis_move_speed_set_point.vw;
  if (chassis.jump_state == STRETCHING || chassis.jump_state == SHRINKING || chassis.jump_state == STRETCHING_AGAIN
      || chassis.jump_state == FALLING) {
    leg->wheel.torque = 0;
  }

  VAL_LIMIT(leg->wheel.torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);

}

static void joint_motors_torque_set_point_cal() {
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point = 0;
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point = 0;
//R
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_joint_out.theta;
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_joint_out.theta_dot;
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_joint_out.x;
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_joint_out.x_dot;
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_joint_out.phi;
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_R.state_variable_joint_out.phi_dot;
  chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;

//L
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_joint_out.theta;
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_joint_out.theta_dot;
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_joint_out.x;
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_joint_out.x_dot;
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_joint_out.phi;
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis.leg_L.state_variable_joint_out.phi_dot;
  chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;

  pid_calc(&chassis.leg_L.ground_pid, chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, chassis.leg_L.L0_set_point);
  chassis.leg_L.vmc.Fxy_set_point.E.Fy_set_point = chassis.leg_L.ground_pid.out + BODY_WEIGHT * GRAVITY_A * 0.5;

  pid_calc(&chassis.leg_R.ground_pid, chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, chassis.leg_R.L0_set_point);
  chassis.leg_R.vmc.Fxy_set_point.E.Fy_set_point = chassis.leg_R.ground_pid.out + BODY_WEIGHT * GRAVITY_A * 0.5;

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
//    chassis.chassis_move_speed_set_point.vw = (float) (get_rc_ctrl().rc.ch[CHASSIS_Z_CHANNEL]) * -RC_TO_VW;
  chassis.imu_set_point.yaw -= (float) (get_rc_ctrl().rc.ch[CHASSIS_Z_CHANNEL]) * -RC_TO_YAW_INCREMENT;
  if (chassis.imu_set_point.yaw >= PI) {
    chassis.imu_set_point.yaw -= 2 * PI;
  } else if (chassis.imu_set_point.yaw <= -PI) {
    chassis.imu_set_point.yaw += 2 * PI;
  }

  chassis.imu_set_point.pitch = (float) (get_rc_ctrl().rc.ch[CHASSIS_PIT_CHANNEL]) * RC_TO_PITCH;
  chassis.imu_set_point.roll = (float) (get_rc_ctrl().rc.ch[CHASSIS_ROLL_CHANNEL]) * RC_TO_ROLL;
  chassis.leg_L.L0_set_point = -(float) (get_rc_ctrl().rc.ch[4]) * RC_TO_L0 + DEFAULT_L0;
  chassis.leg_R.L0_set_point = -(float) (get_rc_ctrl().rc.ch[4]) * RC_TO_L0 + DEFAULT_L0;

  chassis.chassis_move_speed_set_point.vw =
      pid_loop_calc(&chassis.chassis_vw_pid, chassis.imu_reference.yaw_angle, chassis.imu_set_point.yaw, PI,
                    -PI);
  chassis.phi_0_error =
      chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0;
  chassis.steer_compensatory_torque = pid_calc(&chassis.chassis_leg_coordination_pid, chassis.phi_0_error, 0);

  if (chassis.chassis_move_speed_set_point.vx != 0) {
    chassis.imu_set_point.roll =
        atan2f(chassis.chassis_move_speed_reference.vx * chassis.imu_reference.yaw_gyro, GRAVITY_A);
    VAL_LIMIT(chassis.imu_set_point.roll, MIN_ROLL, MAX_ROLL);
  }
  chassis.L0_delta = -pid_calc(&chassis.chassis_roll_pid, chassis.imu_reference.roll_angle,
                               chassis.imu_set_point.roll);

  if (chassis.leg_L.leg_flag.OFF_GROUND_FLAG == 1) {
    chassis.chassis_move_speed_set_point.vw = 0;
    chassis.imu_set_point.yaw = chassis.imu_reference.yaw_angle;
    chassis.imu_set_point.roll = chassis.imu_reference.roll_angle;
  } else {
    chassis.leg_L.L0_set_point -= chassis.L0_delta;
  }

  if (chassis.leg_R.leg_flag.OFF_GROUND_FLAG == 1) {
    chassis.chassis_move_speed_set_point.vw = 0;
    chassis.imu_set_point.yaw = chassis.imu_reference.yaw_angle;
    chassis.imu_set_point.roll = chassis.imu_reference.roll_angle;
  } else {
    chassis.leg_R.L0_set_point += chassis.L0_delta;
  }

  VAL_LIMIT(chassis.leg_L.L0_set_point, MIN_L0, MAX_L0);
  VAL_LIMIT(chassis.leg_R.L0_set_point, MIN_L0, MAX_L0);

  if (switch_is_down(get_rc_ctrl().rc.s[RC_s_R])) {
    chassis.last_mode = chassis.mode;
    chassis.mode = CHASSIS_DISABLE;
  } else if (switch_is_mid(get_rc_ctrl().rc.s[RC_s_R]) && chassis.init_flag == false) {
    chassis.mode = CHASSIS_INIT;
  } else if (switch_is_mid(get_rc_ctrl().rc.s[RC_s_R]) && chassis.init_flag == true) {
    chassis.last_mode = chassis.mode;
    chassis.mode = CHASSIS_ENABLED_LEG;
    chassis.jump_state = NOT_READY;
  } else if ((switch_is_mid(rc_sw_R_last) && switch_is_up(get_rc_ctrl().rc.s[RC_s_R]))) {
    chassis.last_mode = chassis.mode;
    chassis.jump_state = READY;
  }
  rc_sw_R_last = get_rc_ctrl().rc.s[RC_s_R];
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
      joint_K_L[i] *= 0.4f;
    }
    for (int i = 0; i < 6; i++) {
      wheel_K_L[i] = 0;
    }
    for (int i = 2; i < 6; i++) {
      joint_K_L[i] = 0;
    }
  } else if (leg->leg_index == R) {
    for (int i = 0; i < 6; i++) {
      joint_K_R[i] *= 0.4f;
    }
    for (int i = 0; i < 6; i++) {
      wheel_K_R[i] = 0;
    }
    for (int i = 2; i < 6; i++) {
      joint_K_R[i] = 0;
    }
  }
  leg->state_variable_reference.x = 0;
}

static void chassis_off_ground_detection(struct Leg *leg) {
  if (leg->Fn <= 3) {
    chassis_off_ground_handle(leg);
    leg->leg_flag.OFF_GROUND_FLAG = 1;

    buzzer_on(25, 10000);
  } else {
    leg->leg_flag.OFF_GROUND_FLAG = 0;
    buzzer_off();
  }
}

static void chassis_impact_handle(struct Leg *leg) {
  if (leg == NULL) {
    return;
  }

  if (leg->leg_flag.IMPACT_FLAG == 1) {
    leg->ground_pid.p = 1500;
  } else {
    leg->ground_pid.p = CHASSIS_LEG_LO_PID_P;
  }
}

static void chassis_impact_detection(struct Leg *leg) {
  if (leg == NULL) {
    return;
  }

  if (leg->Fn > 30) {
    leg->leg_flag.IMPACT_FLAG = 1;
    buzzer_on(5, 10000);
  }

  if (leg->leg_flag.IMPACT_FLAG == 1 && leg->Fn <= 15) {
    leg->leg_flag.IMPACT_FLAG = 0;
    buzzer_off();
  }

  chassis_impact_handle(leg);
}

static void chassis_motor_cmd_send() {

#if DUBUG
  CAN_cmd_motor(CAN_1,
                CAN_MOTOR_0x200_ID,
                0,
                0,
                0,
                0);

  cyber_gear_control_mode(&cybergears_2[LF_MOTOR_ID], 0, 0, 0, 0, 0);
  cyber_gear_control_mode(&cybergears_2[LB_MOTOR_ID], 0, 0, 0, 0, 0);
  osDelay(2);
  cyber_gear_control_mode(&cybergears_2[RB_MOTOR_ID], 0, 0, 0, 0, 0);
  cyber_gear_control_mode(&cybergears_2[RF_MOTOR_ID], 0, 0, 0, 0, 0);
#else
  CAN_cmd_motor(CAN_1,
                CAN_MOTOR_0x200_ID,
                chassis.leg_L.wheel.motor_3508.give_current,
                chassis.leg_R.wheel.motor_3508.give_current,
                0,
                0);

  cyber_gear_control_mode(&cybergears_2[LF_MOTOR_ID], chassis.leg_L.cyber_gear_data[0].torque, 0, 0, 0, 0);
  cyber_gear_control_mode(&cybergears_2[LB_MOTOR_ID], chassis.leg_L.cyber_gear_data[1].torque, 0, 0, 0, 0);
  osDelay(2);
  cyber_gear_control_mode(&cybergears_2[RB_MOTOR_ID], -chassis.leg_R.cyber_gear_data[1].torque, 0, 0, 0, 0);
  cyber_gear_control_mode(&cybergears_2[RF_MOTOR_ID], -chassis.leg_R.cyber_gear_data[0].torque, 0, 0, 0, 0);
  osDelay(2);
#endif

}

/*******************************************************************************
 *                                  Subtask                                   *
 *******************************************************************************/
static void chassis_init_handle() {
  if (is_chassis_leg_return_to_original_position(&chassis)) {
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

    leg_state_variable_out_get(&chassis.leg_L);
    leg_state_variable_out_get(&chassis.leg_R);

    chassis_motors_torque_set_point_cal(&chassis.leg_L);
    chassis_motors_torque_set_point_cal(&chassis.leg_R);

    chassis.leg_L.wheel.motor_3508.give_current = -chassis.leg_L.wheel.torque * MOTOR_3508_TORQUE_TO_DATA;
    chassis.leg_R.wheel.motor_3508.give_current = -chassis.leg_R.wheel.torque * MOTOR_3508_TORQUE_TO_DATA;

    vmc_inverse_solution(&chassis.leg_L);
    vmc_inverse_solution(&chassis.leg_R);

    leg_fn_cal(&chassis.leg_L, chassis.imu_reference.robot_az);
    leg_fn_cal(&chassis.leg_R, chassis.imu_reference.robot_az);

    if (is_chassis_phi_stable(&chassis.imu_reference)) {
      chassis.init_flag = true;
    } else {
      chassis.leg_L.cyber_gear_data[0].torque = 0;
      chassis.leg_L.cyber_gear_data[1].torque = 0;
      chassis.leg_R.cyber_gear_data[0].torque = 0;
      chassis.leg_R.cyber_gear_data[1].torque = 0;
    }
  } else {
    cyber_gear_control_mode(&cybergears_2[LF_MOTOR_ID], 0, 0, 0, 5, 1);
    osDelay(1);
    cyber_gear_control_mode(&cybergears_2[LB_MOTOR_ID], 0, 0, 0, 5, 1);
    osDelay(1);
    cyber_gear_control_mode(&cybergears_2[RB_MOTOR_ID], 0, 0, 0, 5, 1);
    osDelay(1);
    cyber_gear_control_mode(&cybergears_2[RF_MOTOR_ID], 0, 0, 0, 5, 1);
  }
}

static void chassis_jump_handle() {
  switch (chassis.jump_state) {
    case READY:chassis.leg_L.L0_set_point = 0.1;//todo 保存当前腿长，以免在两腿不同长度下跳跃出错
      chassis.leg_R.L0_set_point = 0.1;
      chassis.leg_L.ground_pid.p = 200;
      chassis.leg_R.ground_pid.p = 200;

      if ((ABS(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot) <= 0.03f
          && ABS(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 - chassis.leg_L.L0_set_point) <= 0.07)
          && (ABS(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot) <= 0.03f
              && ABS(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 - chassis.leg_R.L0_set_point) <= 0.07f)) {
        buzzer_on(5, 1000);
        if (switch_is_mid(get_rc_ctrl().rc.s[RC_s_L])) {
          chassis.jump_state = STRETCHING;
          chassis.jump_flag.offground = 0;
        }
      }
      break;

    case STRETCHING:chassis.leg_L.L0_set_point = 0.23f;
      chassis.leg_R.L0_set_point = 0.23f;

      chassis.leg_L.ground_pid.p = 100000;
      chassis.leg_R.ground_pid.p = 100000;

      if (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 > 0.18
          && chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 > 0.18) {
        chassis.jump_state = SHRINKING;
      }

      chassis.jump_flag.offground = 1;

      break;

    case SHRINKING:chassis.leg_L.L0_set_point = 0.15f;
      chassis.leg_R.L0_set_point = 0.15f;

      chassis.leg_L.ground_pid.p = 1800;
      chassis.leg_L.ground_pid.d = 1800;
      chassis.leg_R.ground_pid.p = 1800;
      chassis.leg_R.ground_pid.d = 1800;

      if (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 < 0.15
          && chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 < 0.15) {
        chassis.jump_state = STRETCHING_AGAIN;
      }

      chassis.jump_flag.offground = 1;

      break;

    case STRETCHING_AGAIN:chassis.leg_L.L0_set_point = DEFAULT_L0;
      chassis.leg_R.L0_set_point = DEFAULT_L0;

      chassis.leg_L.ground_pid.p = 2000;
      chassis.leg_R.ground_pid.p = 2000;

      if (chassis.imu_reference.robot_az <= -5) {
        chassis.jump_state = FALLING;
      }

      chassis.jump_flag.offground = 1;

      break;

    case FALLING:chassis.leg_L.L0_set_point = DEFAULT_L0;
      chassis.leg_R.L0_set_point = DEFAULT_L0;

      chassis.leg_L.ground_pid.p = 200;
      chassis.leg_R.ground_pid.p = 200;

      if ((chassis.leg_R.Fn >= 18 || chassis.leg_L.Fn >= 18) && chassis.imu_reference.robot_az >= 10) {
        chassis.jump_state = LANDING;
      }

      chassis.jump_flag.offground = 1;

      break;

    case LANDING:

      chassis.jump_flag.jump_completed = 1;
      break;

    default:break;
  }

}

static void chassis_enabled_leg_handle() {
  chassis_forward_kinematics();

  chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * 0.2f, wheel_K_L, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * 0.2f, joint_K_L, joint_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * 0.2f, wheel_K_R, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * 0.2f, joint_K_R, joint_fitting_factor);

  if (chassis.jump_flag.offground == 1) {
    chassis_off_ground_detection(&chassis.leg_L);
    chassis_off_ground_detection(&chassis.leg_R);
  }

  chassis_jump_handle();

//  chassis_impact_detection(&chassis.leg_L);
//  chassis_impact_detection(&chassis.leg_R);

  leg_state_variable_reference_get(&chassis.leg_L);
  leg_state_variable_reference_get(&chassis.leg_R);

  leg_state_variable_set_point_set(&chassis.leg_L, chassis.chassis_move_speed_set_point.vx);
  leg_state_variable_set_point_set(&chassis.leg_R, chassis.chassis_move_speed_set_point.vx);

  leg_state_variable_error_get(&chassis.leg_L);
  leg_state_variable_error_get(&chassis.leg_R);

  leg_state_variable_out_get(&chassis.leg_L);
  leg_state_variable_out_get(&chassis.leg_R);

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

  chassis.init_flag = false;

  buzzer_off();

  pid_reset(&chassis.chassis_roll_pid, CHASSIS_ROLL_PID_P, CHASSIS_ROLL_PID_I, CHASSIS_ROLL_PID_D);
  pid_reset(&chassis.chassis_vw_pid, CHASSIS_VW_PID_P, CHASSIS_VW_PID_I, CHASSIS_VW_PID_D);
  pid_reset(&chassis.chassis_leg_coordination_pid,
            CHASSIS_LEG_COORDINATION_PID_P,
            CHASSIS_LEG_COORDINATION_PID_I,
            CHASSIS_LEG_COORDINATION_PID_D);

  mi_motor_clear_err(&cybergears_2[LF_MOTOR_ID]);
  osDelay(1);
  mi_motor_clear_err(&cybergears_2[LB_MOTOR_ID]);
  osDelay(1);
  mi_motor_clear_err(&cybergears_2[RB_MOTOR_ID]);
  osDelay(1);
  mi_motor_clear_err(&cybergears_2[RF_MOTOR_ID]);

  cyber_gear_enable(&cybergears_2[LF_MOTOR_ID]);
  osDelay(1);
  cyber_gear_enable(&cybergears_2[LB_MOTOR_ID]);
  osDelay(1);
  cyber_gear_enable(&cybergears_2[RB_MOTOR_ID]);
  osDelay(1);
  cyber_gear_enable(&cybergears_2[RF_MOTOR_ID]);
  osDelay(1);
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
      case CHASSIS_INIT: {
        chassis_init_handle();
      }
        break;

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
// Turning and tilting body