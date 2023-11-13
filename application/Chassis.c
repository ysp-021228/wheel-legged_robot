/*include*/
#include <stdlib.h>

#include "Chassis.h"
#include "cmsis_os.h"
#include "remote.h"
#include "can_receive.h"
#include "user_lib.h"
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
static void chassis_imu_info_update();
static void chassis_relax_judge();
void chassis_device_offline_handle();
static void chassis_off_ground_detection();
static void chassis_info_update();
static void chassis_motor_info_update();
static void leg_state_variable_reference_get(struct Leg *leg);
static void leg_state_variable_set_point_set(struct Leg *leg, fp32 vx);
static void leg_state_variable_error_get(struct Leg *leg);
static void chassis_motors_torque_set_point_cal(struct Leg *leg);
static void joint_motors_torque_set_point_cal();
static void wheel_motors_torque_set_point_cal(struct Leg *leg);
fp32 cal_leg_theta(fp32 phi0, fp32 phi);
static void chassis_forward_kinematics();
static void chassis_motor_cmd_send();
static void chassis_K_matrix_fitting(fp32 L0, fp32 K[6], const fp32 KL[6][4]);
static void VMC_positive_dynamics(struct VMC *vmc);
static void Matrix_multiply(int rows1, int cols1, fp32 matrix1[rows1][cols1],
                            int rows2, int cols2, fp32 matrix2[rows2][cols2],
                            fp32 result[rows1][cols2]);
static void vmc_inverse_solution(struct Leg *leg);
static void Vmc_Negative_Kinematics(struct VMC *vmc, fp32 w1, fp32 w4);
static void Vmc_Negative_Dynamics(struct VMC *vmc, fp32 T1, fp32 T4);
static void leg_fn_cal(struct Leg *leg, fp32 az);

fp32 wheel_K_L[6] = {-2.999201, -0.204226, -0.046454, -0.110180, 0.193266, 0.069071};
fp32 joint_K_L[6] = {4.178352, 0.313840, 0.120231, 0.278284, 6.250164, 4.915544};

fp32 wheel_K_R[6] = {0, 0, 0, 0, 0, 0};
fp32 joint_K_R[6] = {0, 0, 0, 0, 0, 0};

//fp32 wheel_fitting_factor[6][4] = {
//    {-1673.057334, 232.571401, -47.931995, -0.577519},
//    {123.117076, -34.201064, -0.213162, -0.065369},
//
//    {552.013755, -83.775165, 10.262378, -0.821934},
//    {2364.387111, -354.429253, 15.557227, -0.538798},
//
//    {-129.486870, -46.403887, -4.520816, 3.173940},
//    {-115.799334, 11.347779, -1.362861, 0.378518}
//
//};
//fp32 joint_fitting_factor[6][4] = {
//    {35798.593960, -7437.861413, 597.340243, 6.227063},
//    {-157.136511, -93.793125, 1.340098, 0.974567},
//
//    {3450.517869, -562.796975, 35.500483, 8.306312},
//    {5420.342887, -984.711217, 26.244659, 5.071084},
//
//    {-18022.410517, 2874.336071, -20.249029, 18.600220},
//    {-1943.270245, 305.026358, 0.198070, 0.935925}
//};

//fp32 wheel_fitting_factor[6][4] = {
//    {-1673.057334, 232.571401, -47.931995, -0.877519},
//    {123.117076, -34.201064, -0.213162, -0.065369},
//
//    {552.013755, -83.775165, 20.262378, -1.21934},
//    {2364.387111, -354.429253, 10.557227, -0.538798},
//
//    {-129.486870, -46.403887, -4.520816, 3.173940},
//    {-115.799334, 11.347779, -1.362861, 0.378518}
//
//};
//fp32 joint_fitting_factor[6][4] = {
//    {35798.593960, -7437.861413, 797.340243, 8.227063},
//    {-157.136511, -93.793125, 1.340098, 0.974567},
//
//    {3450.517869, -562.796975, 40.500483, 20.306312},
//    {5420.342887, -984.711217, 40.244659, 5.071084},
//
//    {-18022.410517, 2874.336071, -40.249029, 58.600220},
//    {-1943.270245, 305.026358, 0.198070, 0.935925}
//};

fp32 wheel_fitting_factor[6][4] = {
    {-1400.855481,261.315075,-41.214402,-0.288694},
    {9.204610,-8.650371,-0.942393,-0.069374},

    {528.385588,-82.894130,3.671236,-0.368628},
    {1330.129855,-199.248637,9.223661,-0.577372},

    {2355.209129,-396.705970,14.857754,1.765168},
    {113.638926,-18.709641,0.342018,0.188771}
};
fp32 joint_fitting_factor[6][4] = {
    {11201.880993,-3799.610801,415.555231,1.933342},
    {505.162046,-158.656600,21.450875,0.728957},

    {12959.436110,-2055.367427,91.569851,3.691423},
    {9362.037701,-1582.027062,63.365121,5.941493},

    {-52734.083196,8384.486523,-338.700195,40.690128},
    {-5038.547321,790.796380,-32.273919,2.362930}
};

void chassis_task(void const *pvParameters) {

  vTaskDelay(CHASSIS_TASK_INIT_TIME);

  chassis_init(&chassis);

  TickType_t last_wake_time = xTaskGetTickCount();
  while (1) {
    chassis_info_update();//verified to be correct

    chassis_ctrl_info_get();

//    chassis_relax_judge();

    chassis_device_offline_handle();

    switch (chassis.mode) {
      case CHASSIS_ENABLED_LEG: {
        chassis_enabled_leg_handle();
      }
        break;

      case CHASSIS_UNENABLED_LEG: {
        chassis_enabled_leg_handle();
      }
        break;

      case CHASSIS_OFF_GROUND: {

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

static void chassis_info_update() {
  chassis_imu_info_update();
  chassis_motor_info_update();
  chassis.mileage =
      (-chassis.leg_L.wheel.mileage + chassis.leg_R.wheel.mileage) / 2;//The state variable x should use this value
  if (chassis.chassis_move_speed_set_point.vx != 0 || chassis.chassis_move_speed_set_point.vw != 0) {
    chassis.mileage = 0;
    chassis.leg_L.wheel.mileage = 0;
    chassis.leg_R.wheel.mileage = 0;
  }
}

static void chassis_motor_info_update() {
  chassis.leg_L.wheel.speed = -motor_3508_measure[0].speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  chassis.leg_R.wheel.speed = -motor_3508_measure[1].speed_rpm * BALANCE_RATIO_DEGREE_TO_WHEEL_SPEED;
  chassis.leg_L.wheel.mileage = chassis.leg_L.wheel.mileage + CHASSIS_PERIOD * 0.001 * (chassis.leg_L.wheel.speed);
  chassis.leg_R.wheel.mileage = chassis.leg_R.wheel.mileage + CHASSIS_PERIOD * 0.001 * (chassis.leg_R.wheel.speed);

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
}//verified to be correct

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
      (leg->state_variable_reference.theta - leg->state_variable_reference.theta_last) / (CHASSIS_PERIOD * 0.001f);
  leg->state_variable_reference.theta_ddot =
      (leg->state_variable_reference.theta_dot - leg->state_variable_reference.theta_dot_last)
          / (CHASSIS_PERIOD * 0.001f);

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
  leg->state_variable_set_point.phi = 0;
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

static void chassis_motors_torque_set_point_cal(struct Leg *leg) {
  joint_motors_torque_set_point_cal();
  wheel_motors_torque_set_point_cal(leg);
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

  VAL_LIMIT(leg->wheel.torque, -5, 5);

  if (leg->wheel.torque > 0) {
    leg->wheel.torque += 0.1f;
  } else {
    leg->wheel.torque -= 0.1f;
  }
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
  chassis.leg_L.vmc.Fxy_set_point.E.Fy_set_point = chassis.leg_L.pid.out + BODY_WEIGHT * 9.8 * 0.5;

  pid_calc(&chassis.leg_R.pid, chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, chassis.leg_R.L0_set_point);
  chassis.leg_R.vmc.Fxy_set_point.E.Fy_set_point = chassis.leg_R.pid.out + BODY_WEIGHT * 9.8 * 0.5;

  VMC_positive_dynamics(&chassis.leg_R.vmc);
  VMC_positive_dynamics(&chassis.leg_L.vmc);

  chassis.leg_L.cyber_gear_data[2].torque = chassis.leg_L.vmc.Fxy_set_point.E.Tp_set_point;
  chassis.leg_L.cyber_gear_data[0].torque = chassis.leg_L.vmc.T1_T4_set_point.E.T1_set_point;//F
  chassis.leg_L.cyber_gear_data[1].torque = chassis.leg_L.vmc.T1_T4_set_point.E.T4_set_point;//B

  chassis.leg_R.cyber_gear_data[2].torque = chassis.leg_R.vmc.Fxy_set_point.E.Tp_set_point;
  chassis.leg_R.cyber_gear_data[0].torque = chassis.leg_R.vmc.T1_T4_set_point.E.T1_set_point;//F
  chassis.leg_R.cyber_gear_data[1].torque = chassis.leg_R.vmc.T1_T4_set_point.E.T4_set_point;//B

  VAL_LIMIT(chassis.leg_R.cyber_gear_data[0].torque, -5, 5);
  VAL_LIMIT(chassis.leg_R.cyber_gear_data[1].torque, -5, 5);
  VAL_LIMIT(chassis.leg_L.cyber_gear_data[0].torque, -5, 5);
  VAL_LIMIT(chassis.leg_L.cyber_gear_data[1].torque, -5, 5);
}

static void VMC_positive_dynamics(struct VMC *vmc) {
  if (vmc == NULL) { return; }

  vmc->J_F_to_T.E.x1_1 = L1 * sin(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
      * cos(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
      / (vmc->forward_kinematics.fk_L0.L0
          * sin(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

  vmc->J_F_to_T.E.x1_2 = L1 * sin(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
      * sin(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
      / sin(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

  vmc->J_F_to_T.E.x2_1 = L4 * sin(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
      * cos(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
      / (vmc->forward_kinematics.fk_L0.L0
          * sin(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

  vmc->J_F_to_T.E.x2_2 = L4 * sin(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
      * sin(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
      / sin(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

  Matrix_multiply(2, 2, vmc->J_F_to_T.array, 2, 1, vmc->Fxy_set_point.array, vmc->T1_T4_set_point.array);
}

static void Matrix_multiply(int rows1, int cols1, fp32 matrix1[rows1][cols1],
                            int rows2, int cols2, fp32 matrix2[rows2][cols2],
                            fp32 result[rows1][cols2]) {
  if (cols1 != rows2)
    return;

  // Perform matrix multiplication
  for (int i = 0; i < rows1; ++i) {
    for (int j = 0; j < cols2; ++j) {
      result[i][j] = 0;
      for (int k = 0; k < cols1; ++k) {
        result[i][j] += matrix1[i][k] * matrix2[k][j];
      }
    }
  }
}

fp32 cal_leg_theta(fp32 phi0, fp32 phi) {
  fp32 theta = 0, alpha = 0;//alpha is the Angle at which the virtual joint motor is turned

  alpha = PI / 2 - phi0;

  if (alpha * phi < 0) {
    theta = ABS(alpha) - ABS(phi);
    if ((alpha > 0) && (phi < 0)) {
      theta *= -1;
    } else {

    }
  } else {
    theta = ABS(alpha) + ABS(phi);
    if ((alpha < 0) && (phi < 0)) {
    } else {
      theta *= -1;
    }
  }
  return theta;
}

static void vmc_inverse_solution(struct Leg *leg) {
  if (leg == NULL) {
    return;
  }
  Vmc_Negative_Kinematics(&leg->vmc, leg->cyber_gear_data[0].speed, leg->cyber_gear_data[1].speed);
  //todo 看看电机怎么获取实际扭矩
  Vmc_Negative_Dynamics(&leg->vmc, leg->vmc.T1_T4_set_point.E.T1_set_point, leg->vmc.T1_T4_set_point.E.T4_set_point);
}

static void Vmc_Negative_Kinematics(struct VMC *vmc, fp32 w1, fp32 w4) {
  if (vmc == NULL) {
    return;
  }
  vmc->W_fdb.E.w1_fdb = w1;
  vmc->W_fdb.E.w4_fdb = w4;

  vmc->J_w_to_v.E.x1_1 = (L1 * sinf(vmc->forward_kinematics.fk_phi.phi0) * sinf(vmc->forward_kinematics.fk_phi.phi3)
      * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
      - L4 * cosf(vmc->forward_kinematics.fk_phi.phi0) * sinf(vmc->forward_kinematics.fk_phi.phi2)
          * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4))
      / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);
  vmc->J_w_to_v.E.x1_2 = (L1 * cosf(vmc->forward_kinematics.fk_phi.phi0) * sinf(vmc->forward_kinematics.fk_phi.phi3)
      * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
      + L4 * sinf(vmc->forward_kinematics.fk_phi.phi0) * sinf(vmc->forward_kinematics.fk_phi.phi2)
          * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4))
      / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);
  vmc->J_w_to_v.E.x2_1 = (-L1 * sinf(vmc->forward_kinematics.fk_phi.phi0) * cosf(vmc->forward_kinematics.fk_phi.phi3)
      * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
      + L4 * cosf(vmc->forward_kinematics.fk_phi.phi0) * cosf(vmc->forward_kinematics.fk_phi.phi2)
          * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4))
      / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);
  vmc->J_w_to_v.E.x2_2 = -(L1 * cosf(vmc->forward_kinematics.fk_phi.phi0) * cosf(vmc->forward_kinematics.fk_phi.phi3)
      * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2) * L4
      * sinf(vmc->forward_kinematics.fk_phi.phi0) * cosf(vmc->forward_kinematics.fk_phi.phi2)
      * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4))
      / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);

  Matrix_multiply(2, 2, vmc->J_w_to_v.array, 2, 1, vmc->W_fdb.array, vmc->V_fdb.array);
  vmc->V_fdb.E.w0_fdb /= vmc->forward_kinematics.fk_L0.L0;
}

static void Vmc_Negative_Dynamics(struct VMC *vmc, fp32 T1, fp32 T4) {
  if (vmc == NULL) {
    return;
  }
  vmc->T1_T4_fdb.E.T1_fdb = T1;
  vmc->T1_T4_fdb.E.T4_fdb = T4;

  vmc->J_T_to_F.E.x1_1 =
      vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
          / (L1 * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2));
  vmc->J_T_to_F.E.x1_2 =
      vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
          / (L4 * sinf(vmc->forward_kinematics.fk_phi.phi4 - vmc->forward_kinematics.fk_phi.phi3));
  vmc->J_T_to_F.E.x2_1 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
      / (L1 * sin(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi1));
  vmc->J_T_to_F.E.x2_2 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
      / (L4 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4));

  Matrix_multiply(2, 2, vmc->J_T_to_F.array, 2, 1, vmc->T1_T4_fdb.array, vmc->Fxy_fdb.array);
}

static void chassis_ctrl_info_get() {
  chassis.chassis_move_speed_set_point.vx = (float) (get_rc_ctrl().rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX;
  chassis.chassis_move_speed_set_point.vw = (float) (get_rc_ctrl().rc.ch[CHASSIS_Z_CHANNEL]) * -RC_TO_VW;
  chassis.leg_L.L0_set_point = (float) (get_rc_ctrl().rc.ch[4]) * RC_TO_L0 + 0.18f;
  chassis.leg_R.L0_set_point = (float) (get_rc_ctrl().rc.ch[4]) * RC_TO_L0 + 0.18f;

  if (chassis.chassis_move_speed_set_point.vw == 0) {
    chassis.chassis_move_speed_set_point.vw =
        pid_loop_calc(&chassis.chassis_vw_pid, chassis.imu_reference.yaw_angle, chassis.imu_set_point.yaw, PI, -PI);
  } else {
    chassis.imu_set_point.yaw = chassis.imu_reference.yaw_angle;
  }

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

  chassis.leg_L.L0_set_point = 0.18f;
  chassis.leg_R.L0_set_point = 0.18f;

  chassis.imu_set_point.yaw = chassis.imu_reference.yaw_angle;
}

static void chassis_enabled_leg_handle() {
  chassis_forward_kinematics();//verified to be correct

  chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * 0.2, wheel_K_L, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * 0.2, joint_K_L, joint_fitting_factor);
//  chassis_K_matrix_fitting(0.18f, wheel_K_L, wheel_fitting_factor);
//  chassis_K_matrix_fitting(0.18f, joint_K_L, joint_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * 0.2, wheel_K_R, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * 0.2, joint_K_R, joint_fitting_factor);

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

static void chassis_unable_leg_handle() {

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
          / (CHASSIS_PERIOD * 0.001f);
  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_ddot =
      (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot - chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot_last)
          / (CHASSIS_PERIOD * 0.001f);
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
          / (CHASSIS_PERIOD * 0.001f);
  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_ddot =
      (chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot - chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot_last)
          / (CHASSIS_PERIOD * 0.001f);
  y = chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_y;
  x = chassis.leg_R.vmc.forward_kinematics.fk_point_coordinates.c_x - L5 * 0.5f;
  chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);
}

static void chassis_K_matrix_fitting(fp32 L0, fp32 K[6], const fp32 KL[6][4]) {
  for (int i = 0; i < 6; i++) {
    K[i] = KL[i][0] * powf(L0, 3) + KL[i][1] * powf(L0, 2) + KL[i][2] * powf(L0, 1) + KL[i][3] * powf(L0, 0);
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

static void leg_fn_cal(struct Leg *leg, fp32 az) {
  if (leg == NULL) {
    return;
  }
  fp32 P;
  P = leg->vmc.Fxy_fdb.E.Tp_fdb * sinf(leg->state_variable_reference.theta) / leg->vmc.forward_kinematics.fk_L0.L0
      + leg->vmc.Fxy_fdb.E.Fy_fdb * cosf(leg->state_variable_reference.theta);

  leg->wheel.imu_reference.az =
      az - leg->vmc.forward_kinematics.fk_L0.L0_ddot * cosf(leg->state_variable_reference.theta)
          + 2 * leg->vmc.forward_kinematics.fk_L0.L0_dot * leg->state_variable_reference.theta_dot
              * sinf(leg->state_variable_reference.theta)
          + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_reference.theta_ddot
              * sinf(leg->state_variable_reference.theta)
          + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_reference.theta_dot
              * leg->state_variable_reference.theta_dot * cosf(leg->state_variable_reference.theta);

  leg->Fn = P + WHEEL_WEIGHT * 9.8f + WHEEL_WEIGHT * leg->wheel.imu_reference.az;
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

struct Chassis get_chassis() {
  return chassis;
}
//todo 遥控离线刹车原地不动电机离线直接全部失能

