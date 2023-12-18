#include <stdbool.h>
#include "chassis_algorithm.h"
#include "struct_typedef.h"
#include "user_lib.h"
#include "math.h"
#include "Chassis.h"

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

void Matrix_multiply(int rows1, int cols1, fp32 matrix1[rows1][cols1],
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

void VMC_positive_dynamics(struct VMC *vmc) {
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

void Vmc_Negative_Dynamics(struct VMC *vmc, fp32 T1, fp32 T4) {
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

void Vmc_Negative_Kinematics(struct VMC *vmc, fp32 w1, fp32 w4) {
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

void chassis_K_matrix_fitting(fp32 L0, fp32 K[6], const fp32 KL[6][4]) {
  for (int i = 0; i < 6; i++) {
    K[i] = KL[i][0] * powf(L0, 3) + KL[i][1] * powf(L0, 2) + KL[i][2] * powf(L0, 1) + KL[i][3] * powf(L0, 0);
  }
}

void leg_fn_cal(struct Leg *leg, fp32 az) {
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

bool_t is_chassis_phi_stable(struct IMUReference *imu_reference) {
  if (ABS(imu_reference->pitch_angle) <= 0.04 && ABS(imu_reference->pitch_gyro) <= 0.2) {
    return true;
  } else {
    return false;
  }
}

bool_t is_chassis_leg_return_to_original_position(struct Chassis *chassis) {
  if (ABS(chassis->leg_L.cyber_gear_data[0].angle) <= 0.087267 &&
      ABS(chassis->leg_L.cyber_gear_data[1].angle) <= 0.087267 &&
      ABS(chassis->leg_R.cyber_gear_data[0].angle) <= 0.087267 &&
      ABS(chassis->leg_R.cyber_gear_data[1].angle) <= 0.087267) {
    return true;
  } else {
    return false;
  }
}