#ifndef CHASSIS_ALGORITHM_H_
#define CHASSIS_ALGORITHM_H_

#include "struct_typedef.h"
#include "Chassis.h"

fp32 cal_leg_theta(fp32 phi0, fp32 phi);

void Matrix_multiply(int rows1, int cols1, fp32 matrix1[rows1][cols1],
                     int rows2, int cols2, fp32 matrix2[rows2][cols2],
                     fp32 result[rows1][cols2]);

void VMC_positive_dynamics(struct VMC *vmc);

void Vmc_Negative_Dynamics(struct VMC *vmc, fp32 T1, fp32 T4);

void Vmc_Negative_Kinematics(struct VMC *vmc, fp32 w1, fp32 w4);

void chassis_K_matrix_fitting(fp32 L0, fp32 K[6], const fp32 KL[6][4]);

void leg_fn_cal(struct Leg *leg, fp32 az);

#endif //CHASSIS_ALGORITHM_H_
