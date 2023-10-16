//
// Created by xhuanc on 2021/11/14.
//

#ifndef DEMO1_AHRS_MIDDLEWARE_H
#define DEMO1_AHRS_MIDDLEWARE_H
#include "struct_typedef.h"
//定义 NULL
#ifndef NULL
#define NULL 0
#endif

//定义PI 值
#ifndef PI
#define PI 3.14159265358979f
#endif

//定义 角度(度)转换到 弧度的比例
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

//定义 弧度 转换到 角度的比例
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

extern void AHRS_get_height(fp32 *high);
extern void AHRS_get_latitude(fp32 *latitude);
extern fp32 AHRS_invSqrt(fp32 num);
extern fp32 AHRS_sinf(fp32 angle);
extern fp32 AHRS_cosf(fp32 angle);
extern fp32 AHRS_tanf(fp32 angle);
extern fp32 AHRS_asinf(fp32 sin);
extern fp32 AHRS_acosf(fp32 cos);
extern fp32 AHRS_atan2f(fp32 y, fp32 x);
#endif //DEMO1_AHRS_MIDDLEWARE_H
