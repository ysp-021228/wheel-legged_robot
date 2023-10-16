//
// Created by xhuanc on 2021/10/13.
//

#ifndef DEMO1_GIMBAL_H
#define DEMO1_GIMBAL_H
/*      Include     */

#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "AHRS.h"

/*      define     */

#define GIMBAL_TASK_INIT_TIME 500
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3

//自瞄控制系数
#define AUTO_RADIO_YAW 0.007f
#define AUTO_RADIO_PITCH 0.01f

#define RC_TO_YAW 0.002 //0.02
#define RC_TO_PITCH 0.002 //0.02
#define MAX_ABS_ANGLE 20
#define MIN_ABS_ANGLE -25
#define MAX_RELA_ANGLE 40
#define MIN_RELA_ANGLE -20
//云台转动速度系数
#define GIMBAL_RC_MOVE_RATIO_PIT 0.08f
#define GIMBAL_RC_MOVE_RATIO_YAW 0.3f

//6020电机转过弧度和弧长转化系数
#define GIMBAL_RADIAN_ARC 0.4f

#define GIMBAL_YAW_ANGLE_PID_KP     32.0f//1.9
#define GIMBAL_YAW_ANGLE_PID_KI     0.008f//0
#define GIMBAL_YAW_ANGLE_PID_KD     150.001f//40
#define GIMBAL_YAW_ANGLE_MAX_OUT    500.f
#define GIMBAL_YAW_ANGLE_MAX_IOUT   50.f

#define GIMBAL_YAW_SPEED_PID_KP     100.0f//400
#define GIMBAL_YAW_SPEED_PID_KI     1.0f//30
#define GIMBAL_YAW_SPEED_PID_KD     2.0f//120
#define GIMBAL_YAW_SPEED_MAX_OUT    18000.f
#define GIMBAL_YAW_SPEED_MAX_IOUT   12000.f

//能用的合适pid
//#define GIMBAL_PITCH_ANGLE_PID_KP   30.0f
//#define GIMBAL_PITCH_ANGLE_PID_KI   0.0f  //0
//#define GIMBAL_PITCH_ANGLE_PID_KD   0.0f
//#define GIMBAL_PITCH_ANGLE_MAX_OUT  1000.f
//#define GIMBAL_PITCH_ANGLE_MAX_IOUT 30.f
////
//
//#define GIMBAL_PITCH_SPEED_PID_KP   30.f
//#define GIMBAL_PITCH_SPEED_PID_KI   0.0f
//#define GIMBAL_PITCH_SPEED_PID_KD   0.f
//#define GIMBAL_PITCH_SPEED_MAX_OUT  20000.f
//#define GIMBAL_PITCH_SPEED_MAX_IOUT 3000.f

//又高又硬
#define GIMBAL_PITCH_ANGLE_PID_KP   18.0f
#define GIMBAL_PITCH_ANGLE_PID_KI   0.8f  //0
#define GIMBAL_PITCH_ANGLE_PID_KD   110.0f
#define GIMBAL_PITCH_ANGLE_MAX_OUT  1000.f
#define GIMBAL_PITCH_ANGLE_MAX_IOUT 200.0f
//

#define GIMBAL_PITCH_SPEED_PID_KP   180.f
#define GIMBAL_PITCH_SPEED_PID_KI   0.1f
#define GIMBAL_PITCH_SPEED_PID_KD   80.0f
#define GIMBAL_PITCH_SPEED_MAX_OUT  20000.f
#define GIMBAL_PITCH_SPEED_MAX_IOUT 3000.f

/*      结构体和枚举     */

typedef enum {
    GIMBAL_RELAX=0,//云台失能
    GIMBAL_BACK,
    GIMBAL_ACTIVE,
    GIMBAL_AUTO,//云台自瞄模式
    GIMBAL_BUFF,//云台打符模式
}gimbal_mode_e;

typedef struct {
    motor_6020_t yaw;
    motor_6020_t pitch;
    motor_2006_t trigger;

    gimbal_mode_e mode;
    gimbal_mode_e last_mode;

//    AHRS_Eulr_t*Eulr;   //姿态角

    bool_t yaw_is_back;
    bool_t pitch_is_back;
}gimbal_t;

extern void gimbal_task(void const*pvParameters);


#endif //DEMO1_GIMBAL_H
