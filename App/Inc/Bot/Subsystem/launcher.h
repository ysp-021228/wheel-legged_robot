//
// Created by xhuanc on 2021/11/2.
//

#ifndef DEMO1_LAUNCHER_H
#define DEMO1_LAUNCHER_H

#include "IO/can_receive.h"

/******************define******************/
//后期根据射速限制来修改摩擦轮的最大转速
#define FIRE_SPEED_MAX 6500

#define FIRE_SPEED_LEVEL1 6500
#define FIRE_SPEED_LEVEL2 6500
#define FIRE_SPEED_LEVEL3 6500

#define TRIGGER_CONTINUES_SPEED -4000
#define TRIGGER_REVERSE_SPEED 3000
#define FIRE_L  2
#define FIRE_R  3
#define TRIGGER 0
#define DEGREE_45_TO_ENCODER -36863.1f

//#define FIRE_ON() KeyBoard.Q.click_flag==1
//#define FIRE_OFF() KeyBoard.Q.click_flag==0

//连发准备计时完成
#define CONTINUES_SHOOT_TIMING_COMPLETE() HAL_GetTick()-continue_shoot_time>1500
#define CONTINUES_BLOCKED_JUDGE() (HAL_GetTick()-blocked_start_time>500)
#define TRIGGER_REVERSE_TIME_JUDGE() (HAL_GetTick()-reverse_start_time<50)
//摩擦轮转速PID
#define SHOOT_FIRE_L_PID_KP 30
#define SHOOT_FIRE_L_PID_KI 0.f
#define SHOOT_FIRE_L_PID_KD 0.f
#define SHOOT_FIRE_L_PID_MAX_OUT    16000
#define SHOOT_FIRE_L_PID_MAX_IOUT   0

#define SHOOT_FIRE_R_PID_KP 30
#define SHOOT_FIRE_R_PID_KI 0.f
#define SHOOT_FIRE_R_PID_KD 0.f
#define SHOOT_FIRE_R_PID_MAX_OUT    16000
#define SHOOT_FIRE_R_PID_MAX_IOUT   0

//拨弹电机角度环PID
#define SHOOT_TRI_ANGLE_PID_KP 13
#define SHOOT_TRI_ANGLE_PID_KI 0.f
#define SHOOT_TRI_ANGLE_PID_KD 0.1f
#define SHOOT_TRI_ANGLE_PID_MAX_OUT 3000
#define SHOOT_TRI_ANGLE_PID_MAX_IOUT 0

//拨弹电机速度环PID
#define SHOOT_TRI_SPEED_PID_KP  10
#define SHOOT_TRI_SPEED_PID_KI  0.f
#define SHOOT_TRI_SPEED_PID_KD  0.f
#define SHOOT_TRI_SPEED_PID_MAX_OUT 14000
#define SHOOT_TRI_SPEED_PID_MAX_IOUT 0

/******************struct&enum******************/

typedef enum{
    Fire_OFF=0,
    Fire_ON=1,
}fire_mode_e;

typedef enum{
    SHOOT_CLOSE=0,
    SHOOT_SINGLE,
    SHOOT_ING,
    SHOOT_CONTINUES,
}trigger_cmd;

typedef struct {
    fire_mode_e fire_mode;//摩擦轮状态

    fire_mode_e fire_last_mode;//摩擦轮上一次状态

    trigger_cmd trigger_cmd;    //发射机构单发还是

    motor_2006_t fire_l;

    motor_2006_t fire_r;

    motor_2006_t trigger;

}launcher_t;

extern void launcher_init();
extern void launcher_mode_set();
extern void launcher_control();



#endif //DEMO1_LAUNCHER_H
