//
// Created by xhuanc on 2021/9/27.
//

#ifndef AVG_INFANTRY_CAN_RECEIVE_H
#define AVG_INFANTRY_CAN_RECEIVE_H
#include "struct_typedef.h"
#include "PID.h"
/******************** define *******************/

#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8192
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#define MOTOR_ECD_TO_ANGLE 0.0439453125     //        360  /8192
#define MOTOR_RAD_TO_ANGLE 57.29577951308238 // 360*2PI
#define MOTOR_ANGLE_TO_RAD  0.0174532925199433f
/******************** struct *******************/

//CAN_ID 该枚举不区分CAN_1还是CAN_2
typedef enum
{
    //电机控制 发送ID
    CAN_MOTOR_0x200_ID = 0x200,
    //0X200对应的电机ID
    CAN_CHASSIS_3508_MOTOR_1=0x201,
    CAN_CHASSIS_3508_MOTOR_2=0x202,
    CAN_VECTOR_XYZ_0X100_ID=0x100,

    //电机控制 发送ID
    CAN_MOTOR_0x1FF_ID = 0x1FF,
    //0x1FF对应的电机的ID
    //电机控制 发送ID
    CAN_MOTOR_0x2FF_ID = 0x2FF,
    //0x2FF对应的电机的ID

} can_msg_id_e;

typedef enum {
    CAN_1,
    CAN_2,
}CAN_TYPE;

typedef struct {

    union  {
        uint8_t data[4];
        fp32 value;
    }vx;

    union  {
        uint8_t data[4];
        fp32 value;
    }vw;

    union  {
        uint8_t data[4];
        fp32 value;
    }yaw_relative_angle_get;

    union {
        uint8_t data[2];
        uint16_t value;
    }mode;

    union {
        uint8_t data[4];
        fp32 value;

    }run;

    union {
        uint8_t data[4];
        fp32 value;

    }angle;

}__packed Vector_msg;

typedef struct {

    union{
        uint8_t data[3];
        uint16_t value;
    }distance;

    union{
        uint8_t data;
        uint8_t value
    }dis_status;

}__packed TOF_msg;


//电机的数据
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;

    int32_t round_cnt;   //电机旋转的总圈数
    int32_t total_ecd;   //电机旋转的总编码器数值

    uint16_t offset_ecd;//电机的校准编码值

} motor_measure_t;

struct Motor3508
{
    const motor_measure_t *motor_measure;

    fp32 speed;

    fp32 rpm_set;

    fp32 speed_set;

    fp32 torque;

    pid_t speed_p;

    int16_t give_current;

};

typedef struct
{

     motor_measure_t *motor_measure;

    pid_t angle_p;

    pid_t speed_p;


    fp32 max_relative_angle; //°

    fp32 min_relative_angle; //°

    fp32 relative_angle_get;
    fp32 relative_angle_set; //°

    fp32 absolute_angle_get;
    fp32 absolute_angle_set;//rad

    fp32 gyro_set;  //转速设置
    int16_t give_current; //最终电流值

}motor_6020_t;


typedef struct
{
    motor_measure_t *motor_measure;

    pid_t angle_p;//角度环pid

    pid_t speed_p;//速度环pid


    fp32 speed;//转速期望值

    int16_t give_current;

}motor_2006_t;

/******************** extern *******************/



extern motor_measure_t motor_3508_measure[4];//前2个为底盘电机 后2个为摩擦轮电机
extern motor_measure_t motor_6020_measure[4];
extern motor_measure_t motor_yaw_measure;
extern motor_measure_t motor_pitch_measure;
extern motor_measure_t motor_2006_measure[1];//拨弹电机

extern void CAN_cmd_motor(CAN_TYPE can_type,can_msg_id_e CMD_ID,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern fp32 motor_ecd_to_rad_change(uint16_t ecd, uint16_t offset_ecd);

extern fp32 motor_ecd_to_angle_change(uint16_t ecd,uint16_t offset_ecd);

uint32_t get_free_can_mailbox();

#endif //AVG_INFANTRY_CAN_RECEIVE_H
