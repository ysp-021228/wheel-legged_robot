//
// Created by xhuanc on 2021/9/27.
//

#include "IO/can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "Subsystem/Chassis.h"
#include "Subsystem/Detection.h"
#include "CyberGear.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/******************** define *******************/

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

//平衡兵电机转距闭环控制时反馈报文解析
#define get_balance_motor_force_measure(ptr, data)                                    \
    {                                                                           \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->temperate = (data)[1];                                                                           \
        (ptr)->given_current = (uint16_t)((data)[3] << 8 | (data)[2]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[5] << 8 | (data)[4]);      \
        (ptr)->ecd = (uint16_t)((data)[7] << 8 | (data)[6]);  \
    }

#define get_motor_round_cnt(ptr) \
    {                            \
             if(ptr.ecd-ptr.last_ecd> 4192){ \
                ptr.round_cnt--;                    \
             }                   \
             else if(ptr.ecd-ptr.last_ecd<-4192)    \
             {                   \
                ptr.round_cnt++;            \
             }                   \
             ptr.total_ecd= ptr.round_cnt*8192+ptr.ecd-ptr.offset_ecd;\
                                 \
    }

/******************** variable *******************/



motor_measure_t motor_3508_measure[4];//前2个为底盘电机 后2个为摩擦轮电机
motor_measure_t motor_yaw_measure;
motor_measure_t motor_pitch_measure;
motor_measure_t motor_2006_measure[1];//TRIGGER

static CAN_TxHeaderTypeDef tx_message;
static uint8_t can_send_data[8];
uint32_t get_free_can_mailbox();
extern struct Chassis chassis;
Vector_msg vector_receive_msg;//
TOF_msg tof_msg;

//
void CAN_cmd_motor(CAN_TYPE can_type,
                   can_msg_id_e CMD_ID,
                   int16_t motor1,
                   int16_t motor2,
                   int16_t motor3,
                   int16_t motor4) {
  uint32_t send_mail_box;
  tx_message.StdId = CMD_ID;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  can_send_data[0] = motor1 >> 8;
  can_send_data[1] = motor1;
  can_send_data[2] = motor2 >> 8;
  can_send_data[3] = motor2;
  can_send_data[4] = motor3 >> 8;
  can_send_data[5] = motor3;
  can_send_data[6] = motor4 >> 8;
  can_send_data[7] = motor4;

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
  }

}

void CAN_speed_cmd_motor(CAN_TYPE can_type, can_msg_id_e CMD_ID, int32_t speed) {
  uint32_t send_mail_box;
  tx_message.StdId = CMD_ID;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  can_send_data[0] = 0xA2;
  can_send_data[1] = 0x00;
  can_send_data[2] = 0x00;
  can_send_data[3] = 0x00;
  can_send_data[4] = *(uint8_t *) (&speed);
  can_send_data[5] = *((uint8_t *) (&speed) + 1);
  can_send_data[6] = *((uint8_t *) (&speed) + 2);
  can_send_data[7] = *((uint8_t *) (&speed) + 3);

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
  }
}


void CAN_msg_send(CAN_TYPE can_type, can_msg_id_e CMD_ID, Vector_msg msg) {
  uint32_t send_mail_box;
  tx_message.StdId = CMD_ID;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  switch (CMD_ID) {

    case CAN_VECTOR_XYZ_0X100_ID : {

      tx_message.DLC = 0x08;

      can_send_data[0] = msg.vx.data[0];
      can_send_data[1] = msg.vx.data[1];
      can_send_data[2] = msg.vx.data[2];
      can_send_data[3] = msg.vx.data[3];
      can_send_data[4] = msg.vw.data[0];
      can_send_data[5] = msg.vw.data[1];
      can_send_data[6] = msg.vw.data[2];
      can_send_data[7] = msg.vw.data[3];
      break;
    }

    default: {
      tx_message.DLC = 0x08;
      can_send_data[0] = 0;
      can_send_data[1] = 0;
      can_send_data[2] = 0;
      can_send_data[3] = 0;
      can_send_data[4] = 0;
      can_send_data[5] = 0;
      can_send_data[6] = 0;
      can_send_data[7] = 0;
      break;
    }
  }

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
  }

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;

  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  mi_motor_data_get(hcan, rx_header, rx_data);
  if (hcan == &hcan1) {
    switch (rx_header.StdId) {
      case CAN_CHASSIS_3508_MOTOR_1: {
        get_motor_measure(&motor_3508_measure[0], rx_data);
      }
        break;
      case CAN_CHASSIS_3508_MOTOR_2: {
        get_motor_measure(&motor_3508_measure[1], rx_data);
      }
        break;

      case CAN_VECTOR_XYZ_0X100_ID: {
        vector_receive_msg.vx.data[0] = rx_data[0];
        vector_receive_msg.vx.data[1] = rx_data[1];
        vector_receive_msg.vx.data[2] = rx_data[2];
        vector_receive_msg.vx.data[3] = rx_data[3];

        vector_receive_msg.vw.data[0] = rx_data[4];
        vector_receive_msg.vw.data[1] = rx_data[5];
        vector_receive_msg.vw.data[2] = rx_data[6];
        vector_receive_msg.vw.data[3] = rx_data[7];

        break;
      }

      default: {
        break;
      }
    }
  } else if (hcan == &hcan2) {

    switch (rx_header.StdId) {

      case CAN_CHASSIS_3508_MOTOR_1: {
        get_motor_measure(&motor_3508_measure[0], rx_data);
      }
        break;
      case CAN_CHASSIS_3508_MOTOR_2: {
        get_motor_measure(&motor_3508_measure[1], rx_data);
      }
        break;

      case 0x20A: {
        tof_msg.distance.data[0] = rx_data[0];
        tof_msg.distance.data[1] = rx_data[1];
        tof_msg.distance.data[2] = rx_data[2];
        tof_msg.dis_status.data = rx_data[3];
        break;
      }

      default:break;
    }
  }
}

fp32 motor_ecd_to_rad_change(uint16_t ecd, uint16_t offset_ecd) {
  int32_t relative_ecd = ecd - offset_ecd;
  if (relative_ecd > HALF_ECD_RANGE) {
    relative_ecd -= ECD_RANGE;
  } else if (relative_ecd < -HALF_ECD_RANGE) {
    relative_ecd += ECD_RANGE;
  }

  return relative_ecd * MOTOR_ECD_TO_RAD;
}

fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd) {
  int16_t tmp = 0;
  if (offset_ecd >= 4096) {
    if (ecd > offset_ecd - 4096) {
      tmp = ecd - offset_ecd;
    } else {
      tmp = ecd + 8192 - offset_ecd;
    }
  } else {
    if (ecd > offset_ecd + 4096) {
      tmp = ecd - 8192 - offset_ecd;
    } else {
      tmp = ecd - offset_ecd;
    }
  }
  return (fp32) tmp / 8192.f * 360;
}

uint32_t get_free_can_mailbox() {
  if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET) {
    return CAN_TX_MAILBOX0;
  } else if ((hcan1.Instance->TSR & CAN_TSR_TME1)
      != RESET) { return CAN_TX_MAILBOX1; }
  else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET) { return CAN_TX_MAILBOX2; }
  else { return 0; }
}