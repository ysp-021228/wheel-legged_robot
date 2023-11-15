//
// Created by xhuanc on 2022/1/19.
//

#include "IO/Auto.h"
#include "string.h"
#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "string.h"
#include "Subsystem/Referee.h"
#include "Subsystem/Detection.h"
#include "Filter/filter.h"
extern UART_HandleTypeDef huart1;
extern Referee_info_t Referee;

uint8_t usart1_receive_buf[VISION_BUFFER_SIZE]={0};
uint8_t usart1_send_buf[VISION_BUFFER_SEND]={0};
Vision_info_get Vision_info;
Vision_info_send Vision_send;


bool_t Vision_read_data(uint8_t *ReadFromUsart)
{
    uint16_t data_len;
    if(ReadFromUsart==NULL)
        return 0;

    if(ReadFromUsart[0]==0XA5)//帧头校验
    {
        if(verify_CRC8_check_sum(ReadFromUsart,HEAD_LEN))
        {
            data_len=22;
            if(verify_CRC16_check_sum(ReadFromUsart,data_len))
            {
                Vision_info.frame_header.cmd=ReadFromUsart[1];//CMD_ID
                Vision_info.pitch.data[0]=ReadFromUsart[3];
                Vision_info.pitch.data[1]=ReadFromUsart[4];
                Vision_info.pitch.data[2]=ReadFromUsart[5];
                Vision_info.pitch.data[3]=ReadFromUsart[6];

                Vision_info.yaw.data[0]=ReadFromUsart[7];
                Vision_info.yaw.data[1]=ReadFromUsart[8];
                Vision_info.yaw.data[2]=ReadFromUsart[9];
                Vision_info.yaw.data[3]=ReadFromUsart[10];

                Vision_info.distance.data[0]=ReadFromUsart[11];
                Vision_info.distance.data[1]=ReadFromUsart[12];
                Vision_info.distance.data[2]=ReadFromUsart[13];
                Vision_info.distance.data[3]=ReadFromUsart[14];

                Vision_info.reverse_data[0]=ReadFromUsart[15];
                Vision_info.reverse_data[1]=ReadFromUsart[16];
                Vision_info.reverse_data[2]=ReadFromUsart[17];
                Vision_info.reverse_data[3]=ReadFromUsart[18];
                Vision_info.reverse_data[4]=ReadFromUsart[19];

                detect_handle(DETECT_AUTO_AIM);
            }
        }
    }

    //如果一个数据包出现了多帧数据,则再次读取
    if(*(ReadFromUsart+FRAME_LEN) == 0xA5)
    {
        Vision_read_data(ReadFromUsart+FRAME_LEN);
    }

}

void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);//读取UART1-SR 和UART1-DR; 清除中断标志位

        __HAL_DMA_DISABLE(huart1.hdmarx); //失能dma_rx

        Vision_read_data(&usart1_receive_buf[0]);//解析数据信息

        memset(&usart1_receive_buf[0],0,VISION_BUFFER_SIZE);//置0

        __HAL_DMA_CLEAR_FLAG(huart1.hdmarx,DMA_HISR_TCIF5); //清除传输完成标志位

        __HAL_DMA_SET_COUNTER(huart1.hdmarx, VISION_BUFFER_SIZE);//设置DMA 搬运数据大小 单位为字节

        __HAL_DMA_ENABLE(huart1.hdmarx); //使能DMAR

    }


}
static void auto_init(){
    //接收结构体
    Vision_info.frame_header.head=0xA5;
    Vision_info.frame_header.cmd=0x32;
    Vision_info.yaw.value=0;
    Vision_info.pitch.value=0;
    Vision_info.reverse_data[0]=0;
    Vision_info.reverse_data[1]=0;
    Vision_info.reverse_data[2]=0;
    Vision_info.reverse_data[3]=0;
    Vision_info.reverse_data[4]=0;
    Vision_info.distance.value=0;

    //发送结构体
    Vision_send.frame_header.head=0xA5;
    Vision_send.frame_header.cmd=0x21;
    Vision_send.yaw.value=0.f;
    Vision_send.pitch.value=0.f;
    Vision_send.roll.value=0.f;
    Vision_send.shoot.value=0.0f;
    Vision_send.reverse_data[0]=0;

}
static void vision_send(){
    usart1_send_buf[0]= Vision_send.frame_header.head;
    usart1_send_buf[1]= Vision_send.frame_header.cmd;
    append_CRC8_check_sum(usart1_send_buf,HEAD_LEN);

    usart1_send_buf[3]=Vision_send.pitch.data[0];
    usart1_send_buf[4]=Vision_send.pitch.data[1];
    usart1_send_buf[5]=Vision_send.pitch.data[2];
    usart1_send_buf[6]=Vision_send.pitch.data[3];

    usart1_send_buf[7]=Vision_send.yaw.data[0];
    usart1_send_buf[8]=Vision_send.yaw.data[1];
    usart1_send_buf[9]=Vision_send.yaw.data[2];
    usart1_send_buf[10]=Vision_send.yaw.data[3];

    usart1_send_buf[11]=Vision_send.roll.data[0];
    usart1_send_buf[12]=Vision_send.roll.data[1];
    usart1_send_buf[13]=Vision_send.roll.data[2];
    usart1_send_buf[14]=Vision_send.roll.data[3];

    usart1_send_buf[15]=Vision_send.shoot.data[0];
    usart1_send_buf[16]=Vision_send.shoot.data[1];
    usart1_send_buf[17]=Vision_send.shoot.data[2];
    usart1_send_buf[18]=Vision_send.shoot.data[3];

    usart1_send_buf[19]=0;

    append_CRC16_check_sum(usart1_send_buf,22);
}

void auto_task(void const *pvParameters)
{
    osDelay(AUTO_TASK_INIT_TIME);
    auto_init();
    append_CRC8_check_sum(usart1_send_buf,HEAD_LEN);
    while (1)
    {
//        if(Referee.ShootData.shooter_id==1)
//        {
            Vision_send.shoot.value=26.0f;
//        }
        //装载视觉发送信息          ---> 将Vision_info_send 的装进uart缓冲区
        vision_send();
        usart1_tx_dma_enable(usart1_send_buf,VISION_BUFFER_SEND);
        osDelay(4);
    }

}