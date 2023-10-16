//%%88888888888888888888888888888888888888888888888888888888888888888888888888888%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8%&%BB%B%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%i. .  `iJ8B8B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B8%%Bw''.  '.''`q%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB].. .......'.w$%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BBm^'.........`+B8B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%88' .........."B%BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8Bf .........'.0&88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%B&@8B%8%%%%%%%%%%%%BB8%%w  ........`]CB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%BW@%B%8%%%%%%%%%%%%%%%@~  . .......^88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%BB%8%%8#>d8%%$%8%%%%%%%B8%%b. ........  #8%%%%%8%%}}C,UWW%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%W&%pl` ''''`[LB%%BZq-x{Z@BJ   ..... ..lJ%%B}C}W%W*`...... ]8$%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%O`'`'.    . '  '.'^ ?%%B%b:. ...... ..&@%%B~.... .. .....^` `w#&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%W`.'..'    ... .'.X{BB%L..   ...  ..O%8BBz..            ..:C%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%BB_'......... ' `w@%B%B`  `......`Ib@%%&%JI .           ..QBB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%J' ..... '. Ih@#BB%%{a'  .......'"&}BBBBBBB%B%%hu '.    . ''Q$&%B%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%B%%%%%%%8%8%b^''..' . "+{WW8%%%%&%J       .....0@BBBBBBB%%%%BB%B$w .'.     'h8%%%%B%%%B%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%@^ .'''"..''.''.'.''M%#8B%%%%%%8M`'..     ...h%BBBBB%%%%%%%BBB%BB%C[`   .. '','''.'` B%%%%%%%%%%%%%%%%%%%
//%%%%%%%%Bj`    .'. '......^B%B%%%%%%%%%%k"'.'       ,M@BBB&%%BBBB8%&ZUYhaw#%&W'`  ...........' 8%%%%%%%%%%%%%%%%%%
//%%%%%%%8L``'.   '    . ^`?@%%8%%%%%%B%8L` '        ''.'...'...........'..z@BB@BL.'...        .`_Z%%%%%%%%%%%%%%%%%
//%%%%%%%m;`''''''......."&8%%8%%%%%%%%B%` .       . . .'.  .......    '.'C%%BB%B&J'         ..'':_%@%%%%%%%%%%%%%%%
//%%%%%%%%B%%%%%o ..... .v%B%%%%%%%%%BB%'^..                          .''J&88%%%B%8t.....   0&@88%8%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%f      .'$B%%%%%%%%%%8Q+'. . ... '''..    '`^.`.        a8&8%8%%%%B$`.......kB%%888%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%+'      ^B%%%%%%%%8%%&.'  . '';Y%%%8%BB%BBBBb  . . .   08BB88%%%%%%}'.     .n%88888%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%B8h^      'MB8%%%%%%%8%:.   ......^u&%%B%%BB&h       '.'q%8%%%%%%%%%BZ'.   . .bB88888%%%%%%%%%%%%%%%%%
//%%%%%%%W%$MQh:..     .._B%%%%%%%B#j       .  .U%%88888BB!..    ... v%8%%%%%%%%%%Bl     .'''nw&W@$%%%%%%%%%%%%%%%%%
//%%%%%%%#+''.''`'''... . *$%%%%%%W''        . zB%%%8888B<'`    ....J&8%%%%%%%%%BBw.      ...'.',.b%%%%%%%%%%%%%%%%%
//%%%%%%%B8l.............'."O@%%8W^.  ..     :u&%&%B%%%p' '''..   .bBW%%%%%%%%%%C+.'     ....... %W%%%%%%%%%%%%%%%%%
//%%%%%%%%%&..'.''. `  ..''' n&%QnkhJM&8B@BBB%%88%8888q...        o%88%%%%%%%&%]`.. ...  .  .  'M&%%%%%%%%%%%%%%%%%%
//%%%%%%%B%@ZadJLO888h`'''..   ;%@8%BB%%8&8%%888888%%Q'..       '+&8%%%%%%%W@l. .. .'''C%BW0Xkkq%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%B%%B%8%BWQ '.''''' `>{BW%%B8%%%%%8888BZ`'.     . 'q88%BB%B8B!^. ..'... [&%%BBB%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%B%8%%8%%Bk .'''''`.^`'xY8%%%%%%%%%8f ^       . r$8%%&Bt`^   .. ... Y@&88%%%%8%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%B%%%%B%8B%8%Wl^'.. ......  ''',[J&%8Bv` .       .,%8%_.`' .....   . 'Z&88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%B%%%%%%%%%%%%%%%%%%@k'`''........ .   . +B%%X!.        ..b}%al    '... .    '.`tBB8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%8B{~''`'........:j,'.']BBBJ. .     ..  v%@q ^  '``... .     '.'L8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%@%&BQ` ''  ,#%B%8BB&8B&0' .      ..'<&8&888%88&%%Bu     'w#$%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%B%%%%%%%%%%%%%%%%%%%88WkMB8%%%%%%%%%%f'          .b%8%%%%%%%%%%%%8%@mW@B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%8B%%%B%%8%%%%%B%x`.        . -B8%%%%%%%%%%%B&8%%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#z'`.      .' .B&B8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%8B@M` ..      . ^nOB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%C'.. .     ..']%B%%%8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB#....        ' BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Bq .   .    ' Q%BB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%88888] .^^.   "p@B%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B%%B%$d'"  ^.&BB8%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%BB%8M '^hB%8%88%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%B88%%%%W&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include "Referee.h"
#include "string.h"
#include "CRC8_CRC16.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "Gimbal.h"
#include "Chassis.h"
#include "Detection.h"
#include "key_board.h"
#include "launcher.h"

extern UART_HandleTypeDef huart6;
extern key_board_t KeyBoard;

Graphic_Operate static_update_flag=UI_NONE;
Graphic_Operate one_layer_update_flag=UI_NONE;
Graphic_Operate two_layer_update_flag=UI_NONE;
Graphic_Operate three_layer_update_flag=UI_NONE;

uint8_t usart6_buf[REFEREE_BUFFER_SIZE]={0};

Referee_info_t Referee;

/*
 * UI更新状态
 */
ui_robot_status_t ui_robot_status={

    .gimbal_mode=GIMBAL_RELAX,
    .chassis_mode=CHASSIS_RELAX,
    .block_warning=false,
    .super_cap_value=0.f,
    .shoot_heat_limit=0,

};

/*函数和声明*/
static void referee_unpack_fifo_data(void);
static bool_t Referee_read_data(uint8_t *ReadFromUsart);
static void ui_static_draw();
/*裁判系统主任务*/

extern fp32 INS_angle[3];

//串口中断函数
void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);//读取UART6-SR 和UART6-DR; 清除中断标志位
//        static uint16_t this_time_rx_len = 0;
//
//        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            __HAL_DMA_DISABLE(huart6.hdmarx);
//            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
//            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(huart6.hdmarx);
////            detect_hook(REFEREE_TOE);
//        }
//        else
//        {
//            __HAL_DMA_DISABLE(huart6.hdmarx);
//            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
//            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
//            __HAL_DMA_ENABLE(huart6.hdmarx);
//            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
////            detect_hook(REFEREE_TOE);
//        }

        __HAL_DMA_DISABLE(huart6.hdmarx); //使能dma_rx

        Referee_read_data(&usart6_buf[0]);

        memset(&usart6_buf[0],0,REFEREE_BUFFER_SIZE);//置0

        __HAL_DMA_CLEAR_FLAG(huart6.hdmarx,DMA_LISR_TCIF1); //清除传输完成标志位

        __HAL_DMA_SET_COUNTER(huart6.hdmarx, REFEREE_BUFFER_SIZE);//设置DMA 搬运数据大小 单位为字节

        __HAL_DMA_ENABLE(huart6.hdmarx); //使能DMARx

    }
}

//根据裁判系统信息判断机器人的ID和对应客户端的ID
void judge_team_client(){
    //本机器人为红方
    if(Referee.GameRobotStat.robot_id<10)
    {
        Referee.ids.teammate_hero 		 	= 1;
        Referee.ids.teammate_engineer  = 2;
        Referee.ids.teammate_infantry3 = 3;
        Referee.ids.teammate_infantry4 = 4;
        Referee.ids.teammate_infantry5 = 5;
        Referee.ids.teammate_plane		 	= 6;
        Referee.ids.teammate_sentry		= 7;

        Referee.ids.client_hero 		 	= 0x0101;
        Referee.ids.client_engineer  = 0x0102;
        Referee.ids.client_infantry3 = 0x0103;
        Referee.ids.client_infantry4 = 0x0104;
        Referee.ids.client_infantry5 = 0x0105;
        Referee.ids.client_plane			= 0x0106;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_red:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;
            
            case Referee_engineer_red:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;
            
            case Referee_infantry3_red:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_red:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_red:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_red:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }//本机器人为蓝方
    else{
        Referee.ids.teammate_hero 		 	= 101;
        Referee.ids.teammate_engineer  = 102;
        Referee.ids.teammate_infantry3 = 103;
        Referee.ids.teammate_infantry4 = 104;
        Referee.ids.teammate_infantry5 = 105;
        Referee.ids.teammate_plane		 	= 106;
        Referee.ids.teammate_sentry		= 107;

        Referee.ids.client_hero 		 	= 0x0165;
        Referee.ids.client_engineer  = 0x0166;
        Referee.ids.client_infantry3 = 0x0167;
        Referee.ids.client_infantry4 = 0x0168;
        Referee.ids.client_infantry5 = 0x0169;
        Referee.ids.client_plane			= 0x016A;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_blue:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;

            case Referee_engineer_blue:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;

            case Referee_infantry3_blue:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_blue:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_blue:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_blue:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }
}
int CmdID=0;//数据命令码解析

bool_t Referee_read_data(uint8_t *ReadFromUsart)
{

    uint16_t judge_length;

    if(ReadFromUsart==NULL)
        return 0 ;

    memcpy(&Referee.FrameHeader,ReadFromUsart,Referee_LEN_FRAME_HEAD);

     if(ReadFromUsart[SOF]==REFREE_HEADER_SOF) //判断帧头是否为0xA5
    {
        if(verify_CRC8_check_sum(ReadFromUsart,LEN_HEADER)) //CRC
        {
            judge_length=ReadFromUsart[DATA_LENGTH]+LEN_HEADER+Referee_LEN_CMD_ID+Referee_LEN_FRAME_TAIL;
            if(verify_CRC16_check_sum(ReadFromUsart,judge_length))
            {
//                retval_tf=1;//表示数据可用
                CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)

                switch (CmdID)
                {

                    case Referee_ID_game_state://0x0001 比赛状态 1HZ
                        memcpy(&Referee.GameState,ReadFromUsart+DATA,Referee_LEN_game_state);
                        break;

                    case Referee_ID_game_result://0x0002 比赛结果   比赛结束后发送
                        memcpy(&Referee.GameResult,ReadFromUsart+DATA,Referee_LEN_game_result);
                        break;

                    case Referee_ID_game_robot_survivors://0x0003 机器人状态HP   1HZ
                        memcpy(&Referee.GameRobotHP,ReadFromUsart+DATA,Referee_LEN_game_robot_survivors);
                        break;

                    case Referee_ID_game_dart_state: //0x0004 飞镖发射状态
                        memcpy(&Referee.GameDartStatus,ReadFromUsart+DATA,Referee_LED_game_missile_state);
                        break;

                    case Referee_ID_game_buff: //0x0005 ICRA_BUFF状态     1HZ
                        memcpy(&Referee.GameICRABuff,ReadFromUsart+DATA,Referee_LED_game_buff);
                        break;

                    case Referee_ID_event_data://0x0101 场地事件数据      1HZ
                        memcpy(&Referee.EventData,ReadFromUsart+DATA,Referee_LEN_event_data);
                        break;

                    case Referee_ID_supply_projectile_action://0x0102 场地补给站动作标识数据   动作改变之后发送
                        memcpy(&Referee.SupplyProjectileAction,ReadFromUsart+DATA,Referee_LEN_supply_projectile_action);
                        break;

                    case Referee_ID_supply_warm://0x0104    裁判系统警告数据    己方警告之后发送
                        memcpy(&Referee.RefereeWarning,ReadFromUsart+DATA,Referee_LEN_supply_warm);
                        break;

                    case Referee_ID_dart_shoot_time://0x0105    飞镖发射口倒计时    1HZ
                        memcpy(&Referee.DartRemainingTime,ReadFromUsart+DATA,Referee_LEN_missile_shoot_time);
                        break;

                    case Referee_ID_game_robot_state://0x0201   机器人状态数据     10HZ
                        memcpy(&Referee.GameRobotStat,ReadFromUsart+DATA,Referee_LEN_game_robot_state);
                        judge_team_client();//判断一下机器人所属的队伍和类型 以及对应的机械人id和客户端id
                        break;

                    case Referee_ID_power_heat_data://0x0202    实时功率热量数据    50HZ
                        memcpy(&Referee.PowerHeatData,ReadFromUsart+DATA,Referee_LEN_power_heat_data);
                        break;

                    case Referee_ID_game_robot_pos://0x0203     机器人位置数据     10HZ
                        memcpy(&Referee.GameRobotPos,ReadFromUsart+DATA,Referee_LEN_game_robot_pos);
                        break;

                    case Referee_ID_buff_musk://0x0204  机器人增益数据     1HZ
                        memcpy(&Referee.Buff,ReadFromUsart+DATA,Referee_LEN_buff_musk);
                        break;

                    case Referee_ID_aerial_robot_energy://0x0205    空中机器人能量状态数据 10HZ
                        memcpy(&Referee.AerialRobotEnergy,ReadFromUsart+DATA,Referee_LEN_aerial_robot_energy);
                        break;

                    case Referee_ID_robot_hurt://0x0206     伤害状态数据  伤害发生后发送
                        memcpy(&Referee.RobotHurt,ReadFromUsart+DATA,Referee_LEN_robot_hurt);
                        break;

                    case Referee_ID_shoot_data://0x0207     实时射击数据  射击后发送
                        memcpy(&Referee.ShootData,ReadFromUsart+DATA,Referee_LEN_shoot_data);
                        break;

                    case Referee_ID_bullet_remaining://0x0208   剩余发射数   10HZ周期发送
                        memcpy(&Referee.BulletRemaining,ReadFromUsart+DATA,Referee_LEN_bullet_remaining);
                        break;

                    case Referee_ID_rfid_status://0x0209    机器人RFID状态，1Hz
                        memcpy(&Referee.RfidStatus,ReadFromUsart+DATA,Referee_LEN_rfid_status);
                        break;

                    case Referee_ID_dart_client_directive://0x020A  飞镖机器人客户端指令书, 10Hz
                        memcpy(&Referee.DartClient,ReadFromUsart+DATA,Referee_LEN_dart_client_directive);
                        break;
//
//                    case 0x0301: //测试机器人交互
//                        memcpy(&test_buf,ReadFromUsart+DATA, sizeof(tx_client_string));
//                        break;
                    default:
                        break;
                }
                detect_handle(DETECT_REFEREE);
            }
        }
        if(*(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length +Referee_LEN_FRAME_TAIL) == 0xA5)
        {
            //如果一个数据包出现了多帧数据,则再次读取
            Referee_read_data(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length+ Referee_LEN_FRAME_TAIL);
        }
    }
}


//绘制变量
uint8_t state_first_graphic;//0~7循环 更新的图层数
uint8_t ClientTxBuffer[200];//发送给客户端的数据缓冲区
//绘制数据
//第0层画的字符串 字符串最长只能 30 Byte
/**************************************/
/**
 * 绘制字符串
 * @param graphic
 * @param name
 * @param op_type
 * @param layer
 * @param color
 * @param size
 * @param length
 * @param width
 * @param start_x
 * @param start_y
 * @param character
 */
void String_Graphic(ui_string_t*clientData,
                  const char* name,
                  uint32_t op_type,
                  uint32_t layer,
                  uint32_t color,
                  uint32_t size,
                  uint32_t length,
                  uint32_t width,
                  uint32_t start_x,
                  uint32_t start_y,
                  const char *character)// 数组
{
    ui_graphic_data_struct_t*data_struct=&clientData->graphic_data_struct;
    data_struct->graphic_tpye=UI_CHAR;

    for(char i=0;i<3;i++)
        data_struct->graphic_name[i] = name[i];	//字符索引
    data_struct->operate_tpye=op_type;// 图层操作  1为增加
    data_struct->layer=layer;//在第几图层
    data_struct->color=color;//颜色
    data_struct->start_angle=size;
    data_struct->end_angle=length;
    data_struct->width=width;
    data_struct->start_x=start_x;
    data_struct->start_y=start_y;
    data_struct->radius = 0;
    data_struct->end_x = 0;
    data_struct->end_y = 0;

    memcpy(clientData->data,character,30);
}

/**
 * 绘制腹图像
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param start_angle
 * @param end_angle
 * @param width
 * @param start_x
 * @param start_y
 * @param radius
 * @param end_x
 * @param end_y
 */
void Figure_Graphic(ui_graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
                    const char* name,
                    uint32_t operate_tpye,
                    uint32_t graphic_tpye,//绘制什么图像
                    uint32_t layer,
                    uint32_t color,
                    uint32_t start_angle,
                    uint32_t end_angle,
                    uint32_t width,
                    uint32_t start_x,
                    uint32_t start_y,
                    uint32_t radius,
                    uint32_t end_x,
                    uint32_t end_y)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//字符索引
    graphic->operate_tpye = operate_tpye; //图层操作
    graphic->graphic_tpye = graphic_tpye;         //Char型
    graphic->layer        = layer;//都在第一层
    graphic->color        = color;//变色
    graphic->start_angle  = start_angle;
    graphic->end_angle    = end_angle;
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->radius = radius;
    graphic->end_x  = end_x;
    graphic->end_y  = end_y;
}

static void draw_static_string(ext_string_data_t* ui_string)
{

    switch (state_first_graphic) {
        case 0:{
            char first_line[30]  = {"CHASSIS:"};//
            String_Graphic(&ui_string->clientData, "CL1", static_update_flag, UI_ZERO_LAYER, UI_PINK, 15, strlen(first_line), 2, 320,
                           620, first_line);
        }
            break;
        case 1: {
            char second_line[30] = {" GIMBAL:"};//云台模式
            String_Graphic(&ui_string->clientData, "CL2", static_update_flag, UI_ZERO_LAYER, UI_PINK, 15, strlen(second_line), 2, 320,
                           680, second_line);
            break;
        }
        case 2: {
            char third_line[30]={"   FIRE:"};
            String_Graphic(&ui_string->clientData, "CL3", static_update_flag, UI_ZERO_LAYER, UI_PINK, 15, strlen(third_line), 2, 320,
                           740, third_line);
            break;
        }
        case 3: {
            char fourth_line[30] = {"    LIP:"};
            String_Graphic(&ui_string->clientData, "CL4", static_update_flag, UI_ZERO_LAYER, UI_PINK, 15, strlen(fourth_line), 2, 320,
                           800, fourth_line);
            break;
        }
        case 4:{
            char cap_line[30]={"CAP:"};
            String_Graphic(&ui_string->clientData,"CAP",static_update_flag,UI_ZERO_LAYER,UI_ORANGE,15, strlen(cap_line),2,1920-550,
                           400,cap_line);
            break;
        }
        case 5:{
            char status_line[30]={"OFFLINE"};
            String_Graphic(&ui_string->clientData,"CAP",static_update_flag,UI_ZERO_LAYER,UI_ORANGE,15, strlen(status_line),2,1920-550,
                           400,status_line);
            break;
        }

//
//        case 5:{
//            char cap_line[30]={"CAP:"};
//            String_Graphic(&ui_string->clientData,"CAP",static_update_flag,UI_ZERO_LAYER,UI_ORANGE,15, strlen(cap_line),2,1920-550,
//                           400,cap_line);
//            break;
//        }
//
//        case 5:{
//            char cap_line[30]={"CAP:"};
//            String_Graphic(&ui_string->clientData,"CAP",static_update_flag,UI_ZERO_LAYER,UI_ORANGE,15, strlen(cap_line),2,1920-550,
//                           400,cap_line);
//            break;
//        }
//
//        case 5:{
//            char cap_line[30]={"CAP:"};
//            String_Graphic(&ui_string->clientData,"CAP",static_update_flag,UI_ZERO_LAYER,UI_ORANGE,15, strlen(cap_line),2,1920-550,
//                           400,cap_line);
//            break;
//        }

        default:
        {
            state_first_graphic=0;
        }
            break;
    }
}

/**
 * 绘制浮点数
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param size
 * @param decimal
 * @param width
 * @param start_x
 * @param start_y
 * @param number
 */
void Float_Graphic(ui_graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
                   const char* name,
                   uint32_t operate_tpye,
                   uint32_t graphic_tpye,//绘制什么图像
                   uint32_t layer,
                   uint32_t color,
                   uint32_t size,
                   uint32_t decimal,
                   uint32_t width,
                   uint32_t start_x,
                   uint32_t start_y,
                   float number)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//字符索引
    graphic->operate_tpye = operate_tpye; //图层操作
    graphic->graphic_tpye = graphic_tpye;
    graphic->layer        = layer;//
    graphic->color        = color;//变色
    graphic->start_angle  = size;
    graphic->end_angle    = decimal;//小数有效位
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->number       = number*1000;//浮点类型的要成1000后转换为一个int32类型的
}

/**
 * 绘制整形
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param size
 * @param zero
 * @param width
 * @param start_x
 * @param start_y
 * @param number
 */
void Int_Graphic(ui_graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
                 const char* name,
                 uint32_t operate_tpye,
                 uint32_t graphic_tpye,//绘制什么图像
                 uint32_t layer,
                 uint32_t color,
                 uint32_t size,
                 uint32_t zero,
                 uint32_t width,
                 uint32_t start_x,
                 uint32_t start_y,
                 int32_t number)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//字符索引
    graphic->operate_tpye = operate_tpye; //图层操作
    graphic->graphic_tpye = graphic_tpye;
    graphic->layer        = layer;//都在第一层
    graphic->color        = color;//变色
    graphic->start_angle  = size;
    graphic->end_angle    = zero;
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->number       = number;
}

/*
 * 静元素绘画,即每次都要绘画   所以绘制图像命令每次都要ADD
 */
void zero_layer_draw() {
    ext_string_data_t ui_string;

    if (state_first_graphic >= 5) {
        state_first_graphic = 0;
    }
    //整帧的帧头
    ui_string.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_string.txFrameHeader.data_length =
            sizeof(ext_student_interactive_header_data_t) + sizeof(ui_string_t);
    ui_string.txFrameHeader.seq = 0;//包序号
    memcpy(ClientTxBuffer, &ui_string.txFrameHeader, sizeof(frame_header_struct_t));
    append_CRC8_check_sum(ClientTxBuffer, sizeof(frame_header_struct_t));//头校验

    //数据帧的帧头
    // 命令码 整帧命令的ID是与裁判系统通信的整帧数据的ID Referee_开头
    ui_string.CmdID=Referee_ID_robot_interactive_header_data;//机器人间交互的命令ID 是整个帧的ID

    //此处的ID为数据内容的ID UI_INTERACT开头
    ui_string.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_char_graphic;//机器人交互内容的ID 绘画字符的ID
    ui_string.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_string.dataFrameHeader.receiver_ID=Referee.SelfClient;

    //数据内容填充
    draw_static_string(&ui_string);//绘制静态元素 元素类型为char 字符串类型

    //将除帧头部分放入缓冲区
    memcpy(ClientTxBuffer +  Referee_LEN_FRAME_HEAD, (uint8_t *) &ui_string.CmdID,
           Referee_LEN_CMD_ID + ui_string.txFrameHeader.data_length);
    //对发送数组增加CRC16校验位
    append_CRC16_check_sum(ClientTxBuffer, sizeof(ui_string));
    //串口发送
    usart6_tx_dma_enable(ClientTxBuffer, sizeof(ui_string));

    //第O层绘画的字符串更新
    state_first_graphic++;
}
void seven_aimLine_draw(){
    ext_graphic_seven_data_t ui_aim_line;//发送变量

    //裁判通信帧头处理

    ui_aim_line.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_aim_line.txFrameHeader.data_length=sizeof (ext_student_interactive_header_data_t)+
                                          sizeof(ui_graphic_data_struct_t)*7;
    ui_aim_line.txFrameHeader.seq=0;    //数据内容
    //将画图的内容赋值到的xxx_data_struct中


    memcpy(ClientTxBuffer,&ui_aim_line.txFrameHeader,sizeof(frame_header_struct_t));
    //CRC8
    append_CRC8_check_sum(ClientTxBuffer,sizeof(frame_header_struct_t));
    ui_aim_line.CmdID= Referee_ID_robot_interactive_header_data;

    //数据帧头
    ui_aim_line.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_aim_line.dataFrameHeader.receiver_ID=Referee.SelfClient;
    ui_aim_line.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_seven_graphic;

    Figure_Graphic(&ui_aim_line.clientData[0],"LI1",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2-80,1080/2-80,
                   0,1920/2+80,1080/2-80);
    Figure_Graphic(&ui_aim_line.clientData[1],"LI2",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2-60,1080/2-120,
                   0,1920/2+60,1080/2-120);
    Figure_Graphic(&ui_aim_line.clientData[2],"LI3",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2-40,1080/2-160,
                   0,1920/2+40,1080/2-160);
    Figure_Graphic(&ui_aim_line.clientData[3],"LI4",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2-20,1080/2-200,
                   0,1920/2+20,1080/2-200);
    Figure_Graphic(&ui_aim_line.clientData[4],"LI5",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2-10,1080/2-240,
                   0,1920/2+10,1080/2-240);
    Figure_Graphic(&ui_aim_line.clientData[5],"LI6",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2,1080/2-40,
                   0,1920/2,1080/2-240);
    Figure_Graphic(&ui_aim_line.clientData[6],"LI7",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2-100,1080/2-40,
                   0,1920/2+100,1080/2-40);

    //将除帧头部分放入缓冲区
    memcpy(ClientTxBuffer+Referee_LEN_FRAME_HEAD,(uint8_t *)&ui_aim_line.CmdID,
           Referee_LEN_CMD_ID+ui_aim_line.txFrameHeader.data_length);
    //帧尾CRC16处理
    append_CRC16_check_sum(ClientTxBuffer,sizeof(ui_aim_line));
    //串口发送
    usart6_tx_dma_enable(ClientTxBuffer,sizeof(ui_aim_line));
}

void self_line_draw(){
    ext_graphic_five_data_t ui_selfLine;

    ui_selfLine.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_selfLine.txFrameHeader.data_length=sizeof (ext_student_interactive_header_data_t)+
                                          sizeof(ui_graphic_data_struct_t)*5;
    ui_selfLine.txFrameHeader.seq=0;    //数据内容
    //将画图的内容赋值到的xxx_data_struct中

    memcpy(ClientTxBuffer,&ui_selfLine.txFrameHeader,sizeof(frame_header_struct_t));
    //CRC8
    append_CRC8_check_sum(ClientTxBuffer,sizeof(frame_header_struct_t));
    ui_selfLine.CmdID= Referee_ID_robot_interactive_header_data;

    //数据帧头
    ui_selfLine.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_selfLine.dataFrameHeader.receiver_ID=Referee.SelfClient;
    ui_selfLine.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_five_graphic;

    Figure_Graphic(&ui_selfLine.clientData[1],"SL0",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,563,1080-1078,
                   0,726,1080-839);

    Figure_Graphic(&ui_selfLine.clientData[2],"SL1",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,726,1080-839,
                   0,1257,1080-839);

    Figure_Graphic(&ui_selfLine.clientData[3],"SL2",one_layer_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1257,1080-839,
                   0,1379,1080-1079);

    Figure_Graphic(&ui_selfLine.clientData[4],"SL2",UI_NONE,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1257,1080-839,
                   0,1379,1080-1079);

    Figure_Graphic(&ui_selfLine.clientData[5],"SL2",UI_NONE,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1257,1080-839,
                   0,1379,1080-1079);

    //将除帧头部分放入缓冲区
    memcpy(ClientTxBuffer+Referee_LEN_FRAME_HEAD,(uint8_t *)&ui_selfLine.CmdID,
           Referee_LEN_CMD_ID+ui_selfLine.txFrameHeader.data_length);
    //帧尾CRC16处理
    append_CRC16_check_sum(ClientTxBuffer,sizeof(ui_selfLine));
    //串口发送
    usart6_tx_dma_enable(ClientTxBuffer,sizeof(ui_selfLine));

}

void one_layer_draw()
{
    static uint8_t oneLayerStep=-1;
    oneLayerStep++;

    switch (oneLayerStep) {
        case 0:{
            seven_aimLine_draw();
        }break;

        case 1:{
            self_line_draw();
        }break;

        default:{
            oneLayerStep=-1;
            one_layer_update_flag=UI_MODIFY;
        }break;
    }
}

int32_t cap_percentage=90;

void cap_percentage_draw()
{
    ext_graphic_two_data_t ui_cap_percentage;
    {
        //TODO:暂时这么先，等有电容后修改这部分
        if(KeyBoard.SHIFT.status==KEY_PRESS)
        {
            cap_percentage--;
        }
        else{
            cap_percentage++;
        }
        VAL_LIMIT(cap_percentage,0,100);
    }

    ui_cap_percentage.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_cap_percentage.txFrameHeader.data_length=sizeof (ext_student_interactive_header_data_t)+
                                                sizeof(ui_graphic_data_struct_t)*2;
    ui_cap_percentage.txFrameHeader.seq=0;

    memcpy(ClientTxBuffer,&ui_cap_percentage.txFrameHeader,sizeof(frame_header_struct_t));
    //CRC8
    append_CRC8_check_sum(ClientTxBuffer,sizeof(frame_header_struct_t));

    ui_cap_percentage.CmdID= Referee_ID_robot_interactive_header_data;
    ui_cap_percentage.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_cap_percentage.dataFrameHeader.receiver_ID=Referee.SelfClient;
    ui_cap_percentage.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_two_graphic;

    int32_t end_angle=cap_percentage*360/100;
    if(cap_percentage<=0)
    {
        end_angle=1;
    }
    if(cap_percentage>=50)
    {
        Int_Graphic(&ui_cap_percentage.clientData[0],"CFL",two_layer_update_flag,UI_INT,
                    UI_TWO_LAYER,UI_GREEN,17,0,2,1920-528,308,cap_percentage);
        Figure_Graphic(&ui_cap_percentage.clientData[1],"CAC",two_layer_update_flag,UI_ARC,
                       UI_THREE_LAYER,UI_GREEN,0,end_angle,8,1920-520,300,0,50,50);
    }
    else{
        Int_Graphic(&ui_cap_percentage.clientData[0],"CFL",two_layer_update_flag,UI_INT,
                    UI_TWO_LAYER,UI_RED_BLUE,17,0,2,1920-528,308,cap_percentage);
        Figure_Graphic(&ui_cap_percentage.clientData[1],"CAC",two_layer_update_flag,UI_ARC,
                       UI_THREE_LAYER,UI_RED_BLUE,0,end_angle,8,1920-520,300,0,50,50);
    }

    memcpy(ClientTxBuffer+Referee_LEN_FRAME_HEAD,(uint8_t *)&ui_cap_percentage.CmdID,
           Referee_LEN_CMD_ID + ui_cap_percentage.txFrameHeader.data_length);
    append_CRC16_check_sum(ClientTxBuffer,sizeof(ui_cap_percentage));
    usart6_tx_dma_enable(ClientTxBuffer,sizeof(ui_cap_percentage));
}

void two_layer_draw(){
    static uint8_t twoLayerStep=-1;
    twoLayerStep++;
    switch (twoLayerStep) {
        case 0:{
            cap_percentage_draw();
        }break;

//        case 1:{
//
//        }break;
        default:{
          twoLayerStep=-1;
          two_layer_update_flag=UI_MODIFY;
        }break;
    }

}

void chassis_mode_draw(){

    ext_string_data_t ui_chassis_mode;
    ui_chassis_mode.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_chassis_mode.txFrameHeader.data_length=sizeof (ext_student_interactive_header_data_t)+
                                              sizeof (ext_string_data_t);
    ui_chassis_mode.txFrameHeader.seq=0;
    memcpy(ClientTxBuffer,&ui_chassis_mode.txFrameHeader,sizeof(ext_student_interactive_header_data_t));

    append_CRC8_check_sum(ClientTxBuffer,sizeof (ext_student_interactive_header_data_t));

    ui_chassis_mode.CmdID=Referee_ID_robot_interactive_header_data;//0X301
    ui_chassis_mode.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_char_graphic;
    ui_chassis_mode.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_chassis_mode.dataFrameHeader.receiver_ID=Referee.SelfClient;

    //根据对应模式塞字符串进入ClientData
    switch (ui_robot_status.chassis_mode) {
        case CHASSIS_RELAX:
        {
            char str[30]={" RELAX"};//局部变量
            String_Graphic(&ui_chassis_mode.clientData, "CME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320,680, str);
        }break;

        case CHASSIS_SPIN:
        {
            char str[30]={"  SPIN"};
            String_Graphic(&ui_chassis_mode.clientData, "CME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320,680, str);
        }break;

        case CHASSIS_FOLLOW_GIMBAL:
        {
            char str[30]={"FOLLOW"};
            String_Graphic(&ui_chassis_mode.clientData, "CME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320,680, str);
        }break;

        case CHASSIS_BLOCK:
        {
            char str[30]={" BLOCK"};
            String_Graphic(&ui_chassis_mode.clientData, "CME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320,680, str);
        }break;

        case CHASSIS_ONLY:
        {
            char str[30]={"  ONLY"};
            String_Graphic(&ui_chassis_mode.clientData, "CME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320,680, str);
        }break;

        default:
        {

        }break;
    }

    memcpy(ClientTxBuffer+Referee_LEN_FRAME_HEAD,(uint8_t *)&ui_chassis_mode.CmdID,
           Referee_LEN_CMD_ID+ui_chassis_mode.txFrameHeader.data_length);
    append_CRC16_check_sum(ClientTxBuffer,sizeof(ui_chassis_mode));
    usart6_tx_dma_enable(ClientTxBuffer,sizeof(ui_chassis_mode));
}

void gimbal_mode_draw(){
    ext_string_data_t ui_gimbal_mode;
    ui_gimbal_mode.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_gimbal_mode.txFrameHeader.data_length= sizeof (ext_student_interactive_header_data_t) +
                                              sizeof (ext_string_data_t);
    ui_gimbal_mode.txFrameHeader.seq=0;
    memcpy(ClientTxBuffer, &ui_gimbal_mode.txFrameHeader, sizeof(ext_student_interactive_header_data_t));

    append_CRC8_check_sum(ClientTxBuffer,sizeof (ext_student_interactive_header_data_t));

    ui_gimbal_mode.CmdID=Referee_ID_robot_interactive_header_data;//0X301
    ui_gimbal_mode.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_char_graphic;
    ui_gimbal_mode.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_gimbal_mode.dataFrameHeader.receiver_ID=Referee.SelfClient;

    //根据对应模式塞字符串进入ClientData
    switch (ui_robot_status.gimbal_mode) {
        case GIMBAL_ACTIVE:
        {
            char str[30]={"ACTIVE"};//局部变量
            String_Graphic(&ui_gimbal_mode.clientData, "GME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320, 680, str);
        }break;

        case GIMBAL_AUTO:
        {
            char str[30]={"  AUTO"};
            String_Graphic(&ui_gimbal_mode.clientData, "GME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320, 680, str);
        }break;

        case GIMBAL_BUFF:
        {
            char str[30]={"  BUFF"};
            String_Graphic(&ui_gimbal_mode.clientData, "GME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320, 680, str);
        }break;

        case GIMBAL_RELAX:
        {
            char str[30]={" RELAX"};
            String_Graphic(&ui_gimbal_mode.clientData, "GME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320, 680, str);
        }break;

        default:
        {
            char str[30]={"OFFLINE"};
            String_Graphic(&ui_gimbal_mode.clientData, "GME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320, 680, str);
        }break;
    }

    memcpy(ClientTxBuffer+Referee_LEN_FRAME_HEAD, (uint8_t *)&ui_gimbal_mode.CmdID,
           Referee_LEN_CMD_ID+ui_gimbal_mode.txFrameHeader.data_length);
    append_CRC16_check_sum(ClientTxBuffer,sizeof(ui_gimbal_mode));
    usart6_tx_dma_enable(ClientTxBuffer,sizeof(ui_gimbal_mode));
}
void fire_mode_draw(){
    ext_string_data_t ui_fire_mode;
    ui_fire_mode.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_fire_mode.txFrameHeader.data_length=sizeof (ext_student_interactive_header_data_t)+
                                           sizeof (ext_string_data_t);

    ui_fire_mode.txFrameHeader.seq=0;

    memcpy(ClientTxBuffer,(uint8_t*)&ui_fire_mode.txFrameHeader,sizeof (ext_student_interactive_header_data_t));
    append_CRC8_check_sum(ClientTxBuffer,sizeof(ext_student_interactive_header_data_t));

    ui_fire_mode.CmdID=Referee_ID_robot_interactive_header_data;//0X301
    ui_fire_mode.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_char_graphic;
    ui_fire_mode.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_fire_mode.dataFrameHeader.receiver_ID=Referee.SelfClient;

    switch (ui_robot_status.fire_mode) {
        case Fire_ON:
        {
            char str[30]={"    ON"};//局部变量
            String_Graphic(&ui_fire_mode.clientData, "FME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320, 680, str);
        }break;

        case Fire_OFF:
        {
            char str[30]={"   OFF"};//局部变量
            String_Graphic(&ui_fire_mode.clientData, "FME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320, 680, str);
        }
        default:
        {
            char str[30]={"OFFLINE"};
            String_Graphic(&ui_fire_mode.clientData, "FME", three_layer_update_flag, UI_THREE_LAYER, UI_PINK,
                           15, strlen(str), 2, 320, 680, str);
        }break;
    }

}
void three_layer_draw(){
    static uint8_t threeLayerStep=-1;
    threeLayerStep++;

    switch (threeLayerStep) {
        case 0:
        {
            chassis_mode_draw();
        }break;

        case 1:
        {
            gimbal_mode_draw();
        }

        case 2:
        {
            fire_mode_draw();
        }
        default:{
            threeLayerStep=-1;
        }break;
    }


}
uint32_t static_cnt=0;

void UI_all_draw(){
    static int step=-1;
    step++;
    switch (step) {
        case 0:{
            zero_layer_draw();
        }break;

        case 1:{
            one_layer_draw();
        }break;

        case 2:{
            two_layer_draw();
        }break;

        case 3:{
            three_layer_draw();
        }break;

        default:
        {
            step=-1;
        }break;
    }

}

void UI_dynamic_draw(){
    static int dynamic_step=-1;//绘制动态元素步骤
    //动态元素绘画
    dynamic_step++;
    switch (dynamic_step) {

        case 0://
        {
            two_layer_draw();
        }break;

        default:{
            dynamic_step=-1;
        }break;
    }


}
static void UI_reset_flag()
{
    static_update_flag=UI_ADD;
    one_layer_update_flag=UI_ADD;
    two_layer_update_flag=UI_ADD;
}


static void UI_set_flag()
{
    static_update_flag=UI_NONE;
    one_layer_update_flag=UI_MODIFY;
    two_layer_update_flag=UI_MODIFY;
    three_layer_update_flag=UI_MODIFY;
}

//分为动态和静态，B键一下刷新所有图形
void UI_parse_info()
{

}
void UI_paint_task(void const*argument){
    vTaskDelay(20);
    bool isReset=false;

    while (1) {

        //通过按键开启刷新
        if (KeyBoard.B.click_flag == 1)//B点击一下更新一下所有图形
        {
            UI_reset_flag();
            isReset=true;
            static_cnt = HAL_GetTick();//记录个时间，过段时间后就改为MODIFY
            KeyBoard.B.click_flag = 0;
        }

        if (isReset)//true  刷新所有元素
        {
            UI_all_draw();
            if (HAL_GetTick() - static_cnt > 3000) {//目前3000可以刷新全部元素
                isReset=false;//表示重新绘制完成
            }
        }
        else {
            UI_dynamic_draw();
        }

        vTaskDelay(100);//20ms 底盘功率发送频率是50HZ
    }
}