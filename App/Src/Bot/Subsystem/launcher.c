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
#include <stdbool.h>
#include "Subsystem/launcher.h"
#include "Subsystem/Gimbal.h"
#include "IO/can_receive.h"
#include "Subsystem/Chassis.h"
#include "cmsis_os.h"
#include "IO/remote.h"
#include "key_board.h"
#include "bsp_laser.h"
#include "Filter/filter.h"
uint8_t rc_last_sw_L;
uint8_t rc_last_sw_R;
uint8_t blocked_flag;
uint8_t reverse_flag;
uint32_t continue_shoot_time;//遥控器左边拨杆down的持续时间 或者 鼠标左键按下的持续时间
uint32_t blocked_start_time;
uint32_t reverse_start_time;
first_order_filter_type_t filter_trigger_rpm_in;
extern key_board_t KeyBoard;
extern struct RCCtrl rc_ctrl;
extern gimbal_t gimbal;
extern motor_measure_t motor_2006_measure[1];
extern struct Chassis chassis;

bool is_blocked();
void launcher_relax_handle();
void trigger_block_handle();

//通过赋值进行发射机构的初始化
launcher_t launcher;

int32_t total_ecd_ref;

void launcher_mode_set(){

    //摩擦轮开启时,做拨杆向上拨一下关闭摩擦轮


    //拨轮控制 拨轮转动的条件是 当前摩擦轮是开的
    //要么遥控器拨杆向下播一下
    //要么鼠标左键点一下
    if(launcher.fire_mode==Fire_ON
       &&((switch_is_down(rc_ctrl.rc.s[RC_s_L])||KeyBoard.Mouse_l.status==KEY_CLICK)
          ||KeyBoard.Mouse_l.status==KEY_PRESS
          ||launcher.trigger_cmd==SHOOT_ING))
    {
        if((!switch_is_down(rc_last_sw_L)&&switch_is_down(rc_ctrl.rc.s[RC_s_L]))
           ||KeyBoard.Mouse_l.status==KEY_CLICK&&launcher.trigger_cmd!=SHOOT_ING)//launcher.trigger_cmd!=SHOOT_ING 表示当前不处于单发状态中
        {
            launcher.trigger_cmd=SHOOT_SINGLE;
            continue_shoot_time=HAL_GetTick();//计时
        }

        //计时达到触发连发的时间
        if((switch_is_down(rc_ctrl.rc.s[RC_s_L])&&CONTINUES_SHOOT_TIMING_COMPLETE())
            ||KeyBoard.Mouse_l.status==KEY_PRESS)

        {
            launcher.trigger_cmd=SHOOT_CONTINUES;
        }
    }
    else //摩擦轮没开默认关闭
    {
        launcher.trigger_cmd=SHOOT_CLOSE;
    }

    rc_last_sw_L=rc_ctrl.rc.s[RC_s_L];
    rc_last_sw_R=rc_ctrl.rc.s[RC_s_R];
}
//发射机构控制
void launcher_control(){

    if(gimbal.mode==GIMBAL_RELAX)
    {
        launcher_relax_handle();
    }
    else {

        if (launcher.fire_mode == Fire_ON) {
            launcher.fire_l.speed = -FIRE_SPEED_MAX;
            launcher.fire_r.speed = FIRE_SPEED_MAX;
        } else {
            launcher.fire_l.speed = 0;
            launcher.fire_r.speed = 0;
        }

        if (launcher.trigger_cmd == SHOOT_CLOSE) {
            launcher.trigger.speed = 0;
        }
        else if (launcher.trigger_cmd == SHOOT_SINGLE) { //收到单发命令
            launcher.trigger_cmd = SHOOT_ING;//进入正在单发状态
            total_ecd_ref = launcher.trigger.motor_measure->total_ecd + DEGREE_45_TO_ENCODER;//单发拨动45° 设置期望值
        }
        else if (launcher.trigger_cmd == SHOOT_ING) { //单发状态
            //判断是否到达目标位置
            if (ABS(total_ecd_ref - launcher.trigger.motor_measure->total_ecd) < 2048)//差值小于5° 8192/36*360/5
            {
                if(KeyBoard.Mouse_l.status!=KEY_PRESS)
                {
                    launcher.trigger_cmd = SHOOT_CLOSE;
                    launcher.trigger.speed = 0;
                }
            } else {
                launcher.trigger.speed = pid_calc(&launcher.trigger.angle_p, launcher.trigger.motor_measure->total_ecd,
                                                  total_ecd_ref);
            }
        }

        if (launcher.trigger_cmd == SHOOT_CONTINUES) {
            launcher.trigger.speed = TRIGGER_CONTINUES_SPEED;
            trigger_block_handle();
        }
        first_order_filter_cali(&filter_trigger_rpm_in,launcher.trigger.motor_measure->speed_rpm);

        launcher.trigger.give_current = pid_calc(&launcher.trigger.speed_p,filter_trigger_rpm_in.out,
                                                 launcher.trigger.speed);

        launcher.fire_l.give_current = pid_calc(&launcher.fire_l.speed_p, launcher.fire_l.motor_measure->speed_rpm,
                                                launcher.fire_l.speed);
        launcher.fire_r.give_current = pid_calc(&launcher.fire_r.speed_p, launcher.fire_r.motor_measure->speed_rpm,
                                                launcher.fire_r.speed);

    }
}

//发射机构失能
void launcher_relax_handle(){

    launcher.trigger.give_current=0;
    launcher.fire_r.give_current=0;
    launcher.fire_l.give_current=0;

}

void launcher_init(){
    blocked_flag=false;//堵转标志位置0

    reverse_flag=false;//反转标志位置0

    launcher.trigger_cmd=SHOOT_CLOSE;//初始时发射机构默认关闭

    launcher.fire_last_mode=Fire_OFF;//初始时摩擦轮默认关闭

    launcher.fire_mode=Fire_OFF;//初始时摩擦轮默认关闭

    //获取发射机构电机数据结构体
    launcher.fire_l.motor_measure=&motor_3508_measure[FIRE_L];
    launcher.fire_r.motor_measure=&motor_3508_measure[FIRE_R];
    launcher.trigger.motor_measure=&motor_2006_measure[TRIGGER];

    //发射机构电机PID初始化
    launcher.fire_r.speed_p.p=SHOOT_FIRE_R_PID_KP;
    launcher.fire_r.speed_p.i=SHOOT_FIRE_R_PID_KI;
    launcher.fire_r.speed_p.d=SHOOT_FIRE_R_PID_KD;
    launcher.fire_r.speed_p.max_output=SHOOT_FIRE_R_PID_MAX_OUT;
    launcher.fire_r.speed_p.integral_limit=SHOOT_FIRE_R_PID_MAX_IOUT;

    launcher.fire_l.speed_p.p=SHOOT_FIRE_L_PID_KP;
    launcher.fire_l.speed_p.i=SHOOT_FIRE_L_PID_KI;
    launcher.fire_l.speed_p.d=SHOOT_FIRE_L_PID_KD;
    launcher.fire_l.speed_p.max_output=SHOOT_FIRE_L_PID_MAX_OUT;
    launcher.fire_l.speed_p.integral_limit=SHOOT_FIRE_L_PID_MAX_IOUT;

    launcher.trigger.angle_p.p=SHOOT_TRI_ANGLE_PID_KP;
    launcher.trigger.angle_p.i=SHOOT_TRI_ANGLE_PID_KI;
    launcher.trigger.angle_p.d=SHOOT_TRI_ANGLE_PID_KD;
    launcher.trigger.angle_p.max_output=SHOOT_TRI_ANGLE_PID_MAX_OUT;
    launcher.trigger.angle_p.integral_limit=SHOOT_TRI_ANGLE_PID_MAX_IOUT;

    launcher.trigger.speed_p.p=SHOOT_TRI_SPEED_PID_KP;
    launcher.trigger.speed_p.i=SHOOT_TRI_SPEED_PID_KI;
    launcher.trigger.speed_p.d=SHOOT_TRI_SPEED_PID_KD;
    launcher.trigger.speed_p.max_output=SHOOT_TRI_SPEED_PID_MAX_OUT;
    launcher.trigger.speed_p.integral_limit=SHOOT_TRI_SPEED_PID_MAX_IOUT;

    launcher.trigger.motor_measure->total_ecd=launcher.trigger.motor_measure->offset_ecd=launcher.trigger.motor_measure->ecd;
    //最开始的编码值作为拨轮电机的校准值
    first_order_filter_init(&filter_trigger_rpm_in,1,1);
}

//判断是否堵转
bool is_blocked(){
    if(TRIGGER_CONTINUES_SPEED>0){
        if (blocked_flag==false && launcher.trigger.motor_measure->speed_rpm <= 0.3*TRIGGER_CONTINUES_SPEED){
            blocked_start_time=HAL_GetTick();//获取堵转开始时间
            blocked_flag=true;
        }

        //标识位为1时，已经开始堵转，判断是否堵转达到一定时间，若达到，则判定堵转
        if (blocked_flag==true && launcher.trigger.motor_measure->speed_rpm <= 0.3*TRIGGER_CONTINUES_SPEED){
            if(CONTINUES_BLOCKED_JUDGE()){
                blocked_flag=false;
                return true;
            }
        }
        return false;
    }

    else if(TRIGGER_CONTINUES_SPEED<0){
        //在标识位为0时，电机转速低于阈值时，判定堵转开始
        if (blocked_flag==false && launcher.trigger.motor_measure->speed_rpm >= 0.3*TRIGGER_CONTINUES_SPEED){
            blocked_start_time=HAL_GetTick();//获取堵转开始时间
            blocked_flag=true;
        }

        //标识位为1时，已经开始堵转，判断是否堵转达到一定时间，若达到，则判定堵转
        if (blocked_flag==true && launcher.trigger.motor_measure->speed_rpm >= 0.3*TRIGGER_CONTINUES_SPEED){
            if(CONTINUES_BLOCKED_JUDGE()){
                blocked_flag=false;
                return true;
            }
        }
        return false;
    }
}
//堵转处理函数
void  trigger_block_handle(){
    //判断堵转并且反转标识为0时
    if(is_blocked() && reverse_flag==false){
        reverse_flag=true;//判定开始反转
        reverse_start_time=HAL_GetTick();//获取开始反转时间
    }

    //判定反转开始并且时间没有达到反转结束时间
    if(reverse_flag==true && TRIGGER_REVERSE_TIME_JUDGE()){
        launcher.trigger.speed=TRIGGER_REVERSE_SPEED;//拨单电机设置为反转速度
    }

    else{
        reverse_flag=false;
    }
}

//发射机构热量限制
void launcher_shooting_limit(){

}