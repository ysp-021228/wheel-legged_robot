/* 
 * File:          dan_bian_wu_lian_gan.c
 * Date:          2023-9-20
 * Description:   单边五连杆，以前关节作为A点
 * Author:        LZB
 * Modifications: 
 */

/*
 * 
    一、以后大腿与车体连接点作为A点，前大腿与车体连接作为E点更符合整车的坐标系
   
   
    二、以整车正方向为前，从右侧看:
     ---------状态变量-----------
     1.所有关节顺时针转时编码器为正值
     2.轮子往车头方向走是正位移、速度
     3.以ZJ的图进行倒放，图中的箭头方向就是webot中角度的变大方向
     4.当车体往前倾斜，机体倾角、角速度为正
     5.虚拟关节与轮子连接的线如果在竖直线的左边，形成的角是负的

     ---------机体角度,角速度,加速度----------
     1.三轴加速度沿着三个坐标的正方向时为正
     2.当车体往前倾斜，机体倾角、角速度为正;顺着Y方向的旋转轴旋转，Roll角度和角速度为正

     ---------扭矩---------------
     1.轮子往前转是正的扭矩
     2.2个关节电机顺时针方向转的扭矩是正的

*/

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <math.h>                    //三角运算
#include <string.h>                  //清空结构体操作
#include <stdio.h>                   //打印数据
#include <webots/position_sensor.h>  //测位置
#include <webots/motor.h>            //可获取速度
#include <webots/inertial_unit.h>    //测角度
#include <webots/accelerometer.h>    //测加速度
#include <webots/gyro.h>             //测角速度
#include <webots/camera.h>
#include <webots/keyboard.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP                       3  
#define STATUS_VAL_NUMS                 6       //状态变量个数
#define WHEEL_RADIUS                    0.03f   //轮子半径
#define PAI_VAL                         3.1415926f  //Π的大小
#define BIG_LEG_INITITAL_ANGLE          (60*0.0174444444f) //60°变为弧度，表示做好webots模型后大腿形成的倾角，当然假设一开始做的模型是前后腿完全对称，那么这个宏定义其实是fi_1的补角
#define WHEEL_MOTOR_MAX_TORQUE          5       //轮子电机最大扭矩
#define JOINT_MOTOR_MAX_TORQUE          30      //关节电机最大扭矩
#define MAX_SPEED_TAR                   1.5f    //最大的速度目标值
#define MAX_VIRTUAL_LEG_LENGTH          0.28f   //最大的虚拟腿长
#define NORMAL_VIRTUAL_LEG_LENGTH       0.25f   //常态下的虚拟腿长
#define MIN_VIRTUAL_LEG_LENGTH          0.18f   //最大的虚拟腿长 
 
#define abs(x)                        ( (x>0)? (x) : (-(x)) )               //符号函数  
#define constrain(x, max, min)        ( (x>max)?(max):(x<min)?(min):(x) )   //限幅函数
#define add(x, inc, max)                do{x += inc; if (x>=max) x = max;}while(0)         //x变量递增inc这么多,最多达到max值
#define sub(x, dec, min)                do{x -= dec; if (x<=min) x = min;}while(0)         //x变量递减dec这么多,最小达到min值

/*
 * -----------------------------------------------用户定义的私有变量类型-------------------------------------------------
 */ 
typedef enum PID_type//表示PID的类型
{
    POSITION_PID = 0, //位置式

    DELTA_PID, //增量式
}PID_type_e;
typedef enum PID_Err_Index//表示数组索引的序号,PID运算中会使用到上上次、上次、这次的误差
{
    LLAST = 0,

    LAST,

    NOW,
}PID_Err_Index_e;
typedef enum Jumping_State //表示跳跃的过程,这里就贪图方面看，不全大写了
{
    Normal = 0, //正常状态

    Stretch_ing, //伸腿中
    
    Shrink_ing, //缩腿中

    Kick_ing, //踢腿中

    Fall_ing, //自由落体，是因为在伸腿、缩腿、踢腿中主持力不可信！

    Land, //着陆
}Jumping_State_e;
typedef enum Flag_State//表示一些标志位的状态
{
    FLASE = 0,
    TRUE = 1,
}Flag_State_e;
typedef enum Devices_Name_Index //设备名字数组索引的序号(即Devices_Name_List数组),并且设备名字数组是直接用webots里的名字,而这里就是多+后缀e,
{
    Wheel_Motor_e = 0,
    Wheel_Pos_Sensor_e,
    
    F_Joint_Motor_e,
    F_Joint_Pos_Sensor_e,

    B_Joint_Motor_e,
    B_Joint_Pos_Sensor_e,

    Imu_e,
    Gyro_e,
    Accelerometer_e,
    Camera_e,
    Wheel_Gyro_e,
    
    Devices_Num,//12,但是轮子编码器已经不用了

}Devices_Name_Index_e;
typedef enum Devices_Info_Index //设备信息数组索引的序号,即Motor_Info和Pos_Sensor_Info这两个数组
{
    //这里其实是在说电机或者编码器有三个位置
    F_Joint = 0,
    B_Joint,
    Wheel,
    Virtual,

    Total, //这里其实是想说电机或编码器各自共有4个，包括虚拟电机或虚拟编码器

}Devices_Info_Index_e;
typedef enum Status_Val_Index //状态变量在数组索引的序号,用在Get_Leg_Status_Tar函数、Get_Leg_Status_Vals函数中
{
    th = 0,
    th_dt,
    x,
    v,
    fi,
    fi_dt,
}Status_Val_Index_e;




typedef struct Motor //旋转电机
{
  //转速（单位：弧度/s），不是线速度
  double Spin_Speed;
  
  //扭矩目标值,tar的第一个字母就不大写了,包括fdb(feedback)
  double Torque_tar;
  
  //扭矩反馈值,但用了扭矩控制的电机,调用wb_motor_get_torque_feedback函数返回的是0
  double Torque_fdb;  

}Motor_t;
typedef struct Pos_Sensor //编码器
{
  //求走过的弧度数
  double Total_Radian;

  /*用来差分求电机角速度,只用来获取关节电机的角速度,而不获取轮子的*/
  double This_Radian;
  double Last_Radian;
  
}Pos_Sensor_t;
typedef struct Imu //陀螺仪所有信息,角度、角速度、加速度等
{
    double Pitch;
    double Yaw;
    double Roll;

    //角速度(dt表示一阶导)
    double Pitch_dt;
    double Yaw_dt;
    double Roll_dt;

    /*原始三轴加速度数值*/
    double Raw_ax;
    double Raw_ay;
    double Raw_az;

    /*去除重力加速度成分的加速度数值*/
    double Filter_ax;
    double Filter_ay;
    double Filter_az;

    /*物体竖直方向的加速度*/
    double Object_az;

}Imu_t;




typedef struct PID_t //PID结构体
{
    double p;
    double i;
    double d;

    double set[3];           //目标值,包含NOW， LAST， LLAST上上次
    double get[3];           //测量值
    double err[3];           //误差

    double d_error;
    double pout;             //p输出
    double iout;             //i输出
    double dout;             //d输出

    double pos_out;          //本次位置式输出
    double last_pos_out;     //上次输出
    double delta_u;          //本次增量值
    double delta_out;        //本次增量式输出 = last_delta_out + delta_u
    double last_delta_out;

    double max_err;
    double deadband;             //err < deadband return
    double div;
    int pid_mode;
    double MaxOutput;         //输出限幅
    double IntegralLimit;     //积分限幅

    void (*f_param_init)(struct PID_t *pid,  //PID参数初始化
                         int pid_mode,
                         double maxOutput,
                         double integralLimit,
                         double deadband,
                         double div,
                         double p, double i, double d);
    void (*f_pid_reset)(struct PID_t *pid, double p, double i, double d);		//pid三个参数修改

}PID_t; 
typedef struct Status //状态变量，变量小写好了方便看是哪个角度
{
    union
    {
        double array[STATUS_VAL_NUMS];
        struct
        {
            double th;
            double th_dt;
            double x;
            double v; 
            double fi;
            double fi_dt;
        }e;
    };
}Status_t;
typedef struct Vmc //五连杆运动学
{
    double  L[6];    //五连杆的长度,依次L0~L5,并且L[0]也就是L0

    double  q[5];    //五连杆的角度,依次是q0~q4,并且q[0]也就是q0（对应知乎文章用的fi命名的角度,只是因为状态变量里有fi了,这里就不再这么命名）

    struct   //中间变量
    {
        double  A0, B0, C0, L_BD_sq; //正运动学用到的中间变量,注意L_BD_sq是B点和D点距离的平方

        double  X_B, Y_B, X_C, Y_C, X_D, Y_D; //B、C、D三点坐标
    }Mid_Param;
    
    struct  //变化率,用来求Fn的
    {
        double This_L0_dt, Last_L0_dt, L0_ddt;  //L0的一阶导和二阶导

        double This_th, Last_th, This_th_dt;//腿与竖直方向夹角的一阶导数

        double Last_th_dt, th_ddt;  //腿与竖直方向夹角的二阶导数
    }Acc_fdb; 

    union //关节电机反馈的角速度
    {
        double array[2][1];
        struct 
        {
            double w1_fdb;
            double w4_fdb;
        }e;

    }W_fdb;
    
    union //足端的旋转线速度(其实是垂直于虚拟腿方向的速度➗腿长)和沿着虚拟腿方向的速度
    {
        double array[2][1];
        struct 
        {
            double w0_fdb; //其实是垂直于虚拟腿方向的速度➗腿长
            double vy_fdb;
        }e;

    }V_fdb;

    union //关节电机反馈扭矩
    {
        double array[2][1];
        struct 
        {
            double T1_fdb;
            double T4_fdb;
        }e; 
    }T14_fdb;
    
    union //关节电机期望扭矩
    {
        double array[2][1];
        struct
        {
            double T1_tar;
            double T4_tar;
        }e;
    }T14_tar;

    union //足端反馈力(垂直足端和沿着足端)
    {
        double array[2][1];
        struct
        {
            double Tp_fdb;
            double Fy_fdb;
        }e;
    }Fxy_fdb;

    union //足端期望力(垂直足端和沿着足端)
    {
        double array[2][1];
        struct
        {
            double Tp_tar;
            double Fy_tar;
        }e;
    }Fxy_tar;

    union //关节电机反馈的角速度------->足端的速度
    {
        double array[2][1];
        struct
        {
            double x1_1;
            double x1_2;
            double x2_1;
            double x2_2;
        }e;
    }J_w2v; 

    union //足端期望力------>关节电机期望扭矩
    {
        double array[2][1];
        struct
        {
            double x1_1;
            double x1_2;
            double x2_1;
            double x2_2;
        }e;
    }J_F2T;
    
    union //关节电机反馈扭矩------>足端反馈力
    {
        double array[2][1];
        struct
        {
            double x1_1;
            double x1_2;
            double x2_1;
            double x2_2;
        }e;
    }J_T2F;

}Vmc_t;
typedef struct Leg //腿结构体
{
    double  L0_tar;  //腿长目标值
    double  L0_At_Jump_Request; //跳跃请求瞬间的L0测量值
     
    double  Fn;      //足端支持力

    struct //一些用到的标志位
    {
        int is_speed_tar_not_zero_flag; //速度目标值不为0
        int is_speed_tar_to_zero_flag; //速度目标值突变为0

        int is_off_ground_flag; //是否离地
        int is_za_luo_on_ground_flag; //是否砸落地面
        int is_enable_off_ground_detect; //是否允许离地检测

        int is_jump_request_flag; //是否跳跃
        int the_jumping_flag; //跳跃过程的一个标志
    }Flag;
    
    Motor_t Motor_Info[Total];
  
    Pos_Sensor_t Pos_Sensor_Info[Total];

    Imu_t Wheel_Imu_Info;

    Vmc_t Vmc;

    Status_t Meas, Tar, Err; //状态变量的测量、目标、误差

    double Wheel_Motor_K_Matrix[STATUS_VAL_NUMS]; //轮子的K矩阵

    double Joint_Motor_K_Matrix[STATUS_VAL_NUMS]; //虚拟关节的K矩阵

    PID_t LegLength_PID; //腿长PID

}Leg_t;
typedef struct Chassic //底盘
{
    double v_set; //速度设置

    Leg_t Leg; //由于是单腿的代码,这里就不确定左、右方向

    Imu_t Body_Imu;

}Chassic_t;





/*
----------------------------------------------------------用户定义的函数(可直接使用)---------------------------------------------
*/
void PID_struct_init(
    PID_t* pid, int mode, double maxout, 
    double intergral_limit, double deadband, double div,
    double kp, double ki, double kd);
double PID_Calc(PID_t* pid, double fdb, double ref);

void Matrix_Multiply(int rows1, int cols1, double matrix1[rows1][cols1], 
                     int rows2, int cols2, double matrix2[rows2][cols2], 
                     double result[rows1][cols2]) ;

static void kUpdateBaseOnLeg(double L0, double K[6], const double KL[6][4]) ;





/*
*----------------------------------------------------------用户定义的变量-------------------------------------------------------
*/
int key; //键盘值
double m_Body = 6.f;
double m_Wheel = 0.5f;
const double g = 9.8f; //重力加速度g
const char *Devices_Name_List[] = //webot中设备的名字
{
    "Wheel_Motor",
    "Wheel_Pos_Sensor",

    "F_Joint_Motor",
    "F_Joint_Pos_Sensor",

    "B_Joint_Motor",
    "B_Joint_Pos_Sensor",

    "Imu",    
    "Gyro",
    "Accelerometer",
    "Camera",
    "Wheel_Gyro",
};
// 状态反馈系数关于腿长的拟合关系 K(L0),高阶的系数排在前面,依次三阶、二阶、一阶、常数
const double Wheel_Fitting_Factor[6][4] = {
 {-974.4, 538.4, -353.4, -20.7}, 
 {476.2, -242.7, -11, -3.2},
 {227.7, -80.7, 7.2, -22.1},  
 {1042.2, -393.5, 36.6, -16.4}, 
 {5169.5, -1889.8, 145.2, 28.7},
 {-77.1, 38.6, -8.9, 1.7},
};
const double Joint_Fitting_Factor[6][4] = {
 {4140.1, -2134.4, 392.6, 6.3}, 
 {-25.1, -47.4, 27.7, 1.2}, 
 {3228.7, -1200.4, 109.8, 11.4},   
 {1556.5, -600.7, 56.5, 8.2},  
 {-4166.2, 1442.7, -113.6, 439.3},
 {-98.1, 30.1, -0.8, 4.2},
};
//设备树
WbDeviceTag Devices[Devices_Num];
Chassic_t Chassic = 
{
    //腿的初始目标值
    .Leg.L0_tar = NORMAL_VIRTUAL_LEG_LENGTH,

    //五连杆的长度赋值
    .Leg.Vmc.L[1] = 0.14f, //前大腿
    .Leg.Vmc.L[2] = 0.18f, //前小腿
    .Leg.Vmc.L[3] = 0.18f, //后小腿
    .Leg.Vmc.L[4] = 0.14f, //后大腿
    .Leg.Vmc.L[5] = 0.1f, //两个关节的距离

    //腿的状态标志位
    .Leg.Flag.is_speed_tar_not_zero_flag = FLASE,
    .Leg.Flag.is_speed_tar_to_zero_flag = FLASE,

    .Leg.Flag.is_off_ground_flag = FLASE,
    .Leg.Flag.is_za_luo_on_ground_flag = FLASE,
    .Leg.Flag.is_enable_off_ground_detect = TRUE, 
    
    .Leg.Flag.is_jump_request_flag = FLASE,
    .Leg.Flag.the_jumping_flag = Normal,

}; 





/*
 *  --------------------------------------------------用户定义的获取设备的函数---------------------------------------------
    WB表示webots，设备后面加了s表示有多个的意思，比如Motors
*/
void WB_Get_Motors_Device(void) //获取电机设备
{
    /*轮子电机 INFINITY*/
    Devices[Wheel_Motor_e] = wb_robot_get_device(Devices_Name_List[Wheel_Motor_e]);
    wb_motor_enable_torque_feedback(Devices[Wheel_Motor_e], TIME_STEP);
    wb_motor_set_position(Devices[Wheel_Motor_e], INFINITY);
    
    /*2关节电机*/   
    Devices[F_Joint_Motor_e] = wb_robot_get_device(Devices_Name_List[F_Joint_Motor_e]);
    wb_motor_enable_torque_feedback(Devices[F_Joint_Motor_e], TIME_STEP);
    wb_motor_set_position(Devices[F_Joint_Motor_e], 0);
    
    Devices[B_Joint_Motor_e] = wb_robot_get_device(Devices_Name_List[B_Joint_Motor_e]);
    wb_motor_enable_torque_feedback(Devices[B_Joint_Motor_e], TIME_STEP);
    wb_motor_set_position(Devices[B_Joint_Motor_e], 0); 
    
}
void WB_Get_Pos_Sensors_Device(void) //获取编码器设备
{
    /*轮子编码器,可有可无*/
    Devices[Wheel_Pos_Sensor_e] = wb_robot_get_device(Devices_Name_List[Wheel_Pos_Sensor_e]);
    wb_position_sensor_enable(Devices[Wheel_Pos_Sensor_e], TIME_STEP);

    /*2关节编码器*/
    Devices[F_Joint_Pos_Sensor_e] = wb_robot_get_device(Devices_Name_List[F_Joint_Pos_Sensor_e]);
    wb_position_sensor_enable(Devices[F_Joint_Pos_Sensor_e], TIME_STEP);

    Devices[B_Joint_Pos_Sensor_e] = wb_robot_get_device(Devices_Name_List[B_Joint_Pos_Sensor_e]);
    wb_position_sensor_enable(Devices[B_Joint_Pos_Sensor_e], TIME_STEP);

}
void WB_Get_Imus_Device(void) //获取陀螺仪和角度计设备
{
    Devices[Imu_e] = wb_robot_get_device(Devices_Name_List[Imu_e]);
    wb_inertial_unit_enable(Devices[Imu_e], TIME_STEP);

    Devices[Gyro_e] = wb_robot_get_device(Devices_Name_List[Gyro_e]);
    wb_gyro_enable(Devices[Gyro_e], TIME_STEP);
  
    Devices[Accelerometer_e] = wb_robot_get_device(Devices_Name_List[Accelerometer_e]);
    wb_accelerometer_enable(Devices[Accelerometer_e], TIME_STEP);

    //轮子
    Devices[Wheel_Gyro_e] = wb_robot_get_device(Devices_Name_List[Wheel_Gyro_e]);
    wb_gyro_enable(Devices[Wheel_Gyro_e], TIME_STEP);
}
void WB_Get_Camera_Device(void) //获取第一视角
{
    Devices[Camera_e] = wb_robot_get_device(Devices_Name_List[Camera_e]);
    wb_camera_enable(Devices[Camera_e], TIME_STEP);
}
void WB_Get_KeyBoard_Device(void) //获取键盘
{
    wb_keyboard_enable(TIME_STEP);
}
void WB_Get_All_Device(void) //调用上面函数
{
    WB_Get_Motors_Device();
    WB_Get_Pos_Sensors_Device();
    WB_Get_Imus_Device();
    WB_Get_Camera_Device();
    WB_Get_KeyBoard_Device();
}





/*
 *  --------------------------------------------------用户使用到的设备保存数据-------------------------------------------
    WB表示要调用webots自带的api函数才能获得的数据
*/
void WB_Get_Chassic_Imu_Info(Chassic_t* Chassic) //获取机体的陀螺仪的值
{
    if(Chassic == NULL)
        return;

    const double *imu_ = NULL, *gyro_ = NULL, *acc_ = NULL;

    imu_ = wb_inertial_unit_get_roll_pitch_yaw(Devices[Imu_e]);//返回数组地址，大小是3
    gyro_ = wb_gyro_get_values(Devices[Gyro_e]);//返回数组地址，大小是3
    acc_ = wb_accelerometer_get_values(Devices[Accelerometer_e]);//返回数组地址，大小是3

    if(imu_ != NULL)
    {
        Chassic->Body_Imu.Roll  = imu_[0];//围绕X方向  
        Chassic->Body_Imu.Pitch = imu_[1];//围绕Y方向
        Chassic->Body_Imu.Yaw   = imu_[2];//围绕Y方向 
    }
    
    if(gyro_ != NULL)
    {
        Chassic->Body_Imu.Roll_dt  = gyro_[0];//围绕X方向
        Chassic->Body_Imu.Pitch_dt = gyro_[1];//围绕Y方向
        Chassic->Body_Imu.Yaw_dt = gyro_[2];//围绕Z方向  
    }
    
    if(acc_ != NULL)
    {
        //原始加速度数据
        Chassic->Body_Imu.Raw_ax = acc_[0];
        Chassic->Body_Imu.Raw_ay = acc_[1];
        Chassic->Body_Imu.Raw_az = acc_[2];

        /*往前加速时,Raw_ax为正的,而底盘前倾时Pitch为正*/
        Chassic->Body_Imu.Filter_ax = Chassic->Body_Imu.Raw_ax - g*sin(Chassic->Body_Imu.Pitch);
        
        /*从后往前看，底盘在竖直方向的右边时Roll为正数，与ay正方向相反*/
        Chassic->Body_Imu.Filter_ay = Chassic->Body_Imu.Raw_ay - g*cos(Chassic->Body_Imu.Pitch)*sin(-Chassic->Body_Imu.Roll);

        /*后面部分一直都是竖直向下，与az正方向相反，但是重力加速度的分量会让az方向测得数据是正的*/
        Chassic->Body_Imu.Filter_az = Chassic->Body_Imu.Raw_az - g*cos(Chassic->Body_Imu.Pitch)*cos(Chassic->Body_Imu.Roll);        

        /*机体竖直方向的加速度*/
        Chassic->Body_Imu.Object_az = Chassic->Body_Imu.Filter_ax*sin(Chassic->Body_Imu.Pitch) + \
                                      Chassic->Body_Imu.Filter_ay*sin(-Chassic->Body_Imu.Roll)*cos(Chassic->Body_Imu.Pitch) + \
                                      Chassic->Body_Imu.Filter_az*cos(Chassic->Body_Imu.Pitch)*cos(Chassic->Body_Imu.Roll);
    }

}
void WB_Get_Leg_Wheel_Imu_Info(Leg_t* Leg) //获取一条腿的轮子陀螺仪的值
{  
    if(Leg == NULL)
        return;

    const double *wheel_gyro_ = NULL;

    wheel_gyro_ = wb_gyro_get_values(Devices[Wheel_Gyro_e]);//返回数组地址，大小是3
     
    if(wheel_gyro_ != NULL)
    {
        Leg->Wheel_Imu_Info.Roll_dt = wheel_gyro_[0];//围绕X方向
        Leg->Wheel_Imu_Info.Pitch_dt = wheel_gyro_[1];//围绕Y方向
        Leg->Wheel_Imu_Info.Yaw_dt = wheel_gyro_[2];//围绕Z方向  
    }
    
}
void WB_Get_Leg_Pos_Sensors_Info(Leg_t* Leg) //获取一条腿的所有编码器的值
{
    if(Leg == NULL)
        return;

    //轮子和虚拟关节不需要保存

    //前关节
    Leg->Pos_Sensor_Info[F_Joint].Total_Radian = wb_position_sensor_get_value(Devices[F_Joint_Pos_Sensor_e]);
    Leg->Pos_Sensor_Info[F_Joint].Last_Radian = Leg->Pos_Sensor_Info[F_Joint].This_Radian;
    Leg->Pos_Sensor_Info[F_Joint].This_Radian = Leg->Pos_Sensor_Info[F_Joint].Total_Radian;

    //前关节
    Leg->Pos_Sensor_Info[B_Joint].Total_Radian = wb_position_sensor_get_value(Devices[B_Joint_Pos_Sensor_e]);
    Leg->Pos_Sensor_Info[B_Joint].Last_Radian = Leg->Pos_Sensor_Info[B_Joint].This_Radian;
    Leg->Pos_Sensor_Info[B_Joint].This_Radian = Leg->Pos_Sensor_Info[B_Joint].Total_Radian;

}
void WB_Get_Leg_Motors_Spin_Speed_Info(Leg_t* Leg) //获取一条腿的所有电机的旋转速度
{
    if(Leg == NULL)
        return;

    Leg->Motor_Info[F_Joint].Spin_Speed = (Leg->Pos_Sensor_Info[F_Joint].This_Radian - Leg->Pos_Sensor_Info[F_Joint].Last_Radian) / (TIME_STEP * 0.001f);

    Leg->Motor_Info[B_Joint].Spin_Speed = (Leg->Pos_Sensor_Info[B_Joint].This_Radian - Leg->Pos_Sensor_Info[B_Joint].Last_Radian) / (TIME_STEP * 0.001f);

    Leg->Motor_Info[Wheel].Spin_Speed = Leg->Wheel_Imu_Info.Yaw_dt;

}
void WB_Get_Leg_Motors_Torque_fdb_Info(Leg_t* Leg) //获取一条腿的所有电机的反馈扭矩
{
    //很可惜，用了扭矩控制的电机，调用wb_motor_get_torque_feedback函数会返回0
    //所以只能默认发送的扭矩=反馈的扭矩
}
void WB_Get_Keyboard_Info(void)
{
    key = wb_keyboard_get_key();
}
void WB_Get_Robot_First_Perspective_Image_Info(void) //获取第一视角 
{
    wb_camera_get_image(Devices[Camera_e]);
}
void WB_Get_All_Devices_Info(void)
{
    WB_Get_Chassic_Imu_Info(&Chassic);
    WB_Get_Leg_Wheel_Imu_Info(&Chassic.Leg);
    WB_Get_Leg_Pos_Sensors_Info(&Chassic.Leg);
    WB_Get_Leg_Motors_Spin_Speed_Info(&Chassic.Leg);
    WB_Get_Keyboard_Info();
    WB_Get_Robot_First_Perspective_Image_Info();
}







/*
 *------------------------------------------------------VMC运动学+动力学部分---------------------------------------------------------
*/
//五连杆的正运动学解算,传入q1和q4角度,获取L0和角度q0
void Vmc_Positive_Kinematics(Vmc_t* vmc, const double q1, const double q4) 
{
    if(vmc == NULL)
        return;
        
    double y, x, temp; 
    
    /*----------------------------------------步骤1：获取q1和q4----------------------------------------------*/
    vmc->q[1] = q1;
    vmc->q[4] = q4;

    /*----------------------------------------步骤2：获取B点和D点坐标--------------------------------------------*/
    vmc->Mid_Param.X_B = vmc->L[1]*cos(vmc->q[1]);
    vmc->Mid_Param.Y_B = vmc->L[1]*sin(vmc->q[1]);
    vmc->Mid_Param.X_D = vmc->L[4]*cos(vmc->q[4]) + vmc->L[5];
    vmc->Mid_Param.Y_D = vmc->L[4]*sin(vmc->q[4]);

    /*----------------------------------------步骤3：获取中间变量-----------------------------------------------*/
    vmc->Mid_Param.A0 = 2*vmc->L[2]*(vmc->Mid_Param.X_D - vmc->Mid_Param.X_B);
    vmc->Mid_Param.B0 = 2*vmc->L[2]*(vmc->Mid_Param.Y_D - vmc->Mid_Param.Y_B);
    vmc->Mid_Param.L_BD_sq = pow( (vmc->Mid_Param.X_D - vmc->Mid_Param.X_B), 2 ) + pow( (vmc->Mid_Param.Y_D - vmc->Mid_Param.Y_B), 2 );
    vmc->Mid_Param.C0 = pow( vmc->L[2], 2 ) + vmc->Mid_Param.L_BD_sq - pow( vmc->L[3], 2 );
    
    /*-----------------------------------------步骤4：获取q2角度----------------------------------------------------*/
    temp = pow( vmc->Mid_Param.A0, 2 ) + pow( vmc->Mid_Param.B0, 2 ) - pow( vmc->Mid_Param.C0, 2 );
    y = vmc->Mid_Param.B0 + sqrt( abs(temp) );//temp不一定为正
    x = vmc->Mid_Param.A0 + vmc->Mid_Param.C0;
    vmc->q[2] = 2*atan2(y,x);

    /*-----------------------------------------步骤5：求C点坐标和q3角度-------------------------------------------------*/
    vmc->Mid_Param.X_C = vmc->L[1]*cos(vmc->q[1]) + vmc->L[2]*cos(vmc->q[2]);
    vmc->Mid_Param.Y_C = vmc->L[1]*sin(vmc->q[1]) + vmc->L[2]*sin(vmc->q[2]);
    y = vmc->Mid_Param.Y_C - vmc->Mid_Param.Y_D;//复用y和x变量
    x = vmc->Mid_Param.X_C - vmc->Mid_Param.X_D; 
    vmc->q[3] = atan2(y,x);
  
    /*---------------------------------------步骤6：求出虚拟腿长L0和q0角度-----------------------------------------*/
    //复用这个变量
    temp = pow( (vmc->Mid_Param.X_C - vmc->L[5]*0.5f), 2 ) + pow( vmc->Mid_Param.Y_C , 2 );
    vmc->L[0] = sqrt( abs(temp) );//temp不一定为正
    y = vmc->Mid_Param.Y_C;
    x = vmc->Mid_Param.X_C - vmc->L[5]*0.5f;
    vmc->q[0] = atan2(y,x);

}
//五连杆的正动力学,传入LQR计算得到足端的扭矩,设置J_F2T雅可比矩阵,足端期望力------>关节电机期望扭矩
void Vmc_Positive_Dynamics(Vmc_t* vmc, const double Tp, const double Fy)
{
    if(vmc == NULL)
        return;

    //LQR算出的系统输入
    vmc->Fxy_tar.e.Tp_tar = Tp;
    vmc->Fxy_tar.e.Fy_tar = Fy;

    //J_F2T雅可比矩阵
    vmc->J_F2T.e.x1_1 = vmc->L[1]*sin(vmc->q[1] - vmc->q[2])*cos(vmc->q[0] - vmc->q[3]) / (vmc->L[0]*sin(vmc->q[3] - vmc->q[2]));
    vmc->J_F2T.e.x1_2 = vmc->L[1]*sin(vmc->q[1] - vmc->q[2])*sin(vmc->q[0] - vmc->q[3]) / sin(vmc->q[3] - vmc->q[2]);
    vmc->J_F2T.e.x2_1 = vmc->L[4]*sin(vmc->q[3] - vmc->q[4])*cos(vmc->q[0] - vmc->q[2]) / (vmc->L[0]*sin(vmc->q[3] - vmc->q[2]));
    vmc->J_F2T.e.x2_2 = vmc->L[4]*sin(vmc->q[3] - vmc->q[4])*sin(vmc->q[0] - vmc->q[2]) / sin(vmc->q[3] - vmc->q[2]);

    //计算得到两个关节电机的扭矩
    Matrix_Multiply(2,2,vmc->J_F2T.array, \
                    2,1,vmc->Fxy_tar.array, \
                    vmc->T14_tar.array);

}
//五连杆的逆运动学结算,传入w1和w4角速度,设置J_w2v雅可比矩阵,关节电机反馈的角速度------->足端的速度
void Vmc_Negative_Kinematics(Vmc_t* vmc, const double w1, const double w4)
{
    if(vmc == NULL)
        return;

    //关节反馈的角速度
    vmc->W_fdb.e.w1_fdb = w1;
    vmc->W_fdb.e.w4_fdb = w4;

    //J_w2v雅可比矩阵
    vmc->J_w2v.e.x1_1 = ( vmc->L[1]*sin(vmc->q[0])*sin(vmc->q[3])*sin(vmc->q[1] - vmc->q[2]) - \
                          vmc->L[4]*cos(vmc->q[0])*sin(vmc->q[2])*sin(vmc->q[3] - vmc->q[4]) ) / sin(vmc->q[2] - vmc->q[3]);

    vmc->J_w2v.e.x1_2 = ( vmc->L[1]*cos(vmc->q[0])*sin(vmc->q[3])*sin(vmc->q[1] - vmc->q[2]) + \
                          vmc->L[4]*sin(vmc->q[0])*sin(vmc->q[2])*sin(vmc->q[3] - vmc->q[4]) ) / sin(vmc->q[2] - vmc->q[3]);

    vmc->J_w2v.e.x2_1 = ( -vmc->L[1]*sin(vmc->q[0])*cos(vmc->q[3])*sin(vmc->q[1] - vmc->q[2]) + \
                           vmc->L[4]*cos(vmc->q[0])*cos(vmc->q[2])*sin(vmc->q[3] - vmc->q[4]) ) / sin(vmc->q[2] - vmc->q[3]);

    vmc->J_w2v.e.x2_2 = -( vmc->L[1]*cos(vmc->q[0])*cos(vmc->q[3])*sin(vmc->q[1] - vmc->q[2]) + \
                           vmc->L[4]*sin(vmc->q[0])*cos(vmc->q[2])*sin(vmc->q[3] - vmc->q[4]) ) / sin(vmc->q[2] - vmc->q[3]);

    //求解垂直于L0速度和沿着L0速度
    Matrix_Multiply(2,2,vmc->J_w2v.array, \
                    2,1,vmc->W_fdb.array, \
                    vmc->V_fdb.array);
    vmc->V_fdb.e.w0_fdb /= vmc->L[0]; //将垂直于L0速度------>角速度w0

    //顺便求L0的一阶、二阶变化率
    vmc->Acc_fdb.Last_L0_dt = vmc->Acc_fdb.This_L0_dt;
    vmc->Acc_fdb.This_L0_dt = -vmc->V_fdb.e.vy_fdb; //这里的负号其实是因为逆运动学算出的和差分算出的L0_dt是缺了负号
    vmc->Acc_fdb.L0_ddt = (vmc->Acc_fdb.This_L0_dt - vmc->Acc_fdb.Last_L0_dt) / (TIME_STEP * 0.001f);

}
//五连杆的逆动力学结算,传入反馈的T1和T4你局,设置J_T2F雅可比矩阵,关节电机反馈扭矩------>足端反馈力
void Vmc_Negative_Dynamics(Vmc_t* vmc, const double T1, const double T4)
{
    if(vmc == NULL)
        return;

    //保存反馈的T1和T4扭矩
    vmc->T14_fdb.e.T1_fdb = T1;
    vmc->T14_fdb.e.T4_fdb = T4;

    //J_T2F雅可比矩阵
    vmc->J_T2F.e.x1_1 = vmc->L[0]*sin(vmc->q[0] - vmc->q[2]) / (vmc->L[1]*sin(vmc->q[1] - vmc->q[2]));
    vmc->J_T2F.e.x1_2 = vmc->L[0]*sin(vmc->q[0] - vmc->q[3]) / (vmc->L[4]*sin(vmc->q[4] - vmc->q[3]));
    vmc->J_T2F.e.x2_1 = cos(vmc->q[0] - vmc->q[2]) / (vmc->L[1]*sin(vmc->q[2] - vmc->q[1]));
    vmc->J_T2F.e.x2_2 = cos(vmc->q[0] - vmc->q[3]) / (vmc->L[4]*sin(vmc->q[3] - vmc->q[4]));

    //求出足端反馈力
    Matrix_Multiply(2,2,vmc->J_T2F.array, \
                    2,1,vmc->T14_fdb.array, \
                    vmc->Fxy_fdb.array);
}
//腿的五连杆逆结算,得到足端反馈速度、足端反馈力
void Leg_Vmc_Negative_Settle(Leg_t* Leg)
{
    if(Leg == NULL)
        return;

    Vmc_Negative_Kinematics(&Leg->Vmc, Leg->Motor_Info[F_Joint].Spin_Speed, Leg->Motor_Info[B_Joint].Spin_Speed);

    Vmc_Negative_Dynamics(&Leg->Vmc, Leg->Vmc.T14_tar.e.T1_tar, Leg->Vmc.T14_tar.e.T4_tar);

}








/*--------------------------------------------------------键盘控制---------------------------------------------------------------------------*/
void Key_Ctrl(Chassic_t* Chassic)
{
    if(Chassic == NULL)
        return;

    switch(key)
    {
        case 'W': //前进
            add(Chassic->v_set, 0.01f, MAX_SPEED_TAR);
        break;

        case 'S': //后退
            sub(Chassic->v_set, 0.01f, -MAX_SPEED_TAR); 
        break;

        case 'Q': //压腿
            sub(Chassic->Leg.L0_tar, 0.0003f, MIN_VIRTUAL_LEG_LENGTH); 
        break;

        case 'E': //抬腿
            add(Chassic->Leg.L0_tar, 0.0003f, MAX_VIRTUAL_LEG_LENGTH); 
        break;

        case 'F': //初始腿长
            Chassic->Leg.L0_tar = 0.25f; 
        break; 
    
        case 'R': //跳跃
            if(Chassic->Leg.Flag.the_jumping_flag == Normal)
            {
                if(abs(Chassic->Leg.Meas.e.th * 57.32f) <= 1.5f)
                {
                    Chassic->Leg.Flag.is_jump_request_flag = TRUE; 
                }
            }
            else
                Chassic->Leg.Flag.is_jump_request_flag = FLASE; 
        break;

        default:  //松开W和S,慢慢关闭速度目标值
            if(Chassic->v_set > 0)
            {
                sub(Chassic->v_set, 0.02f, 0);  
            }
            else
            {
                add(Chassic->v_set, 0.02f, 0);  
            }
        break;
    }
}






/*
*-----------------------------------------------------获取状态变量的测量、目标、误差------------------------------------------------------------
*/
//位移部分专门控制,提高刹车效率
void Leg_X_Meas_Ctrl(Leg_t* Leg)
{
    if(Leg == NULL)
        return;

    //速度目标值不为0,清空位移、修改两个标志位
    if(Leg->Tar.e.v != 0)
    {
        Leg->Meas.e.x = 0;

        Leg->Flag.is_speed_tar_not_zero_flag = TRUE;

        Leg->Flag.is_speed_tar_to_zero_flag = FLASE;
    } 
    else
    {
        //速度目标值突变为0
        if(Leg->Flag.is_speed_tar_not_zero_flag == TRUE)
        {
            Leg->Flag.is_speed_tar_to_zero_flag = TRUE;

            Leg->Flag.is_speed_tar_not_zero_flag = FLASE; 
        }
        
        //速度目标值变为0但是这时候是刹车,不要记录位移
        if(Leg->Flag.is_speed_tar_to_zero_flag == TRUE)
        {
            if(abs(Leg->Meas.e.v) <= 0.1f)
            {
               Leg->Flag.is_speed_tar_to_zero_flag = FLASE;

               Leg->Meas.e.x = 0;
            }
        }
        //没有速度目标值或者刹车差不多,开始记录位移
        else
        {
            Leg->Meas.e.x += Leg->Meas.e.v*(TIME_STEP * 0.001f);
        }

    }
}
void Save_Leg_Status_Meas(Leg_t* Leg, const double Meas_array[STATUS_VAL_NUMS])
{
    if(Leg == NULL)
        return;
    
    //Meas_array数组中,传入的位移是0,所以这里暂且保存
    double temp = Leg->Meas.e.x;

    //按照th    th_dt   x   v   fi    fi_dt的顺序    
    memcpy(Leg->Meas.array, Meas_array, STATUS_VAL_NUMS*sizeof(double));

    //对位移进行以下处理
    Leg->Meas.e.x = temp; 
    Leg_X_Meas_Ctrl(Leg);

    //顺便计算th的一阶、二阶变化率
    Leg->Vmc.Acc_fdb.Last_th_dt = Leg->Vmc.Acc_fdb.This_th_dt;
    Leg->Vmc.Acc_fdb.This_th_dt = Leg->Meas.e.th_dt;
    Leg->Vmc.Acc_fdb.th_ddt = (Leg->Vmc.Acc_fdb.This_th_dt - Leg->Vmc.Acc_fdb.Last_th_dt) / (TIME_STEP * 0.001f);

}
void Save_Leg_Status_Tar(Leg_t* Leg, const double Tar_array[STATUS_VAL_NUMS]) 
{
    if(Leg == NULL)
        return;

    //按照th    th_dt   x   v   fi    fi_dt的顺序    
    //只允许改变速度目标
    Leg->Tar.e.v = Tar_array[v];

}
void Save_Leg_Status_Err(Leg_t* Leg)
{
    if(Leg == NULL)
        return;
    
    double *meas_tmp, *tar_temp, *err_temp;

    meas_tmp = Leg->Meas.array;
    tar_temp = Leg->Tar.array;
    err_temp = Leg->Err.array;

    //计算6个状态变量的误差
    for(int i = 0; i < STATUS_VAL_NUMS; i ++)
    {
        err_temp[i] = meas_tmp[i] - tar_temp[i];
    }
}
//计算th角度并返回th,传入三个角度
double Cal_Leg_th_Meas(const double q0, const double fi)
{
    double th = 0, a_er_fa = 0;

    //直腿模型里面a_er_fa是指关节电机转过的角度
    a_er_fa = PAI_VAL*0.5f - q0; //VMC算出的q0并不是直腿模型的a_er_fa，同样仿照直腿模型使得腿往后撤时th角度为正

    if( a_er_fa*fi < 0 )
    {
        th = abs(a_er_fa) + abs(fi); //绝对是正的

        /*底盘后仰（fi<0）且腿前撤（a_er_fa>0），th是负的*/
        if( (a_er_fa > 0) && (fi < 0) )
        {
            //此时th为正不用做处理  
        }
        /*底盘前倾（fi>0）且腿后撤（a_er_fa<0），th是正的*/
        else 
        {
           th *= -1;     
        }
    }
    else
    {
        th = abs(a_er_fa) - abs(fi) ; //如果a_er_fa比较小，会出现临界情况也就是腿是竖直的
        
        /*底盘后仰（fi<0）且腿后撤（a_er_fa<0）*/
        if( (a_er_fa < 0) && (fi < 0) )
        {     
           th *= -1;
        }
        /*底盘前倾（fi>0）且腿前撤（a_er_fa>0）*/
        else 
        {      
           
        }
    }    

    return th;
}
//用户传入单腿测量值
void Get_Leg_Status_Meas_Vals(Leg_t* Leg, Imu_t* Body_Imu)
{
    if(Leg == NULL || Body_Imu == NULL)
        return;

    double Meas_array[STATUS_VAL_NUMS] = {0};

    //对测量值数组赋值
    Leg->Vmc.Acc_fdb.Last_th = Leg->Vmc.Acc_fdb.This_th;
    Meas_array[th] = Cal_Leg_th_Meas(Leg->Vmc.q[0], Body_Imu->Pitch);
    Leg->Vmc.Acc_fdb.This_th = Meas_array[th];

    /*腿倾角速度目前我了解到的有3种获取方法
     *用差分
     *用Vmc_Negative_Kinematics函数获取的Leg->Vmc.V_fdb.e.w0_fdb
     *用Leg->Vmc.V_fdb.e.w0_fdb - Body_Imu->Pitch
     发现在地面上三者一致，但是跳跃中表现良好体态的是差分，而实物里面也应该不是差分获取，要么是第二种要么第三种
     至于为什么后两种在跳跃过程表现并不良好的原因，猜测是跳跃过程PID的问题，就不去探索这个问题了
    */
    Meas_array[th_dt] = (Leg->Vmc.Acc_fdb.This_th - Leg->Vmc.Acc_fdb.Last_th) / (TIME_STEP*0.001f);
    //Meas_array[th_dt] = Leg->Vmc.V_fdb.e.w0_fdb;
    //Meas_array[th_dt] = Leg->Vmc.V_fdb.e.w0_fdb - Body_Imu->Pitch;

    Meas_array[x] = 0;

    Meas_array[v] = Leg->Motor_Info[Wheel].Spin_Speed*WHEEL_RADIUS; //旋转速度----->直线速度

    Meas_array[fi] = Body_Imu->Pitch;

    Meas_array[fi_dt] = Body_Imu->Pitch_dt;

    //传入数组
    Save_Leg_Status_Meas(Leg, Meas_array);
}
//用户传入单腿目标值
void Get_Leg_Status_Tar_Vals(Leg_t* Leg, const double v_tar)
{
    if(Leg == NULL)
        return;
    
    //对目标值数组进行赋值
    double Tar_array[STATUS_VAL_NUMS] = {0};
    Tar_array[v] = v_tar;
 
    //传入数据
    Save_Leg_Status_Tar(Leg, Tar_array);
}
//获取所有腿的状态变量的测量、目标、误差
void Get_Legs_Status_Vals(Chassic_t* Chassic)
{
    /*测量值*/
    Get_Leg_Status_Meas_Vals(&Chassic->Leg, &Chassic->Body_Imu);
    /*目标值*/
    Get_Leg_Status_Tar_Vals(&Chassic->Leg, Chassic->v_set);
    /*误差值*/
    Save_Leg_Status_Err(&Chassic->Leg);
}





/*
*-----------------------------------------------------计算扭矩并保存到电机数据，发送扭矩-----------------------------------------------------
*/
//Motors表示多个的意思,2个实体关节+1个虚拟关节
void Cal_Leg_Joint_Motors_Torque_tar(Leg_t* Leg) 
{
    if(Leg == NULL)
        return;
    
    /*----------------------------------------获取Fxy_tar矩阵的Tp和Fy力-------------------------------*/
    //获取足端的Tp力
    Leg->Vmc.Fxy_tar.e.Tp_tar = 0;
    for(int i = 0; i < STATUS_VAL_NUMS; i++)
    {     
        Leg->Vmc.Fxy_tar.e.Tp_tar += Leg->Err.array[i] * Leg->Joint_Motor_K_Matrix[i];
    }
    Leg->Vmc.Fxy_tar.e.Tp_tar = constrain(Leg->Vmc.Fxy_tar.e.Tp_tar, JOINT_MOTOR_MAX_TORQUE, -JOINT_MOTOR_MAX_TORQUE);
    
    //获取足端的Fy力
    PID_Calc(&Leg->LegLength_PID, Leg->Vmc.L[0], Leg->L0_tar);
    Leg->Vmc.Fxy_tar.e.Fy_tar = Leg->LegLength_PID.pos_out + m_Body*g;//前馈

    /*---------------------------------------获取T14_tar矩阵的T1和T4-----------------------------------*/
    Vmc_Positive_Dynamics(&Leg->Vmc, Leg->Vmc.Fxy_tar.e.Tp_tar, Leg->Vmc.Fxy_tar.e.Fy_tar);
    
    //虚拟关节电机扭矩
    Leg->Motor_Info[Virtual].Torque_tar = Leg->Vmc.Fxy_tar.e.Tp_tar;
    //两个关节电机扭矩
    Leg->Motor_Info[F_Joint].Torque_tar = Leg->Vmc.T14_tar.e.T1_tar;
    Leg->Motor_Info[B_Joint].Torque_tar = Leg->Vmc.T14_tar.e.T4_tar;   

}
void Cal_Leg_Wheel_Motor_Torque_tar(Leg_t* Leg)
{
    if(Leg == NULL)
        return;
     
    Leg->Motor_Info[Wheel].Torque_tar = 0; 
    
    //轮子电机扭矩
    for(int i = 0; i < STATUS_VAL_NUMS; i++)
    {
        Leg->Motor_Info[Wheel].Torque_tar += Leg->Err.array[i] * Leg->Wheel_Motor_K_Matrix[i];
    }             
    Leg->Motor_Info[Wheel].Torque_tar = constrain(Leg->Motor_Info[Wheel].Torque_tar, WHEEL_MOTOR_MAX_TORQUE, -WHEEL_MOTOR_MAX_TORQUE);
}
void Cal_Leg_Motors_Torque_tar(Leg_t* Leg)
{
    if(Leg == NULL)
        return;   
    
    Cal_Leg_Joint_Motors_Torque_tar(Leg);
    Cal_Leg_Wheel_Motor_Torque_tar(Leg);
}
void WB_Send_Actuator_Commands(Leg_t* Leg)
{   
    if(Leg == NULL)
        return;

    wb_motor_set_torque(Devices[Wheel_Motor_e], Leg->Motor_Info[Wheel].Torque_tar);
    
    wb_motor_set_torque(Devices[F_Joint_Motor_e], Leg->Motor_Info[F_Joint].Torque_tar);
    
    wb_motor_set_torque(Devices[B_Joint_Motor_e], Leg->Motor_Info[B_Joint].Torque_tar);
}
//保存反馈扭矩
void Save_Leg_Motors_Torque_fdb(Leg_t* Leg)
{
    if(Leg == NULL)
        return;

    Leg->Motor_Info[Wheel].Torque_fdb = Leg->Motor_Info[Wheel].Torque_tar;
    Leg->Motor_Info[F_Joint].Torque_fdb = Leg->Vmc.T14_fdb.e.T1_fdb;
    Leg->Motor_Info[B_Joint].Torque_fdb = Leg->Vmc.T14_fdb.e.T4_fdb;
    Leg->Motor_Info[Virtual].Torque_fdb = Leg->Motor_Info[Virtual].Torque_tar;
   
}




/*
 *----------------------------------------------------------获取支持力----------------------------------------------------------------
*/
void Cal_Leg_Fn(Leg_t* Leg, Imu_t* Body_Imu)
{
    if(Leg == NULL || Body_Imu == NULL)
        return;
    
    double P;

    P = Leg->Vmc.Fxy_fdb.e.Tp_fdb * sin(Leg->Meas.e.th) / Leg->Vmc.L[0] + \
        Leg->Vmc.Fxy_fdb.e.Fy_fdb * cos(Leg->Meas.e.th);
    
    //轮子的竖直方向加速度
    Leg->Wheel_Imu_Info.Object_az = Body_Imu->Object_az - \
                                    Leg->Vmc.Acc_fdb.L0_ddt * cos(Leg->Meas.e.th) + \
                                    2 * Leg->Vmc.Acc_fdb.This_L0_dt * Leg->Meas.e.th_dt * sin(Leg->Meas.e.th) + \
                                    Leg->Vmc.L[0] * Leg->Vmc.Acc_fdb.th_ddt * sin(Leg->Meas.e.th) + \
                                    Leg->Vmc.L[0] * pow(Leg->Meas.e.th_dt, 2) * cos(Leg->Meas.e.th);
    
    Leg->Fn = P + m_Wheel*g + m_Wheel*Leg->Wheel_Imu_Info.Object_az;
}







/*
* ---------------------------------------------------------离地检测与措施-----------------------------------------------------
*/
int Off_Ground_Detect(Leg_t* Leg)
{
    if(Leg == NULL)
        return;

    if(Leg->Fn <= 20.f)
    {
        Leg->Flag.is_off_ground_flag = TRUE;
    }
    else
    {
        Leg->Flag.is_off_ground_flag = FLASE;
    }
    
    return Leg->Flag.is_off_ground_flag;
}
//保留虚拟关节关于腿倾角的增益
void Off_Ground_Manage(Leg_t* Leg)
{
    if(Leg == NULL)
        return;

    memset(Leg->Wheel_Motor_K_Matrix, \
           0,\
           sizeof(double) * STATUS_VAL_NUMS);

    memset(&Leg->Joint_Motor_K_Matrix[2], \
           0,\
           sizeof(double) *(STATUS_VAL_NUMS-2) );
                                                         
}








/*
* ---------------------------------------------------------砸落地面检测与措施-----------------------------------------------------
*/
int Za_Luo_On_Ground_Detect(Leg_t* Leg)
{
    if(Leg == NULL)
        return;

    //砸落的前提是先离地了
    if(Leg->Flag.is_off_ground_flag == TRUE)
    {
        if(Leg->Fn >= 100.f)
        {
            Leg->Flag.is_za_luo_on_ground_flag = TRUE;
        }
    }
    
    return Leg->Flag.is_za_luo_on_ground_flag;
}
//砸落瞬间，改变一段时间腿长PID，并且清空位移
void Za_Luo_On_Ground_Manage(Leg_t* Leg)
{
    if(Leg == NULL)
        return;

    Leg->LegLength_PID.MaxOutput = 400;
    Leg->LegLength_PID.p = 1000;
    Leg->LegLength_PID.i = 2000;
    Leg->LegLength_PID.IntegralLimit = 100;
    Leg->LegLength_PID.d = 150000;

    Leg->Meas.e.x = 0;
}
//砸落一段时间后的检测
int Za_Luo_Over_Detect(Leg_t* Leg)
{
    if(Leg == NULL)
        return;

    if(Leg->Flag.is_za_luo_on_ground_flag == TRUE)
    {
        if(Leg->Fn >= 60.f && Leg->Fn <= 70.f)
        {
            Leg->Flag.is_za_luo_on_ground_flag = FLASE;
        }
    }
    
    return Leg->Flag.is_za_luo_on_ground_flag;
}
//砸落完成，改变腿长PID为原来的值
void Za_Luo_Over_Manage(Leg_t* Leg)
{
    if(Leg == NULL)
        return;

    Leg->LegLength_PID.MaxOutput = m_Body*g;
    Leg->LegLength_PID.p = 10000;
    Leg->LegLength_PID.i = 0;
    Leg->LegLength_PID.IntegralLimit = 0;
    Leg->LegLength_PID.d = 100000;
}




 


/*
 *------------------------------------------------------------跳跃----------------------------------------------------------------------
*/
void Jump(Leg_t* Leg)
{
    if(Leg == NULL)
        return;

    double temp;
    
    switch (Leg->Flag.the_jumping_flag)
    {
        case Normal:  //正常,枚举为0

            if(Leg->Flag.is_jump_request_flag == TRUE)//开始跳跃
            {   
               Leg->L0_tar = NORMAL_VIRTUAL_LEG_LENGTH; //下面的跳跃调试都是在腿长为0.25m下调的
            
               temp = Leg->Vmc.L[0] - Leg->L0_tar;
               
               if( (abs(Leg->Vmc.Acc_fdb.This_L0_dt) <= 0.01f) && \
                   (abs(temp) <= 0.01f) )
               {
                   Leg->Flag.is_jump_request_flag = FLASE;
               
                   Leg->Flag.the_jumping_flag = Stretch_ing;

                   Leg->L0_At_Jump_Request = Leg->Vmc.L[0]; 

                   Leg->Flag.is_enable_off_ground_detect = FLASE; //必须关闭离地检测
               }             
      
            } 
                
        break;
    
        case Stretch_ing: //伸腿中,枚举为1
            Leg->L0_tar = 0.3f;

            Leg->LegLength_PID.MaxOutput = 230;
            Leg->LegLength_PID.p = 100000;
            Leg->LegLength_PID.d = 500000;

            if(Leg->Vmc.L[0] >= 0.28f) //可以缩腿
            {
                Leg->Flag.the_jumping_flag = Shrink_ing;
            }

            Off_Ground_Manage(Leg);

        break;

        case Shrink_ing: //缩腿中,枚举为2
            Leg->L0_tar = 0.15f;

            Leg->LegLength_PID.MaxOutput = 200;
            Leg->LegLength_PID.p = 15000;
            Leg->LegLength_PID.d = 50000;

            if(Leg->Vmc.L[0] <= 0.15f) //可以踢腿
            {
                Leg->Flag.the_jumping_flag = Kick_ing;
            }

            Off_Ground_Manage(Leg);

        break;

        case Kick_ing: //踢腿中,枚举为3
            Leg->L0_tar = Leg->L0_At_Jump_Request;

            Leg->LegLength_PID.MaxOutput = m_Body*g;
            Leg->LegLength_PID.p = 15000;
            Leg->LegLength_PID.d = 500000;

            if(Leg->Vmc.L[0] > 0.17f) //在这段区间自由落体,dd_Zw非常可信,允许离地检测
            {
                if( (Leg->Wheel_Imu_Info.Object_az <= -5.f) && (Leg->Fn >= -5.f) && (Leg->Fn <= 5.f) )
                {
                    Leg->Flag.the_jumping_flag = Fall_ing;
                }
            }

            Off_Ground_Manage(Leg);

        break;

        case Fall_ing: //自由落体中,枚举为4 
            Leg->LegLength_PID.MaxOutput = m_Body*g;
            Leg->LegLength_PID.p = 3000;
            Leg->LegLength_PID.d = 100000;
            
            if( (Leg->Fn >= 70.f) && (Leg->Wheel_Imu_Info.Object_az >= 10.f) ) //砸落地面瞬间
            {
                Leg->Flag.the_jumping_flag = Land; 
            }
            
            Off_Ground_Manage(Leg);
            
        break;

        case Land: //着陆,枚举为5
        
            //这里就不使用离地砸落时用的方法
            Leg->LegLength_PID.MaxOutput = m_Body*g;
            Leg->LegLength_PID.p = 10000;
            Leg->LegLength_PID.i = 0;
            Leg->LegLength_PID.IntegralLimit = 0;
            Leg->LegLength_PID.d = 100000;
            
            /*如果砸落地面时，发现L0测量值远小于起跳那一瞬间的L0差别很大，
              说明是跳上台阶，L0目标值改成砸落台阶时刻的L0测量值，
              这样就不会出现视野还会上升*/
            if(Leg->L0_At_Jump_Request >= Leg->Vmc.L[0])
            {
               Leg->L0_tar = Leg->Vmc.L[0];
               
               Leg->Flag.the_jumping_flag = Normal;
              
               Leg->Flag.is_enable_off_ground_detect = TRUE;
            }

        break;

        default:
        break;
    }


  

}











/*
 *  -------------------------------------------------------用户打印数据------------------------------------------
*/
void WB_Printf_Info(void)
{ 
    
    printf("--------------------------Begin------------------------------------\n");
    
    // //正运动学
    // printf("q0:%lf  q1:%lf  q2:%lf  q3:%lf  q4:%lf\n",\
            // Chassic.Leg.Vmc.q[0]*57.32f, Chassic.Leg.Vmc.q[1]*57.32f, \
            // Chassic.Leg.Vmc.q[2]*57.32f, Chassic.Leg.Vmc.q[3]*57.32f, Chassic.Leg.Vmc.q[4]*57.32f);
    // //L0
    // printf("L0_meas:%lf   L0_tar:%lf\n", \
            // Chassic.Leg.Vmc.L[0], Chassic.Leg.L0_tar);
            
    // //状态变量
    // printf("th:%lf   th_dt:%lf   x:%lf   v:%lf    fi:%lf    fi_dt:%lf\n",\
    //         Chassic.Leg.Meas.e.th*57.32f, Chassic.Leg.Meas.e.th_dt*57.32f, \
    //         Chassic.Leg.Meas.e.x, Chassic.Leg.Meas.e.v, \
    //         Chassic.Leg.Meas.e.fi*57.32f, Chassic.Leg.Meas.e.fi_dt*57.32f);
    
    // //K增益
    // printf("Wheel_K1:%lf   Wheel_K2:%lf    Wheel_K3:%lf    Wheel_K4:%lf    Wheel_K5:%lf   Wheel_K6:%lf\n", \
            // Chassic.Leg.Wheel_Motor_K_Matrix[0], Chassic.Leg.Wheel_Motor_K_Matrix[1], \
            // Chassic.Leg.Wheel_Motor_K_Matrix[2], Chassic.Leg.Wheel_Motor_K_Matrix[3], \
            // Chassic.Leg.Wheel_Motor_K_Matrix[4], Chassic.Leg.Wheel_Motor_K_Matrix[5]);
    // printf("Joint_K1:%lf   Joint_K2:%lf    Joint_K3:%lf    Joint_K4:%lf    Joint_K5:%lf   Joint_K6:%lf\n", \
            // Chassic.Leg.Joint_Motor_K_Matrix[0], Chassic.Leg.Joint_Motor_K_Matrix[1], \
            // Chassic.Leg.Joint_Motor_K_Matrix[2], Chassic.Leg.Joint_Motor_K_Matrix[3], \
            // Chassic.Leg.Joint_Motor_K_Matrix[4], Chassic.Leg.Joint_Motor_K_Matrix[5]);
     
    // //目标扭矩
    // printf("Wheel_Torque_tar:%lf      Joint_F_Torque_tar:%lf        Joint_B_Torque_tar:%lf       Tp_tar:%lf      Fy_tar:%lf\n",\
            // Chassic.Leg.Motor_Info[Wheel].Torque_tar, \
            // Chassic.Leg.Motor_Info[F_Joint].Torque_tar, \
            // Chassic.Leg.Motor_Info[B_Joint].Torque_tar, \
            // Chassic.Leg.Vmc.Fxy_tar.e.Tp_tar, \
            // Chassic.Leg.Vmc.Fxy_tar.e.Fy_tar );
    
    // //反馈扭矩
    // printf("Wheel_Torque_fdb:%lf      Joint_F_Torque_fdb:%lf        Joint_B_Torque_fdb:%lf       Tp_fdb:%lf      Fy_fdb:%lf\n",\
    //         Chassic.Leg.Motor_Info[Wheel].Torque_tar, \
    //         Chassic.Leg.Motor_Info[F_Joint].Torque_fdb, \
    //         Chassic.Leg.Motor_Info[B_Joint].Torque_fdb, \
    //         Chassic.Leg.Vmc.Fxy_fdb.e.Tp_fdb, \
    //         Chassic.Leg.Vmc.Fxy_fdb.e.Fy_fdb);
    
    ////支持力
    //printf("Fn:%lf\n", Chassic.Leg.Fn);
    
    // //L0变化率
    // printf("L0_dt: %lf  L0_ddt:%lf\n",\
    //         Chassic.Leg.Vmc.Acc_fdb.This_L0_dt, Chassic.Leg.Vmc.Acc_fdb.L0_ddt);
    
    // //th变化率
    // printf("th_dt: %lf  th_ddt:%lf\n",\
    //         Chassic.Leg.Vmc.Acc_fdb.This_th_dt, Chassic.Leg.Vmc.Acc_fdb.th_ddt);
            
    // //腿长PID
    // printf("PID_p:%lf    PID_i:%lf    PID_d:%lf    PID_out:%lf    PID_meas:%lf     PID_tar:%lf\n",\
            // Chassic.Leg.LegLength_PID.p, Chassic.Leg.LegLength_PID.i, Chassic.Leg.LegLength_PID.d, \
            // Chassic.Leg.LegLength_PID.pos_out, Chassic.Leg.Vmc.L[0], Chassic.Leg.L0_tar); 
          
    /*不支持单独打印换行操作,所以用下面方法将数据分隔开*/      
    printf("---------------------------End----------------------------------\n");
}









/*
 * -------------------------------------------------------用户放在main函数里的----------------------------------------------------------
*/
void My_Init_Before_While_Func(void)
{  
    /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
    WB_Get_All_Device();

    PID_struct_init(&Chassic.Leg.LegLength_PID, POSITION_PID, m_Body*g, \
                    0, 0, 0, 10000, 0, 100000); 

}
void My_Ctrl_In_While_Func(void)
{
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
    */
    WB_Get_All_Devices_Info();

    /* Process sensor data here */
    Vmc_Positive_Kinematics(&Chassic.Leg.Vmc, \
                                PAI_VAL - BIG_LEG_INITITAL_ANGLE + Chassic.Leg.Pos_Sensor_Info[F_Joint].This_Radian, \
                                BIG_LEG_INITITAL_ANGLE + Chassic.Leg.Pos_Sensor_Info[B_Joint].This_Radian);

    //多项式拟合
    kUpdateBaseOnLeg(Chassic.Leg.Vmc.L[0]*0.5f, \
                    Chassic.Leg.Wheel_Motor_K_Matrix, \
                    Wheel_Fitting_Factor); 

    kUpdateBaseOnLeg(Chassic.Leg.Vmc.L[0]*0.5f, \
                    Chassic.Leg.Joint_Motor_K_Matrix, \
                    Joint_Fitting_Factor);    
                        
    //砸落检测和处理
    if(Za_Luo_On_Ground_Detect(&Chassic.Leg) == TRUE)
    {
        Za_Luo_On_Ground_Manage(&Chassic.Leg);
    }
        
    //离地检测和处理,注意前往不能if(Chassic.is_enable_off_ground_detect == TRUE && Off_Ground_Detect())
    //因为Off_Ground_Detect()直接改变了is_off_ground_flag标志位,但本意是如下的逻辑
    if(Chassic.Leg.Flag.is_enable_off_ground_detect == TRUE)
    {
        if(Off_Ground_Detect(&Chassic.Leg) == TRUE)
        {
            Off_Ground_Manage(&Chassic.Leg);
        }
    }   
               
    //砸落结束检测和处理
    if(Za_Luo_Over_Detect(&Chassic.Leg) == FLASE)
    {
        Za_Luo_Over_Manage(&Chassic.Leg);
    }

    Jump(&Chassic.Leg);
        
    Key_Ctrl(&Chassic);

    Get_Legs_Status_Vals(&Chassic);

    Cal_Leg_Motors_Torque_tar(&Chassic.Leg);
        
    Leg_Vmc_Negative_Settle(&Chassic.Leg);
        
    Save_Leg_Motors_Torque_fdb(&Chassic.Leg);
        
    Cal_Leg_Fn(&Chassic.Leg, &Chassic.Body_Imu);
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
    */
    WB_Send_Actuator_Commands(&Chassic.Leg);    

    //WB_Printf_Info();       
}


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
    wb_robot_init();

    My_Init_Before_While_Func();
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
    while (wb_robot_step(TIME_STEP) != -1) {

        My_Ctrl_In_While_Func();
        
    };

  /* Enter your cleanup code here */
  /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return 0;
}

/* 带偏移量的绝对值限幅函数 */
static void abs_limit(double* a, double abs_max, double offset) {
    if ( *a > abs_max + offset )
        *a = abs_max + offset;
    if ( *a < -abs_max + offset )
        *a = -abs_max + offset;
}

/* 参数初始化 */
static void pid_param_init(
    PID_t* pid,
    int mode,
    double maxout,
    double intergral_limit,
    double deadband,
    double div,
    double 	kp,
    double 	ki,
    double 	kd) {
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->deadband = deadband;
    pid->div = div;
    pid->pid_mode = mode;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/* 中途更改参数设定(调试) */
static void pid_param_reset(PID_t* pid, double kp, double ki, double kd) {
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    pid->iout = 0;
}

/*pid总体初始化-----------------------------------------------------------------*/
void PID_struct_init(
    PID_t* pid, int mode, double maxout,
    double intergral_limit, double deadband, double div,
    double kp, double ki, double kd) {
    memset(pid, 0, sizeof(PID_t));
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_param_reset;
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, deadband, div, kp, ki, kd);
}

/**
  * @bref. calculate delta PID and position PID
  * @param[in] set： target
  * @param[in] real	measure
  */
double PID_Calc(PID_t* pid, double get, double set) {
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
        
    /* 误差超限控制 */
    if ( pid->max_err != 0 && abs(pid->err[NOW]) > pid->max_err )
        return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;

    if ( pid->pid_mode == POSITION_PID ) { //位置式p
        /* P输出 */
        pid->pout = pid->p * pid->err[NOW];
           /* D输出 */
        pid->d_error = pid->err[NOW] - pid->err[LAST];
        pid->dout = pid->d * pid->d_error;

        /* I输出 */
        if ( pid->deadband != 0 && abs(pid->err[NOW]) >= pid->deadband ) { //积分死区控制
            if ( pid->div != 0 && abs(pid->err[NOW]) <= pid->div )
                pid->iout += pid->i * pid->err[NOW];
            else if ( pid->div == 0 )
                pid->iout += pid->i * pid->err[NOW];
        } else if ( pid->deadband == 0 ) { //积分无死区控制
            if ( pid->div != 0 && abs(pid->err[NOW]) <= pid->div )//积分分离
                pid->iout += pid->i * pid->err[NOW];
            else if ( pid->div == 0 )
                pid->iout += pid->i * pid->err[NOW];
        }
        abs_limit(&(pid->iout), pid->IntegralLimit, 0);
        /* 位置式PID输出 */
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput, 0);
        pid->last_pos_out = pid->pos_out;
    } else if ( pid->pid_mode == DELTA_PID ) { //增量式P
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        abs_limit(&(pid->iout), pid->IntegralLimit, 0);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput, 0);
        pid->last_delta_out = pid->delta_out;
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}

/*
  矩阵乘法运算
  result = matrix1 * matrix2
*/
void Matrix_Multiply(int rows1, int cols1, double matrix1[rows1][cols1], 
                     int rows2, int cols2, double matrix2[rows2][cols2], 
                     double result[rows1][cols2]) 
{
    // Check if multiplication is possible
    if (cols1 != rows2)
        return;
  
    // Perform matrix multiplication
    for(int i = 0; i < rows1; ++i) 
    {
        for(int j = 0; j < cols2; ++j) 
        {
            result[i][j] = 0;
            for(int k = 0; k < cols1; ++k) 
            {
                result[i][j] += matrix1[i][k] * matrix2[k][j];
            }
        }
    }
}

static void kUpdateBaseOnLeg(double L0, double K[6], const double KL[6][4]) 
{
    double K_temp[6];
    for ( int i = 0; i < 6; ++i ) 
    {
        K_temp[i] = KL[i][0] * pow(L0, 3) + \
                    KL[i][1] * pow(L0, 2) + \
                    KL[i][2] * pow(L0, 1) + \
                    KL[i][3] * pow(L0, 0); //系数降序排列

        K[i] = K_temp[i];
    }
    
}