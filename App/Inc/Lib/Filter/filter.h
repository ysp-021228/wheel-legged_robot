//
// Created by xhuanc on 2021/11/17.
//

#ifndef DEMO1_FILTER_H
#define DEMO1_FILTER_H
#include "main.h"
#include "arm_math.h"


/************ 一阶 卡尔曼滤波 *************/

typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
    float B;
    float Q;
    float R;
    float H;
}first_kalman_filter_t;

//创建滤波器
void  first_Kalman_Create(first_kalman_filter_t *p, float T_Q, float T_R);

//往滤波器塞数据
float first_Kalman_Filter(first_kalman_filter_t* p, float dat);


//清空卡尔曼滤波器
//void KalmanClear(first_kalman_filter_t *p);

/************ 一阶 卡尔曼滤波 *************/


/************ 二阶 卡尔曼滤波 *************/

#define mat         arm_matrix_instance_f32 //float
#define mat_64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//转置矩阵
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

typedef struct
{
    float raw_value;
    float filtered_value[2];
    mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} second_kalman_filter_t;

typedef struct
{
    float raw_value;
    float filtered_value[2];
    float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
    float P_data[4];
    float AT_data[4], HT_data[4];
    float A_data[4];//
    float H_data[4];
    float Q_data[4];
    float R_data[4];
} second_kalman_filter_init_t;

void second_kalman_filter_init(second_kalman_filter_t *F, second_kalman_filter_init_t *I);

float* second_kalman_filter_calc(second_kalman_filter_t *F, float signal1, float signal2);


/************ 滑动均值滤波 *************/

#define MAF_MaxSize 100 //滑动队列最大值

//滑动滤波结构体 //均值滤波
typedef struct moving_Average_Filter
{
    float num[MAF_MaxSize];
    uint8_t length;
    uint8_t pot;//当前位置
    float total;
    float aver_num;

}moving_Average_Filter;	//最大设置MAF_MaxSize个

//滑动滤波器对应的操作函数
void average_init(moving_Average_Filter *Aver, uint8_t lenth);
void average_add(moving_Average_Filter *Aver, float add_data);
float average_get(moving_Average_Filter *Aver, uint16_t pre);//获取前n次的数据
void average_clear(moving_Average_Filter *Aver);
void average_fill(moving_Average_Filter *Aver, float temp);//往滑动滤波填充某个值


//中值滤波

#endif

