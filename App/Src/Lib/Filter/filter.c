

#include "main.h"
#include "Inc/Lib/Filter/filter.h"


/************ 一阶卡尔曼滤波*************/

/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *
  * @retval none
  * @attention R固定,Q越大代表越信任测量值，Q无穷代表只用测量值
  *		       Q固定,R越大代表越信任预测值，R无穷代表只用预测值
  */
void first_Kalman_Create(first_kalman_filter_t *p, float T_Q, float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}


/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */

float first_Kalman_Filter(first_kalman_filter_t* p, float dat)
{
    p->X_mid =p->A*p->X_last;                     //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)     状态方程
    p->P_mid = p->A*p->P_last+p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q            观测方程
    p->kg = p->P_mid/(p->P_mid+p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)   更新卡尔曼增益
    p->X_now = p->X_mid + p->kg*(dat-p->X_mid);   //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))  修正估计值
    p->P_now = (1-p->kg)*p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)           更新后验估计协方差
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;							  //输出预测结果x(k|k)
}

/************ 一阶卡尔曼滤波*************/



/************ 二阶卡尔曼滤波 *************/

float matrix_value1;
float matrix_value2;

void kalman_filter_init(second_kalman_filter_t *F, second_kalman_filter_init_t *I)
{
    mat_init(&F->xhat,2,1,(float *)I->xhat_data);
    mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
    mat_init(&F->z,2,1,(float *)I->z_data);
    mat_init(&F->A,2,2,(float *)I->A_data);
    mat_init(&F->H,2,2,(float *)I->H_data);
    mat_init(&F->Q,2,2,(float *)I->Q_data);
    mat_init(&F->R,2,2,(float *)I->R_data);
    mat_init(&F->P,2,2,(float *)I->P_data);
    mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
    mat_init(&F->K,2,2,(float *)I->K_data);
    mat_init(&F->AT,2,2,(float *)I->AT_data);
    mat_trans(&F->A, &F->AT);
    mat_init(&F->HT,2,2,(float *)I->HT_data);
    mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pData[1];
}


// xhatminus==x(k|k-1)  xhat==X(k-1|k-1)
// Pminus==p(k|k-1)     P==p(k-1|k-1)    AT==A'
// HT==H'   K==kg(k)    I=1
//

/**
  *@param 卡尔曼参数结构体
	感觉高阶也是可以这么算倒是
  *@param 角度
  *@param 速度
*/
float *kalman_filter_calc(second_kalman_filter_t *F, float signal1, float signal2)
{
    float TEMP_data[4] = {0, 0, 0, 0};
    float TEMP_data21[2] = {0, 0};
    mat TEMP,TEMP21;

    mat_init(&TEMP,2,2,(float *)TEMP_data);//
    mat_init(&TEMP21,2,1,(float *)TEMP_data21);//

    //获取测量值
    F->z.pData[0] = signal1;//z(k)
    F->z.pData[1] = signal2;//z(k)

    //矩阵乘积是第一个参数乘第二个参数然后将结果放到第三个参数里面
    //1. xhat'(k)= A xhat(k-1)  xhat->指带^的x
    //第一步，根据上一时刻Xhat最优解预测当前xhat
    mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

    //2. P'(k) = A P(k-1) AT + Q
    //第二步，由过去的协方差矩阵计算当前xhat的协方差矩阵
    mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

    //3. K(k) = P'(k) HT / (H P'(k) HT + R)
    //第三步，根据当前协方差矩阵计算卡尔曼增益
    //这一步除了最后一句以外都是拿K当作储存计算结果的一个变量
    mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

    mat_inv(&F->K, &F->P);	//	将公式分母取反再乘
    mat_mult(&F->Pminus, &F->HT, &TEMP);//
    mat_mult(&TEMP, &F->P, &F->K);//

    //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
    //第四步，根据QR，综合预测量和测量值，计算当前xhat最优解
    mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

    //5. P(k) = (1-K(k)H)P'(k)
    //第五步，计算当前最优解的协方差矩阵
    mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
    mat_sub(&F->Q, &F->P, &TEMP);//
    mat_mult(&TEMP, &F->Pminus, &F->P);

    //获取二阶卡尔曼的计算结果
    matrix_value1 = F->xhat.pData[0];
    matrix_value2 = F->xhat.pData[1];

    F->filtered_value[0] = F->xhat.pData[0];
    F->filtered_value[1] = F->xhat.pData[1];
    return F->filtered_value;
}



//参考深圳RP战队的滤波算法

/************ 滑动均值滤波 *************/

/**
  * @brief    average_init
  * @note    滑动滤波器初始化，设置长度
  * @param  None
  * @retval None
  */
void average_init(moving_Average_Filter *Aver, uint8_t length)
{
    uint16_t i;

    for(i = 0; i<MAF_MaxSize; i++)
        Aver->num[i] = 0;

    if(length >MAF_MaxSize)
    {
        length = MAF_MaxSize;
    }

    Aver->length = length;
    Aver->pot = 0;
    Aver->aver_num = 0;
    Aver->total = 0;
}

/**
  * @brief    average_clear
  * @note    滑动滤波器清空
  * @param  None
  * @retval None
  */
void average_clear(moving_Average_Filter *Aver)
{
    uint16_t i;

    for(i = 0; i<MAF_MaxSize; i++)
        Aver->num[i] = 0;

    Aver->pot = 0;
    Aver->aver_num = 0;
    Aver->total = 0;
}

/**
  * @brief    average_fill
  * @note    滑动滤波器填充某个值
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_fill(moving_Average_Filter *Aver, float temp)
{
    uint16_t i;

    for(i = 0; i<(Aver->length); i++)
        Aver->num[i] = temp;

    Aver->pot = 0;
    Aver->aver_num = temp;
    Aver->total = temp*(Aver->length);
}

/**
  * @brief    average_add
  * @note    滑动平均滤波器进入队列，先进先出
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_add(moving_Average_Filter *Aver, float add_data)
{

    Aver->total -=  Aver->num[Aver->pot];
    Aver->total += add_data;

    Aver->num[Aver->pot] = add_data;

    Aver->aver_num = (Aver->total)/(Aver->length);
    Aver->pot++;

    if(Aver->pot == Aver->length)
    {
        Aver->pot = 0;
    }

}

/**
  * @brief    average_get
  * @note    获取第前pre次的数据，如果超出数组长度则取记录的最早的数据
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
float average_get(moving_Average_Filter *Aver, uint16_t pre)
{
    float member;
    uint8_t temp;

    if(Aver->pot != 0)
    {
        temp = Aver->pot-1;
    }
    else
    {
        temp = Aver->length-1;
    }

    if(pre>Aver->length)
        pre = pre % Aver->length;

    if(pre>temp)
    {
        pre = Aver->length+temp-pre;
    }
    else
    {
        pre = temp-pre;
    }

    member = Aver->num[pre];

    return member;
}


