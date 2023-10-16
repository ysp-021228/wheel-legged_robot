//
// Created by xhuanc on 2021/11/24.
//

#ifndef DEMO1_RAMP_H
#define DEMO1_RAMP_H
#include "struct_typedef.h"

typedef  struct
{
    fp32 input;        //输入数据
    fp32 out;          //输出数据
    fp32 min_value;    //限幅最小值
    fp32 max_value;    //限幅最大值
    fp32 frame_period; //时间间隔
}__packed ramp_function_source_t;

void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

#endif //DEMO1_RAMP_H
