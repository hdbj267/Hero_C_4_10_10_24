#ifndef __SortAver_FILTER_H
#define __SortAver_FILTER_H


#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "arm_math.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "usart.h"

void  SortAver_Filter(float value,float *filter,uint8_t n);
void QuiteSort(float* a,int low,int high);
float FindPos(float*a,int low,int high);

#endif

