typedef struct kalman_filter KF;

#ifndef __KALMAN_H
#define __KALMAN_H

#include "math.h"

struct kalman_filter{
	float Q, R[2], K[2], X, x, P, p, yk[2], Sk[2][2];
};

void kalman_filter_init(KF *kf, float Q, float* R, float X, float P);
void kalman_filtering(KF *kf, float* output, float *data1, float *data2, int length);

#endif
