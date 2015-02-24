#include <kalman.h>

void kalman_filter_init(KF *kf, float Q, float* R, float X, float P){
	kf->Q = Q;
	kf->R[0] = R[0];
	kf->R[1] = R[1];
	kf->K[0] = 0;
	kf->K[1] = 0;
	kf->X = X;
	kf->x = 0;
	kf->P = P;
	kf->p = 0;
	kf->yk[0] = 0;
	kf->yk[1] = 0;
	kf->Sk[0][0] = 0;
	kf->Sk[0][1] = 0;
	kf->Sk[1][0] = 0;
	kf->Sk[1][1] = 0;
}

void kalman_measurement_residual(KF *kf, float Z1, float Z2){
	kf->yk[0] = Z1 - kf->x;
	kf->yk[1] = Z2 - kf->x;
}

void kalman_measurement_residual_covariance(KF *kf){
	kf->Sk[0][0] = kf->p + kf->R[0];
	kf->Sk[0][1] = kf->Sk[1][0] = kf->p;
	kf->Sk[1][1] = kf->p + kf->R[1];
}

void kalman_state_predict(KF *kf){
	kf->x = kf->X;
}

void kalman_covariance_predict(KF *kf){
	kf->p = kf->P + kf->Q;
}

void kalman_state_update(KF *kf){
	kf->X = kf->x + kf->K[0] * kf->yk[0] + kf->K[1] * kf->yk[1];
}

void kalman_covariance_update(KF *kf){
	kf->P = (1 - kf->K[0] + kf->K[1]) * kf->p;
}

void kalman_gain(KF *kf){
	float inv_Sk[2][2];
	float det = 1 / (kf->Sk[0][0]*kf->Sk[1][1] - kf->Sk[0][1]*kf->Sk[1][0]);
	inv_Sk[0][0] = det * kf->Sk[1][1];
	inv_Sk[0][1] = -det * kf->Sk[0][1];
	inv_Sk[1][0] = -det * kf->Sk[1][0];
	inv_Sk[1][1] = det * kf->Sk[0][0];
	kf->K[0] = kf->p * inv_Sk[0][0] + kf->p * inv_Sk[1][0];
	kf->K[1] = kf->p * inv_Sk[0][1] + kf->p * inv_Sk[1][1];
}

void kalman_filtering(KF *kf, float* output, float* data1, float* data2, int length){
	for(int i = 0; i < length; i++){
		kalman_state_predict(&kf[i]);
		kalman_covariance_predict(&kf[i]);
		kalman_measurement_residual(&kf[i], data1[i], data2[i]);
		kalman_measurement_residual_covariance(&kf[i]);
		kalman_gain(&kf[i]);
		kalman_state_update(&kf[i]);
		kalman_covariance_update(&kf[i]);
		output[i] = kf[i].X;
	}
}
