/*
#include <kalman.h>
#include <cstdlib>

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
	float det = 1.0f / (kf->Sk[0][0]*kf->Sk[1][1] - kf->Sk[0][1]*kf->Sk[1][0]);
	inv_Sk[0][0] = det * kf->Sk[1][1];
	inv_Sk[0][1] = -det * kf->Sk[0][1];
	inv_Sk[1][0] = -det * kf->Sk[1][0];
	inv_Sk[1][1] = det * kf->Sk[0][0];
	kf->K[0] = kf->p * inv_Sk[0][0] + kf->p * inv_Sk[1][0];
	kf->K[1] = kf->p * inv_Sk[0][1] + kf->p * inv_Sk[1][1];
}

void kalman_filtering(KF *kf, float* output, float* data1, float* data2, int length){

	for(int i = 0; i < length; i++){
		if(fabs(kf[i].X - *data1) < 0.000001f || fabs(kf[i].X - *data2) < 0.000001f){
			return;
		}
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
*/
/*
 * Kalman.cpp
 *
 *  Created on: 2014Š~8€ë24€é
 *      Author: YunKei
 */

#include <kalman.h>
#include <stdio.h>

Kalman::Kalman(double Q, double* R, double X, double P) : _Q(0), __X(0), _x(0), __P(0), _p(0), isOneDim(false){
	_Q = Q;
	_R[0] = R[0];
	_R[1] = R[1];
	_K[0] = 0;
	_K[1] = 0;
	__X = X;
	_x = 0;
	__P = P;
	_p = 0;
	_yk[0] = 0;
	_yk[1] = 0;
	_Sk[0][0] = 0;
	_Sk[0][1] = 0;
	_Sk[1][0] = 0;
	_Sk[1][1] = 0;
	isOneDim = _R[1] < 0 ? true : false;
}

void Kalman::Filtering(double* output, double data1, double data2){

	StatePredict();
	CovariancePredict();
	MeasurementResidual(data1, data2);
	MeasurementResidualCovariance();
	Gain();
	StateUpdate();
	CovarianceUpdate();
	*output = __X;
}

void Kalman::MeasurementResidual(double Z1, double Z2){
	_yk[0] = Z1 - _x;
	if(!isOneDim){
		_yk[1] = Z2 - _x;
	}
}
void Kalman::MeasurementResidualCovariance(){
	_Sk[0][0] = _p + _R[0];
	if(!isOneDim){
		_Sk[0][1] = _Sk[1][0] = _p;
		_Sk[1][1] = _p + _R[1];
	}
}
void Kalman::StatePredict(){
	_x = __X;
}
void Kalman::CovariancePredict(){
	_p = __P + _Q;
}
void Kalman::StateUpdate(){

	if(!isOneDim){
		__X = _x + _K[0] * _yk[0] + _K[1] * _yk[1];
	}
	else{
		__X = _x + _K[0] * _yk[0];
	}
}
void Kalman::CovarianceUpdate(){

	if(!isOneDim){
		__P = (1 - _K[0] + _K[1]) * _p;
	}
	else{
		__P = (1 - _K[0]) * _p;
	}
}
void Kalman::Gain(){
	if(!isOneDim){

		double inv_Sk[2][2];
		double inv_det = _Sk[0][0] *_Sk[1][1] - _Sk[0][1] *_Sk[1][0];
		inv_Sk[0][0] = _Sk[1][1] / inv_det;
		inv_Sk[0][1] = _Sk[0][1] / -inv_det;
		inv_Sk[1][0] = _Sk[1][0] / -inv_det;
		inv_Sk[1][1] = _Sk[0][0] / inv_det;
		_K[0] = _p * inv_Sk[0][0] + _p * inv_Sk[1][0];
		_K[1] = _p * inv_Sk[0][1] + _p * inv_Sk[1][1];
	}
	else{
		double inv_Sk = 1 / _Sk[0][0];
		_K[0] = _p * inv_Sk;
	}
}

