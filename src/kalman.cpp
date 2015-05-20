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

