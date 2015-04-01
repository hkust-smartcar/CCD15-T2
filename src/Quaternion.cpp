/*
 * Quaternion.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Quaternion.h>
#include <cmath>
#include <vector.h>
#include <stdio.h>
#include <libsc/mpu6050.h>
#include <libutil/misc.h>

Quaternion* _mQuaternion;

using namespace libsc;

Quaternion::Quaternion(double interval, Mpu6050* mpu) : Interval(interval){

	_mQuaternion = this;
	m_mpu = mpu;
	m_mpu->Update();
	std::array<float, 3> accel = m_mpu->GetAccel();
//	std::swap(accel[0],accel[1]);
//	accel[1] = -accel[1];
	accel[0] = -accel[0];
	accel[1] = -accel[1];

//	InitAngles[0] = _Euler[0] = Acceleration::getInstance()->getFilteredAngle(0);
//	InitAngles[1] = _Euler[1] = Acceleration::getInstance()->getFilteredAngle(1);
	_Euler[0] = 0;
	_Euler[1] = 0;
	_Euler[2] = 0;
	EulerToQuaternion(_Euler, _Quaternion);
//	_Quaternion[0] = 1;
//	_Quaternion[1] = 0;
//	_Quaternion[2] = 0;
//	_Quaternion[3] = 0;

	double R[2] = {0.00025, 0.0005};
	_EulerKalman[0] = new Kalman(0.0001, R, 0, 1.0);
	R[0] = 0.00025;
	R[1] = 0.0005;
	_EulerKalman[1] = new Kalman(0.0001, R, 0, 1.0);
	R[0] = 0.00025;
	R[1] = 0.0005;
	_EulerKalman[2] = new Kalman(0.0001, R, 0, 1.0);

	DriftCorrectionPid[0] = new Pid(5,5.5,0,0,1000,interval);
	DriftCorrectionPid[1] = new Pid(6,5.5,0,0,1000,interval);
	DriftCorrectionPid[2] = new Pid(7,5.5,0,0,1000,interval);

}

Quaternion* Quaternion::getInstance(){
	return _mQuaternion;
}

Kalman* Quaternion::getKalman(int index){
	return _EulerKalman[index];
}

double Quaternion::getInitAngles(int index){
	return InitAngles[index];
}

double Quaternion::getEuler(int index){
	return _Euler[index];
}

void Quaternion::setEuler(int index, double angle){
	_Euler[index] = angle;
	double quaternion[4];
	EulerToQuaternion(_Euler, quaternion);
	for(int i = 0; i < 4; i++){
		_Quaternion[i] = quaternion[i];
	}
}

void Quaternion::resetQuaternion(){

//	for(int i = 0; i < 2; i++){
//		_EulerKalman[i]->Clear(Acceleration::getInstance()->getFilteredAngle(i));
//	}
//	_EulerKalman[2]->Clear(0);
//	InitAngles[0] = _Euler[0] = Acceleration::getInstance()->getFilteredAngle(0);
//	InitAngles[1] = _Euler[1] = Acceleration::getInstance()->getFilteredAngle(1);
	_Euler[0] = 0;
	_Euler[1] = 0;
	_Euler[2] = 0;
	EulerToQuaternion(_Euler, _Quaternion);
}

double* Quaternion::getQuaternion(){
	return _Quaternion;
}

void Quaternion::getQuaternionConjugate(double* conjugate,double* quaternion){
	conjugate[0] = quaternion[0];
	conjugate[1] = -quaternion[1];
	conjugate[2] = -quaternion[2];
	conjugate[3] = -quaternion[3];
}

void Quaternion::Update(){

	double DeltaQuaternion[4] = {0,0,0,0};
	DeltaQuaternion[0] = 0;
	std::array<float, 3> omega = m_mpu->GetOmega();
//	std::swap(omega[0],omega[1]);
//	omega[1] = -omega[1];

		omega[0] = -omega[0];
		omega[1] = -omega[1];
	for(int i = 1; i < 4; i++){
		DeltaQuaternion[i] = (double)omega[i-1] * 0.01745329251;
	}

	double g[3] = {0,0,1};
	double f[3] = {0,0,0};
	Vector::CrossProduct(f, g, _Euler);
	f[2] = -_Euler[2];
	double acc[3];
	std::array<float, 3> accel = m_mpu->GetAccel();
//	std::swap(accel[0],accel[1]);
//	accel[1] = -accel[1];
	accel[0] = -accel[0];
	accel[1] = -accel[1];

	for(int i = 0; i < 3; i++){
		//acc[i] = Acceleration::getInstance()->getMovingAverageFilter(i)->getAverage();
		acc[i] = (double)accel[i];
	}

//	Vector::Scale(acc, acc, 1 / GRAVITY);
	double mag = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);

	bool valid = mag < 1.2 && mag > 0.8 ? true : false;

	if(valid){
		double z[3];
		z[0] = 2 * (_Quaternion[1] * _Quaternion[3] - _Quaternion[0] * _Quaternion[2]);
		z[1] = 2 * (_Quaternion[0] * _Quaternion[1] + _Quaternion[2] * _Quaternion[3]);
		z[2] = 1 - 2 * (_Quaternion[1] * _Quaternion[1] + _Quaternion[2] * _Quaternion[2]);

		Vector::CrossProduct(z, z, acc);
		for(int i = 0; i < 3; i++){
			temp1[i] = z[i];
			temp2[i] = f[i];
			_EulerKalman[i]->Filtering(&f[i], z[i], f[i]);
			DeltaQuaternion[i+1] += DriftCorrectionPid[i]->pid(0, f[i]);
		}
	}

	QuaternionMultiplication(DeltaQuaternion, _Quaternion, DeltaQuaternion);

	for(int i = 0; i < 4; i++){
		DeltaQuaternion[i] *= Interval /2;
		_Quaternion[i] += DeltaQuaternion[i];
	}

	Normalization(_Quaternion, _Quaternion);
	QuaternionToEuler(_Quaternion, _Euler);
}

void Quaternion::Normalization(double* quaternion, double* quaternionNorm){
	double mag = sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);

	for(int i = 0; i < 4; i++){
		quaternionNorm[i] = quaternion[i] / mag;
		quaternionNorm[i] = libutil::Clamp(-1.0, quaternionNorm[i], 1.0);
	}
}

void Quaternion::QuaternionMultiplication(double* quaternion, double* quaternion1,  double* quaternion2){
	double temp[4];
	temp[0] = quaternion1[0] * quaternion2[0] - quaternion1[1] * quaternion2[1] - quaternion1[2] * quaternion2[2] - quaternion1[3] * quaternion2[3];
	double v[3];
	Vector::CrossProduct(v, &quaternion1[1], &quaternion2[1]);
	temp[1] = quaternion1[0] * quaternion2[1] + quaternion1[1] * quaternion2[0] + v[0];
	temp[2] = quaternion1[0] * quaternion2[2] + quaternion1[2] * quaternion2[0] + v[1];
	temp[3] = quaternion1[0] * quaternion2[3] + quaternion1[3] * quaternion2[0] + v[2];
	for(int i = 0; i < 4; i++){
		quaternion[i] = temp[i];
	}
}

void Quaternion::EulerToQuaternion(double* euler, double* quaternion){
	double CosHalfRoll = cos(euler[0] / 2), SinHalfRoll = sin(euler[0] / 2);
	double CosHalfPitch = cos(euler[1] / 2), SinHalfPitch = sin(euler[1] / 2);
	double CosHalfYaw = cos(euler[2] / 2), SinHalfYaw = sin(euler[2] / 2);
	quaternion[0] = CosHalfRoll * CosHalfPitch * CosHalfYaw - SinHalfRoll * SinHalfPitch * SinHalfYaw;
	quaternion[1] = CosHalfRoll * SinHalfPitch * SinHalfYaw + SinHalfRoll * CosHalfPitch * CosHalfYaw;
	quaternion[2] = CosHalfRoll * SinHalfPitch * CosHalfYaw - SinHalfRoll * CosHalfPitch * SinHalfYaw;
	quaternion[3] = CosHalfRoll * CosHalfPitch * SinHalfYaw + SinHalfRoll * SinHalfPitch * CosHalfYaw;
}

void Quaternion::QuaternionToEuler(double* quaternion, double* euler){
	euler[2] = atan2(2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]), 1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]));
	euler[1] = asin(2 * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3]));
	euler[0] = atan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]), 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]));
}

void Quaternion::QuaternionToMatrix(double* quaternion, double matrix[3][3]){
	matrix[0][0] = 1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
	matrix[0][1] = 2 * (quaternion[1] * quaternion[2] - quaternion[0] * quaternion[3]);
	matrix[0][2] = 2 * (quaternion[0] * quaternion[2] + quaternion[1] * quaternion[3]);
	matrix[1][0] = 2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]);
	matrix[1][1] = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[3] * quaternion[3]);
	matrix[1][2] = 2 * (quaternion[2] * quaternion[3] - quaternion[0] * quaternion[1]);
	matrix[2][0] = 2 * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]);
	matrix[2][1] = 2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]);
	matrix[2][2] = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
}
