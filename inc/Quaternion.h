/*
 * Quaternion.h
 *
 *  Created on: 2014�~8��24��
 *      Author: YunKei
 */
#pragma once
#include "Pid.h"


#pragma once

#include <kalman.h>
#include <libsc/mpu6050.h>

using namespace libsc;
using namespace Math;

#define GRAVITY 9.81

namespace Math{

	class Quaternion{

		public:

			Quaternion(double,Mpu6050*);
			static Quaternion* getInstance();
			void Update();
			double getEuler(int);
			void setEuler(int, double);
			double* getQuaternion();
			void getQuaternionConjugate(double*,double*);
			void resetQuaternion();
			void QuaternionToMatrix(double*, double[3][3]);
			double getInitAngles(int index);
			Kalman* getKalman(int index);
			double temp1[3];
			double temp2[3];

		private:
			Mpu6050* m_mpu;
			double Interval;
			double _Quaternion[4];
			Kalman* _EulerKalman[3];
			double _Euler[3];
			double InitAngles[2];
			double PreAcc[3];
			Pid* DriftCorrectionPid[3];
			void Normalization(double*, double*);
			void QuaternionMultiplication(double*, double*,  double*);
			void EulerToQuaternion(double*, double*);
			void QuaternionToEuler(double*, double*);
	};
};
