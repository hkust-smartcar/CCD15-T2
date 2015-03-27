/*
 * Quaternion.h
 *
 *  Created on: 2014�~8��24��
 *      Author: YunKei
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <kalman.h>
#include <libsc/mpu6050.h>
//#include <Pid.h>
using namespace libsc;
namespace Math{

	class Quaternion{

		public:

			Quaternion(double, Mpu6050*);
			static Quaternion* getInstance();
			void Update();
			double getEuler(int);
			void setEuler(int, double);
			double* getQuaternion();
			void getQuaternionConjugate(double*,double*);
			void resetQuaternion();
			void QuaternionToMatrix(double*, double[3][3]);

		private:

			double Interval;
			double _Quaternion[4];
			Kalman* _EulerKalman[3];
			double _Euler[3];
			void Normalization(double*, double*);
			void QuaternionMultiplication(double*, double*,  double*);
			void EulerToQuaternion(double*, double*);
			void QuaternionToEuler(double*, double*);

			Mpu6050* m_mpu;
	};
}

using namespace Math;

#endif /* QUATERNION_H_ */
