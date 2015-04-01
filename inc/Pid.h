/*
 * Pid.h
 *
 *  Created on: 2014¦~11¤ë8¤é
 *      Author: YunKei
 */

#ifndef PID_H_
#define PID_H_

namespace Math{

	class Pid{

		public:

			Pid(int, double, double, double, double, double);
			static Pid* getInstance(int);
			double setPid(double, double, double);
			double getPid(int);
			double pid(double, double);
			void clear();

		private:

			double Kp, Ki, Kd;
			double Integral;
			double IntegralLimit;
			double PreErr;
			double PreTimeStamp;
			double Period;
			double DefaultPeriod;

	};
};

using namespace Math;

#endif /* PID_H_ */
