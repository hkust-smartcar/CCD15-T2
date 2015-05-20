/*
 * Kalman.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef KALMAN_H_
#define KALMAN_H_


class Kalman{

public:

	Kalman(double, double*, double, double);
	void Filtering(double*, double, double);

private:

	double _Q, _R[2],__X, _x,__P, _p, _K[2], _yk[2], _Sk[2][2];
	bool isOneDim;
	void MeasurementResidual(double, double);
	void MeasurementResidualCovariance();
	void StatePredict();
	void CovariancePredict();
	void StateUpdate();
	void CovarianceUpdate();
	void Gain();
};


#endif /* KALMAN_H_ */
