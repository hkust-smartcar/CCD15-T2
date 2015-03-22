/*
 * Vector.cpp
 *
 *  Created on: 2014�~8��24��
 *      Author: YunKei
 */

#include <vector.h>

void Vector::Add(double* v, double* v1, double* v2){
	for(int i = 0; i < 3; i++){
		v[i] = v1[i] + v2[i];
	}
}

void Vector::Scale(double* v, double* v1, double scale){
	for(int i = 0; i < 3; i++){
		v[i] = v1[i] * scale;
	}
}

void Vector::Multiplication(double* vout, double m[3][3], double* v){

	double temp[3];
	for(int i = 0; i < 3; i++){
		temp[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
	}

	for(int i = 0; i < 3; i++){
		vout[i] = temp[i];
	}
}

double Vector::DotProduct(double* v1, double* v2){
	double v = 0;
	for(int i = 0; i < 3; i++){
		v += v1[i] * v2[i];
	}
	return v;
}

void Vector::CrossProduct(double* v, double* v1, double* v2){
	double temp[3];
	temp[0] = v1[1] * v2[2] - v1[2] * v2[1];
	temp[1] = v1[2] * v2[0] - v1[0] * v2[2];
	temp[2] = v1[0] * v2[1] - v1[1] * v2[0];
	v[0] = temp[0];
	v[1] = temp[1];
	v[2] = temp[2];
}
