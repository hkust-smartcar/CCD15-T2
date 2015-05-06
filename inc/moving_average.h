/*
 * moving_average.h
 *
 *  Created on: Apr 15, 2015
 *      Author: harrison
 */

#pragma once
#include <cstdint>

template <class T>
class MovingAverage{
public:
	MovingAverage(int);
	void Add(T);
	T GetAverage(){
		T sum = 0;
		for(int i=0; i<actualsize; i++){
			sum += vals[i];
		}
		return sum/actualsize;
	}
private:
	int targetsize;
	int actualsize;
	T* vals;
	int last_index;
};

/*
 * moving_average.cpp
 *
 *  Created on: Apr 15, 2015
 *      Author: harrison
 */
//#include "moving_average.h"
//#include <cstdint>

template <class T>
MovingAverage<T>::MovingAverage(int size_):
	targetsize(size_),
	actualsize(0),
	last_index(0)
{
	vals = new T[targetsize];
}

template <class T>
void MovingAverage<T>::Add(T val){
	if(actualsize<targetsize){
		actualsize++;
	}
	vals[last_index++] = val;
	last_index %= targetsize;
}


