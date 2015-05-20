/*

 * moving_average.cpp
 *
 *  Created on: Apr 15, 2015
 *      Author: harrison

#include "moving_average.h"
#include <cstdint>

template <class T>
MovingAverage<T>::MovingAverage(int size_):
	targetsize(size_),
	actualsize(0),
	last_index(0)
{
	vals = new int32_t[targetsize];
}

template <class T>
void MovingAverage<T>::Add(T val){
	if(actualsize<targetsize){
		actualsize++;
	}
	vals[last_index++] = val;
	last_index %= targetsize;
}


*/
