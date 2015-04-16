/*
 * moving_average.cpp
 *
 *  Created on: Apr 15, 2015
 *      Author: harrison
 */
#include "moving_average.h"
#include <cstdint>
MovingAverage::MovingAverage(int size_):
	targetsize(size_),
	actualsize(0),
	last_index(0)
{
	vals = new int32_t[targetsize];
}

void MovingAverage::Add(int32_t val){
	if(actualsize<4){
		actualsize++;
	}
	vals[last_index++] = val;
	last_index %= targetsize;
}


