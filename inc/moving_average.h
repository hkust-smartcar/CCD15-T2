/*
 * moving_average.h
 *
 *  Created on: Apr 15, 2015
 *      Author: harrison
 */

#pragma once
#include <cstdint>

class MovingAverage{
public:
	MovingAverage(int);
	void Add(int32_t);
	int32_t GetAverage(){
		int32_t sum = 0;
		for(int i=0; i<actualsize; i++){
			sum += vals[i];
		}
		return sum/actualsize;
	}
private:
	int targetsize;
	int actualsize;
	int32_t* vals;
	int last_index;
};
