/*
 * app.h
 *
 *  Created on: Mar 3, 2015
 *      Author: harrison
 */
#if defined(MK60DZ10)
#include "k60/car.h"
#elif defined(MKL26Z4)
#include "kl26/car.h"
#endif

#pragma once

class App{
public:
	App();
	int16_t Output_s0(int16_t spdcon[5], uint8_t pid[3], uint16_t time[2]);
	int16_t Output_s1(int16_t spdcon[5], uint8_t pid[3], uint16_t time[2]);
	int16_t Output_b(float* balcon, float* balpid, uint16_t* time, float real_angle, float gyro_angle);
	float Output_speed(int16_t* carspeedcon, float* carspeedpid, int16_t encoder);
	void Update_edge(uint8_t* ccd_data_, uint8_t* edge);
private:
	Car m_car;
	int16_t m_balance_pid_output;

};

