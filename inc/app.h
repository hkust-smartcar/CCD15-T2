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

#include <libutil/positional_pid_controller.h>

#ifndef INC_APP_H_
#define INC_APP_H_

class App{
public:
	App();
	int16_t Output_s0(int16_t spdcon[5], uint8_t pid[3], uint16_t time[2]);
	int16_t Output_s1(int16_t spdcon[5], uint8_t pid[3], uint16_t time[2]);
	int16_t Output_b(float balcon[5], float balpid[3], uint16_t time[2], float real_angle, float gyro_angle);
private:
	Car m_car;
//	libutil::IncrementalPidController<float, int16_t> m_inc_pidcontroller;
	libutil::PositionalPidController<int16_t, int16_t> m_speed_inc_pidcontroller;
	int16_t m_balance_pid_output;

};



#endif /* INC_APP_H_ */
