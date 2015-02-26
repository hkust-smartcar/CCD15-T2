/*
 * app.h
 *
 *  Created on: Feb 25, 2015
 *      Author: harrison
 */

#include <cstdint>
#include <libsc/k60/dir_encoder.h>
#include <libutil/incremental_pid_controller.h>

#ifndef INC_CAR_H_
#define INC_CAR_H_


#define RAD2ANGLE 57.296f

using namespace libsc::k60;
using namespace libbase::k60;

class Car{
public:
	Car();
	int16_t Output_s0(int16_t spdcon[5], uint8_t pid[3], uint16_t time[2]);
	int16_t Output_s1(int16_t spdcon[5], uint8_t pid[3], uint16_t time[2]);
	int16_t Output_b(float balcon[4], uint8_t balpid[3], uint16_t time[2], float real_angle);
	void Run();
private:
	DirEncoder* m_encoder0;
	DirEncoder* m_encoder1;
	int16_t m_encoder_count_c, m_encoder_count0, m_encoder_count1,
				m_encoder_speed_c, m_encoder_speed0, m_encoder_speed1;

	libutil::IncrementalPidController<float, int16_t> m_inc_pidcontroller;
	int16_t m_pid_output;

};





#endif /* INC_CAR_H_ */
