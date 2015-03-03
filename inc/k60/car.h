/*
 * app.h
 *
 *  Created on: Feb 25, 2015
 *      Author: harrison
 */

#include <cstdint>
#include <libbase/k60/mcg.h>
#include <libsc/k60/dir_motor.h>
#include <libsc/k60/dir_encoder.h>
#include <libsc/k60/mpu6050.h>
#include <libsc/k60/mma8451q.h>
#include <libbase/k60/gpio.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/k60/led.h>
#include <libsc/k60/system.h>
#include <libutil/positional_pid_controller.h>


#ifndef INC_CAR_H_
#define INC_CAR_H_


#define RAD2ANGLE 57.296f

using namespace libsc::k60;
using namespace libbase::k60;

class Car{
public:
	Car();

	int16_t m_encoder_count_c, m_encoder_count0, m_encoder_count1,
			m_encoder_speed_c, m_encoder_speed0, m_encoder_speed1,
			m_speed_output;

	Mpu6050 m_mpu6050;
	Led m_led, m_led2, m_led3, m_led4;
	JyMcuBt106 m_com;
	DirMotor m_motor0, m_motor1;
	Gpo pin0_;


	DirEncoder m_encoder0;
	DirEncoder m_encoder1;


private:


};





#endif /* INC_CAR_H_ */
