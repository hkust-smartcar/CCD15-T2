/*
 * app.h
 *
 *  Created on: Feb 25, 2015
 *      Author: harrison
 */

#include <cstdint>
#include <libbase/kl26/mcg.h>
#include <libsc/dir_motor.h>
#include <libsc/dir_encoder.h>
#include <libsc/mpu6050.h>
//#include <libsc/kl26/mma8451q.h>
#include <libbase/kl26/gpio.h>
#include <libsc/kl26/jy_mcu_bt_106.h>
#include <libsc/led.h>
#include <libsc/system.h>
#include <libsc/tsl1401cl.h>
#include <libbase/kl26/uart.h>
#include <libsc/kl26/uart_device.h>
//#include <libsc/kl26/st7735r.h>
//#include <libutil/positional_pid_controller.h>
#include <libutil/remote_var_manager.h>


#ifndef INC_CAR_H_
#define INC_CAR_H_


#define RAD2ANGLE 57.296f

using namespace libsc;
using namespace libsc::kl26;
using namespace libbase::kl26;
using namespace libutil;

class Car{
public:
	Car();

	int16_t m_encoder_count_c, m_encoder_countr, m_encoder_countl,
			m_encoder_speed_c, m_encoder_speed0, m_encoder_speed1,
			m_speed_output;

	uint16_t edge[2]={0,libsc::Tsl1401cl::kSensorW};

	Mpu6050 m_mpu6050;
	libsc::Led m_led, m_led2, m_led3, m_led4;
	JyMcuBt106* m_com;
	DirMotor m_motor0;
	DirMotor m_motor1;
	Tsl1401cl m_ccd;
//	St7735r m_lcd;


	DirEncoder m_encoder0;
	DirEncoder m_encoder1;

	RemoteVarManager* m_varmanager;


private:


};





#endif /* INC_CAR_H_ */
