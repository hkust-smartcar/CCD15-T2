/*
 * app.h
 *
 *  Created on: Feb 25, 2015
 *      Author: harrison
 */
#pragma once

#include <cstdint>
#include <libbase/helper.h>
#include <libbase/kl26/mcg.h>
#include <libsc/dir_motor.h>
#include <libsc/dir_encoder.h>
#include <libsc/mpu6050.h>
#include <libsc/mma8451q.h>
#include <libbase/kl26/gpio.h>
#include <libsc/kl26/jy_mcu_bt_106.h>
#include <libsc/led.h>
#include <libsc/system.h>
#include <libsc/tsl1401cl.h>
#include <libbase/kl26/uart.h>
#include <libsc/kl26/uart_device.h>
#include <libsc/st7735r.h>
#include <libsc/battery_meter.h>
#include <libutil/remote_var_manager.h>
#include <libbase/kl26/adc.h>
#include <libbase/kl26/hardware.h>
#include <libsc/joystick.h>
#include <libsc/button.h>


#define RAD2ANGLE 57.296f

using namespace LIBBASE_NS;
using namespace libsc;
using namespace LIBSC_NS;
using namespace LIBSC_NS;
using namespace libutil;

class Car{
public:
	Car();
	void Sw1Down(int);
	void Sw2Down(int);
	void Sw3Down(int);

	RemoteVarManager* m_varmanager;
	int16_t m_encoder_countr, m_encoder_countl, m_encoder_countr_t, m_encoder_countl_t, m_encoder_count_c, m_encoder_speed_c, m_speed_output;
	int16_t m_encoder_spdcountr = 0, m_encoder_spdcountl = 0;
	libsc::Led m_led, m_led2, m_led3, m_led4;
	Joystick m_joy;
	Button m_sw1,m_sw2,m_sw3;
	Mpu6050 m_mpu6050;
	Mma8451q m_mma8451q;
	DirEncoder m_encoder0;
	DirEncoder m_encoder1;
	DirMotor m_motor_r;
	DirMotor m_motor_l;
	Tsl1401cl m_ccd;

	JyMcuBt106* m_com;

	BatteryMeter m_bat;

	St7735r m_lcd;
	uint16_t edge[2]={0,libsc::Tsl1401cl::kSensorW};

	bool m_car_move_motor;
	bool m_car_move_forward;
	bool m_lcdupdate;

private:
	Mma8451q::Config GetMma8451qConfig(){
		Mma8451q::Config accel_config;
		accel_config.id = 0;
		accel_config.output_data_rate = Mma8451q::Config::OutputDataRate::k800Hz;
		accel_config.i2c_master_ptr = m_mpu6050.GetI2cMaster();
		return accel_config;
	}

	Button::Config GetButtonConfig(int id){
		Button::Config btnconfig;
		btnconfig.id = id;
		btnconfig.is_active_low = true;
		btnconfig.listener_trigger = Button::Config::Trigger::kDown;
		if(id==0){
			btnconfig.listener = std::bind(&Car::Sw1Down, this, std::placeholders::_1);
		}
		else if(id==1){
			btnconfig.listener = std::bind(&Car::Sw2Down, this, std::placeholders::_1);
		}
		else if(id==2){
			btnconfig.listener = std::bind(&Car::Sw3Down, this, std::placeholders::_1);
		}
		return btnconfig;
	}

};

