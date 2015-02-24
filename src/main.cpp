#include <libutil/misc.h>
/*
#include <libbase/kl26/gpio.h>
#include <libsc/kl26/sys_tick_delay.h>
#include <libsc/kl26/ftdi_ft232r.h>
*/

#include <libsc/k60/mpu6050.h>
#include <libbase/k60/gpio.h>
#include <libsc/k60/sys_tick_delay.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/k60/system.h>
#include <libsc/k60/dir_motor.h>
#include "kalman.h"

/*using namespace libsc::kl26;
using namespace libbase::kl26;*/

#define RAD2ANGLE 57.296f

using namespace libsc::k60;
using namespace libbase::k60;

namespace libbase
{
namespace k60
{

Mcg::Config Mcg::GetMcgConfig()
{
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 150000;
	return config;
}

}
}

int main(){
	System::Init();

	Mpu6050::Config gyro_config;
	gyro_config.accel_range = Mpu6050::Config::Range::kSmall;
	gyro_config.gyro_range = Mpu6050::Config::Range::kSmall;

	Mpu6050 gyro(gyro_config);

	std::array<float, 3> accel_, gyro_;
	float real_angle = 0, acc_angle = 0, gyro_angle = 0, gyro_drift_val = 0;
//	SysTickDelay timer;



	JyMcuBt106::Config uartconfig;
	uartconfig.baud_rate = Uart::Config::BaudRate::k115200;
	uartconfig.id = 0;
	JyMcuBt106 com(uartconfig);

	libutil::InitDefaultFwriteHandler(&com);

	Gpo::Config config;
//	config.pin = Pin::Name::kPtd4;
	config.pin = Pin::Name::kPtd0;
	config.is_high = false;

	Gpo led(config);

//	config.pin = Pin::Name::kPtd5;
	config.pin = Pin::Name::kPtd1;
	config.is_high = true;

	Gpo led2(config);

//	config.pin = Pin::Name::kPtd6;
	config.pin = Pin::Name::kPtd2;
	config.is_high = false;

	Gpo led3(config);

//	config.pin = Pin::Name::kPtd7;
	config.pin = Pin::Name::kPtd3;
	config.is_high = true;

	Gpo led4(config);

	DirMotor::Config motor_config;
	motor_config.id = 0;
	DirMotor motor(motor_config);

	motor.SetClockwise(true);
	motor.SetPower(500);

	bool state = false;

	KF m_gyro_kf[3];
	KF m_acc_kf;

//	float value[2] = {0.035f, 0.705495694f};
	float value[2] = {0.035f, 0.205495694f};
	gyro.Update();
	accel_ = gyro.GetAccel();
	acc_angle = accel_[2] * RAD2ANGLE;
	kalman_filter_init(&m_gyro_kf[0], 0.01f, value, acc_angle, 1);

	Timer::TimerInt t = System::Time(), pt = t;

	while(true){
		t = System::Time();
		if(t-pt>=1){
			led.Turn();
			led2.Turn();
			led3.Turn();
			led4.Turn();
			state = !state;

/*			for(int i=0; i<1000000; i++){
				__asm("nop");
			}*/

	//		timer.DelayMs(1000);
			if(t%4==0){
				gyro.Update();
				accel_ = gyro.GetAccel();
				gyro_ = gyro.GetOmega();
				acc_angle = accel_[2] * RAD2ANGLE;
				gyro_angle += gyro_[0] * 0.004f;
				kalman_filtering(&m_gyro_kf[0], &real_angle, &gyro_angle, &acc_angle, 1);
			}

			if(t%50==0)
//				printf("%.4f,%.4f,%.4f\n",accel_[0], accel_[1], accel_[2]);
				printf("%.4f,%.4f,%.4f\n",accel_[2] * RAD2ANGLE, -gyro_angle, real_angle);

			pt = t;
		}

	}

	return 0;
}
