#include "k60/car.h"
#include <libutil/misc.h>


using namespace libsc::k60;
using namespace libbase::k60;
using namespace libutil;

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

JyMcuBt106::Config GetJyMcuBt106Config(){
	JyMcuBt106::Config uartconfig;
	uartconfig.baud_rate = Uart::Config::BaudRate::k115200;
	uartconfig.id = 0;
	return uartconfig;
}

Mpu6050::Config GetMpu6050Config(){
	Mpu6050::Config gyro_config;
	gyro_config.accel_range = Mpu6050::Config::Range::kSmall;
	gyro_config.gyro_range = Mpu6050::Config::Range::kMid;
	gyro_config.cal_drift = true;
	return gyro_config;

}

Led::Config GetLedConfig(int id){
	Led::Config ledconfig;
	ledconfig.id = id;
	return ledconfig;
}

DirEncoder::Config GetDirEncoderConfig(int id){
	DirEncoder::Config econfig1;
	econfig1.id = id;
	return econfig1;
}

DirMotor::Config GetDirMotorConfig(int id){
	DirMotor::Config config;
	config.id = id;
	return config;
}


Car::Car():
		m_encoder_count0(0),
		m_encoder_speed0(0),
		m_encoder_count1(0),
		m_encoder_speed1(0),
		m_encoder_count_c(0),
		m_encoder_speed_c(0),
		m_led(GetLedConfig(0)),
		m_led2(GetLedConfig(1)),
		m_led3(GetLedConfig(2)),
		m_led4(GetLedConfig(3)),
		m_mpu6050(GetMpu6050Config()),
		m_encoder0(GetDirEncoderConfig(0)),
		m_encoder1(GetDirEncoderConfig(1)),
		m_com(GetJyMcuBt106Config()),
		m_motor0(GetDirMotorConfig(0)),
		m_motor1(GetDirMotorConfig(1))

{
	libutil::InitDefaultFwriteHandler(&m_com);

//	Mma8451q::Config accel_config;
//	accel_config.id = 0;
//	accel_config.sensitivity = Mma8451q::Config::Sensitivity::kLow;
//	accel_config.output_data_rate = Mma8451q::Config::OutputDataRate::k800Hz;
//	accel_config.power_mode = Mma8451q::Config::PowerMode::kNormal;
//	Mma8451q accelerometer(accel_config);

//	Gpo::Config pinconfig;
//	pinconfig.pin = Pin::Name::kPtc0;
//	pinconfig.is_high = false;
//	pin0_ = new Gpo(pinconfig);


}
