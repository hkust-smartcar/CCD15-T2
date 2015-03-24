#include <kl26/car.h>

#include <libsc/led.h>
#include <libutil/misc.h>

using namespace libbase;
using namespace libsc;
using namespace libbase::kl26;
using namespace libsc::kl26;
using namespace libutil;

namespace libbase
{
namespace kl26
{

Mcg::Config Mcg::GetMcgConfig()
{
	Mcg::Config config;
	config.external_oscillator_khz = 8000;
	config.core_clock_khz = 48000;
	return config;
}

}
}

Mpu6050::Config GetMpu6050Config(){
	Mpu6050::Config gyro_config;
	gyro_config.accel_range = Mpu6050::Config::Range::kSmall;
	gyro_config.gyro_range = Mpu6050::Config::Range::kMid;
	gyro_config.cal_drift = true;
	return gyro_config;

}

libsc::Led::Config GetLedConfig(int id){
	libsc::Led::Config ledconfig;
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

//St7735r::Config GetSt7735RConfig(){
//	St7735r::Config config;
//	config.is_revert = true;
//	return config;
//}

Car::Car():
				m_varmanager(new RemoteVarManager(7)),
				m_encoder_countr(0),
				m_encoder_speed0(0),
				m_encoder_countl(0),
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
				m_motor0(GetDirMotorConfig(0)),
				m_motor1(GetDirMotorConfig(1)),
				m_ccd(0)
//				m_lcd(GetSt7735RConfig())

{
//	m_varmanager = new RemoteVarManager(7);
	JyMcuBt106::Config uartconfig;
	uartconfig.baud_rate = libbase::kl26::Uart::Config::BaudRate::k115200;
	uartconfig.id = 0;
	uartconfig.rx_isr = std::bind(&RemoteVarManager::OnUartReceiveSingleChar, m_varmanager, std::placeholders::_1);
	m_com = new JyMcuBt106(uartconfig);

	libutil::InitDefaultFwriteHandler(m_com);



}
