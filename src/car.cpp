#include <car.h>
#include <libsc/led.h>
#include <libutil/misc.h>
#include <libbase/kl26/adc.h>


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
	config.core_clock_khz = 100000;
	config.bus_clock_khz = 100000;
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



libbase::kl26::Adc::Config GetAccConfig(){
	libbase::kl26::Adc::Config config;
	config.adc = libbase::kl26::Adc::Name::kAdc0DAd0;
	config.resolution = libbase::kl26::Adc::Config::Resolution::k16Bit;
	return config;
}

libbase::kl26::Adc::Config GetGyroConfig(){
	libbase::kl26::Adc::Config config;
	config.adc = libbase::kl26::Adc::Name::kAdc0Ad6A;
	config.resolution = libbase::kl26::Adc::Config::Resolution::k16Bit;
	return config;
}

Led::Config GetLedConfig(int id){
	Led::Config ledconfig;
	ledconfig.id = id;
	return ledconfig;
}

Joystick::Config GetJoyStickConfig(){
	Joystick::Config joyconfig;
	joyconfig.id = 0;
	joyconfig.is_active_low = true;
	return joyconfig;
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

BatteryMeter::Config GetBatteryConfig(){
	BatteryMeter::Config config;
	config.voltage_ratio = 1.0f/3.0f;
	return config;
}

St7735r::Config GetSt7735RConfig(){
	St7735r::Config config;
	config.is_revert = true;
	return config;
}

Car::Car():
				m_varmanager(new RemoteVarManager(15)),
				m_encoder_countr(0),
				m_encoder_countl(0),
				m_encoder_count_c(0),
				m_encoder_speed_c(0),
				m_speed_output(0),
				m_led(GetLedConfig(0)),
				m_led2(GetLedConfig(1)),
				m_led3(GetLedConfig(2)),
				m_led4(GetLedConfig(3)),
				m_joy(GetJoyStickConfig()),
				m_mpu6050(GetMpu6050Config()),
				m_mma8451q(GetMma8451qConfig()),
				m_encoder0(GetDirEncoderConfig(0)),
				m_encoder1(GetDirEncoderConfig(1)),
				m_motor_r(GetDirMotorConfig(0)),
				m_motor_l(GetDirMotorConfig(1)),
				m_ccd(0),
				m_bat(GetBatteryConfig()),
				m_lcd(GetSt7735RConfig())

{
	/*
	 * Force NVIC interrupt priority for UART higher than PIT
	 */
	NVIC_SetPriority(UART0_IRQn, 0);
	NVIC_SetPriority(PIT_IRQn, 1);

	m_led.SetEnable(true);
	m_led2.SetEnable(false);
	m_led3.SetEnable(true);
	m_led4.SetEnable(false);

	JyMcuBt106::Config uartconfig;
	uartconfig.baud_rate = libbase::kl26::Uart::Config::BaudRate::k115200;
	uartconfig.id = 0;
	uartconfig.rx_isr = std::bind(&RemoteVarManager::OnUartReceiveSingleChar, m_varmanager, std::placeholders::_1);
	m_com = new JyMcuBt106(uartconfig);

	libutil::InitDefaultFwriteHandler(m_com);

//	m_lcd.Clear(0x0);

}
