#include "app.h"
#include "DebugApp.h"
#include <libsc/system.h>
#include <libutil/misc.h>
#include <libsc/led.h>
#include <libsc/joystick.h>
#include <libsc/mpu6050.h>
#include <functional>

using namespace libsc;
using namespace libsc::kl26;
using namespace libbase::kl26;
using namespace libutil;

Led::Config GetLedConfig2(int id){
	libsc::Led::Config ledconfig;
	ledconfig.id = id;
	ledconfig.is_active_low = true;
	return ledconfig;
}

int main(){
	System::Init();
	App m_app;

//	JyMcuBt106::Config uartconfig;
//	uartconfig.baud_rate = libbase::kl26::Uart::Config::BaudRate::k115200;
//	uartconfig.id = 0;
////	uartconfig.rx_isr = std::bind(&RemoteVarManager::OnUartReceiveSingleChar, m_varmanager, std::placeholders::_1);
//	JyMcuBt106* m_com = new JyMcuBt106(uartconfig);
//
//	libutil::InitDefaultFwriteHandler(m_com);

//	Joystick joy();

//	Mpu6050::Config gyro_config;
//	gyro_config.accel_range = Mpu6050::Config::Range::kSmall;
//	gyro_config.gyro_range = Mpu6050::Config::Range::kMid;
//	gyro_config.cal_drift = false;
//
//	Mpu6050 mpu(gyro_config);
//	Led led(GetLedConfig2(0));
//
//	bool verified = mpu.Verify();
//	while(true){
//		if(verified){
//			led.SetEnable(true);
//			mpu.Update();
//			std::array<float,3>omega =  mpu.GetOmega();
//			printf("%f,%f,%f\n\r", omega[0],omega[1],omega[2]);
//			System::DelayMs(5);
//		}
//	}
	return 0;
}
