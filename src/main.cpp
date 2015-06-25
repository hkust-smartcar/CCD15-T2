#include "app.h"
#include <libutil/string.h>
#include <libsc/system.h>
#include <libutil/misc.h>
#include <libsc/led.h>
#include <libsc/joystick.h>
#include <libsc/mpu6050.h>
#include <libsc/kl26/jy_mcu_bt_106.h>
#include <functional>
#include <libbase/kl26/lptmr.h>
#include <libbase/kl26/tpm_pwm.h>
#include <libbase/kl26/gpio.h>
#include <libbase/kl26/hardware.h>


using namespace libsc;
using namespace libsc::kl26;
using namespace libbase::kl26;
using namespace libutil;

/*JyMcuBt106* m_com;
Timer::TimerInt g_prev_t;

void signalisr(Gpi*){
	m_com->SendStr(libutil::String::Format("%s %d\n","Detected", System::Time()));
	if(System::Time() - g_prev_t > 0){
		m_com->SendStr(libutil::String::Format("%s %d\n","Detected", System::Time() - g_prev_t));
		g_prev_t = System::Time();
	}
}*/


int main(){
	System::Init();

//	g_prev_t = 0;

/*	libbase::kl26::TpmPwm::Config tpmconfig;
	tpmconfig.pin = libbase::kl26::Pin::Name::kPta5;
	tpmconfig.period = 1000000000;
	tpmconfig.pos_width = 0;
	tpmconfig.precision = TpmPwm::Config::Precision::kNs;
	tpmconfig.alignment = TpmPwm::Config::Alignment::kCenter;
	TpmPwm pwm(tpmconfig);*/



//	Lptmr::Config lptmrconfig;
//	lptmrconfig.count = UINT16_MAX;
//	Lptmr time(lptmrconfig);


//	JyMcuBt106::Config uartconfig;
//	uartconfig.baud_rate = libbase::kl26::Uart::Config::BaudRate::k115200;
//	uartconfig.id = 0;
//	m_com = new JyMcuBt106(uartconfig);
//
//	Gpi::Config pinconfig;
//	pinconfig.pin = Pin::Name::kPtd0;
//	pinconfig.interrupt = Pin::Config::Interrupt::kBoth;
//	pinconfig.isr = std::bind(&signalisr, std::placeholders::_1);
//	Gpi lptmrpin(pinconfig);
//
//	NVIC_SetPriority(PORTC_PORTD_IRQn, 1);
//	NVIC_SetPriority(PIT_IRQn, 2);
//
//	while(true){
////		time.ClearCounter();
//		System::DelayMs(1000);
//		m_com->SendStr(libutil::String::Format("Running %d\n",System::Time()));
//	}

	App m_app;

/*	JyMcuBt106::Config uartconfig;
	uartconfig.baud_rate = libbase::kl26::Uart::Config::BaudRate::k115200;
	uartconfig.id = 0;
	JyMcuBt106* m_com = new JyMcuBt106(uartconfig);
	libutil::InitDefaultFwriteHandler(m_com);

	Adc::Config adcconfig;
	adcconfig.adc = Adc::Name::kAdc0Ad23;
	Adc* ir = new Adc(adcconfig);

	while(true){
		printf("%f\n", ir->GetResultF());
		System::DelayMs(5);
	}*/

	return 0;
}
