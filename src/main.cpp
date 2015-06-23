#include "app.h"
#include <libsc/system.h>
#include <libutil/misc.h>
#include <libsc/led.h>
#include <libsc/joystick.h>
#include <libsc/mpu6050.h>
#include <libsc/kl26/jy_mcu_bt_106.h>
#include <functional>
#include "OLED.h"

using namespace libsc;
using namespace libsc::kl26;
using namespace libbase::kl26;
using namespace libutil;

int main(){
	System::Init();


//	OLED_Init();
	/*const char* str = "OLED_test";
//
//
	while(true){
//		OLED_6x8Str(30,7,(unsigned char*)str);
//		OLED_6x8Char(30, 7, 'a');
		OLED_image();
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
				asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
						asm("nop");
	}*/
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
