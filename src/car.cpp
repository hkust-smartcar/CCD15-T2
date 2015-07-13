#include <car.h>
#include <libsc/led.h>
#include <libutil/misc.h>
#include <libbase/kl26/adc.h>
#include <libbase/kl26/soft_pwm.h>
#include <libutil/string.h>


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
	config.core_clock_khz = 70000;
	config.bus_clock_khz = 35000;
	return config;
}

}
}

Mpu6050::Config GetMpu6050Config(){
	Mpu6050::Config gyro_config;
	gyro_config.accel_range = Mpu6050::Config::Range::kSmall;
	gyro_config.gyro_range = Mpu6050::Config::Range::kSmall;
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
	ledconfig.is_active_low = true;
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

BatteryMeter::Config GetBatteryConfig(){
	BatteryMeter::Config config;
	config.voltage_ratio = 1.0f/3.0f;
	return config;
}

St7735r::Config GetSt7735RConfig(){
	St7735r::Config config;
	config.is_revert = false;
	return config;
}

SimpleBuzzer::Config GetBuzzerConfig(){
	SimpleBuzzer::Config config;
	config.id = 0;
	config.is_active_low = false;
	return config;
}

Us100::Config GetUltrasonicConfig(){
	Us100::Config config;
	config.id = 0;
	Us100 ultrasonic(config);
	return config;
}



void Car::Sw1Down(int id){
	if(id==0){

	}
}

void Car::Sw2Down(int id){
	if(id==1){
		m_car_move_motor = !m_car_move_motor;
//		m_buzzer.SetBeep(true);
//		System::DelayMs(10);
//		m_buzzer.SetBeep(false);

	}
}

void Car::Sw3Down(int id){
	if(id==2/* && (System::Time() - m_prev_pressed_time > 1000)*/){
//		m_prev_pressed_time = System::Time();
		if(m_car_move_motor){
			m_car_move_forward = !m_car_move_forward;
//			m_buzzer.SetBeep(true);
		}
		else{
			m_lcdupdate = !m_lcdupdate;
		}

//		System::DelayMs(50);
//		m_buzzer.SetBeep(false);

		/*m_lcdupdate = !m_lcdupdate;
		m_buzzer.SetBeep(true);
//		System::DelayMs(50);
//		m_buzzer.SetBeep(false);*/
	}

}

void Car::SelectDown(int){
	m_print_state++;
	m_print_state%=m_num_print_states;
	printf("printstates:%d\n",m_print_state);
//	m_buzzer.SetBeep(true);
//	System::DelayMs(50);
//	m_buzzer.SetBeep(false);
}

void Car::SelectRight(int){
	if(!m_car_move_forward){
		m_shift_balance_angle += 0.5f;
	}

}


void Car::SelectLeft(int){
	if(!m_car_move_forward){
		m_shift_balance_angle -= 0.5f;
	}
}


void Car::GetInfrared(Gpi *){
//	static bool state = true;
//	m_buzzer.SetBeep(true);
//	m_car_move_motor = false;
//	printf("Detected\n");
	m_ir_count++;
//	state=!state;
	/*struct note{
		std::string note;
		uint32_t period;
	};

	SoftPwm::Config buzzercfg;
//	buzzercfg.alignment = SoftPwm::Config::Alignment::kEdge;
//	buzzercfg.is_active_high = true;
	buzzercfg.pit_channel = 1;
	buzzercfg.pin = Pin::Name::kPta12;
	buzzercfg.period = 1000;
	buzzercfg.pos_width = 0;
	SoftPwm buzzer(buzzercfg);
	note notes[] = {
//		0-11
		{"C",61158},
		{"C# / Db",57723},
		{"D",54484},
		{"D# / Eb",51427},
		{"E",48541},
		{"F",45815},
		{"F# / Gb",43245},
		{"G",40818},
		{"G# / Ab",38527},
		{"A",36364},
		{"A# / Bb",34323},
		{"B",32396},
//		12-23
		{"C",30578},
		{"C# / Db",28862},
		{"D",27242},
		{"D# / Eb",25713},
		{"E",24270},
		{"F",22907},
		{"F# / Gb",21622},
		{"G",20409},
		{"G# / Ab",19263},
		{"A",18182},
		{"A# / Bb",17161},
		{"B",16198},
//		24-35
		{"C",15289},
		{"C# / Db",14431},
		{"D",13621},
		{"D# / Eb",12856},
		{"E",12135},
		{"F",11454},
		{"F# / Gb",10811},
		{"G",10204},
		{"G# / Ab",9631},
		{"A",9091},
		{"A# / Bb",8581},
		{"B",8099},
//		36-47
		{"C",7645},
		{"C# / Db",7215},
		{"D",6811},
		{"D# / Eb",6428},
		{"E",6067},
		{"F",5727},
		{"F# / Gb",5405},
		{"G",5102},
		{"G# / Ab",4816},
		{"A",4545},
		{"A# / Bb",4290},
		{"B",4050},
//		48-59
		{"C",3822},
		{"C# / Db",3608},
		{"D",3405},
		{"D# / Eb",3214},
		{"E",3034},
		{"F",2863},
		{"F# / Gb",2703},
		{"G",2551},
		{"G# / Ab",2408},
		{"A",2273},
		{"A# / Bb",2145},
		{"B",2025},
//		60-71
		{"C",1911},
		{"C# / Db",1804},
		{"D",1703},
		{"D# / Eb",1607},
		{"E",1517},
		{"F",1432},
		{"F# / Gb",1351},
		{"G",1276},
		{"G# / Ab",1204},
		{"A",1136},
		{"A# / Bb",1073},
		{"B",1012},
//		72-83
		{"C",956},
		{"C# / Db",902},
		{"D",851},
		{"D# / Eb",804},
		{"E",758},
		{"F",716},
		{"F# / Gb",676},
		{"G",638},
		{"G# / Ab",602},
		{"A",568},
		{"A# / Bb",536},
		{"B",506},
//		84-95
		{"C",478},
		{"C# / Db",451},
		{"D",426},
		{"D# / Eb",402},
		{"E",379},
		{"F",358},
		{"F# / Gb",338},
		{"G",319},
		{"G# / Ab",301},
		{"A",284},
		{"A# / Bb",268},
		{"B",253},
//		96-107
		{"C",239},
		{"C# / Db",225},
		{"D",213},
		{"D# / Eb",201},
		{"E",190},
		{"F",179},
		{"F# / Gb",169},
		{"G",159},
		{"G# / Ab",150},
		{"A",142},
		{"A# / Bb",134},
		{"B",127},
//		108-119
		{"C",119},
		{"C# / Db",113},
		{"D",106},
		{"D# / Eb",100},
		{"E",95},
		{"F",89},
		{"F# / Gb",84},
		{"G",80},
		{"G# / Ab",75},
		{"A",71},
		{"A# / Bb",67},
		{"B",63},
		{" ", 1}
	};

	int octave = 6;
	int notesid[5] = {12*(octave-1) + 12, 12*(octave-1) + 10, 12*(octave-1) + 9, 12*(octave-1) + 10, 12*(octave-1+1) + 1 };
	for(int i=0; i<5; i++){
		printf("%s\n",notes[notesid[i]-1].note.c_str());
		buzzer.SetPeriod(notes[notesid[i]-1].period,(notes[notesid[i]-1].period)*1/100);
		System::DelayMs(300);
	}*/
}

Car::Car():
				m_varmanager(new RemoteVarManager(9)),
				m_encoder_countr(0),
				m_encoder_countl(0),
				m_encoder_countr_t(0),
				m_encoder_countl_t(0),
				m_speed_output(0),
				m_total_speed_error(0),
				m_led(GetLedConfig(0)),
				m_led2(GetLedConfig(1)),
				m_led3(GetLedConfig(2)),
				m_led4(GetLedConfig(3)),
				m_joy(GetJoyStickConfig()),
				m_sw1(GetButtonConfig(0)),
				m_sw2(GetButtonConfig(1)),
				m_sw3(GetButtonConfig(2)),
				m_mpu6050(GetMpu6050Config()),
				m_mma8451q(GetMma8451qConfig()),
				m_encoder0(GetDirEncoderConfig(0)),
				m_encoder1(GetDirEncoderConfig(1)),
				m_motor_l(GetDirMotorConfig(0)),
				m_motor_r(GetDirMotorConfig(1)),
				m_ccd_2(1),
				m_ccd_1(0),
				m_bat(GetBatteryConfig()),
				m_lcd(GetSt7735RConfig()),
				m_buzzer(GetBuzzerConfig()),
				m_infrared(GetInfraredConfig()),
				m_infrared_switch(GetInfraredSwitchConfig()),
				m_car_move_motor(false),
				m_car_move_forward(false),
				m_lcdupdate(false),
				m_shift_balance_angle(0.0f),
				m_prev_pressed_time(0),
				m_ir_count(0),
				m_ultrasonic(GetUltrasonicConfig())
{
	/*
	 * Force NVIC interrupt priority for UART higher than PIT
	 */
	NVIC_SetPriority(UART0_IRQn, 0);
	NVIC_SetPriority(PORTC_PORTD_IRQn, 1);
	NVIC_SetPriority(PIT_IRQn, 2);

	m_led.SetEnable(true);
	m_led2.SetEnable(true);
	m_led3.SetEnable(true);
	m_led4.SetEnable(true);

	m_led.SetEnable(true);
	m_led2.SetEnable(false);
	m_led3.SetEnable(true);
	m_led4.SetEnable(false);

	m_buzzer.SetBeep(true);
	System::DelayMs(50);
	m_buzzer.SetBeep(false);

	m_led.SetEnable(false);
	m_led2.SetEnable(false);
	m_led3.SetEnable(false);
	m_led4.SetEnable(false);

	JyMcuBt106::Config uartconfig;
	uartconfig.baud_rate = libbase::kl26::Uart::Config::BaudRate::k115200;
	uartconfig.id = 0;
	uartconfig.rx_isr = std::bind(&RemoteVarManager::OnUartReceiveSingleChar, m_varmanager, std::placeholders::_1);
	m_com = new JyMcuBt106(uartconfig);

	libutil::InitDefaultFwriteHandler(m_com);

	m_lcd.Clear(0);

	m_ultrasonic.Start();


//	float totalVoltage = 0.0f;
//	for(int i=0;  i<=1000; i++){
//		totalVoltage += m_bat.GetVoltage();
//	}
//	OLED_Init();
//	OLED_6x8Str(0,0,(unsigned char*)(String::Format("%.2fV",totalVoltage/1000).c_str()));
//	OLED_image();
}
