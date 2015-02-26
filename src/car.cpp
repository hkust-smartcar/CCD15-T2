#include "car.h"
#include <libsc/k60/dir_motor.h>
#include <libsc/k60/dir_encoder.h>
#include <libsc/k60/mpu6050.h>
#include <libsc/k60/mma8451q.h>
#include <libbase/k60/gpio.h>
#include <libsc/k60/sys_tick_delay.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include "kalman.h"
#include <libutil/misc.h>
#include <libsc/k60/led.h>
#include <libsc/k60/system.h>

using namespace libsc::k60;
using namespace libbase::k60;

int16_t Car::Output_s(int16_t spdcon[5], uint8_t spdpid[3], uint16_t time[2]){
	uint8_t period;
	int16_t output, temp, encoder_speed;
	if (time[0]==0){
		time[0]=System::Time();
		m_encoder->Update();
		return 0;
	}
	if (spdcon[4]!=spdcon[5]){	//reset summation error if setspeed is changed
		spdcon[4]=spdcon[5];
		spdcon[3]=0;
	}
	spdcon[1]=spdcon[0];
	period=System::Time()-time[0];
	time[0]=System::Time();
	m_encoder->Update();
	m_encoder_count = (int16_t) m_encoder->GetCount() / 2;
	encoder_speed = m_encoder_count*1100000/13312/period;	// 512*104/44=13312/11, rotation per sec *100
	spdcon[0]=spdcon[5]-encoder_speed;
	temp=(spdcon[0]-spdcon[1])/period; //slope
	if (abs(spdcon[0])<400){	//prevent large error
		spdcon[3]=spdcon[3]+(spdcon[0]*period/1000);
	}
	output=(spdpid[0]*spdcon[0]+spdpid[1]*spdcon[3]+spdpid[2]*(temp+spdcon[2])/2)/10;
	spdcon[2]=temp;
	return output;
}
int16_t Car::Output_b(float balcon[4], uint8_t balpid[3], uint16_t time[2], float real_angle){
	uint8_t period;
	int16_t output, temp;
	if (time[1]==0){
		time[1]=System::Time()-10;
		balcon[4]=real_angle;
		return 0;
	}

	balcon[1]=balcon[0];
	period=System::Time()-time[1];
	time[1]=System::Time();
	balcon[0]=balcon[4]-real_angle;	// 512*104/44=13312/11
	temp=(balpid[0]-balpid[1])/period;
	if (abs(balcon[0])<10){	//prevent overshooting in steady state
		balcon[3]=balcon[3]+(balcon[0]*period/100);
	}
	output=balpid[0]*balcon[0]+balpid[1]*balcon[3]/time[1]+balpid[2]*(temp+balcon[2])/2;
	balcon[2]=temp;
	return output;
}

Car::Car(){

}

void Car::Run(){
	Mpu6050::Config gyro_config;
	gyro_config.accel_range = Mpu6050::Config::Range::kSmall;
	gyro_config.gyro_range = Mpu6050::Config::Range::kSmall;
	gyro_config.cal_drift = true;

	Mpu6050 gyro(gyro_config);


	std::array<float, 3> accel_, gyro_;
	float real_angle = 0, acc_angle = 0, gyro_angle = 0;

	Led::Config ledconfig;
	ledconfig.id = 0;
	Led led(ledconfig);

	ledconfig.id = 1;
	Led led2(ledconfig);

	ledconfig.id = 2;
	Led led3(ledconfig);

	ledconfig.id = 3;
	Led led4(ledconfig);


	JyMcuBt106::Config uartconfig;
	uartconfig.baud_rate = Uart::Config::BaudRate::k115200;
	uartconfig.id = 0;
	JyMcuBt106 com(uartconfig);

	libutil::InitDefaultFwriteHandler(&com);



	DirMotor::Config motor_config;
	motor_config.id = 0;
	DirMotor motor(motor_config);

	DirEncoder::Config econfig;
	econfig.id = 0;
	m_encoder = new DirEncoder(econfig);

	/*	DirEncoder::Config econfig1;
	econfig1.id = 1;
	m_encoder1 = new DirEncoder(econfig1);*/



	KF m_gyro_kf[3];
	KF m_acc_kf;

	//	float value[2] = {0.035f, 0.705495694f};
//	float value[2] = {0.035f, 0.205495694f};
//	gyro.Update();
//	accel_ = gyro.GetAccel();
//	acc_angle = accel_[2] * RAD2ANGLE;
//	kalman_filter_init(&m_gyro_kf[0], 0.01f, value, acc_angle, 1);

	Timer::TimerInt t = System::Time(), pt = t;

	int32_t encoder_count = 0, encoder1_count = 0;

	int16_t power=0, u;

	int16_t spdcon[6]={0,0,0,0,0,400};
	/* spdcon[0]=error(k);
	 * spdcon[1]=error(k-1);
	 * spdcon[2]=previous slope of de/dt for low-pass filtering;
	 * spdcon[3]=summation for ki;
	 * spdcon[4]=previous setpoint for reset summation time;
	 * spdcon[5]=setpoint (rotation per sec *100);	*/
	uint8_t spdpid[3]={1,0,0};
	/* pid[0]=kp;
	 * pid[1]=ki;
	 * pid[2]=kd;	*/

	float balcon[5]={0,0,0,0,0};
	/* balcon[0]=error(k);
	 * balcon[1]=error(k-1);
	 * balcon[2]=previous slope of de/dt for low-pass filtering;
	 * balcon[3]=summation for ki;
	   balcon[4]=setpoint	*/
	uint8_t balpid[3]={1,0,0};
	/* pid[0]=kp;
	 * pid[1]=ki;
	 * pid[2]=kd;	*/

	uint16_t time[2]={0,0};
	/* time[0] for spd period;
	 * time[1] for bal period;	*/

//	Mma8451q::Config accel_config;
//	accel_config.id = 0;
//	accel_config.sensitivity = Mma8451q::Config::Sensitivity::kLow;
//	accel_config.output_data_rate = Mma8451q::Config::OutputDataRate::k800Hz;
//	accel_config.power_mode = Mma8451q::Config::PowerMode::kNormal;
//	Mma8451q accelerometer(accel_config);

	Gpo::Config pinconfig;
	pinconfig.pin = Pin::Name::kPtc0;
	pinconfig.is_high = false;
	Gpo pin0(pinconfig);

	while(true)
	{
		t = System::Time();
		if(t-pt>=1){
			if(t%500==0){
				led.Switch();
				led2.Switch();
				led3.Switch();
				led4.Switch();
			}


			if(t%2==0){
				pin0.Turn();
				gyro.Update();
				accel_ = gyro.GetAccel();
				gyro_ = gyro.GetOmega();
				gyro_[0] = -gyro_[0];

				acc_angle = accel_[0] * RAD2ANGLE;
				gyro_angle += gyro_[0] * 0.005f;
//				kalman_filtering(&m_gyro_kf[0], &real_angle, &gyro_angle, &acc_angle, 1);
			}



//			if(t%10==0){
//				printf("%.4f,%.4f,%.4f\n",acc_angle, gyro_angle, real_angle);
//				printf("%.4f,%d\n",acc_angle, accelerometer.IsConnected());
//				printf("%d,%d\n", encoder_count, encoder1_count);
//				u_s=Output_s(spdcon, spdpid, time);
//				u_b=Output_s(balcon, balpid, time, real_angle);
//				power=power+u_s+u_b;
//				power = libutil::Clamp<int16_t>(0,power, 500);
//				printf("%d,%d,%d,%d\n", m_encoder_count, power, encoder_speed, u);
//				motor.SetClockwise(power < 0);
//				motor.SetPower((uint16_t)abs(power));
//				m_encoder->Update();
//				m_encoder_count = m_encoder->GetCount();
//				printf("%d,%d\n", m_encoder_count,power);
//			}

			pt = t;

		}
	}
}
