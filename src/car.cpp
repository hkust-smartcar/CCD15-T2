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
#include <libutil/positional_pid_controller.h>
#include <cmath>
#include <kalman.h>

using namespace libsc::k60;
using namespace libbase::k60;
using namespace libutil;

int16_t Car::Output_s0(int16_t spdcon[5], uint8_t spdpid[3], uint16_t time[4]){
	uint8_t period;
	int16_t output, temp;
	if (time[0]==0){
		time[0]=System::Time();
		m_encoder0->Update();
		return 0;
	}
	if (spdcon[4]!=spdcon[5]){	//reset summation error if setspeed is changed
		spdcon[4]=spdcon[5];
		spdcon[3]=0;
	}
	spdcon[1]=spdcon[0];
	period=System::Time()-time[0];
	time[0]=System::Time();
	m_encoder0->Update();
	m_encoder_count0 = (int16_t) m_encoder0->GetCount()/2;
//	m_encoder_count_c = (m_encoder_count0 + m_encoder_count1)/4;
	m_encoder_speed0 = m_encoder_count0*1100000/13312/period;	// 512*104/44=13312/11, rotation per sec *100
	spdcon[0]=spdcon[5]-m_encoder_speed0;
	temp=(spdcon[0]-spdcon[1])/period; //slope
	if (abs(spdcon[0])<200){	//prevent large error
		spdcon[3]=spdcon[3]+(spdcon[0]*period/1000);
	}
	output=(spdpid[0]*spdcon[0]+spdpid[1]*spdcon[3]+spdpid[2]*(temp+spdcon[2])/2)/100;
	spdcon[2]=temp;
	return output;
}
int16_t Car::Output_s1(int16_t spdcon[5], uint8_t spdpid[3], uint16_t time[4]){
	uint8_t period;
	int16_t output, temp;
	if (time[1]==0){
		time[1]=System::Time();
		m_encoder1->Update();
		return 0;
	}
	if (spdcon[4]!=spdcon[5]){	//reset summation error if setspeed is changed
		spdcon[4]=spdcon[5];
		spdcon[3]=0;
	}
	spdcon[1]=spdcon[0];
	period=System::Time()-time[1];
	time[1]=System::Time();
	m_encoder1->Update();
	m_encoder_count1 = (int16_t) -1*m_encoder1->GetCount()/2;
//	m_encoder_count_c = (m_encoder_count0 + m_encoder_count1)/4;
	m_encoder_speed1 = m_encoder_count1*1100000/13312/period;	// 512*104/44=13312/11, rotation per sec *100
	spdcon[0]=spdcon[5]-m_encoder_speed1;
	temp=(spdcon[0]-spdcon[1])/period; //slope
	if (abs(spdcon[0])<200){	//prevent large error
		spdcon[3]=spdcon[3]+(spdcon[0]*period/1000);
	}
	output=(spdpid[0]*spdcon[0]+spdpid[1]*spdcon[3]+spdpid[2]*(temp+spdcon[2])/2)/100;
	spdcon[2]=temp;
	return output;
}

int16_t Car::Output_b(float balcon[5], float balpid[3], uint16_t time[4], float real_angle, float gyro_angle){
	uint8_t period;
	int16_t output;
	float temp;
	if (time[2]==0){
		time[2]=System::Time()-10;
		balcon[4]=real_angle-25.0f;
		return 0;
	}

	balcon[1]=balcon[0];
	period=System::Time()-time[2];
	time[2]=System::Time();
	balcon[0]=balcon[4]-real_angle;	// 512*104/44=13312/11
/*	temp=(balpid[0]-balpid[1])/period;
	if (abs(balcon[0])<10){	//prevent overshooting in steady state
		balcon[3]=balcon[3]+(balcon[0]*period/100);
	}*/
//	output=balpid[0]*balcon[0]+balpid[1]*balcon[3]*time[2]+balpid[2]*(temp+balcon[2])*10;
	output=(int16_t)(balpid[0]*balcon[0]-balpid[2]*(balcon[0] - balcon[1])/period/*gyro_angle*/);
//	balcon[2]=temp;
	return output;
}

Car::Car():
		m_encoder_count0(0),
		m_encoder_speed0(0),
		m_encoder_count1(0),
		m_encoder_speed1(0),
		m_encoder_count_c(0),
		m_encoder_speed_c(0),
//		m_inc_pidcontroller(97.0f, 1.0f, 0.0f, 0.0f),
		m_speed_inc_pidcontroller(0.0f, 1.0f, 0.4f, 0.0f),
		m_balance_pid_output(0)
{

}

void Car::Run(){
	Mpu6050::Config gyro_config;
	gyro_config.accel_range = Mpu6050::Config::Range::kSmall;
	gyro_config.gyro_range = Mpu6050::Config::Range::kMid;
	gyro_config.cal_drift = true;

	Mpu6050 mpu6050_(gyro_config);


	std::array<float, 3> accel_, gyro_;
	float real_angle = 0, acc_angle = 0, gyro_angle = 0, avg_gyro = 0, total_gyro=0;

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



	DirMotor::Config motor_config0;
	motor_config0.id = 0;
	DirMotor motor0(motor_config0);

	DirMotor::Config motor_config1;
	motor_config1.id = 1;
	DirMotor motor1(motor_config1);

	DirEncoder::Config econfig0;
	econfig0.id = 0;
	m_encoder0 = new DirEncoder(econfig0);

	DirEncoder::Config econfig1;
	econfig1.id = 1;
	m_encoder1 = new DirEncoder(econfig1);



//	KF m_gyro_kf[3];
//	KF m_acc_kf;

	//	float value[2] = {0.035f, 0.705495694f};
	double value[2] = {0.001, 0.2600496668};
	mpu6050_.Update();
	accel_ = mpu6050_.GetAccel();
	acc_angle = accel_[0] * RAD2ANGLE;
	gyro_angle = acc_angle;
//	kalman_filter_init(&m_gyro_kf[0], 0.001f, value, acc_angle, 1);
	Kalman kf_filter(0.001, value, (double)acc_angle, 1);

	Timer::TimerInt t_ = System::Time(), pt_ = t_;

	int32_t encoder_count = 0, encoder1_count = 0;

	int16_t power0=0, power1=0, u_s0=0, u_s1=0, u_b=0;

	int16_t spdcon0[6]={0,0,0,0,0,0};
	/* spdcon[0]=error(k);
	 * spdcon[1]=error(k-1);
	 * spdcon[2]=previous slope of de/dt for low-pass filtering;
	 * spdcon[3]=summation for ki;
	 * spdcon[4]=previous setpoint for reset summation time;
	 * spdcon[5]=setpoint (rotation per sec *100);	*/
	uint8_t spdpid0[3]={3,5,30};
	/* pid[0]=kp;
	 * pid[1]=ki;
	 * pid[2]=kd;	*/
	int16_t spdcon1[6]={0,0,0,0,0,0};
		/* spdcon[0]=error(k);
		 * spdcon[1]=error(k-1);
		 * spdcon[2]=previous slope of de/dt for low-pass filtering;
		 * spdcon[3]=summation for ki;
		 * spdcon[4]=previous setpoint for reset summation time;
		 * spdcon[5]=setpoint (rotation per sec *100);	*/
	uint8_t spdpid1[3]={3,5,30};
		/* pid[0]=kp;
		 * pid[1]=ki;
		 * pid[2]=kd;	*/

	float balcon[5]={0,0,0,0,-25.5f};
	/* balcon[0]=error(k);
	 * balcon[1]=error(k-1);
	 * balcon[2]=previous slope of de/dt for low-pass filtering;
	 * balcon[3]=summation for ki;
	   balcon[4]=setpoint	*/
	float balpid[3]={1.6f,0.0f,0.7f};
	/* pid[0]=kp;
	 * pid[1]=ki;
	 * pid[2]=kd;	*/

	uint16_t time[4]={0,0,0,0};
	/* time[0] for spd period;
	 * time[1] for spd period;
	 * time[2] for bal period;
	 * time[3] for bal period;	*/

//	Mma8451q::Config accel_config;
//	accel_config.id = 0;
//	accel_config.sensitivity = Mma8451q::Config::Sensitivity::kLow;
//	accel_config.output_data_rate = Mma8451q::Config::OutputDataRate::k800Hz;
//	accel_config.power_mode = Mma8451q::Config::PowerMode::kNormal;
//	Mma8451q accelerometer(accel_config);

	Gpo::Config pinconfig;
	pinconfig.pin = Pin::Name::kPtc0;
	pinconfig.is_high = false;
	Gpo pin0_(pinconfig);

	int16_t cou=0;
	uint32_t tc_ = 0;
	std::array<float, 3> offset_;
	while(true)
	{
		t_ = System::Time();
		if(t_-pt_>=1){
			if(tc_%500==0){
				led.Switch();
				led2.Switch();
				led3.Switch();
				led4.Switch();
			}


			if(tc_%5==0){
				pin0_.Turn();
				mpu6050_.Update();
				accel_ = mpu6050_.GetAccel();
				gyro_ = mpu6050_.GetOmega();
				gyro_[0] = -gyro_[0];
//				gyro_[0] = -gyro_[0] -0.488f + 0.0201001387f;
				acc_angle = accel_[0] * RAD2ANGLE;
				gyro_angle += gyro_[0] * 0.007f;
//				gyro_angle += gyro_[0] * 0.0025f;
//				kalman_filtering(&m_gyro_kf[0], &real_angle, &gyro_angle, &acc_angle, 1);
				double temp;
				kf_filter.Filtering(&temp, (double)gyro_angle, (double)acc_angle);
				real_angle = temp;
//				cou+=1;
//				total_gyro=total_gyro+gyro_[0];
//				avg_gyro=total_gyro/cou;
//				u_b=Output_b(balcon, balpid, time, real_angle);
//				m_balance_pid_output = m_inc_pidcontroller.Calc(real_angle);
				/*spdcon0[5] += m_pid_output;
				spdcon1[5] += m_pid_output;*/
				m_balance_pid_output = Output_b(balcon, balpid, time, real_angle, gyro_[0]);
				power0 = m_balance_pid_output;
				power1 = m_balance_pid_output;
			}


//			if(t%3000==0){
//				switch(spdcon0[5]){
//				case 400:	spdcon0[5]=800;
//							spdcon1[5]=800;	break;
//				case 800:	spdcon0[5]=1000;
//							spdcon1[5]=1000;	break;
//				case 1000:	spdcon0[5]=400;
//							spdcon1[5]=400;	break;
//				}
//			}
			if(tc_%100==0){
//				printf("%.4f,%.4f,%.4f,%.4f,%.4f\n",acc_angle, gyro_angle, real_angle, gyro_[0],avg_gyro);
//				printf("%.5f, %0.5f\n", acc_angle, real_angle);
//				printf("%.5f,%.5f,%.5f,%.5f\n", real_angle, acc_angle, gyro_angle, gyro_[0]);
//				printf("%d,%d\n",m_encoder_count0,m_encoder_count1);
				offset_ = mpu6050_.GetOffset();
				printf("%.5f,%.5f,%.5f,%0.5f\n", real_angle, gyro_angle, acc_angle,accel_[0]);
			}
/*			if(tc%4==0){
				u_s0=Output_s0(spdcon0, spdpid0, time);
				power0=power0+u_s0;
				u_s1=Output_s1(spdcon1, spdpid1, time);
				power1=power1+u_s1;
			}*/

			if(tc_%100==0){
				m_encoder0->Update();
				m_encoder1->Update();
				m_encoder_count0 += -m_encoder0->GetCount();
				m_encoder_count1 += m_encoder1->GetCount();
				m_speed_inc_pidcontroller.OnCalc((m_encoder_count0 + m_encoder_count1)/2);
				m_speed_output = m_speed_inc_pidcontroller.GetControlOut();
//				power0 += m_speed_output;
//				power1 += m_speed_output;
			}


			power0 = libutil::Clamp<int16_t>(-1000,power0, 1000);
			power1 = libutil::Clamp<int16_t>(-1000,power1, 1000);

//			if(real_angle < 15.0f || real_angle > 45.0f) {
//				power0 = power1 = 0;
//			}else{
				if(power0<0)
					power0 -= 40;
				else power0 += 40;
				if(power1<0)
					power1 -= 40;
				else power1 += 40;
//			}

			motor0.SetClockwise(power0 < 0); //Right Motor - true forward, false backward
			motor1.SetClockwise(power1 > 0); //Left Motor - false forward, true backward
			motor0.SetPower((uint16_t)abs(power0+60));
			motor1.SetPower((uint16_t)abs(power1+60));


			pt_ = t_;
			tc_++;

		}
	}
}
