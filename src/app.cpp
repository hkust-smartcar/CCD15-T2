/*
 * app.cpp
 *
 *  Created on: Mar 3, 2015
 *      Author: harrison
 */
#include "app.h"
#include <libsc/k60/system.h>
#include <libsc/sys_tick_delay.h>
#include <cmath>
#include "kalman.h"
#include "k60/car.h"

using namespace libsc::k60;
using namespace libbase::k60;
/*

int16_t App::Output_s0(int16_t spdcon[5], uint8_t spdpid[3], uint16_t time[4]){
	uint8_t period;
	int16_t output, temp;
	if (time[0]==0){
		time[0]=System::Time();
		m_car.m_encoder0.Update();
		return 0;
	}
	if (spdcon[4]!=spdcon[5]){	//reset summation error if setspeed is changed
		spdcon[4]=spdcon[5];
		spdcon[3]=0;
	}
	spdcon[1]=spdcon[0];
	period=System::Time()-time[0];
	time[0]=System::Time();
	m_car.m_encoder0.Update();
	m_car.m_encoder_count0 = (int16_t) m_car.m_encoder0.GetCount()/2;
//	m_encoder_count_c = (m_encoder_count0 + m_encoder_count1)/4;
	m_car.m_encoder_speed0 = m_car.m_encoder_count0*1100000/13312/period;	// 512*104/44=13312/11, rotation per sec *100
	spdcon[0]=spdcon[5]-m_car.m_encoder_speed0;
	temp=(spdcon[0]-spdcon[1])/period; //slope
	if (abs(spdcon[0])<200){	//prevent large error
		spdcon[3]=spdcon[3]+(spdcon[0]*period/1000);
	}
	output=(spdpid[0]*spdcon[0]+spdpid[1]*spdcon[3]+spdpid[2]*(temp+spdcon[2])/2)/100;
	spdcon[2]=temp;
	return output;
}
int16_t App::Output_s1(int16_t spdcon[5], uint8_t spdpid[3], uint16_t time[4]){
	uint8_t period;
	int16_t output, temp;
	if (time[1]==0){
		time[1]=System::Time();
		m_car.m_encoder1.Update();
		return 0;
	}
	if (spdcon[4]!=spdcon[5]){	//reset summation error if setspeed is changed
		spdcon[4]=spdcon[5];
		spdcon[3]=0;
	}
	spdcon[1]=spdcon[0];
	period=System::Time()-time[1];
	time[1]=System::Time();
	m_car.m_encoder1.Update();
	m_car.m_encoder_count1 = (int16_t) -1*m_car.m_encoder1.GetCount()/2;
//	m_encoder_count_c = (m_encoder_count0 + m_encoder_count1)/4;
	m_car.m_encoder_speed1 = m_car.m_encoder_count1*1100000/13312/period;	// 512*104/44=13312/11, rotation per sec *100
	spdcon[0]=spdcon[5]-m_car.m_encoder_speed1;
	temp=(spdcon[0]-spdcon[1])/period; //slope
	if (abs(spdcon[0])<200){	//prevent large error
		spdcon[3]=spdcon[3]+(spdcon[0]*period/1000);
	}
	output=(spdpid[0]*spdcon[0]+spdpid[1]*spdcon[3]+spdpid[2]*(temp+spdcon[2])/2)/100;
	spdcon[2]=temp;
	return output;
}
*/

int16_t App::Output_b(float balcon[5], float balpid[3], uint16_t time[4], float real_angle, float gyro_angle){
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

App::App():
	m_car(),
//	m_inc_pidcontroller(97.0f, 1.0f, 0.0f, 0.0f),
	m_speed_inc_pidcontroller(0.0f, 1.0f, 0.4f, 0.0f),
	m_balance_pid_output(0)
{
	std::array<float, 3> accel_, gyro_;
	float real_angle = 0, acc_angle = 0, gyro_angle = 0, avg_gyro = 0, total_gyro=0;


	double value[2] = {0.001, 0.2600496668};
	m_car.m_mpu6050.Update();
	accel_ = m_car.m_mpu6050.GetAccel();
	acc_angle = accel_[2] * RAD2ANGLE;
	gyro_angle = acc_angle;
	Kalman kf_filter(0.001, value, (double)acc_angle, 1);

	Timer::TimerInt t_ = System::Time(), pt_ = t_;

	int32_t encoder_count = 0, encoder1_count = 0;

	int16_t power0=0, power1=0, u_s0=0, u_s1=0, u_b=0;

	int16_t spdcon0[6]={0,0,0,0,0,0};
	 /*spdcon[0]=error(k);
	 * spdcon[1]=error(k-1);
	 * spdcon[2]=previous slope of de/dt for low-pass filtering;
	 * spdcon[3]=summation for ki;
	 * spdcon[4]=previous setpoint for reset summation time;
	 * spdcon[5]=setpoint (rotation per sec *100);
	 */
	uint8_t spdpid0[3]={3,5,30};
	 /*pid[0]=kp;
	  pid[1]=ki;
	 * pid[2]=kd;
	 */
	int16_t spdcon1[6]={0,0,0,0,0,0};
	 /*spdcon[0]=error(k);
	 /* spdcon[1]=error(k-1);
	 * spdcon[2]=previous slope of de/dt for low-pass filtering;
	 * spdcon[3]=summation for ki;
	 * spdcon[4]=previous setpoint for reset summation time;
	 * spdcon[5]=setpoint (rotation per sec *100);
	 */
	uint8_t spdpid1[3]={3,5,30};
	 /*pid[0]=kp;
	 * pid[1]=ki;
	 * pid[2]=kd;
	 */

	float balcon[5]={0,0,0,0,-25.5f};
	 /*balcon[0]=error(k);
	 * balcon[1]=error(k-1);
	 * balcon[2]=previous slope of de/dt for low-pass filtering;
	 * balcon[3]=summation for ki;
	 *  balcon[4]=setpoint
	*/
	float balpid[3]={1.85f,0.0f,0.5f};
	 /*pid[0]=kp;
	 * pid[1]=ki;
	 * pid[2]=kd;
	 */

	uint16_t time[4]={0,0,0,0};
	/* time[0] for spd period;
	 * time[1] for spd period;
	 * time[2] for bal period;
	 * time[3] for bal period;
	*/
	uint32_t tc_ = 0;
	std::array<float, 3> offset_;
	std::array<uint16_t,libsc::k60::LinearCcd::kSensorW> ccd_data_;
	int y = 0;
	double temp;
	while(true)
	{
		t_ = System::Time();
		if(t_-pt_>=1){
			if(tc_%400==0){
				m_car.m_ccd.StartSample();
				while(!m_car.m_ccd.SampleProcess()){}
				ccd_data_ = m_car.m_ccd.GetData();
				uint16_t avg = 0;
				uint32_t sum = 0;
				for(int i=0; i<libsc::k60::LinearCcd::kSensorW; i++){
					sum += (uint32_t)ccd_data_[i];
				}
				avg = (uint16_t) (sum / libsc::k60::LinearCcd::kSensorW);
				libsc::k60::St7735r::Rect rect_;

				uint16_t color = 0;

				for(int i=0; i<libsc::k60::LinearCcd::kSensorW; i++){
					rect_.x = i;
					rect_.y = y;
					rect_.w = 1;
					rect_.h = 1;
					m_car.m_lcd.SetRegion(rect_);
					if(ccd_data_[i] >= avg){
							color = ~0;
					}else{
						color = 0;
					}
					if(ccd_data_[i] < 8000){
						color = 0;
					}else if(ccd_data_[i] > 57000){
						color = ~0;
					}
					m_car.m_lcd.FillColor(color);
				}
				y++;
				y=y%160;
			}

			if(tc_%500==0){
				m_car.m_led.Switch();
				m_car.m_led2.Switch();
				m_car.m_led3.Switch();
				m_car.m_led4.Switch();
			}


			if(tc_%5==0){
				m_car.m_mpu6050.Update();
				accel_ = m_car.m_mpu6050.GetAccel();
				gyro_ = m_car.m_mpu6050.GetOmega();
//				gyro_[1] = -gyro_[1];
				acc_angle = accel_[2] * RAD2ANGLE;
				gyro_angle += gyro_[1] * 0.007f;

				/*kf_filter.Filtering(&temp, (double)gyro_angle, (double)acc_angle);
				real_angle = (float)temp;*/
				//				cou+=1;
				//				total_gyro=total_gyro+gyro_[0];
				//				avg_gyro=total_gyro/cou;
				//				u_b=Output_b(balcon, balpid, time, real_angle);
				//				m_balance_pid_output = m_inc_pidcontroller.Calc(real_angle);
				m_balance_pid_output = Output_b(balcon, balpid, time, real_angle, gyro_[1]);
				power0 = m_balance_pid_output;
				power1 = m_balance_pid_output;
			}

			if(tc_%100==0){
				//				printf("%.4f,%.4f,%.4f,%.4f,%.4f\n",acc_angle, gyro_angle, real_angle, gyro_[0],avg_gyro);
				//				printf("%.5f, %0.5f\n", acc_angle, real_angle);
				//				printf("%.5f,%.5f,%.5f,%.5f\n", real_angle, acc_angle, gyro_angle, gyro_[0]);
				//				printf("%d,%d\n",m_encoder_count0,m_encoder_count1);
//				offset_ = m_car.m_mpu6050.GetOffset();
				printf("%.2f,%.2f\n", gyro_angle, acc_angle);
			}
			/*if(tc_%4==0){
				u_s0=Output_s0(spdcon0, spdpid0, time);
				power0=power0+u_s0;
				u_s1=Output_s1(spdcon1, spdpid1, time);
				power1=power1+u_s1;
			}*/

			if(tc_%100==0){
//				m_car.m_encoder0.Update();
//				m_car.m_encoder1.Update();
//				m_car.m_encoder_count0 += -m_car.m_encoder0.GetCount();
//				m_car.m_encoder_count1 += m_car.m_encoder1.GetCount();
//				m_speed_inc_pidcontroller.OnCalc((m_car.m_encoder_count0 + m_car.m_encoder_count1)/2);
//				m_car.m_speed_output = m_speed_inc_pidcontroller.GetControlOut();
				//				power0 += m_speed_output;
				//				power1 += m_speed_output;
			}


			power0 = libutil::Clamp<int16_t>(-1000,power0, 1000);
			power1 = libutil::Clamp<int16_t>(-1000,power1, 1000);

//			if(real_angle < 15.0f || real_angle > 45.0f) {
//				power0 = power1 = 0;
//			}

			m_car.m_motor0.SetClockwise(power0 < 0); //Right Motor - true forward, false backward
			m_car.m_motor1.SetClockwise(power1 > 0); //Left Motor - false forward, true backward
			m_car.m_motor0.SetPower((uint16_t)abs(power0+40));
			m_car.m_motor1.SetPower((uint16_t)abs(power1+40));


			pt_ = t_;
			tc_++;

		}
	}

}


