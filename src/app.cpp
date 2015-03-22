/*
 * app.cpp
 *
 *  Created on: Mar 3, 2015
 *      Author: harrison
 */
#include "app.h"
#include <libsc/system.h>
#include <libsc/sys_tick_delay.h>
#include <cmath>
#include "kalman.h"
#include "k60/car.h"
#include "Quaternion.h"
#include <libutil/misc.h>


using namespace libsc::k60;
using namespace libbase::k60;

uint16_t App::RpmToPwm1(uint16_t count){
	return (uint16_t)(0.1484715791f*count + 59.6501510982f);
}

uint16_t App::RpmToPwm0(uint16_t count){
	return (uint16_t)(0.1462728045f*count + 34.5814814763f);
}

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

int16_t App::Output_b(float* balcon, float* balpid, uint16_t* time, float real_angle, float gyro_angle){
	uint8_t period;
	int16_t output;
	static int16_t total_output = 0;
	float temp;
	if (time[2]==0){
		time[2]=System::Time()-10;
//		balcon[4]=real_angle-25.0f;
		return 0;
	}


	period=System::Time()-time[2];
	time[2]=System::Time();
	balcon[0]=(balcon[4]+balcon[5])-real_angle;	// 512*104/44=13312/11
	temp=(balpid[0]-balpid[1])/period;
//	if (abs(balcon[0])<10){	//prevent overshooting in steady state
//		balcon[3]=balcon[3]+(balcon[0]*period/100);
//	}
//	output=balpid[0]*balcon[0]+balpid[1]*balcon[3]*time[2]+balpid[2]*(temp+balcon[2])*10;
//	output=(int16_t)(balpid[0]*(balcon[0] - balcon[1]));
	output=(int16_t)(balpid[0]*balcon[0] + balpid[2]*(temp+balcon[2])/2);
	balcon[1]=balcon[0];
	balcon[2]=temp;
	return output;
}

float App::Output_speed(int16_t* carspeedcon, float* carspeedpid, int16_t encoder){
	float output;
	static int16_t total_encoder = 0;
	Timer::TimerInt t = System::Time();
	static Timer::TimerInt pt = System::Time();


	carspeedcon[0]=carspeedcon[4]-encoder;
	total_encoder+=carspeedcon[0];

	output=(float)(carspeedpid[0]*carspeedcon[0] + carspeedpid[1]* total_encoder * (t-pt)/1000.0f);
	pt = t;
	return output;
}

void Update_edge(uint8_t* ccd_data_, uint8_t* edge){
	if(edge[1]==0){
		for (uint8_t i=64; ccd_data_[i]<57000&&i<128; i++){
			edge[1]=i;
		}
	}
	if(edge[0]==libsc::Tsl1401cl::kSensorW){
		for (uint8_t i=64; ccd_data_[i]<57000&&i>=0; i--){
			edge[0]=i;
		}
	}
	while (ccd_data_[edge[0]]<57000){
		edge[0]+=1;
	}
	while (ccd_data_[edge[0]]>57000){
		edge[0]-=1;
	}
	while (ccd_data_[edge[1]]<57000){
		edge[1]-=1;
	}
	while (ccd_data_[edge[1]]>57000){
		edge[1]+=1;
	}
	edge[0] = libutil::Clamp<int8_t>(0, edge[0], libsc::Tsl1401cl::kSensorW);
	edge[1] = libutil::Clamp<int8_t>(0, edge[1], libsc::Tsl1401cl::kSensorW);
}

App::App():
	m_car(),
	m_balance_pid_output(0),
	m_speed_control0(1000,m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal()),
	m_speed_control1(1000,m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal())
{
	printf("skp,real,0,0.0\n");
	printf("skd,real,1,0.0\n");
	printf("ski,real,2,0.0\n");
	printf("bkp,real,3,0.0\n");
	printf("bkd,real,4,0.0\n");
	printf("boff,real,5,0.0\n");

	std::array<float, 3> accel_, gyro_;
	float real_angle = 0, acc_angle = 0, gyro_angle = 0, prev_gyro_angle = 0, avg_gyro = 0, total_gyro=0;

	Quaternion quaternion(0.002, &(m_car.m_mpu6050));


	double value[2] = {0.001, 0.2600496668};
	m_car.m_mpu6050.Update();
	accel_ = m_car.m_mpu6050.GetAccel();
	acc_angle = accel_[2] * RAD2ANGLE;
	gyro_angle = acc_angle;
	Kalman kf_filter(0.001, value, (double)acc_angle, 1);

	Timer::TimerInt t_ = System::Time(), pt_ = t_;

	int32_t encoder_count = 0, encoder1_count = 0;

	int16_t power_l=0, power_r=0, u_s0=0, u_s1=0, u_b=0, turn_power0=0, turn_power1=0, speed_power0=0,speed_power1=0;

	/*spdcon[0]=error(k);
	 * spdcon[1]=error(k-1);
	 * spdcon[2]=previous slope of de/dt for low-pass filtering;
	 * spdcon[3]=summation for ki;
	 * spdcon[4]=previous setpoint for reset summation time;
	 * spdcon[5]=setpoint (rotation per sec *100);
	 */
	int16_t spdcon0[6]={0,0,0,0,0,0};

	/* pid[0]=kp;
	 * pid[1]=ki;
	 * pid[2]=kd;
	 */
	uint8_t spdpid0[3]={3,5,30};

	/*spdcon[0]=error(k);
	 * spdcon[1]=error(k-1);
	 * spdcon[2]=previous slope of de/dt for low-pass filtering;
	 * spdcon[3]=summation for ki;
	 * spdcon[4]=previous setpoint for reset summation time;
	 * spdcon[5]=setpoint (rotation per sec *100);
	 */
	int16_t spdcon1[6]={0,0,0,0,0,0};

	/*pid[0]=kp;
	 * pid[1]=ki;
	 * pid[2]=kd;
	 */
	uint8_t spdpid1[3]={3,5,30};

	/*balcon[0]=error(k);
	 * balcon[1]=error(k-1);
	 * balcon[2]=previous slope of de/dt for low-pass filtering;
	 * balcon[3]=summation for ki;
	 *  balcon[4]=setpoint
	 *  balcon[5]=setpoint offset
	*/
	float balcon[6]={0,0,0,0,0.0f,0};

	/*pid[0]=kp;
	 * pid[1]=ki;
	 * pid[2]=kd;
	 */
	float balpid[3]={0.0f,0.0f,0.0f};

	/*carspeedcon[0]=error(k);
	 * carspeedcon[1]=error(k-1);
	 * carspeedcon[2]=previous slope of de/dt for low-pass filtering;
	 * carspeedcon[3]=summation for ki;
	 * carspeedcon[4]=setpoint;
	*/
	int16_t carspeedcon[5]={0,0,0,0,0};

	/*carspeedpid[0]=kp;
	* carspeedpid[1]=ki;
	* carspeedpid[2]=kd;
	*/
	float carspeedpid[3]={0.1f,0.3f,0.0f};

	/* time[0] for spd period;
	 * time[1] for spd period;
	 * time[2] for bal period;
	 * time[3] for bal period;
	*/
	uint16_t time[4]={0,0,0,0};

	uint32_t tc_ = 0;
	std::array<float, 3> offset_;
	std::array<uint16_t,libsc::Tsl1401cl::kSensorW> ccd_data_;

	enum CCD_COLOR{
		CCD_BLACK = 0,
		CCD_WHITE
	};

	std::array<CCD_COLOR,libsc::Tsl1401cl::kSensorW> color;
	int y = 0;
	Timer::TimerInt gyro_t = 0, gyro_pt = 0;
	double temp;

	m_car.m_ccd.StartSample();

	int16_t speedsp = 0;

	Gpo::Config pinconfig;
	pinconfig.pin = Pin::Name::kPtb22;
	Gpo pin(pinconfig);

	while(true)
	{
		t_ = System::Time();
		if(t_-pt_>=1){
			m_car.m_ccd.SampleProcess();
			if(tc_%10==0) {
				if(m_car.m_ccd.IsImageReady()){
					ccd_data_ = m_car.m_ccd.GetData();
					uint16_t avg = 0;
					uint32_t sum = 0;
					for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
						sum += (uint32_t)ccd_data_[i];
					}
					avg = (uint16_t) (sum / libsc::Tsl1401cl::kSensorW);
					for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
						if(ccd_data_[i] >= avg){
								color[i] = CCD_WHITE;
						}else{
							color[i] = CCD_BLACK;
						}
//						if(ccd_data_[i] < 8000){
//							color = 0;
//						}else if(ccd_data_[i] > 57000){
//							color = ~0;
//						}
					}
					int left_edge = 0;
					int right_edge = 127;
					int cameramid = (0 + 127)/2;
					/*for(int i=1; i<libsc::k60::LinearCcd::kSensorW-1; i++){
						if(color[i]==CCD_BLACK && color[i+1]==CCD_WHITE){
							left_edge = i;
						}
						if(color[i]==CCD_WHITE && color[i+1]==CCD_BLACK){
							right_edge = i;
						}
					}*/
					for (int i=64; i<libsc::Tsl1401cl::kSensorW-1; i++){
						if(color[i]==CCD_WHITE && color[i+1]==CCD_BLACK) right_edge=i;
					}
					for (int i=64; i>=0; i--){
						if(color[i]==CCD_BLACK && color[i+1]==CCD_WHITE) left_edge=i;
					}
					int mid = (left_edge + right_edge)/2 + 2;


					int error = cameramid - mid;
					turn_power0 = -4*error;
					turn_power1 = 4*error;
					/*libsc::k60::St7735r::Rect rect_;
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
//						if(ccd_data_[i] < 8000){
//							color = 0;
//						}else if(ccd_data_[i] > 57000){
//							color = ~0;
//						}
						m_car.m_lcd.FillColor(color);
					}
					y++;
					y=y%160;*/
					m_car.m_ccd.StartSample();
				}
			}



			if(tc_%500==0){
				m_car.m_led.Switch();
				m_car.m_led2.Switch();
				m_car.m_led3.Switch();
				m_car.m_led4.Switch();
			}


			if(tc_%2==0){
				pin.Set(true);

				gyro_t = System::Time();
				m_car.m_mpu6050.Update();

				quaternion.Update();
//				accel_ = m_car.m_mpu6050.GetAccel();
				gyro_ = m_car.m_mpu6050.GetOmega();
				gyro_[1] = -gyro_[1];
				/*				acc_angle = accel_[2] * RAD2ANGLE;
				gyro_angle += gyro_[1] * ((float)(gyro_t - gyro_pt)/1000.0f);
				gyro_pt = gyro_t;
				kf_filter.Filtering(&temp, (double)gyro_angle, (double)acc_angle);*/
				real_angle = (float)quaternion.getEuler(0)*57.2957795131;
				//				cou+=1;
				//				total_gyro=total_gyro+gyro_[0];
				//				avg_gyro=total_gyro/cou;
				//				u_b=Output_b(balcon, balpid, time, real_angle);
//				m_balance_pid_output = m_inc_pidcontroller.Calc(real_angle);
				balpid[0] = m_bkp->GetReal();
				balpid[2] = m_bkd->GetReal();
				balcon[5] = m_boff->GetReal();
				m_balance_pid_output = -Output_b(balcon, balpid, time, real_angle, gyro_[1]);
				m_speed_control0.SetSetpoint(m_balance_pid_output);
				m_speed_control1.SetSetpoint(m_balance_pid_output);
				pin.Set(false);
			}

			if(tc_%50==0){
				//				printf("%.4f,%.4f,%.4f,%.4f,%.4f\n",acc_angle, gyro_angle, real_angle, gyro_[0],avg_gyro);
				//				printf("%.5f, %0.5f\n", acc_angle, real_angle);
				//				printf("%.5f,%.5f,%.5f,%.5f\n", real_angle, acc_angle, gyro_angle, gyro_[0]);
				//				printf("%d,%d\n",m_encoder_count0,m_encoder_count1);
//				offset_ = m_car.m_mpu6050.GetOffset();
//				printf("%.4f,%d,%d,%.4f\n", real_angle, power0, m_balance_pid_output, balcon[5]);
				printf("%.4f,%.4f,%.4f\n",quaternion.getEuler(0)*57.2957795131, quaternion.getEuler(1)*57.2957795131, quaternion.getEuler(2)*57.2957795131);
//				printf("%f,%f,%f,%d\n", m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal(), power_r, m_car.m_encoder_countr);
//				printf("%f\n",real_angle);
			}
//			if(tc_%4==0){
//				u_s0=Output_s0(spdcon0, spdpid0, time);
//				power0=power0+u_s0;
//				u_s1=Output_s1(spdcon1, spdpid1, time);
//				power1=power1+u_s1;
//			}

			if(tc_%3000==0){
//				speedsp += 200;
//				speedsp %= 600;
//				m_speed_control0.SetSetpoint(speedsp);
//				m_speed_control1.SetSetpoint(speedsp);
			}
			if(tc_%2==0){
				m_car.m_encoder0.Update();
				m_car.m_encoder1.Update();
				m_car.m_encoder_countr = -m_car.m_encoder0.GetCount(); //right wheel
				m_car.m_encoder_countl = m_car.m_encoder1.GetCount(); //left wheel

//				balcon[5] = -(float)(m_car.m_encoder_count0 + m_car.m_encoder_count1)/2.0f/500.0f;
//				balcon[5] = Output_speed(carspeedcon, carspeedpid, (m_car.m_encoder_count0 + m_car.m_encoder_count1)/2);
//				speed_power0 = Output_speed(carspeedcon, carspeedpid, (m_car.m_encoder_count0 + m_car.m_encoder_count1)/2);
//				speed_power1 = speed_power0;



				m_speed_control0.SetKp(75.0f);
				m_speed_control0.SetKd(m_skd->GetReal());
				m_speed_control0.SetKi(m_ski->GetReal());
				m_speed_control1.SetKp(75.0f);
				m_speed_control1.SetKd(m_skd->GetReal());
				m_speed_control1.SetKi(m_ski->GetReal());
				int16_t r_val = speedsp + m_speed_control0.Calc(m_car.m_encoder_countr);
				int16_t l_val = speedsp + m_speed_control1.Calc(m_car.m_encoder_countl);
				power_r = sign(r_val) * RpmToPwm0(abs(r_val));
				power_l = sign(l_val) * RpmToPwm1(abs(l_val));
//				printf("%d,%d,%d\n", speedsp, m_car.m_encoder_countr, m_car.m_encoder_countl);

			}

			if(tc_%50==0){
//				ccd_data_=my_car.m_ccd.GetData();
//				Update_edge(ccd_data_, edge);
//				if (edge[0]+edge[i]>libsc::k60::LinearCcd::kSensorW){
//					power0*=1.4;
//					power1*=0.75;
//				}
//				if (edge[0]+edge[i]<libsc::k60::LinearCcd::kSensorW){
//					power0*=0.75;
//					power1*=1.4;
//				}
//				if (edge[0]+edge[i]=libsc::k60::LinearCcd::kSensorW){
//					power0*=1.1;
//					power1*=1.1;
//				}
			}

//			power0 = m_balance_pid_output;
//			power1 = m_balance_pid_output;

			power_l = libutil::Clamp<int16_t>(-1000,power_l, 1000);
			power_r = libutil::Clamp<int16_t>(-1000,power_r, 1000);

/*			if(power0>0) power0+=90;
			if(power1>0) power1+=90;

			if(power0<0) power0-=90;
			if(power1<0) power1-=90;*/

//			if(abs(power_l) >= 950) power_l = 0;
//			if(abs(power_r) >= 950) power_r = 0;

			m_car.m_motor0.SetClockwise(power_l < 0); //Right Motor - true forward, false backward
			m_car.m_motor1.SetClockwise(power_r > 0); //Left Motor - false forward, true backward
			m_car.m_motor0.SetPower((uint16_t)abs(power_l));
			m_car.m_motor1.SetPower((uint16_t)abs(power_r));


			pt_ = t_;
			tc_++;

		}
	}

}


