/*
 * app.cpp
 *
 *  Created on: Mar 3, 2015
 *      Author: harrison
 */
#include <car.h>
#include <cmath>
#include <libsc/system.h>
#include <libsc/sys_tick_delay.h>
#include <libutil/misc.h>
#include <libbase/kl26/gpio.h>
#include "app.h"
#include "kalman.h"
#include "Quaternion.h"
#include "upstand.h"


using namespace libsc::kl26;
using namespace libbase::kl26;

/*
 * For K60_2014_CAMERA ONLY
 * uint16_t App::RpmToPwm1(uint16_t count){
	return (count == 0) ? 0 : (uint16_t)(0.1484715791f*count + 59.6501510982f);
}

uint16_t App::RpmToPwm0(uint16_t count){
	return (count == 0) ? 0 : (uint16_t)(0.1462728045f*count + 34.5814814763f);
}*/

/*For KL26_2015_CCD ONLY*/
uint16_t App::RpmToPwm_R(uint16_t count){
	return (count == 0) ? 0 : (uint16_t)(3.4440590762f*count + 44.4624651753f);
}

uint16_t App::RpmToPwm_L(uint16_t count){
	return (count == 0) ? 0 : (uint16_t)(3.3664769271f*count + 95.3022889986f);
}


int16_t App::Output_b(float* balcon, float* balpid, uint16_t* time, float real_angle, float omega){
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
	balcon[0]=(balcon[4]+balcon[5]+balcon[6])-real_angle;	// 512*104/44=13312/11
	temp=(balpid[0]-balpid[1])/period;
//	if (abs(balcon[0])<10){	//prevent overshooting in steady state
//		balcon[3]=balcon[3]+(balcon[0]*period/100);
//	}
//	output=balpid[0]*balcon[0]+balpid[1]*balcon[3]*time[2]+balpid[2]*(temp+balcon[2])*10;
//	output=(int16_t)(balpid[0]*(balcon[0] - balcon[1]));
	total_output += balcon[0] * period;
	/*(temp+balcon[2])/2)*/
	output=(int16_t)(balpid[0]*balcon[0] + balpid[2] * (temp+balcon[2])/2 + balpid[1] * total_output);
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

	output=(float)(carspeedpid[0]*carspeedcon[0] + carspeedpid[1]* total_encoder * (t-pt)/1000.0f + carspeedpid[2]*(carspeedcon[0] - carspeedcon[1])/(t-pt)*1000.0f);
	carspeedcon[1] = carspeedcon[0];
	pt = t;
	return output;
}

//void Update_edge(uint8_t* ccd_data_, uint8_t* edge){
//	if(edge[1]==0){
//		for (uint8_t i=64; ccd_data_[i]<57000&&i<128; i++){
//			edge[1]=i;
//		}
//	}
//	if(edge[0]==libsc::Tsl1401cl::kSensorW){
//		for (uint8_t i=64; ccd_data_[i]<57000&&i>=0; i--){
//			edge[0]=i;
//		}
//	}
//	while (ccd_data_[edge[0]]<57000){
//		edge[0]+=1;
//	}
//	while (ccd_data_[edge[0]]>57000){
//		edge[0]-=1;
//	}
//	while (ccd_data_[edge[1]]<57000){
//		edge[1]-=1;
//	}
//	while (ccd_data_[edge[1]]>57000){
//		edge[1]+=1;
//	}
//	edge[0] = libutil::Clamp<int8_t>(0, edge[0], libsc::Tsl1401cl::kSensorW);
//	edge[1] = libutil::Clamp<int8_t>(0, edge[1], libsc::Tsl1401cl::kSensorW);
//}

App::App():
	m_car(),
	m_lcd_typewriter(GetLcdTypewriterConfig()),
	m_balance_pid_output(0),
	m_speed_control0(0,m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal()),
	m_speed_control1(0,m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal())
{
	m_lcd_typewriter.WriteString(String::Format("%.3fV\n",m_car.m_bat.GetVoltage()).c_str());
	m_car.m_varmanager->Broadcast(m_car.m_com);

	Quaternion quaternion(0.002, &(m_car.m_mpu6050));
//	Upstand upstand(&(m_car.m_mpu6050));


	double value[2] = {0.001, 0.015};
	m_car.m_mpu6050.Update();
	std::array<float, 3> accel_, gyro_;
	float real_angle = 0, acc_angle = 0, gyro_angle = 0, prev_gyro_angle = 0, avg_gyro = 0, total_gyro=0;

	accel_ = m_car.m_mpu6050.GetAccel();
	acc_angle = accel_[2]/* * RAD2ANGLE*/;
	gyro_angle = 0;//acc_angle;
	real_angle = acc_angle;
	Kalman kf_filterR(0.0001, value, (double)0, 1);
	Kalman kf_filterP(0.0001, value, (double)0, 1);
	Kalman kf_filterY(0.0001, value, (double)0, 1);

	Timer::TimerInt t_ = System::Time(), pt_ = t_;


	int16_t power_l=0, power_r=0, u_s0=0, u_s1=0, u_b=0, turn_power0=0, turn_power1=0;

	/*balcon[0]=error(k);
	 * balcon[1]=error(k-1);
	 * balcon[2]=previous slope of de/dt for low-pass filtering;
	 * balcon[3]=summation for ki;
	 *  balcon[4]=setpoint
	 *  balcon[5]=setpoint offset
	*/
	float balcon[7]={0,0,0,0,-20.0f,0,0};

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
	float carspeedpid[3]={0.0f,0.0f,0.0f};

	/* time[0] for spd period;
	 * time[1] for spd period;
	 * time[2] for bal period;
	 * time[3] for bal period;
	*/
	uint16_t time[4]={0,0,0,0};

	uint32_t tc_ = 0;
	std::array<uint16_t,libsc::Tsl1401cl::kSensorW> ccd_data_;

	enum CCD_COLOR{
		CCD_BLACK = 0,
		CCD_WHITE
	};

	std::array<CCD_COLOR,libsc::Tsl1401cl::kSensorW> color;
	int y = 0;
	Timer::TimerInt gyro_t = 0, gyro_pt = System::Time();;

	m_car.m_ccd.StartSample();

	int16_t speedsp = 0;
	double temp = 0.0;

//	Gpo::Config pinclkcfg;
//	pinclkcfg.pin = Pin::Name::kPte18;
//	Gpo pinclk(pinclkcfg);
//
//	Gpo::Config pinsicfg;
//	pinsicfg.pin = Pin::Name::kPte17;
//	Gpo pinsi(pinsicfg);
//
//	Gpo::Config pinadcfg;
//	pinadcfg.pin = Pin::Name::kPte16;
//	Gpo pinad(pinadcfg);

	while(true)
	{
		t_ = System::Time();
		if(t_-pt_>=1){
			m_car.m_ccd.SampleProcess();
			if(tc_%100==0) {
//				pinad.Turn();
//				pinsi.Turn();
//				pinad.Turn();
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
					/*for(int i=1; i<libsc::kl26::LinearCcd::kSensorW-1; i++){
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

					m_car.m_ccd.StartSample();

					St7735r::Rect rect_;
					uint16_t color = 0;

					for(int i=0; i<Tsl1401cl::kSensorW; i++){
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
//						m_car.m_lcd.FillColor(color);
					}
					y++;
					y=y%160;

				}
			}



			if(tc_%500==0){
				m_car.m_led.Switch();
				m_car.m_led2.Switch();
				m_car.m_led3.Switch();
				m_car.m_led4.Switch();
			}


			if(tc_%2==0){
				value[0] = m_r1->GetReal();
				value[1] = m_r2->GetReal();
				gyro_t = System::Time();
				m_car.m_mpu6050.Update();

				quaternion.Update();
//				upstand.KalmanFilter();
//				real_angle = (float) upstand.GetAngle();
				accel_ = m_car.m_mpu6050.GetAccel();
				gyro_ = m_car.m_mpu6050.GetOmega();
				/*static float angles[3] = {0};
				gyro_[0] = gyro_[0] / RAD2ANGLE;
				gyro_[1] = gyro_[1] / RAD2ANGLE;
				gyro_[2] = gyro_[2] / RAD2ANGLE;
				float f[3];
				f[0] = -angles[1];
				f[1] = angles[0];
				f[2] = -angles[2];
//				acc_angle = accel_[2] * RAD2ANGLE;
				float mag = sqrt(accel_[0] * accel_[0] + accel_[1] * accel_[1] + accel_[2] * accel_[2]);
				bool valid = mag < 1.2f && mag > 0.8f ? true : false;

//				gyro_angle += gyro_[0] / RAD2ANGLE * ((float)(gyro_t - gyro_pt)/1000.0f);

				double a[3] = {0};

				if(valid){
					float z[3];
					z[0] = sin(-angles[1]);
					z[1] = cos(angles[1])*sin(angles[0]);
					z[2] = cos(angles[1])*cos(angles[0]);
					float e[3];

					e[0]=z[1]*accel_[2]-z[2] * accel_[1];
					e[1]=z[2]*accel_[0]-z[0] * accel_[2];
					e[2]=z[0]*accel_[1]-z[1] * accel_[0];
//					kf_filter.Filtering(&temp, (double), (double) (real_angle - accel_[2]));
					kf_filterR.Filtering(&a[0], (double)e[0], (double) f[0]);
					kf_filterP.Filtering(&a[1], (double)e[1], (double) f[1]);
					kf_filterY.Filtering(&a[2], (double)e[2], (double) f[2]);
				}
//				else{
//					temp = 0;
//				}
//				real_angle += (gyro_[1] - 0.25 * a[0]) * ((float)(gyro_t - gyro_pt)/1000.0f);
				angles[0] += (gyro_[0] + 0.1 * a[0]) * ((float)(gyro_t - gyro_pt)/1000.0f);
				angles[1] += (gyro_[1] + 0.1 * a[1]) * ((float)(gyro_t - gyro_pt)/1000.0f);
				angles[2] += (gyro_[2] + 0.1 * a[2]) * ((float)(gyro_t - gyro_pt)/1000.0f);
				gyro_pt = gyro_t;
				real_angle = angles[1];*/

//				real_angle = (float)temp + 0.6f*(acc_angle - (float)temp);
				real_angle = (float)quaternion.getEuler(1);
				//				cou+=1;
				//				total_gyro=total_gyro+gyro_[0];
				//				avg_gyro=total_gyro/cou;
				//				u_b=Output_b(balcon, balpid, time, real_angle);
//				m_balance_pid_output = m_inc_pidcontroller.Calc(real_angle);
				balpid[0] = m_bkp->GetReal();
				balpid[1] = m_bki->GetReal();
				balpid[2] = m_bkd->GetReal();

				balcon[6] = m_boff->GetReal();

				m_balance_pid_output = Output_b(balcon, balpid, time, real_angle, gyro_angle);
//				if(abs(real_angle) < 0.25f){
//					power_l = power_r = 0;
//				}else{
					m_speed_control0.SetSetpoint(m_balance_pid_output);
					m_speed_control1.SetSetpoint(m_balance_pid_output);
//				}

			}

			if(tc_%20==0){
				//				printf("%.4f,%.4f,%.4f,%.4f,%.4f\n",acc_angle, gyro_angle, real_angle, gyro_[0],avg_gyro);
				//				printf("%.5f, %0.5f\n", acc_angle, real_angle);
//				printf("%.5f,%.5f,%.5f,%.5f\n", real_angle, acc_angle, gyro_angle, gyro_[0]);
//				printf("%d,%d\n",m_car.m_encoder_countr,m_car.m_encoder_countl);
//				offset_ = m_car.m_mpu6050.GetOffset();
//				printf("%.4f,%d,%.4f\n", real_angle, m_balance_pid_output, balcon[5]);
//				printf("%.4f,%.4f,%.4f\n",quaternion.getEuler(0)*57.2957795131, quaternion.getEuler(1)*57.2957795131, quaternion.getEuler(2)*57.2957795131);
//				printf("%f,%f,%f,%d,%d\n", m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal(), power_r, m_car.m_encoder_countr);
//				printf("%f,%f,%f,%f,%f,%f\n",accel_[0],accel_[1],accel_[2],gyro_[0],gyro_[1],gyro_[2]);
				printf("$,%f,%f,%f\n",real_angle*RAD2ANGLE,acc_angle*RAD2ANGLE,gyro_angle*RAD2ANGLE);
//				printf("%d\n", upstand.GetAngle());
//				printf("%f\n",balcon[5]);
//				printf("%f,%f,%d,%d\n", real_angle*RAD2ANGLE, balcon[5], m_car.m_encoder_countr, m_car.m_encoder_countl);
//				printf("%d,%d,%d\n", speedsp, m_car.m_encoder_countr, m_car.m_encoder_countl);
			}

			if(tc_%500==0){
				speedsp += 10;
				speedsp %= 800;
				power_r = speedsp;
				power_l = speedsp;
				/*speedsp += 30;
				speedsp %= 120;
				m_speed_control0.SetSetpoint(speedsp);
				m_speed_control1.SetSetpoint(speedsp);*/
			}
			if(tc_%2==0){
				m_car.m_encoder0.Update();
				m_car.m_encoder1.Update();
				m_car.m_encoder_countr = -m_car.m_encoder0.GetCount(); //right wheel
				m_car.m_encoder_countl = m_car.m_encoder1.GetCount(); //left wheel

//				balcon[5] = -(float)(m_car.m_encoder_count0 + m_car.m_encoder_count1)/2.0f/500.0f;

//				speed_power0 = Output_speed(carspeedcon, carspeedpid, (m_car.m_encoder_count0 + m_car.m_encoder_count1)/2);


				m_speed_control0.SetKp(m_skp->GetReal());
//				m_speed_control0.SetKd(m_skd->GetReal());
//				m_speed_control0.SetKi(m_ski->GetReal());
				m_speed_control1.SetKp(m_skp->GetReal());
//				m_speed_control1.SetKd(m_skd->GetReal());
//				m_speed_control1.SetKi(m_ski->GetReal());
				int16_t r_val = speedsp + m_speed_control0.Calc(m_car.m_encoder_countr);
				int16_t l_val = speedsp + m_speed_control1.Calc(m_car.m_encoder_countl);
//				power_r = sign(r_val) * RpmToPwm_R(abs(r_val));
//				power_l = sign(l_val) * RpmToPwm_L(abs(l_val));


			}

			if(tc_%20==0){

				carspeedpid[0] = m_carkp->GetReal();
				carspeedpid[1] = m_carki->GetReal();
				carspeedpid[2] = m_carkd->GetReal();
				carspeedcon[4] = (int16_t) m_carspeed->GetReal();
				balcon[5] = -Output_speed(carspeedcon, carspeedpid, (m_car.m_encoder_countr + m_car.m_encoder_countl)/2);
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


			m_car.m_motor_r.SetClockwise(power_r < 0); //Right Motor - false forward, true backward
			m_car.m_motor_l.SetClockwise(power_l > 0); //Left Motor - true forward, false backward
			m_car.m_motor_r.SetPower((uint16_t)abs(power_r));
			m_car.m_motor_l.SetPower((uint16_t)abs(power_l));


			pt_ = t_;
			tc_++;

		}
	}

}


