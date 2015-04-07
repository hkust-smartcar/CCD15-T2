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
#include <libbase/kl26/pit.h>
#include <libsc/pit_timer.h>
#include <libbase/kl26/clock_utils.h>
#include "app.h"
#include "kalman.h"
#include "Quaternion.h"
#include "upstand.h"


using namespace libsc::kl26;
using namespace libbase::kl26;
using namespace libsc;

/*
 * For K60_2014_CAMERA ONLY
 * uint16_t App::RpmToPwm1(uint16_t count){
	return (count == 0) ? 0 : (uint16_t)(0.1484715791f*count + 59.6501510982f);
}

uint16_t App::RpmToPwm0(uint16_t count){
	return (count == 0) ? 0 : (uint16_t)(0.1462728045f*count + 34.5814814763f);
}*/

/*For KL26_2015_CCD (WITH LOADING) ONLY*/
// Clamp 140/140
uint16_t App::RpmToPwm_R(uint16_t count){
	if(count==0) return 0;
	uint16_t val = (uint16_t)(13.9698f*count + 129.3232f);
	val = val < 140 ? 140 : val;
	return val;
}

uint16_t App::RpmToPwm_L(uint16_t count){
	if(count==0) return 0;
	uint16_t val = (uint16_t)(14.3431f*count + 127.1783f);
	val = val < 140 ? 140 : val;
	return val;
}
/*For KL26_2015_CCD (WITH NO LOADING) ONLY*/
// Clamp 50/100
//uint16_t App::RpmToPwm_R(uint16_t count){
//	return (count == 0) ? 0 : (uint16_t)(6.4718*count + 15.1519f);
//}
//
//uint16_t App::RpmToPwm_L(uint16_t count){
//	return (count == 0) ? 0 : (uint16_t)(5.9219f*count + 58.4286f);
//}


int16_t App::Output_b(float* balcon, float* balpid, uint16_t* time, float real_angle, float omega){
	uint32_t period;
	int16_t output;
	static int16_t total_output = 0;
	static uint32_t prev_time = 0;
	float temp;

	period=System::Time()-prev_time;
	time[2]=System::Time();
	balcon[0]=(balcon[4]+balcon[5]+balcon[6])-real_angle;	// 512*104/44=13312/11
	temp=(balcon[0]-balcon[1])/period;
//	output=balpid[0]*balcon[0]+balpid[1]*balcon[3]*time[2]+balpid[2]*(temp+balcon[2])*10;
//	output=(int16_t)(balpid[0]*(balcon[0] - balcon[1]));
	total_output += balcon[0] * period;
	/*(temp+balcon[2])/2)*/
	output=(int16_t)(balpid[0]*balcon[0] + balpid[2] * omega/* + balpid[1] * total_output*/);
	balcon[1]=balcon[0];
//	balcon[2]=temp;
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

Pit::Config GetPitConfig(const uint8_t pit_channel,
		const Pit::OnPitTriggerListener &isr)
{
	Pit::Config config;
	config.channel = pit_channel;
	config.count = ClockUtils::GetBusTickPerMs() * 5;
	config.isr = isr;
	return config;
}

Pit::Config GetPitConfig2(const uint8_t pit_channel,
		const Pit::OnPitTriggerListener &isr)
{
	Pit::Config config;
	config.channel = pit_channel;
	config.count = ClockUtils::GetBusTickPerMs();
	config.isr = isr;
	return config;
}

void App::PitBalance(Pit* pit){
	if(m_pit_count%2==0){
//		pin->Set();
		m_car.m_mpu6050.Update();
		accel_ = m_car.m_mpu6050.GetAccel();
		gyro_ = m_car.m_mpu6050.GetOmega();
	//	m_car.m_mma8451q.Update();

		upstand->KalmanFilter();
		real_angle = (float) upstand->GetAngle();

		balpid[0] = 230.0f/*m_bkp->GetReal()*/;
		balpid[1] = m_bki->GetReal();
		balpid[2] = 1.8f/*m_bkd->GetReal()*/;

		balcon[6] = 1.0f;

		m_balance_pid_output = Output_b(balcon, balpid, time, real_angle, gyro_[1]);

		power_r = power_l = m_balance_pid_output;

//		pin->Clear();
	}
	if(m_pit_count%2==1){

		if(m_car.m_ccd.IsImageReady()){
			pin->Set();
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
			for(int i=1; i<libsc::Tsl1401cl::kSensorW-1; i++){
				if(color[i]==CCD_BLACK && color[i+1]==CCD_WHITE){
					left_edge = i;
				}
				if(color[i]==CCD_WHITE && color[i+1]==CCD_BLACK){
					right_edge = i;
				}
			}
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
							m_car.m_lcd.FillColor(color);
			}
			y++;
			y=y%160;
			pin->Clear();
		}

	}
	if(m_pit_count%4==0){
		printf("%f,%f,%f,%f,%f,%f,%d,%d\n", real_angle, gyro_[1], upstand->GetAccAngle(), m_bkp->GetReal(), m_bki->GetReal(), m_bkd->GetReal(), power_l, power_r);
	}

	m_pit_count++;
}

void App::PitMoveMotor(Pit* pit){
	m_car.m_ccd.SampleProcess();

	power_l = libutil::Clamp<int16_t>(-1000,power_l, 1000);
	power_r = libutil::Clamp<int16_t>(-1000,power_r, 1000);


	/*
	 * Protection for motors
	 * */

//	if(abs(power_l) >= 950) power_l = 0;
//	if(abs(power_r) >= 950) power_r = 0;


	m_car.m_motor_r.SetClockwise(power_r < 0); //Right Motor - false forward, true backward
	m_car.m_motor_l.SetClockwise(power_l > 0); //Left Motor - true forward, false backward
	m_car.m_motor_r.SetPower((uint16_t)abs(power_r));
	m_car.m_motor_l.SetPower((uint16_t)abs(power_l));

}

App::App():
	m_car(),
	m_lcd_typewriter(GetLcdTypewriterConfig()),
	m_balance_pid_output(0),
	m_speed_control0(0,m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal()),
	m_speed_control1(0,m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal())
{
	m_lcd_typewriter.WriteString(String::Format("%.3fV\n",m_car.m_bat.GetVoltage()).c_str());
	m_car.m_varmanager->Broadcast(m_car.m_com);

	/*
	 * Pin for debugging
	 *
	 */
	Gpo::Config pinadcfg;
	pinadcfg.pin = Pin::Name::kPte19;
	pin = new Gpo(pinadcfg);

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

	m_car.m_mpu6050.Update();
//	acc_angle = accel_[2]/* * RAD2ANGLE*/;
//	gyro_angle = 0;//acc_angle;
//	real_angle = acc_angle;
	upstand = new Upstand(&(m_car.m_mpu6050));




	Timer::TimerInt t_ = System::Time(), pt_ = t_;


	Timer::TimerInt gyro_t = 0, gyro_pt = System::Time();;

	m_car.m_ccd.StartSample();

	int16_t speedsp = 0;
	double temp = 0.0;



	Pit m_pit(GetPitConfig(0, std::bind(&App::PitBalance, this, std::placeholders::_1)));
	Pit m_pit2(GetPitConfig2(1, std::bind(&App::PitMoveMotor, this, std::placeholders::_1)));


	while(true)
	{
		if(t_!=System::Time()){
			t_ = System::Time();
//			m_car.m_ccd.SampleProcess();
			/*if(t_%100==0) {
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
					for(int i=1; i<libsc::kl26::LinearCcd::kSensorW-1; i++){
						if(color[i]==CCD_BLACK && color[i+1]==CCD_WHITE){
							left_edge = i;
						}
						if(color[i]==CCD_WHITE && color[i+1]==CCD_BLACK){
							right_edge = i;
						}
					}
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
			}*/



/*			if(t_%500==0){
				m_car.m_led.Switch();
				m_car.m_led2.Switch();
				m_car.m_led3.Switch();
				m_car.m_led4.Switch();
			}*/


			if(t_%4==0){
//				pin->Set();
				/*gyro_t = System::Time();
//				m_car.m_mpu6050.Update();
//				m_car.m_mma8451q.Update();

				upstand->KalmanFilter();
				real_angle = (float) upstand->GetAngle();
//				accel_ = m_car.m_mma8451q.GetAccel();
				gyro_ = m_car.m_mpu6050.GetOmega();
//				acc_angle = accel_[2] * RAD2ANGLE;

//				gyro_angle += gyro_[0] / RAD2ANGLE * ((float)(gyro_t - gyro_pt)/1000.0f);
				balpid[0] = m_bkp->GetReal();
				balpid[1] = m_bki->GetReal();
				balpid[2] = m_bkd->GetReal();

				balcon[6] = m_boff->GetReal();

				m_balance_pid_output = Output_b(balcon, balpid, time, real_angle, gyro_[1]);
//				if(abs(real_angle) < 0.25f){
//					power_l = power_r = 0;
//				}else{
//					m_speed_control0.SetSetpoint(m_balance_pid_output);
//					m_speed_control1.SetSetpoint(m_balance_pid_output);
				power_r = power_l = m_balance_pid_output;
//				}*/
//				pin->Clear();
			}



//			if(t_%500==0){
//				/*speedsp += 10;
//				speedsp %= 800;
//				power_r = speedsp;
//				power_l = speedsp;*/
//				speedsp += 30;
//				speedsp %= 120;
//				m_speed_control0.SetSetpoint(speedsp);
//				m_speed_control1.SetSetpoint(speedsp);
//			}
			/*if(t_%4==0){
				m_car.m_encoder0.Update();
				m_car.m_encoder1.Update();
				m_car.m_encoder_countr = -m_car.m_encoder0.GetCount(); //right wheel
				m_car.m_encoder_countl = m_car.m_encoder1.GetCount(); //left wheel
				m_car.m_encoder_spdcountr += m_car.m_encoder_countr;
				m_car.m_encoder_spdcountl += m_car.m_encoder_countl;

//				balcon[5] = -(float)(m_car.m_encoder_count0 + m_car.m_encoder_count1)/2.0f/500.0f;

//				speed_power0 = Output_speed(carspeedcon, carspeedpid, (m_car.m_encoder_count0 + m_car.m_encoder_count1)/2);


				m_speed_control0.SetKp(0.18f);
//				m_speed_control0.SetKd(m_skd->GetReal());
//				m_speed_control0.SetKi(m_ski->GetReal());
				m_speed_control1.SetKp(0.2f);
//				m_speed_control1.SetKd(m_skd->GetReal());
//				m_speed_control1.SetKi(m_ski->GetReal());
				int16_t r_val = speedsp + m_speed_control0.Calc(m_car.m_encoder_countr);
				int16_t l_val = speedsp + m_speed_control1.Calc(m_car.m_encoder_countl);
//				power_r = sign(r_val) * RpmToPwm_R(abs(r_val));
//				power_l = sign(l_val) * RpmToPwm_L(abs(l_val));


			}*/

			/*if(t_%20==0){
//				printf("%.4f,%.4f,%.4f,%.4f,%.4f\n",acc_angle, gyro_angle, real_angle, gyro_[0],avg_gyro);
//				printf("%.5f, %0.5f\n", acc_angle, real_angle);
//				printf("%.5f,%.5f,%.5f,%.5f\n", real_angle, acc_angle, gyro_angle, gyro_[0]);
//				printf("%d,%d\n",m_car.m_encoder_countr,m_car.m_encoder_countl);
//				offset_ = m_car.m_mpu6050.GetOffset();
//				printf("%.4f,%d,%.4f\n", real_angle, m_balance_pid_output, balcon[5]);
//				printf("%.4f,%.4f,%.4f\n",quaternion.getEuler(0)*57.2957795131, quaternion.getEuler(1)*57.2957795131, quaternion.getEuler(2)*57.2957795131);
//				printf("%f,%f,%f,%d,%d\n", m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal(), power_r, m_car.m_encoder_countr);
//				printf("%f,%f,%f,%f,%f,%f\n",accel_[0],accel_[1],accel_[2],gyro_[0],gyro_[1],gyro_[2]);
//				printf("$,%f,%f,%f\n",real_angle*RAD2ANGLE,acc_angle*RAD2ANGLE,gyro_angle*RAD2ANGLE);
//				printf("%d\n", upstand.GetAngle());
//				printf("%f\n",balcon[5]);
//				printf("%f,%f,%f,%d,%d\n", real_angle, gyro_[1], balcon[5], m_car.m_encoder_countr, m_car.m_encoder_countl);
//				printf("%f,%f,%f\n",accel_[0],accel_[1],accel_[2]);
//				printf("%d,%d,%d,%d,%d\n", speedsp, m_car.m_encoder_countr, m_car.m_encoder_countl, m_car.m_encoder_spdcountr, m_car.m_encoder_spdcountl);
			}*/

			/*if(t_%20==0){

				carspeedpid[0] = m_carkp->GetReal();
				carspeedpid[1] = m_carki->GetReal();
				carspeedpid[2] = m_carkd->GetReal();
				carspeedcon[4] = (int16_t) m_carspeed->GetReal();
				balcon[5] = -Output_speed(carspeedcon, carspeedpid, (m_car.m_encoder_spdcountr + m_car.m_encoder_spdcountl)/2);
				m_car.m_encoder_spdcountr = m_car.m_encoder_spdcountl = 0;
			}*/

//			power0 = m_balance_pid_output;
//			power1 = m_balance_pid_output;

//			power_l = libutil::Clamp<int16_t>(-1000,power_l, 1000);
//			power_r = libutil::Clamp<int16_t>(-1000,power_r, 1000);
//
//
//			/*
//			 * Protection for motors
//			 * */
//
//			if(abs(power_l) >= 950) power_l = 0;
//			if(abs(power_r) >= 950) power_r = 0;
//
//
//			m_car.m_motor_r.SetClockwise(power_r < 0); //Right Motor - false forward, true backward
//			m_car.m_motor_l.SetClockwise(power_l > 0); //Left Motor - true forward, false backward
//			m_car.m_motor_r.SetPower((uint16_t)abs(power_r));
//			m_car.m_motor_l.SetPower((uint16_t)abs(power_l));


			pt_ = t_;

		}
	}

}


