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

/*For KL26_2015_CCD (WITH LOADING) ONLY*/
// Clamp 140/140
uint16_t App::RpmToPwm_R(uint16_t count){
	if(count==0) return 0;
	uint16_t val = (uint16_t)(13.29f*count + 70.0f);
	val = val < 140 ? 140 : val;
	return val;
}

uint16_t App::RpmToPwm_L(uint16_t count){
	if(count==0) return 0;
	uint16_t val = (uint16_t)(12.17f*count + 70.0f);
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

	carspeedcon[0]=carspeedcon[4]-encoder;
	carspeedcon[3]+=carspeedcon[0];

	output=(float)(carspeedpid[0]*carspeedcon[0] + carspeedpid[1]* carspeedcon[3] + carspeedpid[2]*(carspeedcon[0] - carspeedcon[1]));
	carspeedcon[1] = carspeedcon[0];
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
	config.count = ClockUtils::GetBusTickPerUs() * 1000 * 5;
	config.isr = isr;
	return config;
}

void App::PitBalance(Pit* pit){
	if(m_pit_count%2==0){
//		pin->Set();
		m_car.m_mpu6050.Update();
		m_car.m_mma8451q.Update();
//		pin->Clear();
//		accel_ = m_car.m_mma8451q.GetAccel();
//		accel_ = m_car.m_mpu6050.GetAccel();
		gyro_ = m_car.m_mpu6050.GetOmega();


		upstand->KalmanFilter();
		real_angle = (float) upstand->GetAngle();

		balpid[0] = /*30.0f*//*200.0f*/m_bkp->GetReal();
		balpid[1] = m_bki->GetReal();
		balpid[2] = /*0.0f*//*0.1f*/m_bkd->GetReal();

		balcon[6] = 0.0f;

		m_balance_pid_output = -Output_b(balcon, balpid, time, real_angle, -gyro_[1]);


//		m_movavgr.Add(-m_car.m_encoder0.GetCount());
//		m_movavgl.Add(m_car.m_encoder1.GetCount());
//		m_car.m_encoder_countr = m_movavgr.GetAverage();
//		m_car.m_encoder_countl = m_movavgl.GetAverage();

		power_l = m_balance_pid_output/* + turn_powerl - 400*/;
		power_r = m_balance_pid_output/* + turn_powerr - 400*/;

		power_l = libutil::Clamp<int16_t>(-1000,power_l, 1000);
		power_r = libutil::Clamp<int16_t>(-1000,power_r, 1000);

//		carspeedconr[4] = power_r;
//		carspeedconl[4] = power_l;
		int16_t r_val = carspeedconr[4] + Output_speed(carspeedconr, carspeedpidr, m_car.m_encoder_countr);
		int16_t l_val = carspeedconl[4] + Output_speed(carspeedconl, carspeedpidl, m_car.m_encoder_countl);
//		power_r_pwm = sign(r_val) * RpmToPwm_R(abs(r_val));
//		power_l_pwm = sign(l_val) * RpmToPwm_L(abs(l_val));


	}
	if(m_pit_count%4==0){
		m_car.m_encoder0.Update();
		m_car.m_encoder1.Update();
		m_car.m_encoder_countr = -m_car.m_encoder0.GetCount();
		m_car.m_encoder_countr_t += m_car.m_encoder_countr;
		m_car.m_encoder_countl = m_car.m_encoder1.GetCount();
		m_car.m_encoder_countl_t += m_car.m_encoder_countl;
	}
	if(m_pit_count%2==1){
		pin->Set();
		m_car.m_ccd.StartSample();
		while(!m_car.m_ccd.SampleProcess()){}

//		if(m_car.m_ccd.IsImageReady()){
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
			turn_powerl = -4*error;
			turn_powerr = 4*error;
			if(m_car.m_lcdupdate){
				St7735r::Rect rect_;
				uint16_t color = 0;

				rect_.x = 0;
				rect_.y = 0;
				rect_.w = 24;
				rect_.h = 16;
				m_car.m_lcd.SetRegion(rect_);
				m_lcd_typewriter.WriteString(String::Format("%3d\n",right_edge - left_edge).c_str());

				for(int i=0; i<Tsl1401cl::kSensorW; i++){
					rect_.x = i;
					rect_.y = y+16*3;
					rect_.w = 1;
					rect_.h = 1;
					m_car.m_lcd.SetRegion(rect_);
					if(ccd_data_[i] >= avg){
							color = ~0;
					}else{
						color = 0;
					}
					if(avg < 63){
						color = 0;
					}else if(avg > 191){
						color = ~0;
					}
					if(i==mid){
						color = 0xf800;
					}
						m_car.m_lcd.FillColor(color);
				}
				y++;
				y=y%(160-16*3);
			}
//		}
		pin->Clear();
	}
	if(m_pit_count%200==0){
//		power_r_pwm += 10;
//		power_l_pwm += 10;
//		power_r_pwm = power_r_pwm % 600;
//		power_l_pwm = power_l_pwm % 600;
		power_r_pwm = 500;
		power_l_pwm = 500;
//		carspeedconr[4] = 0;
//		carspeedconl[4] = 0;
	}
	if(m_pit_count%5==0){
		if(m_car.m_lcdupdate){
			St7735r::Rect rect_;

			rect_.x = 0;
			rect_.y = 16;
			rect_.w = 128;
			rect_.h = 16;
			m_car.m_lcd.SetRegion(rect_);

			m_lcd_typewriter.WriteString(String::Format("%d %d %d %d %d\n",
					m_car.m_joy.GetState() == Joystick::State::kUp,
					m_car.m_joy.GetState() == Joystick::State::kDown,
					m_car.m_joy.GetState() == Joystick::State::kLeft,
					m_car.m_joy.GetState() == Joystick::State::kRight,
					m_car.m_joy.GetState() == Joystick::State::kSelect
					).c_str());

			rect_.x = 0;
			rect_.y = 31;
			rect_.w = 128;
			rect_.h = 16;
			m_car.m_lcd.SetRegion(rect_);

			m_lcd_typewriter.WriteString(String::Format("%d %d\n",
					m_car.m_encoder_countr_t,
					m_car.m_encoder_countl_t
					).c_str());
		}
		switch(m_car.m_print_state){

			case 1:
				printf("%f,%f,%f\n",real_angle,upstand->GetAccAngle(),upstand->GetGyroAngle());
				break;
			case 2:
				printf("%d,%d,%d,%d\n",power_r_pwm,power_l_pwm,m_car.m_encoder_countr, m_car.m_encoder_countl);
				break;
			case 0:
				default:
					break;
		}
//		printf("%d,%d,%d,%d\n",power_l,m_balance_pid_output, m_car.m_encoder_countr, m_car.m_encoder_countl);
//		printf("%d,%d,%d,%d\n",carspeedconr[4],carspeedconl[4],m_car.m_encoder_countr, m_car.m_encoder_countl);
	}

	m_pit_count++;
}

void App::PitMoveMotor(Pit* pit){

//	m_car.m_encoder_spdcountr += m_car.m_encoder_countr;
//	m_car.m_encoder_spdcountl += m_car.m_encoder_countl;

	/*
	 * Protection for motors
	 * */

//	if(abs(power_l) >= 1000) power_l = 0;
//	if(abs(power_r) >= 1000) power_r = 0;

//	carspeedconr[4] = 0;
//	carspeedconl[4] = 0;
	if(!m_car.m_car_move_motor){
		power_r_pwm = power_l_pwm = 0;
	}
	m_car.m_motor_r.SetClockwise(power_r_pwm < 0); //Right Motor - false forward, true backward
	m_car.m_motor_l.SetClockwise(power_l_pwm > 0); //Left Motor - true forward, false backward
	m_car.m_motor_r.SetPower((uint16_t)abs(power_r_pwm));
	m_car.m_motor_l.SetPower((uint16_t)abs(power_l_pwm));
	m_pit_count2++;
}

App::App():
	m_car(),
	m_lcd_typewriter(GetLcdTypewriterConfig()),
	m_balance_pid_output(0),
	m_speed_control0(0,m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal()),
	m_speed_control1(0,m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal()),
	m_movavgr(3),
	m_movavgl(3)
{
	St7735r::Rect rect_;
	rect_.x = 88;
	rect_.y = 0;
	rect_.w = 40;
	rect_.h = 16;
	m_car.m_lcd.SetRegion(rect_);
	m_lcd_typewriter.WriteString(String::Format("%.2fV\n",m_car.m_bat.GetVoltage()).c_str());
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

	upstand = new Upstand(&(m_car.m_mpu6050), &(m_car.m_mma8451q));

	Timer::TimerInt t_ = System::Time(), pt_ = t_;
	Timer::TimerInt gyro_t = 0, gyro_pt = System::Time();

	Pit m_pit(GetPitConfig(0, std::bind(&App::PitBalance, this, std::placeholders::_1)));
	Pit m_pit2(GetPitConfig2(1, std::bind(&App::PitMoveMotor, this, std::placeholders::_1)));


	while(true)
	{
	}

}


