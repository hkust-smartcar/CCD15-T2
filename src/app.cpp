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

/*For KL26_2015_CCD*/
uint16_t App::RpmToPwm_R(uint16_t count){
	if(count==0) return 0;
	uint16_t val = (uint16_t)(0.3459f*count + 33.589f);
	//Flat section before straight line
	val = val <= 108 ? 70 : val;
	return val;
}

uint16_t App::RpmToPwm_L(uint16_t count){
	if(count==0) return 0;
	uint16_t val = (uint16_t)(0.3168f*count + 82.325f);
	//Flat section before straight line
	val = val <= 216 ? 150 : val;
	return val;
}

float App::Output_turning(int16_t* turncon, float* turnpid, uint16_t* time){
    uint16_t period;
    int16_t outputcoeff;
    if(time[4] == 0){
        time[4] = System::Time();
        return 0;
    }
    float temp;

    period = System::Time() - time[4];
    temp = (turncon[0] - turncon[1]) / period;

    outputcoeff = turnpid[0] * turncon[0] + turnpid[2] * temp;

    turncon[1] = turncon[0];
    turncon[2] = temp;
    return outputcoeff;
}

int16_t App::Output_b(float* balcon, float* balpid, uint16_t* time, float real_angle, float omega){
	uint32_t period;
	int16_t output;
	static int16_t total_output = 0;
	static uint32_t prev_time = 0;

	period=System::Time()-prev_time;
	time[2]=System::Time();
	balcon[0]=(balcon[4]+balcon[5]+balcon[6])-real_angle;	// 512*104/44=13312/11
	total_output += balcon[0] * period;
	output=(int16_t)(balpid[0]*balcon[0] + balpid[2] * omega/* + balpid[1] * total_output*/);
	balcon[1]=balcon[0];
	return output;
}

int16_t App::Output_speed(int16_t* carspeedcon, float* carspeedpid, int16_t encoder){
	int16_t output;

	carspeedcon[0]=carspeedcon[4]-encoder/40;
	carspeedcon[3]+=carspeedcon[0];

	output=(int16_t) (carspeedpid[0]*carspeedcon[0] + carspeedpid[1]* carspeedcon[3] + carspeedpid[2]*(carspeedcon[0] - carspeedcon[1]));
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

void App::PitBalance(Pit*){
	if(m_pit_count%2==0){
		m_car.m_mpu6050.Update();
		m_car.m_mma8451q.Update();
		m_gyro_ = m_car.m_mpu6050.GetOmega();

		m_upstand->KalmanFilter();
		m_real_angle = (float) m_upstand->GetAngle();

		m_balpid[0] = 200.0f/*m_bkp->GetReal()*/;
		m_balpid[1] = m_bki->GetReal();
		m_balpid[2] = 0.0f/*m_bkd->GetReal()*/;


		m_balance_pid_output = -Output_b(m_balcon, m_balpid, m_time, m_real_angle, -m_gyro_[1]);
		m_balance_pid_output = 0;
		m_power_r = m_balance_pid_output + m_turn_powerr;
		m_power_l = m_balance_pid_output + m_turn_powerl;

	}
	if(m_pit_count%2==0){
		m_car.m_ccd_2.StartSample();
		while(!m_car.m_ccd_2.SampleProcess()){}
		m_ccd_data_2 = m_car.m_ccd_2.GetData();
		if(m_car.m_lcdupdate){
			St7735r::Rect rect_;
			uint16_t color2 = 0;

			for(int i=0; i<Tsl1401cl::kSensorW; i++){
				rect_.x = i;
				rect_.y = m_last_y2[i];
				rect_.w = 1;
				rect_.h = 1;
				m_car.m_lcd.SetRegion(rect_);
				color2 = 0;
				m_car.m_lcd.FillColor(color2);
				m_last_y2[i] = 130-m_ccd_data_2[i]/4;
				rect_.x = i;
				rect_.y = m_last_y2[i];
				rect_.w = 1;
				rect_.h = 1;
				m_car.m_lcd.SetRegion(rect_);
				color2 = ~0;
				m_car.m_lcd.FillColor(color2);
			}
		}
	}

	if(m_pit_count%2==1){
		m_car.m_ccd_1.StartSample();
		while(!m_car.m_ccd_1.SampleProcess()){}

			m_ccd_data_ = m_car.m_ccd_1.GetData();
			m_avg = 0;
			m_sum = 0;
			for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
				m_sum += (uint32_t)m_ccd_data_[i];
			}
			m_avg = (uint16_t) (m_sum / libsc::Tsl1401cl::kSensorW);
			for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
				if(m_ccd_data_[i] >= m_avg-5){
						m_color[i] = CCD_WHITE;
				}else{
					m_color[i] = CCD_BLACK;
				}
			}

			int cameramid = (0 + 127)/2;
			m_right_edge = libsc::Tsl1401cl::kSensorW-1;
			m_left_edge = 0;
			for (int i=m_mid; i<libsc::Tsl1401cl::kSensorW-1; i++){
				if(m_color[i]==CCD_WHITE && m_color[i+1]==CCD_BLACK){
					m_right_edge=i; break;
				}
			}
			for (int i=m_mid; i>=0; i--){
				if(m_color[i]==CCD_BLACK && m_color[i+1]==CCD_WHITE){
					m_left_edge=i; break;
				}
			}
			int route_mid = (m_left_edge + m_right_edge)/2;
	//
	//			mid = (left_edge + right_edge) + 4;
	//			turncon_b[0] = mid-(Tsl1401cl::kSensorW-1);
	//			turn_coeff_b = Output_turning(turncon_f, turnpid_f, time);
	////			encoder_count_t = encoder_countr + encoder_countl;
	//			turn_powerb = turn_coeff_b * encoder_count_t / 500;
	//			turn_powerb = libutil::Clamp<int16_t>(-200,turn_powerb, 200);
	//			turn_powerl = -1 * turn_powerb;
	//			turn_powerr = turn_powerb;

	/* for the second ccd*/
	//			turncon_f[0] = mid-(Tsl1401cl::kSensorW-1);
	//			turn_coeff_f = Output_turning(turncon_f, turnpid_f, time2);
	//			encoder_count_t = encoder_countr + encoder_countl;
	//				turn_powerf = turn_coeff_f * encoder_count_t / 1000;
	//				turn_powerf = libutil::Clamp<int16_t>(-100,turn_powerf, 100);
	//			if (either ccd is all white)
	//				crossing = 1;
	//			else
	//				crossing = 0;
	//			if (turn_powerb>50||crossing){	//help turning
	//				turn_powerl -= turn_powerf;
	//				turn_powerr += turn_powerf;
	//			}
	//			else{	//prepare turning
	//				turn_powerl += turn_powerf;
	//				turn_powerr -= turn_powerf;
	//			}

			int error = cameramid - route_mid;
			m_movavgturn.Add(error);

			m_turn_powerl = 10*m_movavgturn.GetAverage();
			m_turn_powerl = libutil::Clamp<int16_t>(-800,m_turn_powerl, 800);
			m_turn_powerr = -10*m_movavgturn.GetAverage();
			m_turn_powerr = libutil::Clamp<int16_t>(-800,m_turn_powerr, 800);

			if(m_car.m_lcdupdate){
				m_pin->Set();
				St7735r::Rect rect_;
				uint16_t color = 0;

				for(int i=0; i<Tsl1401cl::kSensorW; i++){
					rect_.x = i;
					rect_.y = m_last_y[i];
					rect_.w = 1;
					rect_.h = 1;
					m_car.m_lcd.SetRegion(rect_);
					color = 0;
					m_car.m_lcd.FillColor(color);
					m_last_y[i] = 65-m_ccd_data_[i]/4;
					rect_.x = i;
					rect_.y = m_last_y[i];
					rect_.w = 1;
					rect_.h = 1;
					m_car.m_lcd.SetRegion(rect_);
					color = ~0;
					m_car.m_lcd.FillColor(color);
				}
				m_pin->Clear();
			}

	//			if(m_car.m_lcdupdate){
	//				St7735r::Rect rect_;
	//				uint16_t color = 0;

	//				rect_.x = 0;
	//				rect_.y = 0;
	//				rect_.w = 24;
	//				rect_.h = 16;
	//				m_car.m_lcd.SetRegion(rect_);
	//				m_lcd_typewriter.WriteString(String::Format("%3d\n",right_edge - left_edge).c_str());

	//				for(int i=0; i<Tsl1401cl::kSensorW; i++){
	//					rect_.x = i;
	//					rect_.y = y+16*3;
	//					rect_.w = 1;
	//					rect_.h = 1;
	//					m_car.m_lcd.SetRegion(rect_);
	//					if(ccd_data_[i] >= avg){
	//							color = ~0;
	//					}else{
	//						color = 0;
	//					}
	//					if(avg < 63){
	//						color = 0;
	//					}else if(avg > 191){
	//						color = ~0;
	//					}
	//					if(i==mid){
	//						color = 0xf800;
	//					}
	//						m_car.m_lcd.FillColor(color);
	//				}
	//				y++;
	//				y=y%(160-16*3);
	//			}
	//		}

	}

	if(m_pit_count%8==0){
		m_car.m_encoder0.Update();
		m_car.m_encoder1.Update();
		m_car.m_encoder_countr = -m_car.m_encoder0.GetCount();
		m_car.m_encoder_countr_t += m_car.m_encoder_countr;
		m_car.m_encoder_countl = m_car.m_encoder1.GetCount();
		m_car.m_encoder_countl_t += m_car.m_encoder_countl;

		if(m_car.m_car_move_forward){
			m_car.m_car_speed = 200.0f;
		}else{
			m_car.m_car_speed = 0.0f;
		}

		m_balcon[6] = (float)(m_car.m_car_speed-(m_car.m_encoder_countr + m_car.m_encoder_countl)/2.0f)*0.02f;

//		power_r_pwm += Output_speed(carspeedconr, carspeedpidr, m_car.m_encoder_countr);
//		power_l_pwm += Output_speed(carspeedconl, carspeedpidl, m_car.m_encoder_countl);
	}


	if(m_pit_count%200==0){
//		carspeedconr[4] += 10;
//		carspeedconl[4] += 10;
//		carspeedconr[4] = carspeedconr[4] % 40;
//		carspeedconl[4] = carspeedconl[4] % 40;
//		power_l+= 10;
//		power_r+=10;
//		power_r_pwm = 500;
//		power_l_pwm = 500;
//		carspeedconr[4] = 0;
//		carspeedconl[4] = 0;
	}
	if(m_pit_count%4==1){
		if(m_car.m_lcdupdate){
//			St7735r::Rect rect_;
//
//			rect_.x = 0;
//			rect_.y = 16;
//			rect_.w = 128;
//			rect_.h = 16;
//			m_car.m_lcd.SetRegion(rect_);
//
//			m_lcd_typewriter.WriteString(String::Format("%d %d %d %d %d\n",
//					m_car.m_joy.GetState() == Joystick::State::kUp,
//					m_car.m_joy.GetState() == Joystick::State::kDown,
//					m_car.m_joy.GetState() == Joystick::State::kLeft,
//					m_car.m_joy.GetState() == Joystick::State::kRight,
//					m_car.m_joy.GetState() == Joystick::State::kSelect
//					).c_str());
//
//			rect_.x = 0;
//			rect_.y = 31;
//			rect_.w = 128;
//			rect_.h = 16;
//			m_car.m_lcd.SetRegion(rect_);

//			m_lcd_typewriter.WriteString(String::Format("%d %d\n",
//					m_car.m_encoder_countr_t,
//					m_car.m_encoder_countl_t
//					).c_str());

//			St7735r::Rect rect_;
//
//			rect_.x = 0;
//			rect_.y = 16;
//			rect_.w = 128;
//			rect_.h = 16;
//			m_car.m_lcd.SetRegion(rect_);
//
//			m_lcd_typewriter.WriteString(String::Format("%4d %4d\n",
//					turn_powerl,
//					turn_powerr
//					).c_str());

		}
		switch(m_car.m_print_state){

			case 1:
				printf("%f,%f,%f\n",m_real_angle,m_upstand->GetAccAngle(),m_upstand->GetGyroAngle());
				break;
			case 2:
				printf("%d,%d,%d,%d,%d,%d\n",m_power_r,m_power_l,m_car.m_encoder_countr, m_car.m_encoder_countl, m_car.m_encoder_countr_t, m_car.m_encoder_countl_t);
				break;
			case 0:
				default:
					break;
		}
	}

	m_pit_count++;
}

void App::PitMoveMotor(Pit*){
	/*
	 * Protection for motors
	 * */

//	if(abs(power_l) >= 1000) power_l = 0;
//	if(abs(power_r) >= 1000) power_r = 0;

	m_power_r_pwm = sign(m_power_l)*(int16_t)RpmToPwm_R((uint16_t)abs(m_power_r));
	m_power_l_pwm = sign(m_power_l)*(int16_t)RpmToPwm_L((uint16_t)abs(m_power_l));

	if(!m_car.m_car_move_motor){
		m_power_r_pwm = m_power_l_pwm = 0;
		m_power_r = m_power_l = 0;
	}
	m_car.m_motor_r.SetClockwise(m_power_r_pwm < 0); //Right Motor - false forward, true backward
	m_car.m_motor_l.SetClockwise(m_power_l_pwm > 0); //Left Motor - true forward, false backward
	m_car.m_motor_r.SetPower((uint16_t)abs(m_power_r_pwm));
	m_car.m_motor_l.SetPower((uint16_t)abs(m_power_l_pwm));
	m_pit_count2++;
}

App::App():
	m_car(),
	m_lcd_typewriter(GetLcdTypewriterConfig()),
	m_balance_pid_output(0),
	m_speed_control0(0,m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal()),
	m_speed_control1(0,m_skp->GetReal(),m_skd->GetReal(),m_ski->GetReal()),
	m_movavgr(3),
	m_movavgl(3),
	m_movavgturn(10)
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
	pinadcfg.pin = Pin::Name::kPtc0;
	m_pin = new Gpo(pinadcfg);

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

	m_upstand = new Upstand(&(m_car.m_mpu6050), &(m_car.m_mma8451q));

	Pit m_pit(GetPitConfig(0, std::bind(&App::PitBalance, this, std::placeholders::_1)));
	Pit m_pit2(GetPitConfig2(1, std::bind(&App::PitMoveMotor, this, std::placeholders::_1)));

	while(true)
	{
	}

}


