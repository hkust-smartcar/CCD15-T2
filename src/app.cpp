/*
 * app.cpp
 *
 *  Created on: Mar 3, 2015
 *      Author: harrison
 */
#include <car.h>
#include <cmath>
#include <climits>
#include <libsc/system.h>
#include <libsc/sys_tick_delay.h>
#include <libutil/misc.h>
#include <libbase/kl26/gpio.h>
#include <libbase/kl26/pit.h>
#include <libsc/pit_timer.h>
#include <libbase/kl26/clock_utils.h>
#include "app.h"
#include "kalman.h"
//#include "Quaternion.h"
#include "upstand.h"
#include "medianFilter.h"
#include "Eigen/Eigen"
using Eigen::Vector3f;


using namespace libsc::kl26;
using namespace libbase::kl26;
using namespace libsc;

/*For KL26_2015_CCD*/
uint16_t App::RpmToPwm_R(uint16_t count){
//	if(count==0) return 0;
	uint16_t val = (uint16_t)(0.3459f*count + 33.589f);
	//Flat section before straight line
//	val = count <= 108 ? 70 : val;
//	val = val <= 70 ? 70 : val;
	return val;
}

uint16_t App::RpmToPwm_L(uint16_t count){
//	if(count==0) return 0;
	uint16_t val = (uint16_t)(0.3168f*count + 82.325f);
	//Flat section before straight line
//	val = count <= 216 ? 150 : val;
//	val = val <= 150 ? 150 : val;
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
	balcon[0]=(balcon[4]+balcon[5]+balcon[6]+m_car.m_shift_balance_angle)-real_angle;	// 512*104/44=13312/11
	total_output += balcon[0] * period;

	float error = omega;
	static float prev_error = 0.0f;
	prev_error = balpid[2] * 0.4f * error + 0.6f * prev_error;

//	output=(int16_t)(balpid[0]*balcon[0] + balpid[2] * omega/* + balpid[1] * total_output*/);
	output=(int16_t)(balpid[0]*balcon[0] + prev_error);

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
void App::Update_edge(uint16_t* m_ccd_data, uint16_t* edge_data){
	edge_data[0] = 10;
	edge_data[1] = libsc::Tsl1401cl::kSensorW-11;
	int16_t max=-1, min=INT16_MAX;
	for (int i=10; i<libsc::Tsl1401cl::kSensorW-11; i++){
		if(((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i-1]) > max){
			edge_data[0] = i;
			max = ((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i-1]);
		}
		if(((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i-1]) < min){
			edge_data[1] = i;
			min = ((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i-1]);
		}
	}
//	If left edge is on right of right edge, then ignore
	if(edge_data[0]>edge_data[1]){
		edge_data[0] = 10;
		edge_data[1] = libsc::Tsl1401cl::kSensorW-11;
	}
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
	m_car.m_buzzer.SetBeep(false);
	if(m_pit_count%2==0){
		m_car.m_mpu6050.Update();
		m_car.m_mma8451q.Update();
		m_gyro_ = m_car.m_mpu6050.GetOmega();

		m_upstand->KalmanFilter();
		m_real_angle = (float) m_upstand->GetAngle();

		m_balpid[0] = 250.0f/*m_bkp->GetReal()*/;
		m_balpid[1] = 0.0f/*m_bki->GetReal()*/;
		m_balpid[2] = 4.0f/*m_bkd->GetReal()*/;


		m_balance_pid_output = -Output_b(m_balcon, m_balpid, m_time, m_real_angle, -m_gyro_[1]);
//		m_balance_pid_output = 0;
		m_power_r = m_balance_pid_output + m_turn_powerr;
		m_power_l = m_balance_pid_output + m_turn_powerl;

	}
//	Every 20ms for the two ccds to finish sampling
	if(m_pit_count%4==3){
		m_car.m_ccd_2.StartSample();
		while(!m_car.m_ccd_2.SampleProcess()){}
		m_ccd_data_2 = m_car.m_ccd_2.GetData();

		uint16_t cameramid = 127/2;
		m_prev_edge_data_2[0] = m_edge_data_2[0];
		m_prev_edge_data_2[1] = m_edge_data_2[1];
		Update_edge(m_ccd_data_2.data(), m_edge_data_2);
		m_route_mid_2 = (m_edge_data_2[0] + m_edge_data_2[1])/2;

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
				m_last_y2[i] = 80-m_ccd_data_2[i]/4;
				rect_.x = i;
				rect_.y = m_last_y2[i];
				rect_.w = 1;
				rect_.h = 1;
				m_car.m_lcd.SetRegion(rect_);
				color2 = ~0;
				m_car.m_lcd.FillColor(color2);
			}

			if(m_edge_data_2[0]!=m_edge_data_1[0]){
				for(int i=m_edge_data_1[0]; i<=m_edge_data_2[0]; i++){
					rect_.x = i;
					rect_.y = (m_last_y2[m_edge_data_2[0]] - m_last_y[m_edge_data_1[0]])*(i - m_edge_data_1[0])/(m_edge_data_2[0]-m_edge_data_1[0]) + m_last_y[m_edge_data_1[0]];
					rect_.w = 2;
					rect_.h = 2;
					m_car.m_lcd.SetRegion(rect_);
					color2 = 0b0000011111111111;
					m_car.m_lcd.FillColor(color2);
				}
			}
			if(m_edge_data_2[1]!=m_edge_data_1[1]){
				for(int i=m_edge_data_2[1]; i<=m_edge_data_1[1]; i++){
					rect_.x = i;
					rect_.y = (m_last_y2[m_edge_data_2[1]] - m_last_y[m_edge_data_1[1]])*(i - m_edge_data_1[1])/(m_edge_data_2[1]-m_edge_data_1[1]) + m_last_y[m_edge_data_1[0]];
					rect_.w = 2;
					rect_.h = 2;
					m_car.m_lcd.SetRegion(rect_);
					color2 = 0b0000011111111111;
					m_car.m_lcd.FillColor(color2);
				}
			}
		}
	}

	if(m_pit_count%4==1){
		St7735r::Rect rect_;
		uint16_t color2 = 0;
		if(m_edge_data_2[0]!=m_edge_data_1[0]){
			for(int i=m_edge_data_1[0]; i<=m_edge_data_2[0]; i++){
				rect_.x = i;
				rect_.y = (m_last_y2[m_edge_data_2[0]] - m_last_y[m_edge_data_1[0]])*(i - m_edge_data_1[0])/(m_edge_data_2[0]-m_edge_data_1[0]) + m_last_y[m_edge_data_1[0]];
				rect_.w = 2;
				rect_.h = 2;
				m_car.m_lcd.SetRegion(rect_);
				m_car.m_lcd.FillColor(color2);
			}
		}
		if(m_edge_data_2[1]!=m_edge_data_1[1]){
			for(int i=m_edge_data_2[1]; i<=m_edge_data_1[1]; i++){
				rect_.x = i;
				rect_.y = (m_last_y2[m_edge_data_2[1]] - m_last_y[m_edge_data_1[1]])*(i - m_edge_data_1[1])/(m_edge_data_2[1]-m_edge_data_1[1]) + m_last_y[m_edge_data_1[0]];
				rect_.w = 2;
				rect_.h = 2;
				m_car.m_lcd.SetRegion(rect_);
				m_car.m_lcd.FillColor(color2);
			}
		}
		m_car.m_ccd_1.StartSample();
		while(!m_car.m_ccd_1.SampleProcess()){}

		m_ccd_data_raw = m_car.m_ccd_1.GetData();
		uint16_t c[libsc::Tsl1401cl::kSensorW];
		uint16_t s[libsc::Tsl1401cl::kSensorW];
		for(int i = 0; i < libsc::Tsl1401cl::kSensorW; i++ ){
			c[i] = m_ccd_data_raw[i];
		}
		medianFilter(c, s , libsc::Tsl1401cl::kSensorW);
		for(int i = 0; i < libsc::Tsl1401cl::kSensorW; i++ ){
			m_ccd_data_1[i] = s[i];
		}

		m_avg = 0;
		m_sum = 0;
		for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
			m_sum += (uint32_t)m_ccd_data_1[i];
		}
		m_avg = (uint16_t) (m_sum / libsc::Tsl1401cl::kSensorW);
//		for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
//			if(m_ccd_data_[i] >= m_avg-5){
//					m_color[i] = CCD_WHITE;
//			}else{
//				m_color[i] = CCD_BLACK;
//			}
//		}

		m_prev_edge_data_1[0] = m_edge_data_1[0];
		m_prev_edge_data_1[1] = m_edge_data_1[1];
		Update_edge(m_ccd_data_1.data(), m_edge_data_1);
		m_route_mid_1 = (m_edge_data_1[0] + m_edge_data_1[1])/2;

		float trust = 0.0f;
		m_movavgturn.Add(m_mid - m_route_mid_1);
		int error = (int)(trust * (m_mid - m_route_mid_1) + (1.0f-trust) * (m_mid - m_route_mid_2));

		/*
		 * 90 degree detection
		 */
		int total_white_2 = 0;
		int total_white_1 = 0;
		int right_white = 0;
		int left_white = 0;
		for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
			if(m_ccd_data_1[i] > m_threshold_1){
				total_white_1++;
			}
			if(m_ccd_data_1[i] > m_avg){
				if(i>=(127-10)){
					right_white++;
				}
				if(i<=10){
					left_white++;
				}
			}
		}
		for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
			if(m_ccd_data_2[i] > m_threshold_2){
				total_white_2++;
			}
		}

		if(total_white_1 >= 940 && total_white_2 >= 60 &&
				(abs(m_edge_data_1[1] - m_edge_data_1[0]) - abs(m_prev_edge_data_1[1] - m_prev_edge_data_1[0])) > 13 &&
				/*((right_white >= 6) ^ (left_white >= 6))*/
				(
					(((m_prev_edge_data_1[0] - m_edge_data_1[0]) > 13) && abs(m_prev_edge_data_1[1] - m_edge_data_1[1]) <= 1)
					^
					(((m_edge_data_1[1] - m_prev_edge_data_1[1]) > 13) && abs(m_prev_edge_data_1[0] - m_edge_data_1[0]) <= 1)
				)
		){
			m_triggered_90 = true;
//			m_car.m_buzzer.SetBeep(true);
			m_hold_error = (int)(5.0f*error);
			m_hold_count = 3;
		}

/*		if(m_triggered_90){
			if(
					(m_edge_data_1[0] < 10 && m_edge_data_1[1] > 110) ||
					(m_edge_data_1[0] > 18 && m_edge_data_1[1] > 118)
			){

			}
		}*/



/*		int sum_if_diff_is_positive = 0;
		for (int i=0; i<libsc::Tsl1401cl::kSensorW-1; i++){
			if((m_ccd_data_1[i] - m_ccd_data_2[i]) > 0){
				sum_if_diff_is_positive++;
			}
		}
		if(sum_if_diff_is_positive>100 && m_hold_count == 0){
//				m_car.m_buzzer.SetBeep(true);
//			if(m_pit_count - m_prev_pit_count){
//					m_car.m_buzzer.SetBeep(true);
//			}
//			m_prev_pit_count = m_pit_count;
			m_hold_count = 1;
			m_hold_error = (int)(2.0f*error);
		}
		if(sum_if_diff_is_positive>100 || m_hold_count > 0){
			m_hold_count--;
			error = m_hold_error;
		}*/

		/*
		 * Cross road detection
		 */
		if((abs(m_edge_data_2[1] - m_edge_data_2[0]) - abs(m_prev_edge_data_2[1] - m_prev_edge_data_2[0])) > 50){
			m_car.m_buzzer.SetBeep(true);
			m_hold_error = m_prev_edge_data_2[1] - m_prev_edge_data_2[0];
			m_hold_count = 20;
		}

		if(m_hold_count > 0){
			m_hold_count--;
			error = m_hold_error;
		}

		if(m_car.m_car_move_forward){
			m_turn_powerl = (int16_t)(-((25.0f+m_speedInMetrePerSecond*1.5f)*error + (3.0f+m_speedInMetrePerSecond*1.0f)*(error - m_turn_prev_error)/0.02f));
//				m_turn_powerl = libutil::Clamp<int16_t>(-800,m_turn_powerl, 800);
			m_turn_powerr = (int16_t)(((25.0f+m_speedInMetrePerSecond*1.5f)*error + (3.0f+m_speedInMetrePerSecond*1.0f)*(error - m_turn_prev_error)/0.02f));
//				m_turn_powerr = libutil::Clamp<int16_t>(-800,m_turn_powerr, 800);
			m_turn_prev_error = error;
		}
		if(m_car.m_lcdupdate){
//			m_pin->Set();
			St7735r::Rect rect_;
			uint16_t color = 0;

			for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
				rect_.x = i;
				rect_.y = m_last_y[i];
				rect_.w = 1;
				rect_.h = 1;
				m_car.m_lcd.SetRegion(rect_);
				color = 0;
				m_car.m_lcd.FillColor(color);
				rect_.x = i;
				rect_.y = 159-m_ccd_data_1[i]/4;
				rect_.w = 1;
				rect_.h = 1;
				m_car.m_lcd.SetRegion(rect_);
				color = ~0;
				m_car.m_lcd.FillColor(color);
				m_last_y[i] = 159-m_ccd_data_1[i]/4;
			}
//			m_pin->Clear();
		}

	}

	if(m_pit_count%4==0){
		m_car.m_encoder0.Update();
		m_car.m_encoder1.Update();
		m_car.m_encoder_countr = -m_car.m_encoder0.GetCount();
		m_car.m_encoder_countr_t += m_car.m_encoder_countr;
		m_car.m_encoder_countl = m_car.m_encoder1.GetCount();
		m_car.m_encoder_countl_t += m_car.m_encoder_countl;

		if(m_car.m_car_move_forward){
//			if(abs(m_turn_powerl)> 400){
//				m_car.m_car_speed = 380.0f;
//			}else{
//				m_car.m_car_speed = 380.0f;
//			}

//			m_car.m_car_speed = (int16_t)((1.5f * 40.0f / 1000.0f * 100.0f) * 1210.0f / 18.8f);

//			m_movavgspeed.Add(m_car.m_car_speed-(m_car.m_encoder_countr + m_car.m_encoder_countl)/2);
//			m_car.m_total_speed_error += m_movavgspeed.GetAverage();
//			m_car.m_total_speed_error = libutil::Clamp<int16_t>(-1000,m_car.m_total_speed_error,1000);
//			float Kp = 0.03;
//			float Ki = 0.0005;
//			float Ki = 0.001;
//			if(m_movavgspeed.GetAverage() > 200){
//				Kp = 0.03;
//			}
//			m_balcon[6] = (float)(m_movavgspeed.GetAverage()*Kp + m_car.m_total_speed_error * Ki);
//			m_balcon[6] = libutil::Clamp<float>(-8.0f,m_balcon[6],8.0f);
//			m_movavgspeed_output.Add(m_balcon[6]);
//			m_balcon[6] = m_movavgspeed_output.GetAverage();
//			m_balcon[6] = 0;
//			m_car.m_car_speed = 2;

			m_speedInMetrePerSecond = (m_car.m_encoder_countr + m_car.m_encoder_countl)/2.0f * 0.188f / 1210.0f / 0.02f;
//			Avoid acceleration over 2ms-2
			float maxAcceleration = 1.0f;
			m_acceleration = (m_speed_setpoint-m_speedInMetrePerSecond)/0.02f;
			m_total_speed += m_acceleration * 0.02f;
			m_total_speed = libutil::Clamp<float>(-300.0f,m_total_speed,300.0f);
			m_acceleration = libutil::Clamp<float>(-1.0f,(0.007f * m_acceleration + /*0.3f * 0.5f * (m_acceleration) + 0.5f * (m_prev_speed) + */0.005f * m_total_speed /* - 0.0001f * 0.8f*m_speedInMetrePerSecond/0.02f + 0.2f * m_prev_speed + 0.0f * m_total_speed*/),maxAcceleration);
//			m_prev_speed = (m_speed_setpoint-m_speedInMetrePerSecond)/0.02;
//			m_prev_speed = 0.3f * 0.5f * (m_acceleration - m_prev_speed) + 0.5f * (m_prev_speed);
			//			m_prev_speed = - 0.8f*m_speedInMetrePerSecond/0.02f + 0.2f * m_prev_speed;
			//			m_acceleration = m_acceleration + 0.001f * m_total_speed;
			m_balcon[6] = atan(m_acceleration/9.81f)*RAD2ANGLE;
		}else{
			m_car.m_car_speed = 0.0f;
			m_balcon[6] = 0;
		}

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
				printf("%f,%f,%f\n",m_speed_setpoint,m_balcon[6],m_speedInMetrePerSecond);
				break;
			case 2:
				printf("%f,%f,%f,%f\n",m_car.m_shift_balance_angle, m_real_angle,m_upstand->GetAccAngle(),m_upstand->GetGyroAngle());
				break;
			case 3:
				printf("E%f,%f,%f\n",m_speed_setpoint,m_balcon[6],m_speedInMetrePerSecond);
//				printf("%d,%d,%d,%d,%f,%f\n",m_power_r_pwm,m_power_l_pwm,m_car.m_encoder_countr, m_car.m_encoder_countl,m_balcon[6],m_speedInMetrePerSecond);
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

	if(m_stop->GetInt()!=0){
		m_car.m_car_move_motor = false;
	}

	m_power_r_pwm = sign((int)m_power_r)*(int16_t)RpmToPwm_R((uint16_t)abs(m_power_r));
	m_power_l_pwm = sign((int)m_power_l)*(int16_t)RpmToPwm_L((uint16_t)abs(m_power_l));

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
	m_movavgspeed(3),
	m_movavgr(3),
	m_movavgl(3),
	m_movavgturn(5),
	m_movavgspeed_output(10),
	m_prevSpeedInMetrePerSecond(0),
	m_speedInMetrePerSecond(0),
	m_prev_speed(0),
	m_acceleration(0),
	m_total_speed(0),
	m_speed_setpoint(1.0f),
	m_turn_prev_error(0),
	m_triggered_90(false),
	m_hold_error(0),
	m_hold_count(0),
	m_threshold_1(0),
	m_threshold_2(0)
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
	pinadcfg.pin = Pin::Name::kPtc3;
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

	/*
	 * Sample CCD1 for average to get black and white threshold
	 */
	m_car.m_ccd_1.StartSample();
	while(!m_car.m_ccd_1.SampleProcess()){}
	System::DelayMs(20);
	m_car.m_ccd_1.StartSample();
	while(!m_car.m_ccd_1.SampleProcess()){}
	m_ccd_data_1 = m_car.m_ccd_1.GetData();
	uint16_t sum = 0;
	for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
		sum += m_ccd_data_1[i];
	}
	m_threshold_1 = sum / libsc::Tsl1401cl::kSensorW;

	uint16_t color = ~0;

	for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
		rect_.x = i;
		rect_.y = 159-m_threshold_1/4;
		rect_.w = 1;
		rect_.h = 1;
		m_car.m_lcd.SetRegion(rect_);
		m_car.m_lcd.FillColor(color);
	}

	/*
	 * Sample CCD2 for average to get black and white threshold
	 */
	m_car.m_ccd_2.StartSample();
	while(!m_car.m_ccd_2.SampleProcess()){}
	System::DelayMs(20);
	m_car.m_ccd_2.StartSample();
	while(!m_car.m_ccd_2.SampleProcess()){}
	m_ccd_data_2 = m_car.m_ccd_2.GetData();
	sum = 0;
	for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
		sum += m_ccd_data_2[i];
	}
	m_threshold_2 = sum / libsc::Tsl1401cl::kSensorW;

	color = ~0;

	for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
		rect_.x = i;
		rect_.y = 80-m_threshold_2/4;
		rect_.w = 1;
		rect_.h = 1;
		m_car.m_lcd.SetRegion(rect_);
		m_car.m_lcd.FillColor(color);
	}

	Pit m_pit(GetPitConfig(0, std::bind(&App::PitBalance, this, std::placeholders::_1)));
	Pit m_pit2(GetPitConfig2(1, std::bind(&App::PitMoveMotor, this, std::placeholders::_1)));

	while(true)
	{
	}

}


