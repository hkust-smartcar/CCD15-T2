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
//#include "Eigen/Eigen"
//using Eigen::Vector3f;


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
	uint16_t val = (uint16_t)(0.36f*count + 80.325f);
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
	balcon[0] = (balcon[4]+balcon[5]+balcon[6]+m_car.m_shift_balance_angle)-real_angle;	// 512*104/44=13312/11
	balcon[0] = tan(balcon[0]*1.0f/RAD2ANGLE)*90;
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
void App::Update_edge(uint16_t* m_ccd_data, uint16_t* edge_data, int ccdNumber){
	int deadzone = 0;
	if(ccdNumber == 1){
		deadzone = 15;
	}
	if(ccdNumber == 2){
		deadzone = 20;
	}
	edge_data[0] = deadzone;
	edge_data[1] = libsc::Tsl1401cl::kSensorW-deadzone-1;
	int16_t maxLeft=-1, maxRight=-1;

	int16_t sum = 0;
	int numberOfSamples = 1;
	for(int i=m_mid-1; i>=m_mid-numberOfSamples; i--){
		sum += (int16_t)(m_ccd_data[i]);
	}
	/*for(int i=m_mid; i>numberOfSamples+deadzone; i--){
		if(((int16_t)m_ccd_data[i] - sum/numberOfSamples) > maxLeft){
			edge_data[0] = i;
			maxLeft = ((int16_t)m_ccd_data[i] - sum/numberOfSamples);
			sum = sum + (int16_t)m_ccd_data[i-numberOfSamples-1] - (int16_t)m_ccd_data[i-1];
		}
	}*/
	for(int i=m_mid; i>deadzone+1; i--){
		if(((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i-1]) > maxLeft){
			if(((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i-1]) >= 10){
				edge_data[0] = i;
				maxLeft = ((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i-1]);
//				break;
			}
		}
	}
	sum = 0;
	for(int i=m_mid+1; i<=m_mid+1+numberOfSamples; i++){
		sum += (int16_t)(m_ccd_data[i]);
	}
/*	for(int i=m_mid; i<=libsc::Tsl1401cl::kSensorW-deadzone-numberOfSamples; i++){
		if(((int16_t)m_ccd_data[i] - sum/numberOfSamples) > maxRight){
			edge_data[1] = i;
			maxRight = ((int16_t)m_ccd_data[i] - sum/numberOfSamples);
			sum = sum + (int16_t)m_ccd_data[i+numberOfSamples] - (int16_t)m_ccd_data[i];
		}
	}*/
	for(int i=m_mid; i<libsc::Tsl1401cl::kSensorW-deadzone-1; i++){
		if(((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i+1]) > maxRight){

			if(((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i+1]) >= 10){
				edge_data[1] = i;
				maxRight = ((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i+1]);
//				break;
			}
		}
	}

/*	for (int i=numberOfSamples; i<libsc::Tsl1401cl::kSensorW-1; i++){
		if(((int16_t)m_ccd_data[i] - sum/numberOfSamples) > maxLeft){
			edge_data[0] = i;
			maxLeft = ((int16_t)m_ccd_data[i] - sum/numberOfSamples);
		}
		if(((int16_t)m_ccd_data[i-numberOfSamples] - (sum - m_ccd_data[i-numberOfSamples] + m_ccd_data[i+1])/numberOfSamples) > maxRight){
			edge_data[1] = i;
			maxRight = ((int16_t)m_ccd_data[i-numberOfSamples] - (sum - m_ccd_data[i-numberOfSamples] + m_ccd_data[i+1])/numberOfSamples);
		}
		sum = sum - (int16_t)m_ccd_data[i-numberOfSamples] + (int16_t)m_ccd_data[i];

	}*/

	int innerBlackLeft = -1, innerBlackRight = -1;

/*	for(int i=0; i<m_mid; i++){
		if(((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i+1]) > innerBlackLeft){

			if(((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i+1]) >= 10){
				edge_data[0] = i;
				innerBlackLeft = ((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i+1]);
			}
		}
	}

	for(int i=libsc::Tsl1401cl::kSensorW-1; i>m_mid; i--){
		if(((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i-1]) > innerBlackRight){

			if(((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i-1]) >= 10){
				edge_data[1] = i;
				innerBlackRight = ((int16_t)m_ccd_data[i] - (int16_t)m_ccd_data[i-1]);
			}
		}
	}*/
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
	config.count = ClockUtils::GetBusTickPerUs() * 1000 * 1;
	config.isr = isr;
	return config;
}

void App::PitBalance(Pit*){
	if(m_hold_count <= 0){
		m_car.m_buzzer.SetBeep(false);
	}
	if(m_pit_count%2==0){
		m_car.m_mpu6050.Update();
		m_car.m_mma8451q.Update();
		m_gyro_ = m_car.m_mpu6050.GetOmega();

		m_upstand->KalmanFilter();
		m_real_angle = (float) m_upstand->GetAngle();


		m_balpid[1] = 0.0f/*m_bki->GetReal()*/;
		if(m_speedInMetrePerSecond>=m_speed_setpoint*0.8f){
			m_balpid[0] = 400.0f/*m_bkp->GetReal()*/;
			m_balpid[2] = 10.0f/*m_bkd->GetReal()*/;
		}else{
			m_balpid[0] = 400.0f/*m_bkp->GetReal()*/;
			m_balpid[2] = 10.0f;
		}


		m_balance_pid_output = -Output_b(m_balcon, m_balpid, m_time, m_real_angle, -m_gyro_[1]);
//		m_balance_pid_output = 0;
	}
//	Every 20ms for the two ccds to finish sampling
	if(m_pit_count%4==3){
		m_car.m_ccd_2.StartSample();
		while(!m_car.m_ccd_2.SampleProcess()){}
		m_ccd_data_raw = m_car.m_ccd_2.GetData();
		uint16_t c[libsc::Tsl1401cl::kSensorW];
		uint16_t s[libsc::Tsl1401cl::kSensorW];
		for(int i = 0; i < libsc::Tsl1401cl::kSensorW; i++ ){
			c[i] = m_ccd_data_raw[i];
		}
		medianFilter(c, s , libsc::Tsl1401cl::kSensorW);
		for(int i = 0; i < libsc::Tsl1401cl::kSensorW; i++ ){
			m_ccd_data_2[i] = s[i];
		}
		for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
			if(i<20 || i>libsc::Tsl1401cl::kSensorW-20-1){
				m_ccd_data_2[i] = 0;
			}
		}


		Update_edge(m_ccd_data_2.data(), m_edge_data_2, 2);
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

		m_ccd_data_1 = m_car.m_ccd_1.GetData();
		for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
			if(i<15 || i>libsc::Tsl1401cl::kSensorW-15-1){
				m_ccd_data_1[i] = 0;
			}
		}

/*		m_ccd_data_raw = m_car.m_ccd_1.GetData();
		uint16_t c[libsc::Tsl1401cl::kSensorW];
		uint16_t s[libsc::Tsl1401cl::kSensorW];
		for(int i = 0; i < libsc::Tsl1401cl::kSensorW; i++ ){
			c[i] = m_ccd_data_raw[i];
		}
		medianFilter(c, s , libsc::Tsl1401cl::kSensorW);
		for(int i = 0; i < libsc::Tsl1401cl::kSensorW; i++ ){
			m_ccd_data_1[i] = s[i];
		}*/

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
		Update_edge(m_ccd_data_1.data(), m_edge_data_1,1);
		m_route_mid_1 = (m_edge_data_1[0] + m_edge_data_1[1])/2;

		/*
		 * Change trust based on error
		 */
		float trust = 0.0f;
		trust = (m_speed_setpoint - m_speedInMetrePerSecond)/m_speed_setpoint * 0.0f + abs(m_turn_error)/30 * 1.0f/* + m_balcon[0] / 5.0f * 0.4f*/;

		/*
		 * Change trust of closer CCD based on width of the closer CCD
		 * So that the car won't go out of boundary
		 */

		int16_t width_1 = (int16_t)m_edge_data_1[1] - (int16_t)m_edge_data_1[0];
		int16_t width_2 = (int16_t)m_edge_data_2[1] - (int16_t)m_edge_data_2[0];
		/*
		 * Width needs to be bigger than 0, so that won't mess up with single black line case
		 */
		if(width_1 > 0 && width_1 < 80){
			trust = 0.0f;
		}

		if(width_1 < 0){
			trust = 0.0f;
		}

		if(width_2 < 0){
			trust = 1.0f;
		}

		/*
		 * Clamp trust so that must sum up to 1
		 */
		trust = libutil::Clamp<float>(0.0f,trust,1.0f);
		int m_turn_error = (int)(trust * (m_mid - m_route_mid_1) + (1.0f-trust) * (m_mid - m_route_mid_2));

		/*
		 * Middle black detection
		 */
		if(abs(m_edge_data_1[1]-m_edge_data_1[0])<15 || abs(m_edge_data_2[1]-m_edge_data_2[0])<15){
			m_hold_error = (int)m_turn_prev_error;
			m_hold_count = 5;
//			m_car.m_buzzer.SetBeep(true);
		}

		/*
		 * 90 degree detection
		 */
		int total_white_2 = 0;
		int total_white_1 = 0;
		int right_white = 0;
		int left_white = 0;

		for(int i=15; i<libsc::Tsl1401cl::kSensorW-15; i++){
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
		for(int i=20; i<libsc::Tsl1401cl::kSensorW-20; i++){
			if(m_ccd_data_2[i] > m_threshold_2){
				total_white_2++;
			}
		}

		if(total_white_2 == 0 && m_ccd_1_entered_black && m_ccd_1_dropped_edge && m_ccd_2_dropped_edge){
			m_turn_error = 0;
			m_ccd_2_entered_black=true;
		}

		if(total_white_1 == 0){
			m_turn_error = 0;
			m_ccd_1_entered_black=true;
		}


		if(m_edge_data_2[0] == 20 && m_prev_edge_data_2[0] >= 20 && m_edge_data_2[1] != 107 && total_white_2 != 0){
			m_last_left_maxed = (int)m_edge_data_2[0];
			m_last_right_maxed = 0;
			m_ccd_2_dropped_edge = true;
		}

		if(m_edge_data_2[1] == 107 && m_edge_data_2[0] != 20 && total_white_2 != 0){
			m_last_left_maxed = 0;
			m_last_right_maxed = (int)m_edge_data_2[1];
			m_ccd_2_dropped_edge = true;
		}

		if(m_edge_data_1[0] == 15 && m_edge_data_1[1] != 112 && total_white_1 != 0){
			m_ccd_1_dropped_edge = true;
		}

		if(m_edge_data_1[1] == 112 && m_edge_data_1[0] != 15 && total_white_1 != 0){
			m_ccd_1_dropped_edge = true;
		}


		if(
				m_ccd_2_entered_black &&
				m_ccd_2_dropped_edge &&
				m_ccd_1_dropped_edge &&
				m_hold_count == 0 &&
				!m_triggered_90
		){
			m_ccd_2_dropped_edge = false;
			m_ccd_1_dropped_edge = false;
			m_ccd_1_entered_black = false;
			m_ccd_2_entered_black = false;
			m_ccd_2_entered_90_black = false;
			m_triggered_90 = true;
			m_car.m_buzzer.SetBeep(true);
//			m_speed_setpoint = 1.0f;
			if(m_last_left_maxed == 20){
//				m_hold_error = (int)8.0f*((int)m_mid - m_last_left_maxed);
				m_power_l_pwm = -400;
				m_power_r_pwm = 400;
				printf("turn left %d %d %d %d %d %d %d\n",m_turn_powerl,m_turn_powerr, m_power_l, m_power_r, m_balance_pid_output, m_power_l_pwm, m_power_r_pwm);
			}

			if(m_last_right_maxed == 107){
//				m_hold_error = (int)8.0f*((int)m_mid - m_last_right_maxed);
				m_power_l_pwm = 400;
				m_power_r_pwm = -400;
				printf("turn right %d %d %d %d %d %d %d\n",m_turn_powerl,m_turn_powerr, m_power_l, m_power_r, m_balance_pid_output, m_power_l_pwm, m_power_r_pwm);
			}

//			m_hold_error = (int)800.0f*(m_last_error_not_full_width);
			m_hold_count = 10;
		}

		m_prev_edge_data_2[0] = m_edge_data_2[0];
		m_prev_edge_data_2[1] = m_edge_data_2[1];


		/*
		 * Cross road detection
		 */
		if(
				// If left edge is on right of right edge, then this maybe a cross road
//				(m_edge_data_2[0]>m_edge_data_2[1] && (m_edge_data_2[0] - m_edge_data_2[1])>10) ||
				// If sudden change in width, then this maybe a cross road
				/*((abs(m_edge_data_1[1] - m_edge_data_1[0]) - abs(m_prev_edge_data_1[1] - m_prev_edge_data_1[0])) > 10 &&*/
				abs(m_edge_data_1[1] - m_edge_data_1[0]) >= 110 &&
				abs(m_edge_data_2[1] - m_edge_data_2[0]) >= 110

		)
		{
//			m_car.m_buzzer.SetBeep(true);
			m_hold_error = 0;
//			m_hold_count = 10;
		}

		if(m_hold_count > 0){
			m_hold_count--;
			if(m_hold_count==1){
				m_speed_setpoint = 2.2f;
				m_triggered_90 = false;
				m_car.m_buzzer.SetBeep(false);
			}
//			m_turn_error = m_hold_error;
//			m_turn_powerl = -20;
//			m_turn_powerr = 20;
		}

/*		if(abs(m_turn_error)>10){
			m_speed_setpoint = 2.0f;
		}else{
			m_speed_setpoint = 2.3f;
		}*/

		if(m_car.m_car_move_forward && !m_triggered_90){
			m_turn_powerl = (int16_t)(-((40.5f+m_speedInMetrePerSecond*6.0f+abs(m_turn_error)*0.0f)*m_turn_error + (8.0f+0.0f*abs(m_balcon[0])+m_speedInMetrePerSecond*m_speedInMetrePerSecond*0.5f)*(m_turn_error - m_turn_prev_error)/0.02f));
//				m_turn_powerl = libutil::Clamp<int16_t>(-800,m_turn_powerl, 800);
			m_turn_powerr = (int16_t)(((40.5f+m_speedInMetrePerSecond*6.0f+abs(m_turn_error)*0.0f)*m_turn_error + (8.0f+0.0f*(m_balcon[0])+m_speedInMetrePerSecond*m_speedInMetrePerSecond*0.5f)*(m_turn_error - m_turn_prev_error)/0.02f));
//				m_turn_powerr = libutil::Clamp<int16_t>(-800,m_turn_powerr, 800);
			m_turn_prev_error = m_turn_error;
		}else{
			m_turn_powerl = m_turn_powerr = 0;
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
			m_movavgspeed.Add((int16_t)(m_speed_setpoint-m_speedInMetrePerSecond)/0.02f);
			m_speed_error = (float)m_movavgspeed.GetAverage();

			float speedKp = 31.0f;
			float speedKd = 0.0f;
			float speedKi = 17.5f;
			float speedDt = 0.02f;

			m_total_speed += (0.05f*m_speed_error + 0.95f * m_prev_speed) * speedDt;
			m_total_speed = libutil::Clamp<float>(-60.0f,m_total_speed,60.0f);

//			m_speed_output = (int16_t)(speedKp * m_speed_error + speedKi * m_total_speed);
			float a = speedKp + speedKd / speedDt + speedKi * speedDt;
			float b = -speedKp - 2*speedKd / speedDt;
			float c = speedKd / speedDt;
			
//			m_speed_output += (int16_t)(a * m_speed_error + b * m_prev_speed + c * m_prev_speed_2);

			// Avoid acceleration over 3ms-2
			float maxAcceleration = 0.0f;
			if(m_speedInMetrePerSecond <= m_speed_setpoint*0.6f){
				maxAcceleration = 4.0f;
			}else{
				maxAcceleration = 4.0f;
			}

//			Affect balance only when speed is > setpoint * 80%
//			if(m_speedInMetrePerSecond >= m_speed_setpoint*0.6f){

//			Otherwise, use angle to do acceleration for maximum acceleration
//			}else{

					m_acceleration = libutil::Clamp<float>(-maxAcceleration,(0.05f * (0.2f*m_speed_error + 0.8f * m_prev_speed) + 0.0f * (m_speed_error-m_prev_speed)/0.02f + 0.93f * m_total_speed * 0.02f),maxAcceleration);
					m_prev_speed_2 = m_prev_speed;
					m_prev_speed = (0.2f*m_speed_error + 0.8f * m_prev_speed);
//					m_prev_speed = (m_speed_setpoint-m_speedInMetrePerSecond)/0.02;
//					m_prev_speed = 0.3f * 0.5f * (m_acceleration - m_prev_speed) + 0.5f * (m_prev_speed);
//					m_prev_speed = - 0.8f*m_speedInMetrePerSecond/0.02f + 0.2f * m_prev_speed;
//					m_acceleration = m_acceleration + 0.001f * m_total_speed;
					m_balcon[6] = atan(m_acceleration/9.81f)*RAD2ANGLE;
//					m_speed_output = 0;
//			}

		}else{
			m_car.m_car_speed = 0.0f;
			m_balcon[6] = 0;
			m_speed_output = 0;
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
				printf("%f,%f\n",m_speed_setpoint,m_speedInMetrePerSecond);
				break;
			case 2:
				printf("%f,%f,%f,%f,%f\n",m_car.m_shift_balance_angle, m_real_angle,m_upstand->GetAccAngle(),m_upstand->GetGyroAngle(), m_balcon[0]);
				break;
			case 3:
				printf("%d,%d,%d,%d,%f,%f\n",m_power_r_pwm,m_power_l_pwm,m_car.m_encoder_countr, m_car.m_encoder_countl,m_balcon[6],m_speedInMetrePerSecond);
				break;
			case 4:
				printf("%d,%d\n",m_edge_data_2[1] - m_edge_data_2[0],m_edge_data_1[1] - m_edge_data_1[0]);
				break;
			case 5:
				printf("E%d,%d\n",m_edge_data_2[1] - m_edge_data_2[0],m_edge_data_1[1] - m_edge_data_1[0]);
				break;
			case 6:
				printf("%d,%d,%d,%d\n",m_edge_data_2[0], m_edge_data_2[1],m_edge_data_1[0], m_edge_data_1[1]);
				break;
			case 0:
			default:
				break;
		}
	}

	m_pit_count++;

	if(m_turn_powerr < 0 && !m_triggered_90){
		m_turn_powerr = 0;
	}
	if(m_turn_powerl < 0 && !m_triggered_90){
		m_turn_powerl = 0;
	}
	m_power_r = m_balance_pid_output /*- m_speed_output */+ m_turn_powerr;
	m_power_l = m_balance_pid_output /*- m_speed_output */+ m_turn_powerl;

	if(!m_triggered_90){
		m_power_r_pwm = sign((int)m_power_r)*(int16_t)RpmToPwm_R((uint16_t)abs(m_power_r));
		m_power_l_pwm = sign((int)m_power_l)*(int16_t)RpmToPwm_L((uint16_t)abs(m_power_l));
	}

	if(m_stop->GetInt()!=0){
		m_car.m_car_move_motor = false;
	}else if(m_start->GetInt()!=0){
		m_car.m_car_move_forward = true;
	}

	if(!m_car.m_car_move_motor){
		m_power_r_pwm = m_power_l_pwm = 0;
		m_power_r = m_power_l = 0;
	}

	m_car.m_motor_r.SetClockwise(m_power_r_pwm < 0); //Right Motor - false forward, true backward
	m_car.m_motor_l.SetClockwise(m_power_l_pwm > 0); //Left Motor - true forward, false backward
	m_car.m_motor_r.SetPower((uint16_t)abs(m_power_r_pwm));
	m_car.m_motor_l.SetPower((uint16_t)abs(m_power_l_pwm));
}

void App::PitMoveMotor(Pit*){
	/*
	 * Protection for motors
	 * */

//	if(abs(power_l) >= 1000) power_l = 0;
//	if(abs(power_r) >= 1000) power_r = 0;


	m_pit_count2++;
}

App::App():
	m_car(),
	m_lcd_typewriter(GetLcdTypewriterConfig()),
	m_balance_pid_output(0),
	m_movavgspeed(5),
	m_movavgr(3),
	m_movavgl(3),
	m_movavgturn(5),
	m_movavgspeed_output(10),
	m_prevSpeedInMetrePerSecond(0),
	m_speedInMetrePerSecond(0),
	m_prev_speed(0),
	m_prev_speed_2(0),
	m_acceleration(0),
	m_speed_error(0),
	m_total_speed(0),
	m_speed_setpoint(2.2f),
	m_speed_output(0),
	m_turn_error(0),
	m_turn_prev_error(0),
	m_triggered_90(false),
	m_hold_error(0),
	m_hold_count(0),
	m_threshold_1(0),
	m_threshold_2(0),
	m_ccd_1_entered_black(false),
	m_ccd_2_entered_black(false),
	m_last_error_not_full_width(0),
	m_last_left_maxed(0),
	m_last_right_maxed(0),
	m_ccd_2_dropped_edge(false),
	m_ccd_1_dropped_edge(false)
{
	float totalVoltage = 0.0f;
	for(int i=0;  i<=1000; i++){
		totalVoltage += m_car.m_bat.GetVoltage();
	}
	St7735r::Rect rect_;
	rect_.x = 88;
	rect_.y = 0;
	rect_.w = 40;
	rect_.h = 16;
	m_car.m_lcd.SetRegion(rect_);
	m_lcd_typewriter.WriteString(String::Format("%.2fV\n",totalVoltage/1000).c_str());
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
//	m_threshold_1 = 127;

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
//	m_threshold_2 = 127;

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


