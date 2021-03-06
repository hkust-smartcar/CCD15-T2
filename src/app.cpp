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
#include "upstand.h"
#include "medianFilter.h"
#include "OLED.h"


using namespace libsc::kl26;
using namespace libbase::kl26;
using namespace libsc;

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
	m_actual_bal_error = (balcon[4]+balcon[5])-real_angle;
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

void App::GetBlocks(uint16_t* m_ccd_data, int ccdNumber){
	int diff[127] = {0};
	for(int i=0; i<127; i++){
		diff[0] = (int)m_ccd_data[i+1] - (int)m_ccd_data[i];
	}
	int windowSize = 20;
	int sum = 0;
	for(int i=0; i<windowSize; i++){
		sum += (int)m_ccd_data[i];
	}
}

uint16_t App::Get_mid(uint16_t* m_ccd_data, int ccdNumber, uint16_t* midData){
	uint16_t deadzone = 0;
	uint16_t regionCount = 0;
	uint16_t regionEdge[20] = {0};
	uint16_t deltaMid[2] = {0};
	int8_t RorF = 0;
	for(int i = 0; i< 10; i++){
		midData[i] = 0;
	}
	if(ccdNumber == 1)
		deadzone = 0;
	else if(ccdNumber == 2)
		deadzone = 20;
	regionEdge[0] = deadzone;
	for (uint16_t i = deadzone; i<126-deadzone; i++){
		if(regionCount == 20){
			break;
		}
		if (regionCount == 0 && abs(m_ccd_data[i+1]-m_ccd_data[i])>9){
			regionEdge[regionCount] = i+1;
			midData[regionCount] = (regionEdge[regionCount] + regionEdge[regionCount-1])/2;
			regionCount++;
			if (m_ccd_data[i+1] - m_ccd_data[i] > 9)
				RorF = -1;
			else RorF = 1;
		}
		if (regionCount != 0 && m_ccd_data[i+1] - m_ccd_data > (RorF*9){
			regionEdge[regionCount] = i+1;
			midData[regionCount] = (regionEdge[regionCount] + regionEdge[regionCount-1])/2;
			regionCount++;
			RorF *= -1;
		}
	}
	regionEdge[regionCount] = 127-deadzone;
	midData[regionCount] = (regionEdge[regionCount] + regionEdge[regionCount-1])/2;*/
	m_total_white_1 = 0;
	for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
		if(m_ccd_data_1[i] >= m_avg){
			m_color[i] = CCD_WHITE;
			m_total_white_1++;
		}else{
			m_color[i] = CCD_BLACK;
		}
	}
	for(int i=1; i<libsc::Tsl1401cl::kSensorW-1; i++){
		if(m_color[i+1] != m_color[i]){
			regionEdge[regionCount] = i;
			midData[regionCount] = (regionEdge[regionCount] + regionEdge[regionCount-1])/2;
			regionCount++;
		}
	}

	m_regionTotalNumber[ccdNumber] = regionCount;

	if(regionCount <= 0){
		return m_nowMid[ccdNumber];
	}

	int closestMid = 63;
	int16_t min = INT16_MAX;
	for(int i=0; i<regionCount; i++){
		if((int16_t)(abs((int)midData[i] - (int)m_nowMid[ccdNumber])) < min){
			min = (int16_t)(abs((int)midData[i] - (int)m_nowMid[ccdNumber]));
			closestMid = i;
//			printf("%d %d %d\n",m_regionTotalNumber[ccdNumber], midData[closestMid], min);
		}
	}

//	if(abs((int)midData[closestMid] - (int)m_nowMid[ccdNumber]) < 3){
		return midData[closestMid];
//	}else{
//		return m_nowMid[ccdNumber];
//	}
}
void App::Analysis(uint16_t* region, float* now_mid){
	uint16_t state_count1 = 0;
	uint16_t state_count2 = 0;
	m_prev_state = m_state;
	if (region[1] == 5){
		if (abs(now_mid[1] - 64) < 10)
			m_state = MIDDLELINE;
		else if (abs(now_mid[1] - 64) >= 10)
			m_state = OBSTACLE;
		return;
	}
	else if (region[1] == 1 && region[2] == 1){
		m_state = SLOPE;
		return;
	}
	else if (region[1] == 1){
		state_count1++;
		if (abs(now_mid[2] - 64) < 5)
			m_state = STRAIGHT;
		else if (abs(now_mid[2] - 64) >= 5)
			m_state = TURN2;
		return;
	}
	else if (region[2] == 1){
		state_count2++;
		if (abs(now_mid[1] - 64) < 10)
			m_state = STRAIGHT;
		else if (abs(now_mid[1] - 64) >= 10)
			m_state = TURN1;
		return;
	}
	if (state_count2 > 15){
		m_found_cross = true;
	}
	else if (state_count2 > 0){
		m_found_blackline = true;
	}
	if (state_count1 > 15 && m_found_cross == true){
		m_found_cross = false;
		m_state = CROSS;
		state_count1 = 0;
		state_count2 = 0;
		return;
	}
	else if (state_count1 > 0 && m_found_blackline == true){
		m_found_blackline = false;
		m_state = NINETYDEG;
		state_count1 = 0;
		state_count2 = 0;
		return;
	}
	m_state = UNKNOWN;
	return;
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

void App::PitBalance(Pit*){
	m_found_edges = false;

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
			m_balpid[0] = 78.0f/*m_bkp->GetReal()*/;
			m_balpid[2] = 1.5f/*m_bkd->GetReal()*/;
		}else{
			m_balpid[0] = 78.0f/*m_bkp->GetReal()*/;
			m_balpid[2] = 1.5f;
		}


		m_balance_pid_output = -Output_b(m_balcon, m_balpid, m_time, m_real_angle, -m_gyro_[1]);
//		m_balance_pid_output = 0;
	}
//	Every 20ms for the two ccds to finish sampling
	if(m_pit_count%4==3){
		m_car.m_ccd_2.StartSample();
		while(!m_car.m_ccd_2.SampleProcess()){}
		m_ccd_data_raw_2 = m_car.m_ccd_2.GetData();

		int shiftCcd2 = 2;
		for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
			if(i<shiftCcd2){
				m_ccd_data_2[i] = 0;
			}else{
				m_ccd_data_2[i] = m_ccd_data_raw_2[i-shiftCcd2];
			}

		}

		for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
			if(i<20 || i>libsc::Tsl1401cl::kSensorW-20-1){
				m_ccd_data_2[i] = 0;
			}
		}

//		m_nowMid[2] = m_route_mid_1;
//		m_route_mid_2 = Get_mid(m_ccd_data_2.data(), 2, mid_data2);

		m_avg_2 = 0;
		m_sum_2 = 0;
		for(int i=20; i<libsc::Tsl1401cl::kSensorW-20; i++){
			m_sum_2 += (uint32_t)m_ccd_data_2[i];
		}
		m_avg_2 = (uint16_t) (m_sum_2 / libsc::Tsl1401cl::kSensorW);
//		for(int i=20; i<libsc::Tsl1401cl::kSensorW-20; i++){
//			if(m_ccd_data_2[i] >= m_avg_2){
//				m_color_2[i] = CCD_WHITE;
//			}else{
//				m_color_2[i] = CCD_BLACK;
//			}
//		}

//		m_found_obstacle = false;


		if(m_car.m_lcdupdate){
//			St7735r::Rect rect_;
//			uint16_t color2 = 0;
//
//			for(int i=0; i<Tsl1401cl::kSensorW; i++){
//				rect_.x = i;
//				rect_.y = m_last_y2[i];
//				rect_.w = 1;
//				rect_.h = 1;
//				m_car.m_lcd.SetRegion(rect_);
//				color2 = 0;
//				m_car.m_lcd.FillColor(color2);
//				m_last_y2[i] = 80-m_ccd_data_2[i]/4;
//				rect_.x = i;
//				rect_.y = m_last_y2[i];
//				rect_.w = 1;
//				rect_.h = 1;
//				m_car.m_lcd.SetRegion(rect_);
//				color2 = ~0;
//
//				if(i<=m_edge_data_2[0]){
//					color2 = 0b0000000000011111;
//				}
//				if(i>=m_edge_data_2[1]){
//					color2 = 0b0000011111100000;
//				}
//				if(i<m_edge_data_2[0] || i>m_edge_data_2[1]){
//					color2 = 0;
//				}
//				if(i>=m_edge_data_2[2] && i<=m_edge_data_2[3]){
//					color2 = 0b1111100000000000;
//				}
//				m_car.m_lcd.FillColor(color2);
//			}

		}


	}

	if(m_pit_count%4==1){

		m_car.m_ccd_1.StartSample();
		while(!m_car.m_ccd_1.SampleProcess()){}

		m_ccd_data_raw = m_car.m_ccd_1.GetData();

		int shiftCcd1 = 0;
		for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
			if(i < shiftCcd1){
				m_ccd_data_1[i] = 0;
			}else{
				m_ccd_data_1[i] = m_ccd_data_raw[i-shiftCcd1];
			}

		}

/*		for(int i=0; i<libsc::Tsl1401cl::kSensorW; i++){
			if(i<15 || i>libsc::Tsl1401cl::kSensorW-15-1){
				m_ccd_data_1[i] = 0;
			}
		}*/


		m_avg = 0;
		m_sum = 0;
		for(int i=15; i<libsc::Tsl1401cl::kSensorW-15; i++){
			m_sum += (uint32_t)m_ccd_data_1[i];
		}
		m_avg = (uint16_t) (m_sum / libsc::Tsl1401cl::kSensorW);

		m_found_middle_line = false;

		m_car.m_led.SetEnable(false);
		m_car.m_led2.SetEnable(false);
		m_car.m_led3.SetEnable(false);
		m_car.m_led4.SetEnable(false);

		if(m_found_edges){
//			printf("Edges\n");
			m_car.m_led.SetEnable(true);
			m_car.m_led2.SetEnable(false);
			m_car.m_led3.SetEnable(false);
			m_car.m_led4.SetEnable(true);
		}
		if(m_state==STRAIGHT){
			m_car.m_led.SetEnable(true);
			m_car.m_led2.SetEnable(false);
			m_car.m_led3.SetEnable(true);
			m_car.m_led4.SetEnable(true);
		}
		if(m_found_middle_line){
//			m_route_mid_1 = m_edge_data_1[2];
//			printf("Middle Line\n");
			m_car.m_led.SetEnable(false);
			m_car.m_led2.SetEnable(true);
			m_car.m_led3.SetEnable(false);
			m_car.m_led4.SetEnable(false);
		}
		/*if(m_found_obstacle_1 || m_found_obstacle_2){
			if(m_found_obstacle_2){
//				m_trust = 0.0f;
				if(abs(m_edge_data_2[1]-m_edge_data_2[3]) > abs(m_edge_data_2[2]-m_edge_data_2[0])){
					m_route_mid_2 = (m_edge_data_2[1]+m_edge_data_2[3])/2;
				}else{
					m_route_mid_2 = (m_edge_data_2[2]+m_edge_data_2[0])/2;
				}
			}else{
				m_trust = 1.0f;
				if(abs(m_edge_data_1[1]-m_edge_data_1[3]) > abs(m_edge_data_1[2]-m_edge_data_1[0])){
					m_route_mid_1 = (m_edge_data_1[1]+m_edge_data_1[3])/2;
				}else{
					m_route_mid_1 = (m_edge_data_1[2]+m_edge_data_1[0])/2;
				}
			}
//			printf("Obstacle\n");
//			m_car.m_led.SetEnable(false);
			m_car.m_led2.SetEnable(true);
			m_car.m_led3.SetEnable(true);
//			m_car.m_led4.SetEnable(false);
		}*/

//		if(m_state!=m_last_print_state){
//			m_last_print_state = m_state;
//			static char * enumStrings[] = {};
//			printf("%s\n",enumStrings[m_state]);
//		}


		/*
		 * Change trust based on error
		 */
		m_nowMid[1] = m_route_mid_1;
		m_route_mid_1 = Get_mid(m_ccd_data_1.data(), 1, mid_data1);


//		m_turn_error_1 = ((int)m_mid - (int)m_route_mid_1);
//		m_turn_error_2 = ((int)m_mid - (int)m_route_mid_2);
//		m_trust = abs(m_turn_error_1)/5 * 1.0f/* + m_actual_bal_error / 5.0f * 0.6f*/;
//		m_trust = 1.0f;
		/*
		 * Change trust of closer CCD based on width of the closer CCD
		 * So that the car won't go out of boundary
		 */

		int16_t width_1 = (int16_t)m_edge_data_1[1] - (int16_t)m_edge_data_1[0];
		int16_t width_2 = (int16_t)m_edge_data_2[1] - (int16_t)m_edge_data_2[0];


		/*
		 * Clamp trust so that must sum up to 1
		 */
		m_trust = libutil::Clamp<float>(0.0f,m_trust,1.0f);
		if (m_state == TURN1 || m_state == OBSTACLE)
			m_trust = 1.0f;
		else if (m_state == TURN2)
			m_trust = 0.0f;
//		m_turn_error = (int)(m_trust * m_turn_error_1 + (1.0f-m_trust) * m_turn_error_2);
		m_turn_error = ((int)m_mid - (int)m_route_mid_1);

		/*
		 * Obstacle detection
		 */
/*		if((m_found_obstacle_2) && m_hold_count==0){
			m_hold_error = (int)(m_mid - m_route_mid_2);
			m_hold_count = 8;
//			m_car.m_buzzer.SetBeep(true);
		}
		if((m_found_obstacle_1) && m_hold_count==0){
			m_hold_error = (int)2*(m_mid - m_route_mid_1);
			m_hold_count = 8;
//			m_car.m_buzzer.SetBeep(true);
		}*/

//		m_total_white_2 = 0;
//		m_total_white_1 = 0;
//		m_right_white = 0;
//		m_left_white = 0;
//		m_total_black_1 = 0;
//
//		for(int i=15; i<libsc::Tsl1401cl::kSensorW-15; i++){
//			if(m_ccd_data_1[i] > libutil::Clamp<uint16_t>(0,m_avg,255-255/4)){
//				m_total_white_1++;
//			}
//			if(m_ccd_data_1[i] < libutil::Clamp<uint16_t>(255/4,m_avg,255)){
//				m_total_black_1++;
//			}
//			if(m_ccd_data_1[i] > m_threshold_1){
//				if(i>=(127-35)){
//					m_right_white++;
//				}
//				if(i<=35){
//					m_left_white++;
//				}
//			}
//		}
//		for(int i=20; i<libsc::Tsl1401cl::kSensorW-20; i++){
//			if(m_ccd_data_2[i] > m_threshold_2){
//				m_total_white_2++;
//			}
//		}
//
//		if(m_total_white_1 == 98 && m_state != MIDDLELINE){
//			m_prev_state = m_state;
//			m_state = WHITE;
//		}

//		if(m_total_black_1 == 98 || m_total_white_2 == 0){
//			if(m_total_black_1 == 98){
//				m_edge_data_1[0] = 0;
//				m_edge_data_1[1] = 127;
//				m_edge_data_1[2] = 63;
//				m_edge_data_1[3] = 63;
//				m_route_mid_1 = (m_edge_data_1[0] + m_edge_data_1[1])/2;
//			}
//			if(m_total_white_2 == 0){
//				m_edge_data_2[0] = 0;
//				m_edge_data_2[1] = 127;
//				m_edge_data_2[2] = 63;
//				m_edge_data_2[3] = 63;
//				m_route_mid_2 = (m_edge_data_2[0] + m_edge_data_2[1])/2;
//			}
//			m_turn_error = 0;
////			m_hold_error = 0;
////			m_hold_count = 0;
//		}

		/*
		 * Cross road detection
		 */
//		if(m_total_white_1 >= 110)
//		{
//			m_turn_kp = 2.0f;
//			m_car.m_buzzer.SetBeep(true);
////			m_hold_error = 0;
////			m_hold_count = 7;
//			printf("Crossroad Detected\n");
//			m_car.m_led.SetEnable(true);
//			m_car.m_led2.SetEnable(true);
//			m_car.m_led3.SetEnable(true);
//			m_car.m_led4.SetEnable(true);
//		}

		/*
		 * 90 degree detection
		 */
//		if(m_total_black_1 == 98){
//			m_ccd_1_entered_black = true;
//			m_prev_state = m_state;
//			m_state = BLACK;
//			m_entered_black_line_time = m_pit_count;
//			printf("Detected Black\n");
//		}
//
//		if((m_left_white >= 20 || m_right_white >= 20) && m_total_white_1 >= 88 && m_ccd_1_entered_black && (m_pit_count - m_entered_black_line_time) > 150){
//			m_ccd_1_dropped_edge = true;
//			printf("CCD1 Dropped Edge\n");
//			m_prev_dropped_line_time = m_pit_count;
////			m_prev_state = m_state;
////			m_state = DROPPEDLINE;
//		}
//
///*		if((m_pit_count - m_prev_dropped_line_time) > 200 || (m_pit_count - m_entered_black_line_time) > 50){
////			m_ccd_2_dropped_edge = m_ccd_1_dropped_edge = false;
//			m_ccd_2_entered_black = m_ccd_1_entered_black = false;
//		}*/
//
//		if(m_ccd_1_dropped_edge){
//			m_ccd_1_dropped_edge = false;
//			m_ccd_1_entered_black = false;
//
//			if(m_left_white >= 20){
//				printf("90 turn left\n");
//				m_hold_error = (int)(5.0f*((int)m_mid - 0));
//				m_hold_count = 18;
//			}else if(m_right_white >= 20){
//				printf("90 turn right\n");
//				m_hold_error = (int)(5.0f*((int)m_mid - 127));
//				m_hold_count = 18;
//			}
//
//			m_car.m_buzzer.SetBeep(true);
////			printf("90 degree Detected\n");
////			m_car.m_led.SetEnable(false);
////			m_car.m_led2.SetEnable(false);
////			m_car.m_led3.SetEnable(true);
////			m_car.m_led4.SetEnable(true);
//		}


		if(m_hold_count > 0){
			m_hold_count--;
			if(m_hold_count==1){
				m_triggered_90 = false;
				m_car.m_buzzer.SetBeep(false);
//				m_hold_turn_l = m_hold_turn_r = 0;
				m_found_obstacle_2 = m_found_obstacle_1 = false;
				m_ccd_1_dropped_edge = false;
			}
/*			if(m_found_obstacle_2 || m_found_obstacle_1){
				if(m_hold_count == 2){
					m_hold_error = 0;
				}
			}*/
			m_turn_error = m_hold_error;
//			if(m_triggered_90){
//				m_turn_powerl = m_hold_turn_l;
//				m_turn_powerr = m_hold_turn_r;
//			}
		}


		if(m_car.m_car_move_forward){
			m_turn_powerl = (int16_t)(-((m_turn_kp+m_speedInMetrePerSecond*0.5f)*m_turn_error + (m_turn_kd+m_speedInMetrePerSecond*0.7f)*(m_turn_error - m_turn_prev_error)/0.02f));
//				m_turn_powerl = libutil::Clamp<int16_t>(-800,m_turn_powerl, 800);
			m_turn_powerr = (int16_t)(((m_turn_kp+m_speedInMetrePerSecond*0.5f)*m_turn_error + (m_turn_kd+m_speedInMetrePerSecond*0.7f)*(m_turn_error - m_turn_prev_error)/0.02f));
//				m_turn_powerr = libutil::Clamp<int16_t>(-800,m_turn_powerr, 800);
			m_turn_prev_error = m_turn_error;
		}else{
			m_turn_powerl = m_turn_powerr = 0;
		}

		m_prev_state = m_state;
		m_prev_avg = m_avg;

		m_prev_edge_data_1[0] = m_edge_data_1[0];
		m_prev_edge_data_1[1] = m_edge_data_1[1];
		m_prev_edge_data_1[2] = m_edge_data_1[2];
		m_prev_edge_data_1[3] = m_edge_data_1[3];

		m_prev_edge_data_2[0] = m_edge_data_2[0];
		m_prev_edge_data_2[1] = m_edge_data_2[1];
		m_prev_edge_data_2[2] = m_edge_data_2[2];
		m_prev_edge_data_2[3] = m_edge_data_2[3];

		if(m_car.m_lcdupdate){
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
//				rect_.y = 159-m_ccd_data_1[i]/4;
				rect_.y = 156;
				rect_.w = 1;
				rect_.h = 4;
				m_car.m_lcd.SetRegion(rect_);
				color = ~0;
				if(m_color[i]==CCD_BLACK){
					color = 0;
				}
				for(int j=0; j<m_regionTotalNumber[1]; j++){
					if(i==mid_data1[j]){
						color = 0b1111100000000000;
					}
				}
				if(i==m_route_mid_1){
					color = 0b0000000000011111;
				}
				m_car.m_lcd.FillColor(color);
				m_last_y[i] = 159-m_ccd_data_1[i]/4;
			}

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

			m_total_speed += (0.1f*m_speed_error + 0.9f * m_prev_speed) * speedDt;
			m_total_speed = libutil::Clamp<float>(-40.0f,m_total_speed,40.0f);

//			m_speed_output = (int16_t)(speedKp * m_speed_error + speedKi * m_total_speed);
			float a = speedKp + speedKd / speedDt + speedKi * speedDt;
			float b = -speedKp - 2*speedKd / speedDt;
			float c = speedKd / speedDt;
			
//			m_speed_output += (int16_t)(a * m_speed_error + b * m_prev_speed + c * m_prev_speed_2);

			// Avoid acceleration over 3ms-2
			float maxAcceleration = 0.0f;
			if(m_speedInMetrePerSecond <= m_speed_setpoint*0.6f){
				maxAcceleration = 2.0f;
			}else{
				maxAcceleration = 2.0f;
			}

//			Affect balance only when speed is > setpoint * 80%
//			if(m_speedInMetrePerSecond >= m_speed_setpoint*0.6f){

//			Otherwise, use angle to do acceleration for maximum acceleration
//			}else{

					m_acceleration = libutil::Clamp<float>(-maxAcceleration*2,(0.01f * (0.35f*m_speed_error + 0.65f * m_prev_speed) + 0.0f * (m_speed_error-m_prev_speed)/0.02f + 0.3f * m_total_speed * 0.02f),maxAcceleration);
					m_prev_speed_2 = m_prev_speed;
					m_prev_speed = (0.35f*m_speed_error + 0.65f * m_prev_speed);

//					m_prev_speed = (m_speed_setpoint-m_speedInMetrePerSecond)/0.02;
//					m_prev_speed = 0.3f * 0.5f * (m_acceleration - m_prev_speed) + 0.5f * (m_prev_speed);
//					m_prev_speed = - 0.8f*m_speedInMetrePerSecond/0.02f + 0.2f * m_prev_speed;
//					m_acceleration = m_acceleration + 0.001f * m_total_speed;
					m_balcon[6] = -atan(m_acceleration/9.81f)*RAD2ANGLE;
//					m_balcon[6] = atan((0.15f*m_acceleration + 0.85f * m_prev_speed)/9.81f)*RAD2ANGLE;
//					m_prev_speed = (0.15f*m_acceleration + 0.85f * m_prev_speed);
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

		switch(m_car.m_print_state){
			case 1:
				printf("%f,%f\n",m_speed_setpoint,m_speedInMetrePerSecond);
				break;
			case 2:
				printf("%f,%f,%f,%f,%f\n",m_car.m_shift_balance_angle, m_real_angle,m_upstand->GetAccAngle(),m_upstand->GetGyroAngle(), m_actual_bal_error);
				break;
			case 3:
				printf("%d,%d,%d,%d,%f,%f\n",m_power_l_pwm,m_power_r_pwm,m_car.m_encoder_countr, m_car.m_encoder_countl,m_balcon[6],m_speedInMetrePerSecond);
				break;
			case 4:
				printf("%d,%d\n",m_edge_data_2[1] - m_edge_data_2[0],m_edge_data_1[1] - m_edge_data_1[0]);
				break;
			case 5:
				printf("E%d,%d\n",m_edge_data_2[1] - m_edge_data_2[0],m_edge_data_1[1] - m_edge_data_1[0]);
				break;
			case 6:
				printf("%d,%d,%d,%d,%d,%d\n",m_edge_data_2[0], m_edge_data_2[1],m_edge_data_2[2],m_edge_data_1[0], m_edge_data_1[1],m_edge_data_1[2]);
				break;
			case 7:
				printf("%d %d\n",(m_edge_data_2[1] + m_edge_data_2[0])/2,(m_edge_data_1[1] + m_edge_data_1[0])/2);
				break;
			case 8:
				printf("%d %d\n", (m_mid - m_route_mid_1), (m_mid - m_route_mid_2));
				break;
			case 9:
				printf("%d %d %d %d %d %d %d\n",m_turn_powerl,m_turn_powerr, m_power_r, m_power_l, m_balance_pid_output, m_power_r_pwm, m_power_l_pwm);
				break;
			case 10:
				printf("%d %d %d %d\n", m_total_white_1, m_total_white_2, m_left_white, m_right_white);
				break;
			case 11:
				printf("%d\n",m_total_black_1);
				break;
			case 12:
				{
					Byte buf[128 + 2];
					buf[0] = 'L';
					for(int i=0; i<128; i++){
						buf[i + 1] = m_ccd_data_1[i];
					}
					buf[sizeof(buf) - 1] = '\n';
					m_car.m_com->SendBuffer(buf, sizeof(buf));
				}
				break;
			case 0:
			default:
				break;
		}
	}



/*	if(m_turn_powerr < 0 && !m_triggered_90){
		m_turn_powerr = 0;
	}
	if(m_turn_powerl < 0 && !m_triggered_90){
		m_turn_powerl = 0;
	}*/
	m_power_l = m_balance_pid_output /*- m_speed_output */+ m_turn_powerr;
	m_power_r = m_balance_pid_output /*- m_speed_output */+ m_turn_powerl;

	m_power_l_pwm = (int16_t)(1.25f * m_power_l);
	m_power_r_pwm = (int16_t)(1.0f * m_power_r);

	if(m_stop->GetInt()!=0){
		m_car.m_car_move_motor = false;
	}else if(m_start->GetInt()!=0){
		m_car.m_car_move_forward = true;
	}

	if(!m_car.m_car_move_motor){
		m_power_l_pwm = m_power_r_pwm = 0;
		m_power_l = m_power_r = 0;
	}

	m_car.m_motor_l.SetClockwise(m_power_l_pwm < 0); //Right Motor - false forward, true backward
	m_car.m_motor_r.SetClockwise(m_power_r_pwm > 0); //Left Motor - true forward, false backward
	m_car.m_motor_l.SetPower((uint16_t)abs(m_power_l_pwm));
	m_car.m_motor_r.SetPower((uint16_t)abs(m_power_r_pwm));

	m_pit_count++;
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
	m_hold_turn_l(0),
	m_hold_turn_r(0),
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
	m_turn_error_1(0),
	m_turn_error_2(0),
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
	m_ccd_1_dropped_edge(false),
	m_entered_black_angle(0.0f),
	m_turn_kp(5.0f),
	m_turn_kd(0.05f),
	m_found_middle_line(false),
	m_found_obstacle_1(false),
	m_found_obstacle_2(false),
	m_found_edges(false),
	m_state(UNKNOWN),
	m_prev_state(UNKNOWN),
	m_last_print_state(UNKNOWN),
	m_trust(0.0f),
	m_total_white_2(0),
	m_total_white_1(0),
	m_left_white(0),
	m_right_white(0)
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
//	m_threshold_1 = 100;

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
//	m_threshold_2 = 100;

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

	while(true)
	{
	}

}


