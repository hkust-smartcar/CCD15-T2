/*
 * app.h
 *
 *  Created on: Mar 3, 2015
 *      Author: harrison
 */

#pragma once
#include "car.h"
#include <libsc/lcd_typewriter.h>
#include <libutil/string.h>
#include <libutil/positional_pid_controller.h>
#include <libbase/kl26/pit.h>
#include "upstand.h"
#include "moving_average.h"

using namespace libutil;
using namespace libbase::kl26;

class App{
public:
	App();
	int sign(int x){
		return (x>0) -  (x<0);
	}
	void Update_edge(uint16_t* m_ccd_data, uint16_t* edge_data);
	uint16_t RpmToPwm_R(uint16_t count);
	uint16_t RpmToPwm_L(uint16_t count);
	int16_t Output_s0(int16_t spdcon[5], uint8_t pid[3], uint16_t time[2]);
	int16_t Output_s1(int16_t spdcon[5], uint8_t pid[3], uint16_t time[2]);
	int16_t Output_b(float* balcon, float* balpid, uint16_t* time, float real_angle, float  omega);
	float Output_turning(int16_t* turncon, float* turnpid, uint16_t* time);
	int16_t Output_speed(int16_t* carspeedcon, float* carspeedpid, int16_t encoder);
	void PitBalance(Pit* pit);
	void PitMoveMotor(Pit* pit);
private:
	Car m_car;
	LcdTypewriter m_lcd_typewriter;
	int16_t m_balance_pid_output;

	RemoteVarManager::Var* m_bkp = m_car.m_varmanager->Register("bkp",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* m_bkd = m_car.m_varmanager->Register("bkd",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* m_stop = m_car.m_varmanager->Register("stop",RemoteVarManager::Var::Type::kInt);


	Gpo* m_pin;

	std::array<float, 3> m_accel_, m_gyro_;
	float m_real_angle = 0, m_acc_angle = 0, m_gyro_angle = 0, m_prev_gyro_angle = 0, m_avg_gyro = 0, m_total_gyro=0;

	Upstand* m_upstand;

	int16_t m_power_l=0, m_power_r=0, m_u_s0=0, m_u_s1=0, m_u_b=0, m_turn_powerl=0, m_turn_powerr=0;
	int16_t m_power_l_pwm=0, m_power_r_pwm=0;
	int16_t m_turn_powerb=0,m_turn_powerf=0;
	float m_turn_coeff_b,m_turn_coeff_f;
	int16_t m_speedsp = 0;
	/* edge_data[0] = left_edge;
	 * edge_data[1] = right_edge;
	 */
	uint16_t m_prev_edge_data_1[2] = {0,127};
	uint16_t m_edge_data_1[2] = {0,127};
	uint16_t m_route_mid_1;
	uint16_t prev_edge_data_2[2] = {0,127};
	uint16_t edge_data_2[2] = {0,127};
	uint16_t m_route_mid_2;
	uint16_t m_avg = 0;
	uint32_t m_sum = 0;
	uint16_t m_threshold_1, m_threshold_2;

	int16_t m_mid = 64;
	int16_t m_encoder_count_t;

	uint32_t m_pit_count = 0, m_pit_count2 = 0;

	/*balcon[0]=error(k);
	 * balcon[1]=error(k-1);
	 * balcon[2]=previous slope of de/dt for low-pass filtering;
	 * balcon[3]=summation for ki;
	 *  balcon[4]=setpoint
	 *  balcon[5]=setpoint offset
	*/
	float m_balcon[7]={0,0,0,0,27.6f,0,0};

	/*pid[0]=kp;
	 * pid[1]=ki;
	 * pid[2]=kd;
	 */
	float m_balpid[3]={0.0f,0.0f,0.0f};

	/*carspeedcon[0]=error(k);
	 * carspeedcon[1]=error(k-1);
	 * carspeedcon[2]=previous slope of de/dt for low-pass filtering;
	 * carspeedcon[3]=summation for ki;
	 * carspeedcon[4]=setpoint;
	*/
	int16_t m_carspeedconr[5]={0,0,0,0,0};

	/*carspeedpid[0]=kp;
	* carspeedpid[1]=ki;
	* carspeedpid[2]=kd;
	*/
	float m_carspeedpidr[3]={0.1f,0.0f,0.0f};

	/*carspeedcon[0]=error(k);
	 * carspeedcon[1]=error(k-1);
	 * carspeedcon[2]=previous slope of de/dt for low-pass filtering;
	 * carspeedcon[3]=summation for ki;
	 * carspeedcon[4]=setpoint;
	*/
	int16_t m_carspeedconl[5]={0,0,0,0,0};

	/*carspeedpid[0]=kp;
	* carspeedpid[1]=ki;
	* carspeedpid[2]=kd;
	*/
	float m_carspeedpidl[3]={0.1f,0.0f,0.0f};

	float m_prevSpeedInMetrePerSecond,m_speedInMetrePerSecond,m_prev_speed;

	float m_acceleration;

	float m_total_speed;

	float m_speed_setpoint;

	/* time[0] for spd period;
	 * time[1] for spd period;
	 * time[2] for bal period;
	 * time[3] for bal period;
	 * m_time[4] for turning period
	*/
	uint16_t m_time[6]={0,0,0,0,0,0};
	uint16_t m_time2[5]={0,0,0,0,0};

	/* turncon[0]=error(k);
	 * turncon[1]=error(k-1);
	 * turncon[2]=previous slope of de/dt for low-pass filtering;
	 * turncon[3]=midpoint;
	 */
	bool m_crossing=0;
	int16_t m_turncon_f[4]={0,0,0,libsc::Tsl1401cl::kSensorW-1};
	int16_t m_turncon_b[4]={0,0,0,libsc::Tsl1401cl::kSensorW-1};

	/*pid[0]=kp;
	 * pid[1]=ki;
	 * pid[2]=kd;
	 */
	float m_turnpid_f[3]={0.0f,0.0f,0.0f};
	float m_turnpid_b[3]={1.0f,0.0f,1.0f};

	bool m_triggered_90;

	int m_turn_prev_error;
	int m_hold_error;
	int m_hold_count;
	int m_prev_pit_count;

	uint16_t m_last_y[128]={0};
	uint16_t m_last_y2[128]={0};

	MovingAverage<int16_t> m_movavgspeed,m_movavgr,m_movavgl;
	MovingAverage<int> m_movavgturn;
	MovingAverage<float> m_movavgspeed_output;

	enum CCD_COLOR{
		CCD_BLACK = 0,
		CCD_WHITE
	};
	std::array<uint16_t,libsc::Tsl1401cl::kSensorW> m_ccd_data_1, m_ccd_data_raw;

	std::array<uint16_t,libsc::Tsl1401cl::kSensorW> m_ccd_data_2;


	std::array<CCD_COLOR,libsc::Tsl1401cl::kSensorW> m_color;
	int m_y = 0;

	LcdTypewriter::Config GetLcdTypewriterConfig(){
		LcdTypewriter::Config config;
		config.is_text_wrap = false;
		config.lcd = &(m_car.m_lcd);
		return config;
	}



//	Kalman m_encoder_r_filter, m_encoder_l_filter;
};

