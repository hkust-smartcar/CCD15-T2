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
	uint16_t RpmToPwm_R(uint16_t count);
	uint16_t RpmToPwm_L(uint16_t count);
	int16_t Output_s0(int16_t spdcon[5], uint8_t pid[3], uint16_t time[2]);
	int16_t Output_s1(int16_t spdcon[5], uint8_t pid[3], uint16_t time[2]);
	int16_t Output_b(float* balcon, float* balpid, uint16_t* time, float real_angle, float  omega);
	int16_t Output_turning(int16_t* turncon, float* turnpid, uint16_t* time);
	float Output_speed(int16_t* carspeedcon, float* carspeedpid, int16_t encoder);
	void Update_edge(uint8_t* ccd_data_, uint8_t* edge);
	void PitBalance(Pit* pit);
	void PitMoveMotor(Pit* pit);
private:
	Car m_car;
	LcdTypewriter m_lcd_typewriter;
	int16_t m_balance_pid_output;

	RemoteVarManager::Var* m_skp = m_car.m_varmanager->Register("skp",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* m_skd = m_car.m_varmanager->Register("skd",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* m_ski = m_car.m_varmanager->Register("ski",RemoteVarManager::Var::Type::kReal);

	RemoteVarManager::Var* m_skpl = m_car.m_varmanager->Register("skpl",RemoteVarManager::Var::Type::kReal);

	RemoteVarManager::Var* m_bkp = m_car.m_varmanager->Register("bkp",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* m_bkd = m_car.m_varmanager->Register("bkd",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* m_bki = m_car.m_varmanager->Register("bki",RemoteVarManager::Var::Type::kReal);

	RemoteVarManager::Var* m_boff = m_car.m_varmanager->Register("boff",RemoteVarManager::Var::Type::kReal);

	RemoteVarManager::Var* m_carkp = m_car.m_varmanager->Register("carkp",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* m_carkd = m_car.m_varmanager->Register("carkd",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* m_carki = m_car.m_varmanager->Register("carki",RemoteVarManager::Var::Type::kReal);

	RemoteVarManager::Var* m_carspeed = m_car.m_varmanager->Register("carspeed",RemoteVarManager::Var::Type::kReal);

	RemoteVarManager::Var* m_r1 = m_car.m_varmanager->Register("r1",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* m_r2 = m_car.m_varmanager->Register("r2",RemoteVarManager::Var::Type::kReal);

	RemoteVarManager::Var* m_q = m_car.m_varmanager->Register("q",RemoteVarManager::Var::Type::kInt);

	PositionalPidController<int16_t,int16_t> m_speed_control0;
	PositionalPidController<int16_t,int16_t> m_speed_control1;

	Gpo* pin;

	std::array<float, 3> accel_, gyro_;
	float real_angle = 0, acc_angle = 0, gyro_angle = 0, prev_gyro_angle = 0, avg_gyro = 0, total_gyro=0;

	Upstand* upstand;

	int16_t power_l=0, power_r=0, u_s0=0, u_s1=0, u_b=0, turn_powerl=0, turn_powerr=0;
	int16_t power_l_pwm=0, power_r_pwm=0;
	int16_t turn_powerb,turn_powerf,turn_coeff_b,turn_coeff_f
	int16_t speedsp = 0;

	int m_pit_count = 0, m_pit_count2 = 0;

	/*balcon[0]=error(k);
	 * balcon[1]=error(k-1);
	 * balcon[2]=previous slope of de/dt for low-pass filtering;
	 * balcon[3]=summation for ki;
	 *  balcon[4]=setpoint
	 *  balcon[5]=setpoint offset
	*/
	float balcon[7]={0,0,0,0,24.5f,0,0};

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
	int16_t carspeedconr[5]={0,0,0,0,0};

	/*carspeedpid[0]=kp;
	* carspeedpid[1]=ki;
	* carspeedpid[2]=kd;
	*/
	float carspeedpidr[3]={1.2f,0.04f,0.0f};

	/*carspeedcon[0]=error(k);
	 * carspeedcon[1]=error(k-1);
	 * carspeedcon[2]=previous slope of de/dt for low-pass filtering;
	 * carspeedcon[3]=summation for ki;
	 * carspeedcon[4]=setpoint;
	*/
	int16_t carspeedconl[5]={0,0,0,0,0};

	/*carspeedpid[0]=kp;
	* carspeedpid[1]=ki;
	* carspeedpid[2]=kd;
	*/
	float carspeedpidl[3]={1.5f,0.04f,0.0f};

	/* time[0] for spd period;
	 * time[1] for spd period;
	 * time[2] for bal period;
	 * time[3] for bal period;
	*/
	uint16_t time[4]={0,0,0,0};

	MovingAverage m_movavgr,m_movavgl;

	std::array<uint16_t,libsc::Tsl1401cl::kSensorW> ccd_data_;
	enum CCD_COLOR{
		CCD_BLACK = 0,
		CCD_WHITE
	};

	std::array<CCD_COLOR,libsc::Tsl1401cl::kSensorW> color;
	int y = 0;

	LcdTypewriter::Config GetLcdTypewriterConfig(){
		LcdTypewriter::Config config;
		config.is_text_wrap = false;
		config.lcd = &(m_car.m_lcd);
		return config;
	}



//	Kalman m_encoder_r_filter, m_encoder_l_filter;
};

