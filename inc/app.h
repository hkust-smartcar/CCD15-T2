/*
 * app.h
 *
 *  Created on: Mar 3, 2015
 *      Author: harrison
 */

#pragma once
#include "car.h"
#include <libutil/positional_pid_controller.h>

using namespace libutil;

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
	int16_t Output_b(float* balcon, float* balpid, uint16_t* time, float real_angle);
	float Output_speed(int16_t* carspeedcon, float* carspeedpid, int16_t encoder);
	void Update_edge(uint8_t* ccd_data_, uint8_t* edge);
private:
	Car m_car;
	int16_t m_balance_pid_output;

	RemoteVarManager::Var* m_skp = m_car.m_varmanager->Register("skp",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* m_skd = m_car.m_varmanager->Register("skd",RemoteVarManager::Var::Type::kReal);
	RemoteVarManager::Var* m_ski = m_car.m_varmanager->Register("ski",RemoteVarManager::Var::Type::kReal);

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

	PositionalPidController<int16_t,int16_t> m_speed_control0;
	PositionalPidController<int16_t,int16_t> m_speed_control1;
};

