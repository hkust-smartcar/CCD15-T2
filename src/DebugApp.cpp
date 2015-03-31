/*
 * DebugApp.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: harrison
 */

#include "DebugApp.h"
#include <libsc/system.h>
#include <libutil/string.h>

DebugApp::DebugApp():
	m_car(),
	m_lcd_typewriter(getLcdTypewriterConfig()){
	// TODO Auto-generated constructor stub
	Timer::TimerInt t = System::Time();
	Timer::TimerInt pt = t;
	while(true){
		t = System::Time();
		if(t-pt>1){
			pt = t;

			if(t%1000==0){
				m_car.m_encoder0.Update();
				m_car.m_encoder1.Update();
//				m_lcd_typewriter.WriteString(String::Format("encoder:").c_str());
//				m_lcd_typewriter.WriteString(String::Format("encoder: %d %d\n",m_car.m_encoder0.GetCount(), m_car.m_encoder1.GetCount()).c_str());
				printf("encoder: %d %d\n",m_car.m_encoder0.GetCount(), m_car.m_encoder1.GetCount());
				m_car.m_led.Switch();
				m_car.m_led2.Switch();
				m_car.m_led3.Switch();
				m_car.m_led4.Switch();
			}
		}
	}
}

DebugApp::~DebugApp() {
	// TODO Auto-generated destructor stub
}

