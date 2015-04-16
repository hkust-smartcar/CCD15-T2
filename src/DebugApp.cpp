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
	}

}

DebugApp::~DebugApp() {
	// TODO Auto-generated destructor stub
}

