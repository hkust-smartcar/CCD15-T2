/*
 * DebugApp.h
 *
 *  Created on: Mar 31, 2015
 *      Author: harrison
 */
#pragma once
#include <car.h>
#include <libsc/lcd_typewriter.h>


using namespace libsc;
using namespace libutil;

class DebugApp {
public:
	DebugApp();
	~DebugApp();
private:
	Car m_car;
	LcdTypewriter m_lcd_typewriter;
	LcdTypewriter::Config getLcdTypewriterConfig(){
		LcdTypewriter::Config config;
		config.is_text_wrap = false;
		config.lcd = &(m_car.m_lcd);
		return config;
	}
};
