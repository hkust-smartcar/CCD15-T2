//#include <hidef.h>
//#include "derivative.h"
#pragma once
#include <libbase/helper.h>
#include <libbase/kl26/gpio.h>
#include <libbase/kl26/spi_master.h>
using namespace LIBBASE_NS;

#define CS_PIN libbase::kl26::Pin::Name::kPtc4
#define SCK_PIN  libbase::kl26::Pin::Name::kPtc5
#define SOUT_PIN  libbase::kl26::Pin::Name::kPtc6
#define DC_PIN libbase::kl26::Pin::Name::kPtc7
#define RST_PIN libbase::kl26::Pin::Name::kPtc8


//#define CLK           PTS_PTS7       //接D0
//#define MOSI          PTS_PTS6       //接D1
//#define RST           PTS_PTS5       //接RST
//#define DC            PTS_PTS4       //接DC
//#define OLED_dir      DDRS

#define LED_IMAGE_WHITE       1
#define LED_IMAGE_BLACK       0

#define LED_MAX_ROW_NUM      64
#define LED_MAX_COLUMN_NUM  128

void OLED_WrDat(unsigned char Data);
void OLED_WrCmd(unsigned char Data);
void OLED_SetPos(unsigned char X, unsigned char Y);
void OLED_Fill(unsigned char ucData);
void SetAddressingMode(unsigned char ucData);
void SetColumnAddress(unsigned char a, unsigned char b);
void SetPageAddres(unsigned char a, unsigned char b);
void SetStartLine(unsigned char ucData);
void SetContrastControl(unsigned char ucData);
void SetChargePump(unsigned char ucData);
void SetSegmentRemap(unsigned char ucData);
void SetEntireDisplay(unsigned char ucData);
void SetInverseDisplay(unsigned char ucData);
void SetMultiplexRatio(unsigned char ucData);
void SetDisplayOnOff(unsigned char ucData);
void SetStartPage(unsigned char ucData);
void SetCommonRemap(unsigned char ucData);
void SetDisplayOffset(unsigned char ucData);
void SetDisplayClock(unsigned char ucData);
void SetPrechargePeriod(unsigned char ucData);
void SetCommonConfig(unsigned char ucData);
void SetVCOMH(unsigned char ucData);
void SetNop(void);
void OLED_Init(void);

void OLED_6x8Char(unsigned char X1, unsigned char Y1, unsigned char Data1);
void OLED_6x8Str(unsigned char X1, unsigned char Y1, unsigned char ucStr[]);
void OLED_8x16Char(unsigned char X1, unsigned char Y1, unsigned char Data1);
void OLED_16x16Char(unsigned char X1,unsigned char Y1,unsigned char c[2]);
void OLED_PutString(unsigned char X1,unsigned char Y1,unsigned char *s); 
void OLED_image(void);
void OLED_PutPixel(unsigned char x,unsigned char y);
void OLED_Rectangle(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2);
