#include "OLED.h"
#include "6x8.h"
#include "8x16.h"
#include "GB1616.h"
#include "picture.h"

namespace{

SpiMaster::Config GetSpiConfig2(){
	SpiMaster::Config config;
	config.sck_pin = SCK_PIN;
	config.sout_pin = SOUT_PIN;
	config.pcs_pin  = CS_PIN;
	config.baud_rate_khz = 10000;
	config.is_sck_capture_first = true;
	config.is_sck_idle_low = false;
	return config;
}

SpiMaster* spi;
LIBBASE_MODULE(Gpo)* rst;
LIBBASE_MODULE(Gpo)* dc;

}

/*************************************************************/
/*                    向OLED屏写入数据                       */
/*************************************************************/
void OLED_WrDat(unsigned char Data)
{
    unsigned char j = 8;
    dc->Set();
    spi->ExchangeData(0, (uint16_t)Data);
}

/*************************************************************/
/*                    向OLED屏写入命令                       */
/*************************************************************/
void OLED_WrCmd(unsigned char Data)
{
    dc->Clear();
    spi->ExchangeData(0, (uint16_t)Data);
}


/*************************************************************/
/*                          清屏命令                         */
/*************************************************************/
void OLED_Fill(unsigned char ucData)
{
    unsigned char ucPage,ucColumn;
    
    for(ucPage = 0; ucPage < 8; ucPage++)
    {
        OLED_WrCmd(0xb0 + ucPage);  //0xb0+0~7表示页0~7
        OLED_WrCmd(0x00);           //0x00+0~16表示将128列分成16组其地址在某组中的第几列
        OLED_WrCmd(0x10);           //0x10+0~16表示将128列分成16组其地址所在第几组
        for(ucColumn = 0; ucColumn < 128; ucColumn++)
        {
            OLED_WrDat(ucData);
        }
    }
} 


/*************************************************************/
/*             设定显示坐标，X,Y分别为横坐标和纵坐标         */
/*************************************************************/
void OLED_SetPos(unsigned char X, unsigned char Y)
{ 
    OLED_WrCmd(0xb0 + Y);
    OLED_WrCmd(((X & 0xf0) >> 4) | 0x10);
    OLED_WrCmd((X & 0x0f) | 0x00); 
} 

//以下初始化的函数是由OLED厂家提供的初始化过程
//使用时可以不必理会这些函数的具体意义
//如果做一些高级应用，请参照OLED屏的手册进行分析
void SetAddressingMode(unsigned char ucData)
{
    OLED_WrCmd(0x20);        // Set Memory Addressing Mode
    OLED_WrCmd(ucData);      // Default => 0x02
                            // 0x00 => Horizontal Addressing Mode
                            // 0x01 => Vertical Addressing Mode
                            // 0x02 => Page Addressing Mode
}

void SetColumnAddress(unsigned char a, unsigned char b)
{
    OLED_WrCmd(0x21);        // Set Column Address
    OLED_WrCmd(a);           // Default => 0x00 (Column Start Address)
    OLED_WrCmd(b);           // Default => 0x7F (Column End Address)
}

void SetPageAddres(unsigned char a, unsigned char b){
	 OLED_WrCmd(0x22);        // Set Page Address
	 OLED_WrCmd(a);           // Default => 0x00 (Column Start Address)
	 OLED_WrCmd(b);           // Default => 0x7F (Column End Address)
}

void SetStartLine(unsigned char ucData)
{
    OLED_WrCmd(0x40|ucData); // Set Display Start Line
                            // Default => 0x40 (0x00)
}

void SetContrastControl(unsigned char ucData)
{
    OLED_WrCmd(0x81);        // Set Contrast Control
    OLED_WrCmd(ucData);      // Default => 0x7F
}

void SetChargePump(unsigned char ucData)
{
    OLED_WrCmd(0x8D);        // Set Charge Pump
    OLED_WrCmd(0x10|ucData); // Default => 0x10
                            // 0x10 (0x00) => Disable Charge Pump
                            // 0x14 (0x04) => Enable Charge Pump
}

void SetSegmentRemap(unsigned char ucData)
{
    OLED_WrCmd(0xA0|ucData); // Set Segment Re-Map
                            // Default => 0xA0
                            // 0xA0 (0x00) => Column Address 0 Mapped to SEG0
                            // 0xA1 (0x01) => Column Address 0 Mapped to SEG127
}

void SetEntireDisplay(unsigned char ucData)
{
    OLED_WrCmd(0xA4|ucData); // Set Entire Display On / Off
                            // Default => 0xA4
                            // 0xA4 (0x00) => Normal Display
                            // 0xA5 (0x01) => Entire Display On
}

void SetInverseDisplay(unsigned char ucData)
{
    OLED_WrCmd(0xA6|ucData); // Set Inverse Display On/Off
                            // Default => 0xA6
                            // 0xA6 (0x00) => Normal Display
                            // 0xA7 (0x01) => Inverse Display On
}

void SetMultiplexRatio(unsigned char ucData)
{
    OLED_WrCmd(0xA8);        // Set Multiplex Ratio
    OLED_WrCmd(ucData);      // Default => 0x3F (1/64 Duty)
}

void SetDisplayOnOff(unsigned char ucData)
{
    OLED_WrCmd(0xAE|ucData); // Set Display On/Off
                            // Default => 0xAE
                            // 0xAE (0x00) => Display Off
                            // 0xAF (0x01) => Display On
}

void SetStartPage(unsigned char ucData)
{
    OLED_WrCmd(0xB0|ucData); // Set Page Start Address for Page Addressing Mode
                            // Default => 0xB0 (0x00)
}

void SetCommonRemap(unsigned char ucData)
{
    OLED_WrCmd(0xC0|ucData); // Set COM Output Scan Direction
                            // Default => 0xC0
                            // 0xC0 (0x00) => Scan from COM0 to 63
                            // 0xC8 (0x08) => Scan from COM63 to 0
}

void SetDisplayOffset(unsigned char ucData)
{
    OLED_WrCmd(0xD3);        // Set Display Offset
    OLED_WrCmd(ucData);      // Default => 0x00
}

void SetDisplayClock(unsigned char ucData)
{
    OLED_WrCmd(0xD5);        // Set Display Clock Divide Ratio / Oscillator Frequency
    OLED_WrCmd(ucData);      // Default => 0x80
                            // D[3:0] => Display Clock Divider
                            // D[7:4] => Oscillator Frequency
}

void SetPrechargePeriod(unsigned char ucData)
{
    OLED_WrCmd(0xD9);        // Set Pre-Charge Period
    OLED_WrCmd(ucData);      // Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
                            // D[3:0] => Phase 1 Period in 1~15 Display Clocks
                            // D[7:4] => Phase 2 Period in 1~15 Display Clocks
}

void SetCommonConfig(unsigned char ucData)
{
    OLED_WrCmd(0xDA);        // Set COM Pins Hardware Configuration
    OLED_WrCmd(0x02|ucData); // Default => 0x12 (0x10)
                            // Alternative COM Pin Configuration
                            // Disable COM Left/Right Re-Map
}

void SetVCOMH(unsigned char ucData)
{
    OLED_WrCmd(0xDB);        // Set VCOMH Deselect Level
    OLED_WrCmd(ucData);      // Default => 0x20 (0.77*VCC)
}

void SetNop(void)
{
    OLED_WrCmd(0xE3);        // Command for No Operation
}

/*************************************************************/
/*                      初始化OLED屏                         */
/*************************************************************/
void OLED_Init(void)        
{
	Gpo::Config rstconfig;
	rstconfig.pin = RST_PIN;
	rst = new Gpo(rstconfig);

	Gpo::Config dcconfig;
	dcconfig.pin = DC_PIN;
	dc = new Gpo(dcconfig);

    unsigned char i;
    rst->Clear();
    
    for(i=0;i<100;i++)
    {
        asm("nop");         //等待屏复位
    }
    
    rst->Set();

    spi = new SpiMaster(GetSpiConfig2());


    SetDisplayOnOff(0x00);     // Display Off (0x00/0x01)
    SetDisplayClock(0x80);     // Set Clock as 100 Frames/Sec
    SetMultiplexRatio(0x3F);   // 1/64 Duty (0x0F~0x3F)
    SetDisplayOffset(0x00);    // Shift Mapping RAM Counter (0x00~0x3F)
    SetStartLine(0x00);        // Set Mapping RAM Display Start Line (0x00~0x3F)
    SetChargePump(0x04);       // Enable Embedded DC/DC Converter (0x00/0x04)
    SetAddressingMode(0x02);   // Set Page Addressing Mode (0x00/0x01/0x02)
    SetSegmentRemap(0x01);     // Set SEG/Column Mapping     0x00左右反置 0x01正常
    SetCommonRemap(0x08);      // Set COM/Row Scan Direction 0x00上下反置 0x08正常
    SetCommonConfig(0x10);     // Set Sequential Configuration (0x00/0x10)
    SetContrastControl(0xCF);  // Set SEG Output Current
    SetPrechargePeriod(0xF1);  // Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    SetVCOMH(0x40);            // Set VCOM Deselect Level
    SetEntireDisplay(0x00);    // Disable Entire Display On (0x00/0x01)
    SetInverseDisplay(0x00);   // Disable Inverse Display On (0x00/0x01)
    SetDisplayOnOff(0x01);     // Display On (0x00/0x01)
    OLED_Fill(0x00);           // 清屏
    OLED_SetPos(0,0);
} 

void OLED_PutPixel(unsigned char x,unsigned char y)
{
	unsigned char data1;  //data1当前点的数据

	OLED_SetPos(x,y);
	data1 = 0x01<<(y%8);
	OLED_WrCmd(0xb0+(y>>3));
	OLED_WrCmd(((x&0xf0)>>4)|0x10);
	OLED_WrCmd((x&0x0f)|0x00);
	OLED_WrDat(data1);
}

void OLED_Rectangle(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2)
{
	unsigned char n;

	OLED_SetPos(x1,y1>>3);
	for(n=x1;n<=x2;n++)
	{
		OLED_WrDat(0x01<<(y1%8));
//		if(gif == 1) 	delay_ms(50);
	}
	OLED_SetPos(x1,y2>>3);
  for(n=x1;n<=x2;n++)
	{
	  OLED_WrDat(0x01<<(y2%8));
//		if(gif == 1) 	delay_ms(5);
	}

}
 
/*************************************************************/
/*                     显示6X8的字符                         */
/*************************************************************/
void OLED_6x8Char(unsigned char X1, unsigned char Y1, unsigned char Data1)
{
    unsigned char xx, temp;     
       
    temp = Data1-32;
    if(X1 > 122)
    {
        X1 = 0;
        Y1++;
    }
    
    OLED_SetPos(X1, Y1);
    
    for(xx = 0; xx < 6; xx++)
    {     
        OLED_WrDat(F6x8[temp][xx]);  
    }
}

/*************************************************************/
/*                     显示6X8的字符串                       */
/*************************************************************/
void OLED_6x8Str(unsigned char X1, unsigned char Y1, unsigned char *ucStr)
{
    unsigned char yy, temp1; 

    while(*ucStr)
    {
        temp1 = *ucStr - 32;
        if(X1 > 122)
        {
            X1 = 0;
            Y1++;
        }
        
        OLED_SetPos(X1, Y1);
        
        for(yy = 0; yy < 6; yy++)
        {     
            OLED_WrDat(~(F6x8[temp1][yy]));
        }
        X1 += 6;
        ucStr++;
    }
        
}



/*************************************************************/
/*                     显示8X16的字符串                      */
/*************************************************************/
void OLED_8x16Char(unsigned char X1, unsigned char Y1, unsigned char Data1)
{
    unsigned char yy, temp1;

    Y1 <<= 1;
    
    temp1 = Data1 - 32;
    if(X1 > 120)
    {
        X1 = 0;
        Y1 += 2;
    }
    OLED_SetPos(X1, Y1);   
    
    for(yy = 0; yy < 8; yy++) 
    {
        OLED_WrDat(F8X16[(temp1 << 4) + yy]);
    }
    
    OLED_SetPos(X1, Y1 + 1);   
    
    for(yy = 0; yy < 8; yy++) 
    {
        OLED_WrDat(F8X16[(temp1 << 4) + yy + 8]);
    }
    X1 += 8;

}

/*************************************************************/
/*                     显示16X16的字符                       */
/*************************************************************/
void OLED_16x16Char(unsigned char X1,unsigned char Y1,unsigned char c[2])
{
    unsigned char xx, k;

    Y1 <<= 1;
    if(X1 > 112)
    {
        X1 = 0;
        Y1 += 2;
    }
    
    /*for (k=0;k<64;k++) //64标示自建汉字库中的个数，循环查询内码
    { 
        if ((codeGB_16[k].Index[0]==c[0])&&(codeGB_16[k].Index[1]==c[1]))
        { 

            OLED_SetPos(X1, Y1);
            for(xx = 0; xx < 16; xx++)               
                OLED_WrDat(codeGB_16[k].Msk[xx]);   
            
            OLED_SetPos(X1,Y1 + 1); 
            for(xx = 0;xx <16;xx++)          
                OLED_WrDat(codeGB_16[k].Msk[16+xx]);
            break;
        }
    }	*/

}

/*************************************************************/
/*                         显示字符串                        */
/*************************************************************/
void OLED_PutString(unsigned char X1,unsigned char Y1,unsigned char *s) 
{
    unsigned char l=0;
    while(*s) 
    {
        if( *s < 0x80) 
        {
            OLED_8x16Char(X1,Y1,*s);
            s++;X1+=8;
            if(X1 > 120)
            {
                X1 = 0;
                Y1+=1;
            }

        }
        else
        {
            OLED_16x16Char(X1,Y1,(unsigned char*)s);
            s+=2;X1+=16;
            if(X1 > 112)
            {
                X1 = 0;
                Y1+=1;
            }
        }
    }
}

/*************************************************************/
/*                       OLED显示图片                        */
/*************************************************************/
void OLED_image(void) 
{
//	OLED_SetPos(0,0);
/*	for(int i=0; i<64; i++){
		SetColumnAddress((unsigned char)i, (unsigned char)(i));
		SetPageAddres((unsigned char)(i/8), (unsigned char)(i/8));
		OLED_WrDat((unsigned char)(1<<(i%8)));
	}*/
//	OLED_WrDat(1);
//    unsigned int xx,yy;
//    for(yy=0;yy<8;yy++)
//    {
//        OLED_SetPos(0,yy);
//        for(xx=0;xx<128;xx++)
//            OLED_WrDat(dianbiao[yy*128+xx]);
//    }
} 

