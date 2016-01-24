/*
 * main.cpp
 *
 * Author: Peter
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include <libsc/st7735r.h>
#include <libsc/lcd_console.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/futaba_s3010.h>
#include <libsc/alternate_motor.h>
#include <libsc/k60/jy_mcu_bt_106.h>



#define BLACK 0x0000
#define WHITE 0xFFFF





namespace libbase
{
	namespace k60
	{

		Mcg::Config Mcg::GetMcgConfig()
		{
			Mcg::Config config;
			config.external_oscillator_khz = 50000;
			config.core_clock_khz = 150000;
			return config;
		}

	}
}

using namespace libsc;
using namespace libbase::k60;

// pointers
LcdTypewriter *mylcdtypewriter = 0;
FutabaS3010 *myservo = 0;
AlternateMotor *mymotor = 0;
libsc::k60::JyMcuBt106 *mybluetooth = 0;

//function

//var
const char *string = "hihi";

bool listener(const Byte *, const size_t);

int main(void)
{
	System::Init();
    System::Time();

    //led
	Led::Config ledConfig;
	ledConfig.id = 3;
	ledConfig.is_active_low = false;
    Led myled(ledConfig);
    //lcd
    St7735r::Config st7735rConfig;
    St7735r mylcd(st7735rConfig);

    LcdTypewriter::Config lcdtypewriterConfig;
    lcdtypewriterConfig.lcd = &mylcd;
    LcdTypewriter mylcdtypewriter(lcdtypewriterConfig);

       /*
       St7735r *lcd = 0;
       LcdConsole::Config lcdConsoleConfig;
       lcdConsoleConfig.text_color = WHITE;
       lcdConsoleConfig.bg_color = BLACK;
       lcdConsoleConfig.lcd = lcd;
       LcdConsole myLcdConsole(lcdConsoleConfig);
     //cannot run this
      */

   //servo
    FutabaS3010::Config servoConfig;
    servoConfig.id = 0;
    FutabaS3010 myservo(servoConfig);
    int angle=600;
    myservo.SetDegree(angle);
   //motor
    AlternateMotor::Config motorConfig;
    motorConfig.id = 0;
    AlternateMotor mymotor(motorConfig);
    mymotor.SetClockwise(false);
    int speed = 0 ;
    mymotor.SetPower(speed);
    //buletooth
    /*libsc::k60::JyMcuBt106::Config BluetoothConfig;
    BluetoothConfig.id = 0;
    BluetoothConfig.baud_rate = Uart::Config::BaudRate::k115200;
    BluetoothConfig.tx_buf_size = 200;
    BluetoothConfig.rx_isr = listener ;
    libsc::k60::JyMcuBt106 myBluetooth(BluetoothConfig);*/

     while (true){
		int t = System::Time();

		//mylcdtypewriter.WriteString(string);

		/* if(t%5==0){
			angle+=100;
			myservo.SetDegree(angle);

		}*/
		if(t%1000==0){
			myled.Switch();
		 }
        }





return 0;
}

bool listener(const Byte *data, const size_t size)
{
	switch(data[0]){
	case'a':
		mylcdtypewriter->WriteString(string);
	}
	return true;
}



