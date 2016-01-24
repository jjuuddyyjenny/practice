/*
 * main.cpp
 *
 * Author: Thomas
 *
 */

#include <cassert>
#include <cstring>
#include <cstdio>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libbase/k60/gpio.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/st7735r.h>
#include <libsc/futaba_s3010.h>
#include <libsc/alternate_motor.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/tsl1401cl.h>
#include <cmath>

namespace libbase {
	namespace k60 {

		Mcg::Config Mcg::GetMcgConfig() {
			Mcg::Config config;
			config.external_oscillator_khz = 50000;
			config.core_clock_khz = 150000;
			return config;
		}

	}
}

using namespace libsc;
using namespace libbase::k60;

enum State_t {BE, LE, RE, NE, CR};
const uint16_t EX = 10;
const uint16_t THRES_MIN = 15;

Gpo led0, led1, led2, led3;
Gpo buzzer;
Gpi joy_down, joy_left, joy_right, joy_center, joy_up;
St7735r *lcd = 0;
LcdTypewriter *writer = 0;
FutabaS3010 *servo = 0;
AlternateMotor *motor = 0;
Tsl1401cl *ccd1 = 0;
libsc::k60::JyMcuBt106 *bluetooth = 0;
std::array<uint16_t, Tsl1401cl::kSensorW> ccd_data;
uint16_t angle = 1050; //angle in [600, 1500], middle is 1050
uint16_t speed;
int16_t error, center, thres;
uint16_t bref, wref;
bool motorEnabled;
State_t state;

bool listener(const Byte *, const size_t);
void configuration(void);
void print_ccd(uint16_t);
void edge(uint16_t*, uint16_t*);
void edge1(uint16_t*, uint16_t*);
void updateAngle(void);
void updateSpeed(void);

inline int mmax(int a, int b) {
	return (a > b) ? a : b;
}

inline int mmin(int a, int b) {
	return (a < b) ? a : b;
}

int main(void) {
 	uint16_t count = 0;
	uint16_t left, right;
	Timer::TimerInt t = 0;
	bool ispressed = false;

	System::Init();
	configuration();

	while (1) {
		if (t == System::Time()) continue;
		t = System::Time();

		if (t % 10 == 0) {
			if (!ispressed) {
				if (!joy_down.Get()) {
					bluetooth->SendStr("joy stick down!\n");
					ispressed = true;
				}
				else if (!joy_left.Get()) {
					bluetooth->SendStr("joy stick left!\n");
					ispressed = true;
				}
				else if (!joy_right.Get()) {
					bluetooth->SendStr("joy stick right!\n");
					ispressed = true;
				}
				else if (!joy_center.Get()) {
					bluetooth->SendStr("joy stick center!\n");
					ispressed = true;
				}
				else if (!joy_up.Get()) {
					bluetooth->SendStr("joy stick up!\n");
					ispressed = true;
				}
			}
			else {
				if (joy_down.Get() && joy_left.Get() && joy_right.Get()
					&& joy_center.Get() && joy_up.Get())
					ispressed = false;
			}
		}

		if (t % 10 == 0) {
			ccd1->StartSample();
			while (!ccd1->SampleProcess());
			//print_ccd(Lcd::kBlack);
			ccd_data = ccd1->GetData();
			//print_ccd(Lcd::kGreen);
			edge(&left, &right);
			updateAngle();
			updateSpeed();
		}

		if (t % 10 == 0) {
			switch (count) {
			case 0:
			led2.Set(true);
			led0.Set(false);
			break;
			case 1:
			led0.Set(true);
			led1.Set(false);
			break;
			case 2:
			led1.Set(true);
			led3.Set(false);
			break;
			case 3:
			led3.Set(true);
			led2.Set(false);
			break;
			}
			count = (count + 1) % 4;
		}
	}

	return 0;
}

bool listener(const Byte *data, const size_t size) {
	char message[128];
	switch (data[0]) {
		case '&':
			motorEnabled = true;
			if (speed > 500) {
				speed = 500;
				motor->SetPower(speed);
				bluetooth->SendStrLiteral("motor enabled with default power = 200\n");
			}
			else {
				motor->SetPower(speed);
				sprintf(message, "motor enabled with power = %d\n", speed);
				bluetooth->SendStr(message);
			}
			motor->SetPower(speed);
			break;
		case ' ':
			if (motor->GetPower()) {
				//speed = 0;
				motor->SetPower(0);
				bluetooth->SendStrLiteral("STOP\n");
			}
			motorEnabled = false;
			break;
		case ',':
			if (speed >= 50) {
				speed -= 50;
				if (motorEnabled)
					motor->SetPower(speed);
				sprintf(message, "power = %d\n", speed);
				bluetooth->SendStr(message);
			}
			else bluetooth->SendStrLiteral("power is already 0\n");
			break;
		case '.':
			if (speed <= 750) {
				speed += 50;
				if (motorEnabled)
					motor->SetPower(speed);
				sprintf(message, "power = %d\n", speed);
				bluetooth->SendStr(message);
			}
			else bluetooth->SendStrLiteral("already full powered\n");
			break;
		case 'j':
			if (angle >= 625) {
				angle = (angle > 1050) ? 1050 : angle - 25;
				servo->SetDegree(angle);
				sprintf(message, "angel = %d\n", angle);
				bluetooth->SendStr(message);
			}
			else bluetooth->SendStrLiteral("No more left\n");
			break;
		case 'l':
			if (angle <= 1475) {
				angle = (angle < 1050) ? 1050 : angle + 25;
				servo->SetDegree(angle);
				sprintf(message, "angle = %d\n", angle);
				bluetooth->SendStr(message);
			}
			else bluetooth->SendStrLiteral("No more right\n");
			break;
		case 'i':
			angle = 1050;
			servo->SetDegree(angle);
			break;
		case 'q':
			sprintf(message, "power = %d\n", speed);
			bluetooth->SendStr(message);
			break;
		default:
		;
	}
	return true;
}

void print_ccd(uint16_t color) {
	for (uint16_t i = 0; i < Tsl1401cl::kSensorW; i++) {
		lcd->SetRegion(Lcd::Rect(i, (255 - ccd_data[i]) / 2,
			1, 1));
		lcd->FillPixel(&color, 1);
	}
	lcd->SetRegion(Lcd::Rect(0, 0, 128, 160));
}

void edge(uint16_t *ledge, uint16_t *redge) {
	uint16_t max = 0, min = 0, ccd_data_p[Tsl1401cl::kSensorW];
	uint16_t l = 0;
	uint16_t r = Tsl1401cl::kSensorW - 1;
	for (uint16_t i = 0; i < Tsl1401cl::kSensorW; i++) {
		int16_t j, cnt;
		ccd_data_p[i] = 0;
		for (j = -5, cnt = 0; j <= 5; j++) {
			if (i + j < 0) continue;
			if (i + j >= Tsl1401cl::kSensorW) break;
			ccd_data_p[i] += ccd_data[i + j];
			cnt++;
		}
		ccd_data_p[i] /= cnt;
	}
	for (uint16_t i = 1; i < Tsl1401cl::kSensorW; i++) {
		if (ccd_data_p[i] > ccd_data_p[max])
			max = i;
		if (ccd_data_p[i] < ccd_data_p[min])
			min = i;
	}
	if (max != 0) {
		for (uint16_t i = 0; i < max; i++) {
			if (ccd_data_p[i + 1] - ccd_data_p[i] >
				ccd_data_p[l + 1] - ccd_data_p[l])
				l = i;
		}
	}
	if (max != Tsl1401cl::kSensorW - 1) {
		for (uint16_t i = Tsl1401cl::kSensorW - 1; i > max; i--) {
			if (ccd_data_p[i - 1] - ccd_data_p[i] >
				ccd_data_p[r - 1] - ccd_data_p[r])
				r = i;
		}
	}
	lcd->SetRegion(Lcd::Rect(*ledge, 131, 1, 29));
	lcd->FillColor(Lcd::kBlack);
	lcd->SetRegion(Lcd::Rect(*redge, 131, 1, 29));
	lcd->FillColor(Lcd::kBlack);
	int16_t ld = ccd_data_p[l + 1] - ccd_data_p[l];
	int16_t rd = ccd_data_p[r - 1] - ccd_data_p[r];
	if (ccd_data_p[max] - ccd_data_p[min] < 35) {
		if (error > 0) {
			//*ledge = *redge = Tsl1401cl::kSensorW - 1;
			//*ledge = Tsl1401cl::kSensorW / 2;
			*ledge = 0;
			*redge = Tsl1401cl::kSensorW - 1;
		}
		else if (error < 0) {
			//*redge = *ledge = 0;
			//*redge = Tsl1401cl::kSensorW / 2;
			*ledge = 0;
			*redge = Tsl1401cl::kSensorW - 1;
		}
		state = NE;
		//speed = 200;
		//if (motorEnabled) motor->SetPower(speed);
	}
	else if (ld > rd * 1.4 && state != RE) {
		*redge = Tsl1401cl::kSensorW - 1;
		state = LE; //only left edge
		//speed = 200;
		//if (motorEnabled) motor->SetPower(speed);
	}
	else if (ld * 1.4 < rd && state != LE) {
		*ledge = 0;
		state = RE; //only right edge
		//speed = 200;
		//if (motorEnabled) motor->SetPower(speed);
	}
	else {
		state = BE;
		//speed = 250;
		//if (motorEnabled) motor->SetPower(speed);
	}
	if (state == BE) {
		*ledge = (l * 50 + *ledge * 50) / 100;
		*redge = (r * 50 + *redge * 50) / 100;
	}
	error = *ledge + *redge - Tsl1401cl::kSensorW;
	lcd->SetRegion(Lcd::Rect(*ledge, 131, 1, 29));
	lcd->FillColor(Lcd::kYellow);
	lcd->SetRegion(Lcd::Rect(*redge, 131, 1, 29));
	lcd->FillColor(Lcd::kYellow);
}

void edge1(uint16_t *ledge, uint16_t *redge) {
	uint16_t max = 0, min = 0, ccd_data_p[Tsl1401cl::kSensorW];
	uint16_t l = 0;
	uint16_t r = Tsl1401cl::kSensorW - 1;
	for (uint16_t i = 0; i < Tsl1401cl::kSensorW; i++) {
		int16_t j, cnt;
		ccd_data_p[i] = 0;
		for (j = -5, cnt = 0; j <= 5; j++) {
			if (i + j < 0) continue;
			if (i + j >= Tsl1401cl::kSensorW) break;
			ccd_data_p[i] += ccd_data[i + j];
			cnt++;
		}
		ccd_data_p[i] /= cnt;
	}
	for (uint16_t i = 1; i < Tsl1401cl::kSensorW; i++) {
		if (ccd_data_p[i] > ccd_data_p[max])
			max = i;
		if (ccd_data_p[i] < ccd_data_p[min])
			min = i;
	}
	if (max != 0) {
		for (uint16_t i = 0; i < max; i++) {
			if (ccd_data_p[i + 1] - ccd_data_p[i] >
				ccd_data_p[l + 1] - ccd_data_p[l])
				l = i;
		}
	}
	if (max != Tsl1401cl::kSensorW - 1) {
		for (uint16_t i = Tsl1401cl::kSensorW - 1; i > max; i--) {
			if (ccd_data_p[i - 1] - ccd_data_p[i] >
				ccd_data_p[r - 1] - ccd_data_p[r])
				r = i;
		}
	}
	lcd->SetRegion(Lcd::Rect(*ledge, 131, 1, 29));
	lcd->FillColor(Lcd::kBlack);
	lcd->SetRegion(Lcd::Rect(*redge, 131, 1, 29));
	lcd->FillColor(Lcd::kBlack);
	int16_t ld = ccd_data_p[mmin(l + 2, Tsl1401cl::kSensorW - 1)]
				- ccd_data_p[mmax(l - 1, 0)];
	int16_t rd = ccd_data_p[mmax(r - 2, 0)]
				- ccd_data_p[mmin(r + 1, Tsl1401cl::kSensorW - 1)];
	if (ccd_data_p[max] - ccd_data_p[min] < 35) {
		*ledge = 0;
		*redge = Tsl1401cl::kSensorW - 1;
		state = NE;
	}
	else if (ld > rd * 1.4 && state != RE) {
		*redge = Tsl1401cl::kSensorW - 1;
		state = LE; //only left edge
	}
	else if (ld * 1.4 < rd && state != LE) {
		*ledge = 0;
		state = RE; //only right edge
	}
	else {
		state = BE;
	}
	if (state == BE) {
		*ledge = (l * 30 + *ledge * 70) / 100;
		*redge = (r * 30 + *redge * 70) / 100;
	}
	error = *ledge + *redge - Tsl1401cl::kSensorW;
	lcd->SetRegion(Lcd::Rect(*ledge, 131, 1, 29));
	lcd->FillColor(Lcd::kYellow);
	lcd->SetRegion(Lcd::Rect(*redge, 131, 1, 29));
	lcd->FillColor(Lcd::kYellow);
}

void updateAngle(void) {
	char message[128];
	uint16_t angle_new;
	switch (state) {
		case BE:
			angle_new = 1200 + error * 12;
			if (angle_new > 1750) angle_new = 1750;
			if (angle_new < 650) angle_new = 650;
			break;
		case LE:
			angle_new = 1750;
			break;
		case RE:
			angle_new = 650;
			break;
		case NE:
				angle_new = 1200;
			break;
		case CR:
			angle_new = 1200;
	}
	if ((angle < 1200 && angle_new > 1200) ||
		(angle > 1200 && angle_new < 1200))
		angle_new = 1200;
	if (angle != angle_new) {
		servo->SetDegree(angle_new);
		//sprintf(message, "angle updated to %d\n", angle);
		//bluetooth->SendStr(message);
		angle = angle_new;
	}
}

void updateSpeed(void) {
	speed = 350 - std::abs(error) * 1.5;
	if (speed > 400) speed = 400;
	if (motorEnabled) {
		motor->SetPower(speed);
	}
}

void configuration(void) {
	Gpo::Config led_config;
	led_config.pin = Pin::Name::kPte12;
	led_config.is_high = true;
	led0 = Gpo(led_config);
	led_config.pin = Pin::Name::kPte11;
	led1 = Gpo(led_config);
	led_config.pin = Pin::Name::kPte10;
	led2 = Gpo(led_config);
	led_config.pin = Pin::Name::kPte9;
	led3 = Gpo(led_config);

	Gpo::Config buzzer_config;
	buzzer_config.pin = Pin::Name::kPta8;
	buzzer_config.is_high = false;
	buzzer = Gpo(buzzer_config);

	Gpi::Config joystick_config;
	joystick_config.pin = Pin::Name::kPtc4; //down
	joy_down = Gpi(joystick_config);
	joystick_config.pin = Pin::Name::kPtc5; //left
	joy_left = Gpi(joystick_config);
	joystick_config.pin = Pin::Name::kPtc6; //right
	joy_right = Gpi(joystick_config);
	joystick_config.pin = Pin::Name::kPtc7; //center
	joy_center = Gpi(joystick_config);
	joystick_config.pin = Pin::Name::kPtc8; //up
	joy_up = Gpi(joystick_config);

	St7735r::Config st7735r_config;
	st7735r_config.is_revert = false;
	st7735r_config.is_bgr = false;
	lcd = new St7735r(st7735r_config);
	LcdTypewriter::Config lcd_config;
	lcd_config.lcd = lcd;
	lcd_config.is_text_wrap = true;
	writer = new LcdTypewriter(lcd_config);
	lcd->SetRegion(Lcd::Rect(0, 0, 128, 160));
	lcd->FillColor(Lcd::kBlack);
	lcd->SetRegion(Lcd::Rect(0, 128, 128, 3));
	lcd->FillColor(Lcd::kWhite);
	
	FutabaS3010::Config servo_config;
	servo_config.id = 0;
	servo = new FutabaS3010(servo_config);
	angle = 1050;
	servo->SetDegree(angle);

	AlternateMotor::Config motor_config;
	motor_config.id = 1;
	motor = new AlternateMotor(motor_config);
	motor->SetClockwise(false);
	speed = 0;
	motor->SetPower(speed);
	motorEnabled = false;
	
	libsc::k60::JyMcuBt106::Config bluetooth_config;
	bluetooth_config.id = 0;
	bluetooth_config.baud_rate = Uart::Config::BaudRate::k115200;
	bluetooth_config.tx_buf_size = 200;
	bluetooth_config.rx_isr = listener;
	bluetooth = new libsc::k60::JyMcuBt106(bluetooth_config);

	ccd1 = new Tsl1401cl(0);

	//state initialisation
	state = BE;
	center = Tsl1401cl::kSensorW / 2;
	thres = THRES_MIN + 10;
}
