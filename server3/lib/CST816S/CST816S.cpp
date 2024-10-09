/*
   MIT License

  Copyright (c) 2021 Felix Biego

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "Arduino.h"
#include <Wire.h>
#include <FunctionalInterrupt.h>

#include "CST816S.h"

//avoid the defines, as per https://docs.arduino.cc/learn/contributions/arduino-writing-style-guide#variables
const uint8_t TOUCH_CMD_SLEEP = 0x03;
const uint8_t TOUCH_REGISTER_SLEEP = 0xA5;
const uint8_t TOUCH_REGISTER_WORK = 0x00;
const uint8_t TOUCH_INDEX_GESTURE = 0x01;	//defined by reverse engineering..
const uint8_t TOUCH_REGISTER_NUMBER = 0x02;

const uint8_t TOUCH_IRQ_EN_TOUCH = 0x40;	//gives a lot of events, in itself only provide touch (TOUCH_CONTACT) info. also include gesture info
const uint8_t TOUCH_IRQ_EN_CHANGE = 0x20;	//gives or adds the release (TOUCH_UP) info
const uint8_t TOUCH_IRQ_EN_MOTION = 0x10;	//seems to add the GESTURE_TOUCH_BUTTON events, add long press-while-still-touched gestures
const uint8_t TOUCH_IRQ_EN_LONGPRESS = 0x01;	//seems to do nothing..?

const uint8_t MOTION_MASK_CONTINUOUS_LEFT_RIGHT = 0x04;
const uint8_t MOTION_MASK_CONTINUOUS_UP_DOWN = 0x02;
const uint8_t MOTION_MASK_DOUBLE_CLICK = 0x01;	//add hardware based double click

const uint8_t TOUCH_REGISTER_VERSION = 0x15;
const uint8_t TOUCH_REGISTER_CHIP_ID = 0xA7;		//chip ID
const uint8_t TOUCH_REGISTER_FW_VERSION = 0xA9;		//firmware version
const uint8_t TOUCH_REGISTER_MOTION_MASK = 0xEC;		//motion mask register
const uint8_t TOUCH_REGISTER_IRQ_CTL = 0xFA;		//interrupt control
const uint8_t TOUCH_REGISTER_AUTOSLEEP = 0xFE;		//can be used to disable auto sleep, might be quicker (?) but will consume more power


/*!
    @brief  Constructor for CST816S
	@param	sda
			i2c data pin
	@param	scl
			i2c clock pin
	@param	rst
			touch reset pin
	@param	irq
			touch interrupt pin
*/
CST816S::CST816S(int sda, int scl, int rst, int irq) {
  _sda = sda;
  _scl = scl;
  _rst = rst;
  _irq = irq;

}

/*!
    @brief  read touch data
*/
void CST816S::read_touch() {
  byte data_raw[8];
  i2c_read(CST816S_ADDRESS, 0x01, data_raw, 6);

  data.gestureID = data_raw[0];
  data.points = data_raw[1];
  data.event = data_raw[2] >> 6;
  data.x = ((data_raw[2] & 0xF) << 8) + data_raw[3];
  data.y = ((data_raw[4] & 0xF) << 8) + data_raw[5];

  /*
  Serial.print("GestureID; ");
  Serial.print(data_raw[0]);
  Serial.print(" points; ");
  Serial.print(data_raw[1]);
  Serial.print(" event; ");
  Serial.println(data_raw[2]);
  */
}

/*!
    @brief  handle interrupts
*/
void IRAM_ATTR CST816S::handleISR(void) {
  _event_available = true;

}

/*!
    @brief  initialize the touch screen
	@param	interrupt
			type of interrupt FALLING, RISING..
*/
void CST816S::begin(int interrupt) {
  Wire.begin(_sda, _scl);

  pinMode(_irq, INPUT);
  pinMode(_rst, OUTPUT);

  digitalWrite(_rst, HIGH );
  delay(50);
  digitalWrite(_rst, LOW);
  delay(5);
  digitalWrite(_rst, HIGH );
  delay(50);

  i2c_read(CST816S_ADDRESS, 0x15, &data.version, 1);
  delay(5);
  i2c_read(CST816S_ADDRESS, 0xA7, data.versionInfo, 3);

  attachInterrupt(_irq, std::bind(&CST816S::handleISR, this), interrupt);
}

/*!
    @brief  check for a touch event
*/
bool CST816S::available() {
  if (_event_available) {
    read_touch();
    _event_available = false;
    return true;
  }
  return false;
}

/**
 * Set the CST816 chip in sleep mode, and prepares the ESP32 for a deep sleep with touch screen as a wakeup source
 */
bool CST816S::deepsleepprep() {

	//bool bOk = writeRegister((uint8_t)TOUCH_REGISTER_SLEEP, (uint8_t)TOUCH_CMD_SLEEP);


	//if (!bOk) {
	//	Serial.println("Deep Sleep request failed..");
	//}

	sleep();
	Serial.println(_rst);
	Serial.println(_irq);

	if (_rst != -1) {
		/*
		//from https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
		However, on ESP32/S2/C3/S3/C2, this function cannot be used to hold the state of a digital GPIO during Deep-sleep.
		Even if this function is enabled, the digital GPIO will be reset to its default state when the chip wakes up from Deep-sleep.
		If you want to hold the state of a digital GPIO during Deep-sleep, please call gpio_deep_sleep_hold_en.
		*/
		//gpio_hold_en((gpio_num_t)m_iPIN_RESET);
		gpio_deep_sleep_hold_en();
	}
	if (_irq != -1) {
		//ensure we'll wake when ESP32 deep sleep is started
		esp_sleep_enable_ext0_wakeup((gpio_num_t)_irq, 0);
	}

	return true;
}


/*!
    @brief  put the touch screen in standby mode
*/
void CST816S::sleep() {
  digitalWrite(_rst, LOW);
  delay(5);
  digitalWrite(_rst, HIGH );
  delay(50);
  byte standby_value = 0x03;
  i2c_write(CST816S_ADDRESS, 0xA5, &standby_value, 1);
}

/**PWM
*  @brief  set brightness of backlight
*/
void CST816S::setBL(uint8_t Value)
{
	if (Value < 0 || Value > 100)
	{
		printf("SET_PWM Error \r\n");
	}
	else
	{
		analogWrite(LED_BL, Value * 2.55);
	}
}

/*!
    @brief  get the gesture event name
*/
String CST816S::gesture() {
  switch (data.gestureID) {
    case NONE:
      return "NONE";
      break;
    case SWIPE_DOWN:
      return "SWIPE DOWN";
      break;
    case SWIPE_UP:
      return "SWIPE UP";
      break;
    case SWIPE_LEFT:
      return "SWIPE LEFT";
      break;
    case SWIPE_RIGHT:
      return "SWIPE RIGHT";
      break;
    case SINGLE_CLICK:
      return "SINGLE CLICK";
      break;
    case DOUBLE_CLICK:
      return "DOUBLE CLICK";
      break;
    case LONG_PRESS:
      return "LONG PRESS";
      break;
    default:
      return "UNKNOWN";
      break;
  }
}

/*!
    @brief  read data from i2c
	@param	addr
			i2c device address
	@param	reg_addr
			device register address
	@param	reg_data
			array to copy the read data
	@param	length
			length of data
*/
uint8_t CST816S::i2c_read(uint16_t addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
  Wire.beginTransmission(addr);
  Wire.write(reg_addr);
  if ( Wire.endTransmission(true))return -1;
  Wire.requestFrom(addr, length, true);
  for (int i = 0; i < length; i++) {
    *reg_data++ = Wire.read();
  }
  return 0;
}

/*!
    @brief  write data to i2c
	@brief  read data from i2c
	@param	addr
			i2c device address
	@param	reg_addr
			device register address
	@param	reg_data
			data to be sent
	@param	length
			length of data
*/
uint8_t CST816S::i2c_write(uint8_t addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t length)
{
  Wire.beginTransmission(addr);
  Wire.write(reg_addr);
  for (int i = 0; i < length; i++) {
    Wire.write(*reg_data++);
  }
  if ( Wire.endTransmission(true))return -1;
  return 0;
}