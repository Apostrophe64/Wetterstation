#include "LiquidCrystal_MCP23017.h"
#include <inttypes.h>
#include <Arduino.h>
#include <Wire.h>

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

LiquidCrystal_MCP23017::LiquidCrystal_MCP23017()
{

	m_u8Backlightval = LCD_NOBACKLIGHT; //LCD_BACKLIGHT;
}

void LiquidCrystal_MCP23017::begin(uint8_t u8Addr, uint8_t u8Cols, uint8_t u8Rows, uint8_t u8Rs, uint8_t u8Rw, uint8_t u8En, uint8_t u8Charsize)
 {
	m_u8Addr = u8Addr;
	m_u8Cols = u8Cols;
	m_u8Rows = u8Rows;
	m_u8Rs   = u8Rs;
	m_u8Rw   = u8Rw;
	m_u8En   = u8En;
	m_u8Charsize = u8Charsize;

	Wire.begin();
	Wire.setClock(100000); // 400KHz schaft das Wintek2704 LCD, oder der NodeMCU nicht. :-(

	Wire.beginTransmission(m_u8Addr);
	Wire.write(0x00); // IODIRA register
	Wire.write(0x00); // set all of port A to outputs
	Wire.endTransmission();

	m_u8Displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

	if (m_u8Rows > 1)
	{
		m_u8Displayfunction |= LCD_2LINE;
	}

	// for some 1 line displays you can select a 10 pixel high font
	if ((m_u8Charsize != 0) && (m_u8Rows == 1))
	{
		m_u8Displayfunction |= LCD_5x10DOTS;
	}

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
	delay(50);

	// Now we pull both RS and R/W low to begin commands
	expanderWrite(m_u8Backlightval);	// reset expanderand turn backlight off (Bit 8 =1)
	delay(10);

	//put the LCD into 4 bit mode
	// this is according to the hitachi HD44780 datasheet
	// figure 24, pg 46

	// we start in 8bit mode, try to set 4 bit mode
	write4bits(0x03 << 4);
	delayMicroseconds(4500); // wait min 4.1ms

	// second try
	write4bits(0x03 << 4);
	delayMicroseconds(4500); // wait min 4.1ms

	// third go!
	write4bits(0x03 << 4);
	delayMicroseconds(150);

	// finally, set to 4-bit interface
	write4bits(0x02 << 4);

	// set # lines, font size, etc.
	command(LCD_FUNCTIONSET | m_u8Displayfunction);

	// turn the display on with no cursor or blinking default
	m_u8Displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	display();

	// clear it off
	clear();

	// Initialize to default text direction (for roman languages)
	m_u8Displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

	// set the entry mode
	command(LCD_ENTRYMODESET | m_u8Displaymode);

	home();
}

/********** high level commands, for the user! */
void LiquidCrystal_MCP23017::clear()
{
	command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	delayMicroseconds(2000);  // this command takes a long time!
}

void LiquidCrystal_MCP23017::home()
{
	command(LCD_RETURNHOME);  // set cursor position to zero
	delayMicroseconds(2000);  // this command takes a long time!
}

void LiquidCrystal_MCP23017::setCursor(uint8_t u8Col, uint8_t u8Row)
{
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if (u8Row > m_u8Rows)
	{
		u8Row = m_u8Rows-1;    // we count rows starting w/0
	}
	command(LCD_SETDDRAMADDR | (u8Col + row_offsets[u8Row]));
}

// Turn the display on/off (quickly)
void LiquidCrystal_MCP23017::noDisplay()
{
	m_u8Displaycontrol &= ~LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | m_u8Displaycontrol);
}
void LiquidCrystal_MCP23017::display()
{
	m_u8Displaycontrol |= LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | m_u8Displaycontrol);
}

// Turns the underline cursor on/off
void LiquidCrystal_MCP23017::noCursor()
{
	m_u8Displaycontrol &= ~LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | m_u8Displaycontrol);
}
void LiquidCrystal_MCP23017::cursor()
{
	m_u8Displaycontrol |= LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | m_u8Displaycontrol);
}

// Turn on and off the blinking cursor
void LiquidCrystal_MCP23017::noBlink()
{
	m_u8Displaycontrol &= ~LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | m_u8Displaycontrol);
}
void LiquidCrystal_MCP23017::blink()
{
	m_u8Displaycontrol |= LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | m_u8Displaycontrol);
}

// These commands scroll the display without changing the RAM
void LiquidCrystal_MCP23017::scrollDisplayLeft(void)
{
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void LiquidCrystal_MCP23017::scrollDisplayRight(void)
{
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void LiquidCrystal_MCP23017::leftToRight(void)
{
	m_u8Displaymode |= LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | m_u8Displaymode);
}

// This is for text that flows Right to Left
void LiquidCrystal_MCP23017::rightToLeft(void)
{
	m_u8Displaymode &= ~LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | m_u8Displaymode);
}

// This will 'right justify' text from the cursor
void LiquidCrystal_MCP23017::autoscroll(void)
{
	m_u8Displaymode |= LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | m_u8Displaymode);
}

// This will 'left justify' text from the cursor
void LiquidCrystal_MCP23017::noAutoscroll(void)
{
	m_u8Displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | m_u8Displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LiquidCrystal_MCP23017::createChar(uint8_t u8Location, uint8_t charmap[])
{
	u8Location &= 0x7; // we only have 8 locations 0-7
	command(LCD_SETCGRAMADDR | (u8Location << 3));
	for (int i=0; i<8; i++) {
		write(charmap[i]);
	}
}

// Turn the (optional) backlight off/on
void LiquidCrystal_MCP23017::noBacklight(void)
{
	m_u8Backlightval=LCD_NOBACKLIGHT;
	expanderWrite(0);
}

void LiquidCrystal_MCP23017::backlight(void)
{
	m_u8Backlightval=LCD_BACKLIGHT;
	expanderWrite(0);
}

/*********** mid level commands, for sending data/cmds */

inline void LiquidCrystal_MCP23017::command(uint8_t u8Value)
{
	send(u8Value, 0);
}

inline size_t LiquidCrystal_MCP23017::write(uint8_t u8Value)
{
	send(u8Value, m_u8Rs);
	return 1;
}


/************ low level data pushing commands **********/

// write either command or data
void LiquidCrystal_MCP23017::send(uint8_t u8Value, uint8_t u8Mode)
{
	uint8_t u8Highnib = u8Value&0xf0;
	uint8_t u8Lownib = (u8Value <<4 ) & 0xf0;
	write4bits(u8Highnib | u8Mode);
	write4bits(u8Lownib  | u8Mode);
}

void LiquidCrystal_MCP23017::write4bits(uint8_t u8Value)
{
	expanderWrite(u8Value);
	pulseEnable(u8Value);
}

void LiquidCrystal_MCP23017::expanderWrite(uint8_t u8Data)
{
	Wire.beginTransmission(m_u8Addr);
	Wire.write(0x12); 		// address port A
	Wire.write((int)(u8Data) | m_u8Backlightval);
	Wire.endTransmission();
}

void LiquidCrystal_MCP23017::pulseEnable(uint8_t u8Data)
{
	expanderWrite(u8Data | m_u8En);	// En high
	delayMicroseconds(1);		// enable pulse must be >450ns

	expanderWrite(u8Data & ~ m_u8En);	// En low
	delayMicroseconds(50);		// commands need > 37us to settle
}

void LiquidCrystal_MCP23017::load_custom_character(uint8_t char_num, uint8_t *rows)
{
	createChar(char_num, rows);
}

void LiquidCrystal_MCP23017::setBacklight(uint8_t u8NewVal)
{
	if (u8NewVal) {
		backlight();		// turn backlight on
	} else {
		noBacklight();		// turn backlight off
	}
}

void LiquidCrystal_MCP23017::printstr(const char c[])
{
	//This function is not identical to the function used for "real" I2C displays
	//it's here so the user sketch doesn't have to be changed
	print(c);
}
