#ifndef wintec2704_ic2_h
#define wintec2704_i2c_h

#include <LiquidCrystal_MCP23017.h>

#define Rs  1 //B00000001  // Register select bit
#define Rw  2 //B00000010  // Read/Write bit
#define En1 4 //B00000100  // Enable1 bit
#define En2 8 //B00001000  // Enable2 bit

class Wintec2704_I2C
{
  public:
    Wintec2704_I2C(uint8_t u8Addr);

	void init();
	void begin();
  void clear();
  void home();

  void noDisplay();
  void display();
  void noBlink();
  void blink();
	void noCursor();
	void cursor();
	void scrollDisplayLeft();
	void scrollDisplayRight();
	void leftToRight();
	void rightToLeft();
	void autoscroll();
	void noAutoscroll();
	void createChar(unsigned char, const char* charmap);
  void createChar_P (uint8_t, const char *);
  void createChar_P (uint8_t, const uint8_t *);
  void setCursor(unsigned char col, unsigned char row);
	int write(unsigned char val);

  void print(const __FlashStringHelper *);
  void println(const __FlashStringHelper *);

	void print(const char* chText);
	void println(const char* chText);

  void print(float fValue, int nLen);
  void print(int nValue);
	void backlight();
	void noBacklight();

  private:
     LiquidCrystal_MCP23017 m_Lcd1;
     LiquidCrystal_MCP23017 m_Lcd2;

	bool		m_bInit;
	uint8_t  	m_u8Addr;
  uint8_t 	m_u8Col;
  uint8_t 	m_u8Row;
};
#endif
