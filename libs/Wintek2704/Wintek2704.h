#ifndef wintec2704_h
#define wintec2704_h
#include <LiquidCrystal.h>

class wintec2704
{
  public:
    wintec2704();
    void begin();
	void init();
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
	void createChar(unsigned char, const unsigned char[]);
  void createChar_P (uint8_t, const char *);
  void createChar_P (uint8_t, const uint8_t *);
  void setCursor(unsigned char col, unsigned char row);
	int write(unsigned char val);

  void print(const __FlashStringHelper* chText);
	void print(const char* chText);
  void print(float fValue, int nLen);
  void print(int nValue);
	void backlight();
	void noBacklight();

  private:
     LiquidCrystal m_Lcd1;
     LiquidCrystal m_Lcd2;

    unsigned char m_col;
    unsigned char m_row;
    bool m_bInit;
};
#endif
