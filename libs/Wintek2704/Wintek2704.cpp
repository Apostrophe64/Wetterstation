#include "Wintek2704.h"

#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
//////////////////////////////////////////////////////////////////////////
// Pollin Net IO Board
#if (0)
//  rs, rw, enable, d4, d5, d6, d7);
wintec2704::wintec2704() : m_Lcd1(19, 18, 24, 20, 21, 22, 23),
                           m_Lcd2(19, 18, 27, 20, 21, 22, 23),
                           m_col(0),
                           m_row(0),
						   m_bInit(false)
{
}
#else
//  rs, rw, enable, d4, d5, d6, d7);
wintec2704::wintec2704() : m_Lcd1(3, 2, 8, 4, 5, 6, 7),
                           m_Lcd2(3, 2, 9, 4, 5, 6, 7),
                           m_col(0),
                           m_row(0),
                           m_bInit(false)
{
}
#endif

//////////////////////////////////////////////////////////////////////////
void  wintec2704::begin()
{
 	if (!m_bInit)
    {
		m_Lcd1.begin(27, 2);
		m_Lcd2.begin(27, 2);
		m_bInit = true;
	}
}
//////////////////////////////////////////////////////////////////////////
void wintec2704::clear()
{
	m_row = 0;
	m_col = 0;
    m_Lcd1.clear();
    m_Lcd2.clear();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::home()
{
	m_row = 0;
	m_col = 0;
    m_Lcd1.home();
    m_Lcd2.home();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::noDisplay()
{
    m_Lcd1.noDisplay();
    m_Lcd2.noDisplay();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::display()
{
    m_Lcd1.display();
    m_Lcd2.display();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::noBlink()
{
    m_Lcd1.noBlink();
    m_Lcd2.noBlink();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::blink()
{
    m_Lcd1.blink();
    m_Lcd2.blink();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::noCursor()
{
    m_Lcd1.noCursor();
    m_Lcd2.noCursor();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::cursor()
{
	if (m_row < 2)
	{
		m_Lcd1.cursor();
		m_Lcd2.noCursor();
	}
	else
	{
		m_Lcd1.noCursor();
		m_Lcd2.cursor();
	}
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::setCursor(unsigned char col, unsigned char row)
{
    if (row < 2)
       m_Lcd1.setCursor(col, row);
    else
       m_Lcd2.setCursor(col, row-(unsigned char)2);

    m_col = col;
    m_row = row;
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::scrollDisplayLeft()
{
	m_Lcd1.scrollDisplayLeft();
	m_Lcd2.scrollDisplayLeft();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::scrollDisplayRight()
{
	m_Lcd1.scrollDisplayRight();
	m_Lcd2.scrollDisplayRight();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::leftToRight()
{
	m_Lcd1.leftToRight();
	m_Lcd2.leftToRight();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::rightToLeft()
{
	m_Lcd1.rightToLeft();
	m_Lcd2.rightToLeft();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::autoscroll()
{
	m_Lcd1.autoscroll();
	m_Lcd2.autoscroll();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::noAutoscroll()
{
	m_Lcd1.noAutoscroll();
	m_Lcd2.noAutoscroll();
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::createChar(unsigned char location, const unsigned char charmap[])
{
#if (1)
  m_Lcd1.createChar(location, (uint8_t*)charmap);
  m_Lcd2.createChar(location, (uint8_t*)charmap); 
#else
	location &= 0x7; // we only have 8 locations 0-7
	m_Lcd1.command(LCD_SETCGRAMADDR | (location << 3));
	for (int i=0; i<8; i++)
	{
		m_Lcd1.write(charmap[i]);
	}

	m_Lcd2.command(LCD_SETCGRAMADDR | (location << 3));
	for (int i=0; i<8; i++)
	{
		m_Lcd2.write(charmap[i]);
	}
#endif
}

//////////////////////////////////////////////////////////////////////////
// custom bitmaps in PROGMEM
void wintec2704::createChar_P (uint8_t location, const char *bitmap)
{
//    createChar_P (location, (const uint8_t *)(bitmap));
  m_Lcd1.createChar(location, bitmap);
  m_Lcd2.createChar(location, bitmap); 
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::createChar_P (uint8_t location, const uint8_t *bitmap)
{
  m_Lcd1.createChar(location, (uint8_t*)bitmap);
  m_Lcd2.createChar(location, (uint8_t*)bitmap); 
}

//////////////////////////////////////////////////////////////////////////
int wintec2704::write(unsigned char val)
{
	int s = 0;
	if (m_row < 2)
		s = (int)m_Lcd1.write(val);
	else
		s = (int)m_Lcd2.write(val);

	return s;
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::print(const __FlashStringHelper *ifsh)
{
  PGM_P p = reinterpret_cast<PGM_P>(ifsh);
  while (1)
	{
    unsigned char c = pgm_read_byte(p++);
    if (c == 0)
		 	break;
    write(c);
  }
}
/*
//////////////////////////////////////////////////////////////////////////
void wintec2704::print(const __FlashStringHelper* chText)
{
    if (m_row < 2)
       m_Lcd1.print(chText);
    else
       m_Lcd2.print(chText);
}
*/

//////////////////////////////////////////////////////////////////////////
void wintec2704::print(const char* chText)
{
    if (m_row < 2)
       m_Lcd1.print(chText);
    else
       m_Lcd2.print(chText);
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::print(float fValue, int nPre)
{
    if (m_row < 2)
       m_Lcd1.print(fValue, nPre);
    else
       m_Lcd2.print(fValue, nPre);
}

//////////////////////////////////////////////////////////////////////////
void wintec2704::print(int nValue)
{
    if (m_row < 2)
       m_Lcd1.print(nValue);
    else
       m_Lcd2.print(nValue);
}
//////////////////////////////////////////////////////////////////////////
void  wintec2704::init()
{
 	if (!m_bInit)
    {
		m_Lcd1.begin(27, 2);
		m_Lcd2.begin(27, 2);
		m_bInit = true;
	}
}

//////////////////////////////////////////////////////////////////////////
void  wintec2704::backlight()
{
	// Not implementet yet
}

//////////////////////////////////////////////////////////////////////////
void  wintec2704::noBacklight()
{
	// Not implementet yet
}
