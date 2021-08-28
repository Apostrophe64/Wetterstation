#include "Wintek2704_MCP23017.h"

// initialize the library with the numbers of the interface pins
Wintec2704_I2C::Wintec2704_I2C(uint8_t u8Addr) : m_u8Addr(u8Addr), m_u8Col(0), m_u8Row(0)
{

}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::begin()
{
	if (!m_bInit)
    {
		// set up the LCD's I2C Adress, number of columns and rows, Rs, Rw, En1, En2
		m_Lcd1.begin(m_u8Addr, 27, 2, Rs, Rw, En1);
		m_Lcd2.begin(m_u8Addr, 27, 2, Rs, Rw, En2);
		m_bInit = true;
	}
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::clear()
{
	m_u8Row = 0;
	m_u8Col = 0;
  m_Lcd1.clear();
  m_Lcd2.clear();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::home()
{
	m_u8Row = 0;
	m_u8Col = 0;
  m_Lcd1.home();
  m_Lcd2.home();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::noDisplay()
{
    m_Lcd1.noDisplay();
    m_Lcd2.noDisplay();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::display()
{
    m_Lcd1.display();
    m_Lcd2.display();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::noBlink()
{
    m_Lcd1.noBlink();
    m_Lcd2.noBlink();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::blink()
{
    m_Lcd1.blink();
    m_Lcd2.blink();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::noCursor()
{
    m_Lcd1.noCursor();
    m_Lcd2.noCursor();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::cursor()
{
	if (m_u8Row < 2)
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
void Wintec2704_I2C::setCursor(unsigned char col, unsigned char row)
{
    if (row < 2)
       m_Lcd1.setCursor(col, row);
    else
       m_Lcd2.setCursor(col, row-(unsigned char)2);

    m_u8Col = col;
    m_u8Row = row;
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::scrollDisplayLeft()
{
	m_Lcd1.scrollDisplayLeft();
	m_Lcd2.scrollDisplayLeft();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::scrollDisplayRight()
{
	m_Lcd1.scrollDisplayRight();
	m_Lcd2.scrollDisplayRight();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::leftToRight()
{
	m_Lcd1.leftToRight();
	m_Lcd2.leftToRight();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::rightToLeft()
{
	m_Lcd1.rightToLeft();
	m_Lcd2.rightToLeft();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::autoscroll()
{
	m_Lcd1.autoscroll();
	m_Lcd2.autoscroll();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::noAutoscroll()
{
	m_Lcd1.noAutoscroll();
	m_Lcd2.noAutoscroll();
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::createChar(unsigned char location, const char *charmap)
{
	location &= 0x7; // we only have 8 locations 0-7
	m_Lcd1.command(LCD_SETCGRAMADDR | ((location % 8) << 3));
	for (int i=0; i<8; i++)
	{
		m_Lcd1.write(charmap[i]);
	}

	m_Lcd2.command(LCD_SETCGRAMADDR | ((location % 8) << 3));
	for (int i=0; i<8; i++)
	{
		m_Lcd2.write(charmap[i]);
	}

}

//////////////////////////////////////////////////////////////////////////
// custom bitmaps in PROGMEM
void Wintec2704_I2C::createChar_P (uint8_t location, const char *bitmap)
{
    createChar_P (location, (const uint8_t *)(bitmap));
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::createChar_P (uint8_t location, const uint8_t *bitmap)
{
    m_Lcd1.command (LCD_SETCGRAMADDR | ((location % 8) * 8));
    for (uint8_t n = 0; n < 8; n++)
	{
        m_Lcd1.write (pgm_read_byte(bitmap + n));
    }

    m_Lcd2.command (LCD_SETCGRAMADDR | ((location % 8) * 8));
    for (uint8_t n = 0; n < 8; n++)
	{
        m_Lcd2.write (pgm_read_byte(bitmap + n));
    }
}

//////////////////////////////////////////////////////////////////////////
int Wintec2704_I2C::write(unsigned char val)
{
	int s = 0;
	if (m_u8Row < 2)
		s = (int)m_Lcd1.write(val);
	else
		s = (int)m_Lcd2.write(val);

	return s;
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::print(const __FlashStringHelper *ifsh)
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

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::print(const char* chText)
{
    if (m_u8Row < 2)
       m_Lcd1.print(chText);
    else
       m_Lcd2.print(chText);
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::println(const __FlashStringHelper *ifsh)
{
  PGM_P p = reinterpret_cast<PGM_P>(ifsh);
  while (1) {
    unsigned char c = pgm_read_byte(p++);
		//sLine += c;
    if (c == 0)
		 	break;
		write(c);
  }
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::println(const char* chText)
{
    if (m_u8Row < 2)
       m_Lcd1.print(chText);
    else
       m_Lcd2.print(chText);
    if (m_u8Row < 3)
	{
		m_u8Row++;
		m_u8Col = 0;
		setCursor(m_u8Col, m_u8Row);
	}
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::print(float fValue, int nPre)
{
    if (m_u8Row < 2)
       m_Lcd1.print(fValue, nPre);
    else
       m_Lcd2.print(fValue, nPre);
}

//////////////////////////////////////////////////////////////////////////
void Wintec2704_I2C::print(int nValue)
{
    if (m_u8Row < 2)
       m_Lcd1.print(nValue);
    else
       m_Lcd2.print(nValue);
}
//////////////////////////////////////////////////////////////////////////
void  Wintec2704_I2C::init()
{
	if (!m_bInit)
    {
		// set up the LCD's I2C Adress, number of columns and rows, Rs, Rw, En1, En2
		m_Lcd1.begin(m_u8Addr, 27, 2, Rs, Rw, En1);
		m_Lcd2.begin(m_u8Addr, 27, 2, Rs, Rw, En2);
		m_bInit = true;
	}
}
//////////////////////////////////////////////////////////////////////////
void  Wintec2704_I2C::backlight()
{
	// Not implementet yet
}

//////////////////////////////////////////////////////////////////////////
void  Wintec2704_I2C::noBacklight()
{
	// Not implementet yet
}
