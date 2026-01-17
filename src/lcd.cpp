//
//  lcd.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 22.11.2013.
//
//
#include <stdint.h>
#include "lcd.h"

void lcd_putc(const char c)
{
  lcd.write(c);
}
void lcd_puts(const char *s)
/* print string on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = *s++) ) {
        lcd.print(c);
    }

}/* lcd_puts */
// von Atmega8_SPI_WL_mult_Temp_RC

void lcd_putint(uint8_t zahl)
{
char string[4];
  int8_t i;                             // schleifenzähler
 
  string[3]='\0';                       // String Terminator
  for(i=2; i>=0; i--) 
  {
    string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
    zahl /= 10;
  }
lcd_puts(string);
}
void lcd_putint16(uint16_t zahl)
{
char string[8];
  int8_t i;                             // schleifenzähler
 
  string[7]='\0';                       // String Terminator
  for(i=6; i>=0; i--) 
  {
    string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
    zahl /= 10;
  }
lcd_puts(string);
}




void lcd_putint2(uint8_t zahl)	//zweistellige Zahl
{
	char string[3];
	int8_t i;								// Schleifenzähler
	zahl%=100;								// 2 hintere Stelle
	//  string[4]='\0';                     // String Terminator
	string[2]='\0';							// String Terminator
	for(i=1; i>=0; i--) {
		string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
		zahl /= 10;
	}
	lcd_puts(string);
}

void lcd_putint1(uint8_t zahl)	//einstellige Zahl
{
	//char string[5];
	char string[2];
	zahl%=10;								//  hinterste Stelle
	string[1]='\0';							// String Terminator
	string[0]=zahl +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
	lcd_puts(string);
}

void lcd_puthex(uint8_t zahl)
{
	//char string[5];
	char string[3];
	uint8_t i,l,h;                             // schleifenzähler
	
	string[2]='\0';                       // String Terminator
	l=(zahl % 16);
	if (l<10)
	string[1]=l +'0';  
	else
	{
	l%=10;
	string[1]=l + 'A'; 
	
	}
	zahl /=16;
	h= zahl % 16;
	if (h<10)
	string[0]=h +'0';  
	else
	{
	h%=10;
	string[0]=h + 'A'; 
	}
	
	
	lcd_puts(string);
}
void lcd_putint999(uint16_t zahl)
{
   char string[4];
   int8_t i;                             // schleifenzähler
   
   string[3]='\0';                       // String Terminator
   for(i=2; i>=0; i--)
   {
      string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
      zahl /= 10;
   }
   lcd_puts(string);
}


void lcd_putint12(uint16_t zahl)
{
   char string[5];
   int8_t i;                             // schleifenzähler
   
   string[4]='\0';                       // String Terminator
   for(i=3; i>=0; i--)
   {
      string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
      zahl /= 10;
   }
   lcd_puts(string);
}




