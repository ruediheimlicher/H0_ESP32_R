//
//  lcd.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 22.11.2013.
//
//
#include <stdint.h>
#include <LiquidCrystal_I2C.h>

extern LiquidCrystal_I2C lcd;

void lcd_putc(const char c);
void lcd_puts(const char *s);
void lcd_putint(uint8_t zahl);
void lcd_putint12(uint16_t zahl);
void lcd_putint16(uint16_t zahl);
void lcd_putint2(uint8_t zahl);	//zweistellige Zahl
void lcd_putint1(uint8_t zahl);	//einstellige Zahl
void lcd_puthex(uint8_t zahl);
void lcd_putint999(uint16_t zahl);
