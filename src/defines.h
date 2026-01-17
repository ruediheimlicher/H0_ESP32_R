//
//  text.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 22.11.2013.
//
//

#ifndef DEFINES_h
#define DEFINES_h


#define BOARD_TEENSY 2 //TEENSY
#define BOARD_6 6


#define OSZIA_PIN 22

//#define BOARD BOARD_6
#define OZIA_PIN  22
#define ANZEIGE_POT     1
#define ANZEIGE_TAST    0
#define ANZEIGE_CALIB   2
#define ANZEIGE_ADC     3
#define ANZEIGE_SLAVE   4
#define ANZEIGE_EEPROM  5

#define ANZEIGE_LEVEL  6
#define ANZEIGE_DATA    7

#define HOMESCREEN      0
#define MODELLSCREEN    1 // SYMPEL, TROTTLE ...
#define FUNKTIONSCREEN  2 // YAW,PITCH ...
#define AKTIONSCREEN    3 // LEVEL, EXPO ...


#define MODUSSCREEN     5

   #define POT0LO 620  // Min wert vom ADC Pot 0
   #define POT0HI 3400 // Max wert vom ADC Pot 0


   #define POT1LO 620  // Min wert vom ADC Pot 1
   #define POT1HI 3480 // Max wert vom ADC Pot 1



#define POTLO   1300
#define POTHI  2900


//Impulslaenge, ms
#define PPMLO  850  // Minwert ms fuer Impulslaenge
#define PPMHI  2150 // Maxwert ms fur Impulslaenge

// 
#define  MODELL         0
#define  SIM            1
#define  CALIB          2

#define SAVE            1
#define CANCEL          0
#define CHANGED         2
#define CALIB_START     3
#define CALIB_END       4

#define PPM_DIR_PIN     3
#define PPM_DATA_PIN    4  // PPM-Eingang an Pin 4

#define  NUM_CHANNELS   8;
#define  FRAME_LENGTH   40000; // 20 ms
#define  SYNC_PULSE     30;     // 300 µs



#define EEPROM_WRITE 0
#define EEPROM_READ 1

#define EEPROMINDEX_U 0x10
#define EEPROMINDEX_O 0x20
#define EEPROMINDEX_M 0x30

#define EEPROMLEVELSETTINGS 0x40
#define EEPROMEXPOSETTINGS 0x48

#define EEPROMSLAVEINDEX_M 0x56

// defines for PINS
// links
#define PITCH_PIN A3 // PSB2: A6
#define YAW_PIN A2   // PCB2: A3

// rechts
#define ROLL_PIN A1
#define THROTTLE_PIN A0

#define TASTATUR_PIN A2
#define TASTE_OFF 0
#define TASTE_ON 1

#define BATT_PIN A6

#define SLAVEDELAY      0



#define arrow_width 16
#define arrow_height 16

static unsigned char pfeil_right[] = 
{
   0x00,0x02,0x06,0x0e,0x1e,0x3e,0x7e,0xfe,0xfe,0x7e,0x3e,0x1e,
 0x0e,0x06,0x02,0x00
};
static unsigned char pfeil_left[] = 
{
0x80,0xc0,0xe0,0xf0,0xf8,0xfc,0xfe,0xff,0xfe,0xfc,0xf8,0xf0,
 0xe0,0xc0,0x80,0x00
   };

// https://xbm.jazzychad.net

   static unsigned char pfeil_up[] = 
   {
      0x80,0x00,0xc0,0x01,0xe0,0x03,0xf0,0x07,0xf8,0x0f,0xfc,0x1f,0xfe,0x3f,0x00,0x00
   };



 static unsigned char pfeil_down[] = 
  {
 0x00,0x00,0xfe,0x3f,0xfc,0x1f,0xf8,0x0f,0xf0,0x07,0xe0,0x03,
 0xc0,0x01,0x80,0x00
  };





#define arrow_left_width 8
#define arrow_left_height 8

#endif


