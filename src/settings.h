//
//  expo.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 01.03.2023.
//
//
#include <stdint.h>
/*
#define TASTE01     67
#define TASTE02     109
#define TASTE03     163
#define TASTE04     253
#define TASTE05     360
#define TASTE06     484
#define TASTE07     628
#define TASTE08     742
#define TASTE09     827
#define TASTEL      899
#define TASTE00     946
#define TASTER      993
*/

/*
#define TASTE01     28
#define TASTE02     62
#define TASTE03     110
#define TASTE04     188
#define TASTE05     255
#define TASTE06     358
#define TASTE07     462
#define TASTE08     525
#define TASTE09     588
#define TASTEL      637
#define TASTE00     659
#define TASTER      680
*/

// Tastatur 3x3
#define TASTE01     120
#define TASTE02     168
#define TASTE03     251
#define TASTE04     320
#define TASTE05     408
#define TASTE06     484
#define TASTE07     546
#define TASTE08     597
#define TASTE09     660

// Bits von sendestatus
#define RUN          0
#define PAUSE        1
#define PAKET        2
#define IMPULS       3
#define ADC_OK       4
#define USB_OK       5
#define ENDEPAKET    7

// bits von programmstatus

#define MOTOR_ON        1
#define STOP_ON         2
#define LOCALTASK       3
#define EXTERNTASK      4


// Bits von masterstatus
#define  USBPRESENT_BIT             4 //  USB eingesteckt
#define  AKKU_LOW_BIT               5 //  Akku ist low
#define  TIMEOUT_BIT                6 //  Beep nach ablauf des Timeout
#define  HALT_BIT                   7 //  Bit 7

#define TIMEOUT               12000             // 2800 ticks fuer 1 minute
#define MANUELLTIMEOUT        32 // Loopled-counts bis Manuell zurueckgesetzt wird. 50: ca. 30s


#define ANZAHLMODELLE 4
#define NUM_SERVOS    4
#define KANALSETTINGBREITE 32
// Bits von eepromstatus



// Bits von displaystatus
#define UHR_UPDATE         0
#define BATTERIE_UPDATE    1



#define UPDATESCREEN    5 // Bit in status wird gesetzt wenn eine Taste gedrueckt ist, reset wenn update ausgefuehrt

#define SETTINGWAIT     6  // Bit in status wird gesetzt bis Taste 5 3 * gedrueckt ist
