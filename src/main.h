//
//  main.h
//  H032_ESP32_R
//
//  Created by Ruedi Heimlicher on 22.11.2013.
//
//
#include <stdint.h>



typedef struct canal_struct 
{
  uint16_t speedA;
  uint16_t speedB;
  uint16_t rx;
  uint16_t ry;

  uint8_t node_id;

  uint16_t x;
  uint16_t y;
}canal_struct;

//uint8_t       curr_model; // aktuelles modell

#define BATTX        118
#define BATTY        2
#define BATTH        44
#define BATTB        7

