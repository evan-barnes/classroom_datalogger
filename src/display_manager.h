#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

#define DISPLAY_CLK 5
#define DISPLAY_DI 6
#define DISPLAY_CS 9

#define BLACK 0
#define WHITE 1

Adafruit_SharpMem display(DISPLAY_CLK, DISPLAY_DI, DISPLAY_CS, 400, 240);