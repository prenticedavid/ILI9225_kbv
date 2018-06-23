# ILI9225_kbv
Library for ILI9225 in SPI mode

Install "Adafruit_GFX.h" library

The Ebay board has pins that mate with the analog and power header of an Arduino. 
Early boards had an LED pin that takes 20mA.   Fine for Uno, Mega.   Too much for Zero, Due.

Boards with LED pin require 5V on the LED pin for the backlight.
All boards will work with either 5V or 3.3V logic.   (onboard 74HC245 buffer)

Current boards have NC in the A0 position precviously occupied by LED.   Backlight is always on.

Either use bit-banged GPIO on A0-A5 with the board plugged into headers
Or use the proper hardware SPI pins.   The hardware constructor defaults to MY cs, dc, rst wiring)

Hardware constructor:
  ILI9225_kbv(int8_t cs=10, int8_t dc=9, int8_t rst=8);

Software constructor:
  ILI9225_kbv(int8_t cs, int8_t dc, int8_t mosi, int8_t sclk, int8_t rst, int8_t led=0);

Bit-banged SPI is appallingly slow on a Uno / Mega.   Hardware SPI is reasonable.
Bit-banged SPI is actually faster on the STM32 Core than hardware SPI.
MapleCore is better for hardware SPI.
