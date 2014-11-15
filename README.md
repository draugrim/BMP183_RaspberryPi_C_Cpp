This is a library for interfacig the BMP183 Barometric Pressure + Temp sensor
( http://www.adafruit.com/products/1900 ) with the Raspberry Pi (tested on RPi B+)
using C++ (spidev). Code written on Qt 4.8 (should be portable to other environment
without major problems)

Credits:
[1] The core library was written by Limor Fried/Ladyada for Adafruit Industries for
interfacing the BMP183 with the Arduino (see disclaimer below [A]).

  ----> https://github.com/adafruit/Adafruit_BMP183_Library

[2] To interface with the Raspberry, functions from the mcp3008Spi library by Hussam Al-Hertani
were also used: 

  ----> https://github.com/halherta/RaspberryPi-mcp3008Spi


[A] "This is a library for the Adafruit BMP183 Barometric Pressure + Temp sensor

Designed specifically to work with the Adafruit BMP183 Breakout 
  ----> http://www.adafruit.com/products/1900

These Sensors use SPI to communicate, 4 pins are required to interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Check out the links above for our tutorials and wiring diagrams 

Written by Limor Fried/Ladyada for Adafruit Industries.  
BSD license, all text above must be included in any redistribution"

[B]"The mcp3008Spi class enables the Raspberry Pi to communicate with 
the MCP3008 SPI ADC using spidev. The class can be easily modified to
enable the Raspberry Pi to communicate with other SPI devices.

For more information please visit : 
http://hertaville.com/2013/07/24/interfacing-an-spi-adc-mcp3008-chip-to-the-raspberry-pi-using-c/"