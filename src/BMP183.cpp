
#include "BMP183.h"
#include <qmath.h>

using namespace std;

/*************************************************
* Default constructor. Set member variables to
* default values.
* Code taken integraly from the mcp30008Spi library, from "halherta"
* https://github.com/halherta/RaspberryPi-mcp3008Spi
**********************************************/

BMP183::BMP183(){
    this->mode = SPI_MODE_0 ;
    this->bitsPerWord = 8;
    this->speed = 1000000;
    this->spifd = -1;
    this->spiOpen(std::string("/dev/spidev0.0"));
}

/*************************************************
* Overloaded constructor. Let user set member variables
* Code taken integraly from the mcp30008Spi library, from "halherta"
* https://github.com/halherta/RaspberryPi-mcp3008Spi
* ***********************************************/
BMP183::BMP183(std::string devspi, unsigned char spiMode, unsigned int spiSpeed, unsigned char spibitsPerWord){
    this->mode = spiMode ;
    this->bitsPerWord = spibitsPerWord;
    this->speed = spiSpeed;
    this->spifd = -1;
    this->spiOpen(devspi);
}

/**********************************************
* Destructor
* Code taken integraly from the mcp30008Spi library, from "halherta"
* https://github.com/halherta/RaspberryPi-mcp3008Spi
* ********************************************/
BMP183::~BMP183(){
    this->spiClose();
}

/**********************************************************
* spiOpen() : called by the constructor.
* Opens the spidev device and sets up the spidev interface.
* Private member variables must be set appropriately by
* constructor before calling this function.
* Code taken integraly from the mcp30008Spi library, from "halherta"
* https://github.com/halherta/RaspberryPi-mcp3008Spi
* *********************************************************/
int BMP183::spiOpen(std::string devspi){
    int statusVal = -1;
    this->spifd = open(devspi.c_str(), O_RDWR);
    if(this->spifd < 0){
        perror("could not open SPI device");
        exit(1);
    }

    statusVal = ioctl (this->spifd, SPI_IOC_WR_MODE, &(this->mode));
    if(statusVal < 0){
        perror("Could not set SPIMode (WR)...ioctl fail");
        exit(1);
    }

    statusVal = ioctl (this->spifd, SPI_IOC_RD_MODE, &(this->mode));
    if(statusVal < 0) {
      perror("Could not set SPIMode (RD)...ioctl fail");
      exit(1);
    }

    statusVal = ioctl (this->spifd, SPI_IOC_WR_BITS_PER_WORD, &(this->bitsPerWord));
    if(statusVal < 0) {
      perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
      exit(1);
    }

    statusVal = ioctl (this->spifd, SPI_IOC_RD_BITS_PER_WORD, &(this->bitsPerWord));
    if(statusVal < 0) {
      perror("Could not set SPI bitsPerWord(RD)...ioctl fail");
      exit(1);
    }

    statusVal = ioctl (this->spifd, SPI_IOC_WR_MAX_SPEED_HZ, &(this->speed));
    if(statusVal < 0) {
      perror("Could not set SPI speed (WR)...ioctl fail");
      exit(1);
    }

    statusVal = ioctl (this->spifd, SPI_IOC_RD_MAX_SPEED_HZ, &(this->speed));
    if(statusVal < 0) {
      perror("Could not set SPI speed (RD)...ioctl fail");
      exit(1);
    }
    return statusVal;
}

/***********************************************************
* spiClose(): close the spidev interface.
* Called in destructor
* Code taken integraly from the mcp30008Spi library, from "halherta"
* https://github.com/halherta/RaspberryPi-mcp3008Spi
* *********************************************************/

int BMP183::spiClose(){
    int statusVal = -1;
    statusVal = close(this->spifd);
        if(statusVal < 0) {
      perror("Could not close SPI device\n");
      exit(1);
    }
    return statusVal;
}

/***********************************************************
* Code from the original Adafruit_BMP183 library (for Arduino)
* https://github.com/adafruit/Adafruit_BMP183_Library
* *********************************************************/
bool BMP183::begin(bmp183_mode_t mode) {

  // check mode range
  if ((mode > BMP183_MODE_ULTRAHIGHRES) || (mode < 0)) {
    mode = BMP183_MODE_ULTRAHIGHRES;
  }
  // set the mode indicator
  oversampling = mode;

  if (read8(0xD0) != 0x55) {
      printf("Error detecting device. Aborting!\n");
      return false;
  }

  // read calibration data
  ac1 = read16(BMP183_REGISTER_CAL_AC1);
  ac2 = read16(BMP183_REGISTER_CAL_AC2);
  ac3 = read16(BMP183_REGISTER_CAL_AC3);
  ac4 = read16(BMP183_REGISTER_CAL_AC4);
  ac5 = read16(BMP183_REGISTER_CAL_AC5);
  ac6 = read16(BMP183_REGISTER_CAL_AC6);

  b1 = read16(BMP183_REGISTER_CAL_B1);
  b2 = read16(BMP183_REGISTER_CAL_B2);

  mb = read16(BMP183_REGISTER_CAL_MB);
  mc = read16(BMP183_REGISTER_CAL_MC);
  md = read16(BMP183_REGISTER_CAL_MD);

#if (BMP183_DEBUG == 1)
  printf("ac1 = %d\n", ac1);
  printf("ac2 = %d\n", ac2);
  printf("ac3 = %d\n", ac3);
  printf("ac4 = %d\n", ac4);
  printf("ac5 = %d\n", ac5);
  printf("ac6 = %d\n", ac6);

  printf("b1 = %d\n", b1);
  printf("b2 = %d\n", b2);

  printf("mb = %d\n", mb);
  printf("mc = %d\n", mc);
  printf("md = %d\n", md);
#endif

    return true;
}

/***********************************************************
* Code from the original Adafruit_BMP183 library (for Arduino)
* https://github.com/adafruit/Adafruit_BMP183_Library
* *********************************************************/
uint16_t BMP183::readRawTemperature(void) {
    write8(BMP183_REGISTER_CONTROL, BMP183_REGISTER_READTEMPCMD);
    //_delay_ms(5);
    Sleeper::msleep(5000);

#if BMP183_DEBUG == 1
    printf("Raw temp: %d\n", read16(BMP183_REGISTER_TEMPDATA));
#endif
    return read16(BMP183_REGISTER_TEMPDATA);
}

/***********************************************************
* Code from the original Adafruit_BMP183 library (for Arduino)
* https://github.com/adafruit/Adafruit_BMP183_Library
* *********************************************************/
uint32_t BMP183::readRawPressure(void) {
    uint32_t raw;

    write8(BMP183_REGISTER_CONTROL, BMP183_REGISTER_READPRESSURECMD + (oversampling << 6));

    if (oversampling == BMP183_MODE_ULTRALOWPOWER)
        Sleeper::msleep(5000);     //_delay_ms(5);
    else if (oversampling == BMP183_MODE_STANDARD)
        Sleeper::msleep(8000);     //_delay_ms(8);
    else if (oversampling == BMP183_MODE_HIGHRES)
        Sleeper::msleep(14000);    //_delay_ms(14);
    else
        Sleeper::msleep(26000);    //_delay_ms(26);

    raw = read16(BMP183_REGISTER_PRESSUREDATA);
    raw <<= 8;
    raw |= read8(BMP183_REGISTER_PRESSUREDATA+2);
    raw >>= (8 - oversampling);

#if BMP183_DEBUG == 1
    printf("Raw pressure: %d\n", raw);
#endif
    return raw;
}

/***********************************************************
* Code from the original Adafruit_BMP183 library (for Arduino)
* https://github.com/adafruit/Adafruit_BMP183_Library
* *********************************************************/
int32_t BMP183::getPressure(void) {
    int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
    uint32_t B4, B7;

    UT = readRawTemperature();
    UP = readRawPressure();

#if BMP183_DEBUG == 1
    // use datasheet numbers!
    UT = 27898;
    UP = 23843;
    ac6 = 23153;
    ac5 = 32757;
    mc = -8711;
    md = 2868;
    b1 = 6190;
    b2 = 4;
    ac3 = -14383;
    ac2 = -72;
    ac1 = 408;
    ac4 = 32741;
    oversampling = 0;
#endif

    // perform temperature calculations
    X1=(UT-(int32_t)(ac6))*((int32_t)(ac5))/pow(2,15);
    X2=((int32_t)mc*pow(2,11))/(X1+(int32_t)md);
    B5=X1 + X2;

#if BMP183_DEBUG == 1
    printf("X1 = %d\n", X1);
    printf("X2 = %d\n", X2);
    printf("B5 = %d\n", B5);
#endif

    // perform pressure calcs
    B6 = B5 - 4000;
    X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
    X2 = ((int32_t)ac2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

#if BMP183_DEBUG == 1
    printf("B6 = %d\n", B6);
    printf("X1 = %d\n", X1);
    printf("X2 = %d\n", X2);
    printf("B3 = %d\n", B3);
#endif

    X1 = ((int32_t)ac3 * B6) >> 13;
    X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

#if BMP183_DEBUG == 1
    printf("X1 = %d\n", X1);
    printf("X2 = %d\n", X2);
    printf("B4 = %d\n", B4);
    printf("B7 = %d\n", B7);
#endif

    if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;

#if BMP183_DEBUG == 1
    printf("p = %d\n", p);
    printf("X1 = %d\n", X1);
    printf("X2 = %d\n", X2);
#endif

    p = p + ((X1 + X2 + (int32_t)3791)>>4);
#if BMP183_DEBUG == 1
    printf("p = %d\n", p);
#endif
    return p;
}

/***********************************************************
* Code from the original Adafruit_BMP183 library (for Arduino)
* https://github.com/adafruit/Adafruit_BMP183_Library
* *********************************************************/
float BMP183::getTemperature(void) {
    int32_t UT, X1, X2, B5;     // following ds convention
    float temp;

    UT = readRawTemperature();

#if BMP183_DEBUG == 1
    // use datasheet numbers!
    UT = 27898;
    ac6 = 23153;
    ac5 = 32757;
    mc = -8711;
    md = 2868;
#endif

    // step 1
    X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) / pow(2,15);
    X2 = ((int32_t)mc * pow(2,11)) / (X1+(int32_t)md);
    B5 = X1 + X2;
    temp = (B5+8)/pow(2,4);
    temp /= 10;

    return temp;
}

/***********************************************************
* Code from the original Adafruit_BMP183 library (for Arduino)
* https://github.com/adafruit/Adafruit_BMP183_Library
* *********************************************************/
float BMP183::getAltitude(float sealevelPressure) {
    float altitude;
    float pressure = getPressure(); // in Si units for Pascal
    pressure /= 100;
    altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));
    return altitude;
}


/**********************************************
 * This function substitute the function SPIxfer from the
 * original Adafruit_BMP183 library. It was taken integraly
 * from the mcp30008Spi library, from "halherta"
 * https://github.com/halherta/RaspberryPi-mcp3008Spi
 **********************************************/
int BMP183::spiWriteRead( unsigned char *data, int length){

  struct spi_ioc_transfer spi[length];
  int i = 0;
  int retVal = -1;

// one spi transfer for each byte
  for (i = 0 ; i < length ; i++) {
      spi[i].tx_buf        = (unsigned long)(data + i);     // transmit from "data"
      spi[i].rx_buf        = (unsigned long)(data + i) ;    // receive into "data"
      spi[i].len           = sizeof(*(data + i)) ;
      spi[i].delay_usecs   = 0 ;
      spi[i].speed_hz      = this->speed ;
      spi[i].bits_per_word = this->bitsPerWord ;
      spi[i].cs_change     = 0;
  }

 retVal = ioctl (this->spifd, SPI_IOC_MESSAGE(length), &spi) ;

 if(retVal < 0){
     printf("Problem transmitting spi data..ioctl");
     exit(1);
 }
return retVal;

}

/***********************************************************
* MODIFIED from the original Adafruit_BMP183 library (for Arduino)
* https://github.com/adafruit/Adafruit_BMP183_Library
* *********************************************************/
uint8_t BMP183::read8(uint8_t reg) {
    uint8_t value;
    //digitalWrite(_cs, LOW);
    //SPIxfer(0x80 | reg);
    //value = SPIxfer(0x00);
    //digitalWrite(_cs, HIGH);
    unsigned char data[2];
    data[0] = 0x80 | reg;
    data[1] = 0x00;
    spiWriteRead(data, 2);
    value = data[1];

    return value;
}

/***********************************************************
* MODIFIED from the original Adafruit_BMP183 library (for Arduino)
* https://github.com/adafruit/Adafruit_BMP183_Library
* *********************************************************/
uint16_t BMP183::read16(uint8_t reg) {
    uint16_t value = 0x0000;
    //  digitalWrite(_cs, LOW);
    //  SPIxfer(0x80 | reg);
    //  value = SPIxfer(0x00);
    //  value <<= 8;
    //  value |= SPIxfer(0x00);
    //  digitalWrite(_cs, HIGH);
    unsigned char data[3];
    data[0] = 0x80 | reg;
    data[1] = 0x00;
    data[2] = 0x00;
    spiWriteRead(data, 3);
    value = data[1];            //merge data[1] & data[2]
    value <<= 8;
    value |= data[2];

    return value;
}

/***********************************************************
* MODIFIED from the original Adafruit_BMP183 library (for Arduino)
* https://github.com/adafruit/Adafruit_BMP183_Library
* *********************************************************/
void BMP183::write8(uint8_t reg, uint8_t value) {
    //  digitalWrite(_cs, LOW);
    //  SPIxfer(reg & 0x7F);
    //  SPIxfer(value);
    //  digitalWrite(_cs, HIGH);
    unsigned char data[2];
    data[0] = reg & 0x7F;
    data[1] = value;
    spiWriteRead(data, 2);
}
