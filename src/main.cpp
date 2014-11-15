#include <QCoreApplication>
#include "stdint.h"
#include "BMP183.h"

#define SENSORS_PRESSURE_SEALEVELHPA    (1013.25F)      /**<  Average sea level pressure in hPa*/

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);


    printf("Starting application\n");

    BMP183 bmp("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
    if (!bmp.begin()) {
        printf("No BMP183 detected. Aborting!");
        return 0;
    }

    int i = 50;

    uint32_t pressure;
    float temperature;
    float altitude;

    while (i>0) {
        pressure    = bmp.getPressure();
        temperature = bmp.getTemperature();
        altitude    = bmp.getAltitude(SENSORS_PRESSURE_SEALEVELHPA);

        printf("Pressure:    %d Pa\n", pressure);
        printf("Temperature: %f C\n", temperature);
        printf("Altitude:    %f mPa\n\n", altitude);

        Sleeper::msleep(1000000);

        i--;
    }


    return a.exec();
}
