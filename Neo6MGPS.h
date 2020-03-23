#ifndef NEO6MGPS_H
#define NEO6MGPS_H

#include <Arduino.h>
#include <AltSoftSerial.h>
#include <TinyGPS++.h>

void lat_lon_to_x_y_m(double, double, double&, double&);

class Neo6MGPS {
  public:
    AltSoftSerial ss;
    TinyGPSPlus gps;

    Neo6MGPS(int, int);

    void begin(uint16_t);
    bool try_read_gps(double&, double&, double&, uint8_t num_gps=4);
};

#endif
