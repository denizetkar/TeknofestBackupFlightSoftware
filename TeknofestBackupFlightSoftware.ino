#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C
static float ground_level_pressure_hpa;
// Enumeration of flight states
enum FlightState: uint8_t {
  _BEFORE_FLIGHT = 0,
  _FLYING = 1,
  _FALLING_FAST = 2,
  _FALLING_SLOW = 3,
  _MAIN_COMP_SAFE_FAIL = 4
};
FlightState FLIGHT_STATE = FlightState::_BEFORE_FLIGHT;

void setup() {
  Serial.begin(2000000);
  while(!Serial);

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
  delay(200);
  ground_level_pressure_hpa = bmp.readPressure() / 100;

  // ODR = 125Hz
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X2,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  //TODO: Wait for _BEFORE_FLIGHT signal from the main computer
  //      WITHOUT timeout!

  //TODO: Wait for initial _FLYING signal from the main computer
  //      WITHOUT timeout!

  //TODO: Get FLIGHT_STATE from main flight computer in a loop here.
  //TODO: Loop forever until taking over the recovery management,
  //      IF watchdog signal timeouts OR main computer hands over
  //      the control as a soft failure (because of lack of sensor data).
}

void loop() {

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature(), 6);
  Serial.print(" *C");

  Serial.print(F("\tPressure = "));
  Serial.print(bmp.readPressure(), 6);
  Serial.print(" Pa");

  Serial.print(F("\tHeight from ground = "));
  Serial.print(bmp.readAltitude(ground_level_pressure_hpa), 6); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println();

  //TODO: Modify FLIGHT_STATE according to the altitude estimate.
  //TODO: If recovery conditions are met, then initiate recovery (drogue&main recovery)!
  delay(8);
}
