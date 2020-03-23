//------------------------------------------------------------------------------------------------
//---------------------includes-------------------------------------------------------------------

#include <Adafruit_BMP280.h>
#include <NeoSWSerial.h>

#include "Neo6MGPS.h"

#define TSA_TIME_IN 0
#define TSA_RESPONSE_TIME_OUT 5000
#define PJON_MAX_PACKETS 0
#define PJON_PACKET_MAX_LENGTH 16
#include <PJON.h>

#include "EarthPositionFilter.h"

//------------------------------------------------------------------------------------------------
//---------------------definitions----------------------------------------------------------------

#define BMP280_ALT_VAR 2500
#define GPS_ALT_VAR 9

#define MIN_NUM_GPS 6

#define GPS_TX_PIN 8
#define GPS_RX_PIN 9
#define GPS_BAUD_RATE 9600

#define MAIN_COMP_TX_PIN 2
#define MAIN_COMP_RX_PIN 4
#define MAIN_COMP_BAUD_RATE 9600
#define REDUNDANT_COMP_BUS_ID 44
#define MAIN_COMP_BUS_ID 45

#define MAIN_COMP_TIMEOUT 1000

//------------------------------------------------------------------------------------------------
//---------------------setup and loop objects-----------------------------------------------------

Adafruit_BMP280 bmp; // I2C

// The Neo-6M GPS object
Neo6MGPS neo6m(GPS_TX_PIN, GPS_RX_PIN);

// Kalman Filter object for Altitude
EarthPositionFilter alt_filter;

//------------------------------------------------------------------------------------------------
//---------------------setup and loop constants---------------------------------------------------

//------------------------------------------------------------------------------------------------
//---------------------setup and loop variables---------------------------------------------------

// Enumeration of flight states
enum FlightState: uint8_t {
  _BEFORE_FLIGHT = 0,
  _FLYING = 1,
  _FALLING_FAST = 2,
  _FALLING_SLOW = 3,
  _MAIN_COMP_SAFE_FAIL = 4
};
FlightState FLIGHT_STATE = FlightState::_BEFORE_FLIGHT;
uint32_t last_read_time, now;

double temp = 15.0, pressure = 101325.0, altitude = 0.0, lat_m, lon_m, alt_m;
double ground_level_pressure_hpa, ground_alt_m;

//------------------------------------------------------------------------------------------------
//---------------------other functions------------------------------------------------------------

void receiver_function(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
  /* Make use of the payload before sending something, the buffer where payload points to is
     overwritten when a new message is dispatched */
  FLIGHT_STATE = payload[0];
};

//------------------------------------------------------------------------------------------------
//---------------------setup function-------------------------------------------------------------

void setup() {
  // The serial object to communicate with the main computer
  NeoSWSerial main_s(MAIN_COMP_TX_PIN, MAIN_COMP_RX_PIN);
  PJON<ThroughSerialAsync> secure_ms(REDUNDANT_COMP_BUS_ID);

  Serial.begin(230400);
  while (!Serial);

  Serial.println(F("Initializing GPS module..."));
  neo6m.begin(GPS_BAUD_RATE);

  Serial.println(F("Initializing the barometer sensor..."));
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
  bmp.readPressure(&ground_level_pressure_hpa);
  ground_level_pressure_hpa /= 100;

  // ODR = 125Hz
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X2,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  // Make sure our GPS module uses enough GPS satellites and initialize current position.
  Serial.println(F("Searching for GPS satellites..."));
  while (true) {
    if (neo6m.try_read_gps(lat_m, lon_m, alt_m, MIN_NUM_GPS)) {
      // FOUND at least 'MIN_NUM_GPS' GPS satellites!
      alt_filter.set_pos_m(alt_m);
      ground_alt_m = alt_m;
      break;
    }
  }
  neo6m.ss.end();

  Serial.println(F("Initializing communication with main computer..."));
  main_s.begin(MAIN_COMP_BAUD_RATE);
  secure_ms.strategy.set_serial(&main_s);
  secure_ms.include_sender_info(false);
  secure_ms.set_receiver(receiver_function);
  secure_ms.begin();

  Serial.println(F("Waiting for _BEFORE_FLIGHT signal from the main computer..."));
  while (!(secure_ms.receive() == PJON_ACK && FLIGHT_STATE == FlightState::_BEFORE_FLIGHT));

  Serial.println(F("Waiting for _FLYING signal from the main computer..."));
  while (!(secure_ms.receive() == PJON_ACK && FLIGHT_STATE == FlightState::_FLYING));

  // Get FLIGHT_STATE from main flight computer in a loop.
  // Loop until taking over the recovery management,
  // IF watchdog signal timeouts OR main computer hands over
  // the control as a soft failure (because of lack of sensor data).
  uint8_t main_state;
  Serial.println(F("Polling the main computer for FLIGHT_STATE until it fails..."));
  last_read_time = millis();
  while (millis() - last_read_time < MAIN_COMP_TIMEOUT) {
    if (main_s.available()) {
      main_state = main_s.read();
      if (main_state == FlightState::_MAIN_COMP_SAFE_FAIL)
        break;
      FLIGHT_STATE = main_state;
      last_read_time = millis();
    }
  }
  main_s.end();
  neo6m.ss.begin(GPS_BAUD_RATE);

  // TODO: TAKE OVER ALL OF THE CONTROLS FROM THE MAIN COMPUTER !!!
  last_read_time = micros();
}

//-----------------------------------------------------------------------------------------------
//---------------------loop function-------------------------------------------------------------

void loop() {

  now = micros();
  // constant velocity process model
  alt_filter.set_deltat((now - last_read_time) / 1000000.0);
  alt_filter.predict(0.0);

  bmp.readTemperature(&temp);
  Serial.print(F("Temperature(*C): "));
  Serial.print(temp);

  bmp.readPressure(&pressure);
  Serial.print(F("\tPressure(Pa): "));
  Serial.print(pressure);

  // Get barometer data to update the altitude estimate
  if (bmp.readAltitude(&altitude, ground_level_pressure_hpa)) {
    alt_filter.set_R(BMP280_ALT_VAR);
    alt_filter.update(altitude + ground_alt_m);
  }
  // Get GPS data if available and 'update' the position and velocity 'prediction's
  if (neo6m.try_read_gps(lat_m, lon_m, alt_m)) {
    Serial.print(F("\tGPS altitude(m): "));
    Serial.print(alt_m, 4);
    alt_filter.set_R(GPS_ALT_VAR);
    alt_filter.update(alt_m);
  }
  Serial.print(F("\tFiltered altitude(m): "));
  Serial.println(alt_filter.get_pos_m(), 4); /* Adjusted to local forecast! */

  last_read_time = now;
  //TODO: Modify FLIGHT_STATE according to the altitude estimate.
  //TODO: If recovery conditions are met, then initiate recovery (drogue&main recovery)!
  delay(8);
}
