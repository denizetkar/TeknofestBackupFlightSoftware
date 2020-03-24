//------------------------------------------------------------------------------------------------
//---------------------includes-------------------------------------------------------------------

#include <Adafruit_BMP280.h>

#include "Neo6MGPS.h"

#define TSA_TIME_IN 0
#define TSA_RESPONSE_TIME_OUT 10000
#define PJON_MAX_PACKETS 0
#define PJON_PACKET_MAX_LENGTH 16
#include <PJON.h>

#include "EarthPositionFilter.h"
#include <Servo.h>

#include <float.h>
#include <Deque.h>

//------------------------------------------------------------------------------------------------
//---------------------definitions----------------------------------------------------------------

#define BMP280_DATA_PERIOD_US 8000

#define BMP280_ALT_VAR 2500
#define GPS_ALT_VAR 9

#define MIN_NUM_GPS 6

#define GPS_TX_PIN 2
#define GPS_RX_PIN 3
#define GPS_BAUD_RATE 9600

#define MAIN_COMP_TX_PIN 8
#define MAIN_COMP_RX_PIN 9
#define MAIN_COMP_BAUD_RATE 31250
#define REDUNDANT_COMP_BUS_ID 44
#define MAIN_COMP_BUS_ID 45

#define MAIN_COMP_TIMEOUT_MS 1000
#define MAIN_S_FRAME_BUF_LEN 2

#define CONTROLLER0_PINS 4
#define CONTROLLER1_PINS 5
#define CONTROLLER2_PINS 6
#define CONTROLLER3_PINS 7

#define APOGEE_DETECTION_DT_MS 500
#define MAIN_RECOVERY_ALTITUDE 600

//------------------------------------------------------------------------------------------------
//---------------------setup and loop objects-----------------------------------------------------

Adafruit_BMP280 bmp; // I2C

// The Neo-6M GPS object
Neo6MGPS neo6m(GPS_TX_PIN, GPS_RX_PIN);

// Kalman Filter object for Altitude
EarthPositionFilter alt_filter;

// Servo object for controlling fins
Servo fin_servo0, fin_servo1, fin_servo2, fin_servo3;

// deque object for storing last 3 altitude measurements
Deque<double, 3> alt_q;

//------------------------------------------------------------------------------------------------
//---------------------setup and loop constants---------------------------------------------------

//------------------------------------------------------------------------------------------------
//---------------------setup and loop variables---------------------------------------------------

// Enumeration of flight states
enum FlightState : uint8_t {
  _BEFORE_FLIGHT = 0,
  _FLYING = 1,
  _FALLING_FAST = 2,
  _FALLING_SLOW = 3,
  _MAIN_COMP_SAFE_FAIL = 4
};
FlightState FLIGHT_STATE = FlightState::_BEFORE_FLIGHT;
uint8_t frame_buffer[PJON_PACKET_MAX_LENGTH];
uint32_t last_loop_time, last_alt_time;

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
  alt_q.resize(alt_q.max_length());
  for (int i = 0; i < alt_q.max_length(); ++i) {
    alt_q[i] = -DBL_MAX;
  }

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
  uint8_t main_state, batch_len;
  Serial.println(F("Polling the main computer for FLIGHT_STATE until it fails..."));
  last_loop_time = millis();
  while (millis() - last_loop_time < MAIN_COMP_TIMEOUT_MS) {
    batch_len = secure_ms.strategy.receive_frame(frame_buffer, MAIN_S_FRAME_BUF_LEN);
    if (batch_len == 1) {
      main_state = frame_buffer[0];
      if (main_state == FlightState::_MAIN_COMP_SAFE_FAIL)
        break;
      else if (main_state < FlightState::_MAIN_COMP_SAFE_FAIL) {
        FLIGHT_STATE = main_state;
        last_loop_time = millis();
      }
    }
  }
  main_s.end();
  neo6m.ss.begin(GPS_BAUD_RATE);

  // TODO: TAKE OVER ALL OF THE CONTROLS FROM THE MAIN COMPUTER !!!
  fin_servo0.attach(CONTROLLER0_PINS);
  fin_servo1.attach(CONTROLLER1_PINS);
  fin_servo2.attach(CONTROLLER2_PINS);
  fin_servo3.attach(CONTROLLER3_PINS);
  // Set all fins at default angle (0 degrees)
  fin_servo0.writeMicroseconds(1500);
  fin_servo1.writeMicroseconds(1500);
  fin_servo2.writeMicroseconds(1500);
  fin_servo3.writeMicroseconds(1500);
  last_loop_time = micros();
  last_alt_time = millis();
}

//-----------------------------------------------------------------------------------------------
//---------------------loop function-------------------------------------------------------------

void loop() {

  // constant velocity process model
  alt_filter.set_deltat((micros() - last_loop_time) / 1000000.0);
  alt_filter.predict(0.0);

  Serial.print(F("FLIGHT_STATE: "));
  Serial.print(FLIGHT_STATE);

  bmp.readTemperature(&temp);
  Serial.print(F("\tTemperature(*C): "));
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

  // Modify FLIGHT_STATE according to the altitude estimate.
  // If recovery conditions are met, then initiate recovery (drogue&main recovery)!
  if (FLIGHT_STATE == FlightState::_FLYING) {
    if (millis() - last_alt_time >= APOGEE_DETECTION_DT_MS) {
      last_alt_time += APOGEE_DETECTION_DT_MS;
      alt_q.pop_front();
      alt_q.push_back(alt_filter.get_pos_m());
      if (alt_q[2] < alt_q[1] && alt_q[1] < alt_q[0]) {
        Serial.println(F("Apogee reached!"));

        // TODO: Initiate drogue recovery HERE!!!!!!!!!!!!!!
        FLIGHT_STATE = FlightState::_FALLING_FAST;
      }
    }
  } else if (FLIGHT_STATE == FlightState::_FALLING_FAST) {
    if (alt_filter.get_pos_m() < ground_alt_m + MAIN_RECOVERY_ALTITUDE) {
      Serial.println(F("Less than 600m to ground!"));

      //TODO: Initiate main recovery HERE!!!!!!!!!!!!!!
      FLIGHT_STATE = FlightState::_FALLING_SLOW;
    }
  } else if (FLIGHT_STATE == FlightState::_FALLING_SLOW) {
  } else { /*FlightState::_MAIN_COMP_SAFE_FAIL*/ }

  // Wait for BMP280 data to be ready
  while (micros() - last_loop_time < BMP280_DATA_PERIOD_US);
  last_loop_time += BMP280_DATA_PERIOD_US;
}
