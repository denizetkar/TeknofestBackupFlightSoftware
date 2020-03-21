//------------------------------------------------------------------------------------------------
//---------------------includes-------------------------------------------------------------------

#include <Adafruit_BMP280.h>
#include <NeoSWSerial.h>

#define TSA_TIME_IN 0
#define TSA_RESPONSE_TIME_OUT 5000
#define PJON_MAX_PACKETS 0
#define PJON_PACKET_MAX_LENGTH 16
#include <PJON.h>

//------------------------------------------------------------------------------------------------
//---------------------definitions----------------------------------------------------------------

#define MAIN_COMP_TX_PIN 2
#define MAIN_COMP_RX_PIN 4
#define MAIN_COMP_BAUD_RATE 38400
#define REDUNDANT_COMP_BUS_ID 44
#define MAIN_COMP_BUS_ID 45

#define MAIN_COMP_TIMEOUT 1000

//------------------------------------------------------------------------------------------------
//---------------------setup and loop objects-----------------------------------------------------

Adafruit_BMP280 bmp; // I2C

// The serial object to communicate with the main computer
NeoSWSerial main_s(MAIN_COMP_TX_PIN, MAIN_COMP_RX_PIN);

//------------------------------------------------------------------------------------------------
//---------------------setup and loop constants---------------------------------------------------

//------------------------------------------------------------------------------------------------
//---------------------setup and loop variables---------------------------------------------------

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
uint32_t last_read_time;

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
  PJON<ThroughSerialAsync> secure_ms(REDUNDANT_COMP_BUS_ID);

  Serial.begin(230400);
  while (!Serial);

  Serial.println(F("Initializing communication with main computer..."));
  main_s.begin(MAIN_COMP_BAUD_RATE);
  secure_ms.strategy.set_serial(&main_s);
  secure_ms.include_sender_info(false);
  secure_ms.set_receiver(receiver_function);
  secure_ms.begin();

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
  ground_level_pressure_hpa = bmp.readPressure() / 100;

  // ODR = 125Hz
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X2,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

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

  // TODO: TAKE OVER ALL OF THE CONTROLS FROM THE MAIN COMPUTER !!!
}

//-----------------------------------------------------------------------------------------------
//---------------------loop function-------------------------------------------------------------

void loop() {

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature(), 6);
  Serial.print(F(" *C"));

  Serial.print(F("\tPressure = "));
  Serial.print(bmp.readPressure(), 6);
  Serial.print(F(" Pa"));

  Serial.print(F("\tHeight from ground = "));
  Serial.print(bmp.readAltitude(ground_level_pressure_hpa), 6); /* Adjusted to local forecast! */
  Serial.println(F(" m"));

  Serial.println();

  //TODO: Modify FLIGHT_STATE according to the altitude estimate.
  //TODO: If recovery conditions are met, then initiate recovery (drogue&main recovery)!
  delay(8);
}
