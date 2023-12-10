/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by pcbreflux
   Modified to work with NimBLE
   Modified for V2 by Chreece
   Modified by countrysideboy: Code cleanups, Chop Data, Encryption
   BLE advertisement format from https://bthome.io/

*/

#include "BTHome.h"

#define DEVICE_NAME "MansNotHot"  // The name of the sensor
#define TRIGGER_BASED 1
#define TRIGGER_DEBOUNCE_MS 50
// #define ENABLE_ENCRYPT // Remove this line for no encryption
String BIND_KEY = "231d39c1d7cc1ab1aee224cd096db932"; // Change this key with a string containing 32 of: a-f and 0-9 characters (hex) this will be asked in HA

BTHome bthome;

const int en_button = 0;
const int externalLED = 21;
const int externalTrigger = 18;

void setup() {
  Serial.begin(115200);

  pinMode(externalTrigger, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(externalTrigger), externalTriggerEvent, FALLING);

  pinMode(externalLED, OUTPUT); 
  digitalWrite(externalLED, HIGH);
#ifdef ENABLE_ENCRYPT
  bthome.begin(DEVICE_NAME, true, BIND_KEY);
#else
  bthome.begin(DEVICE_NAME, false, "", TRIGGER_BASED);
#endif
}

int externalTriggerLastEventMillis = 0;
void externalTriggerEvent() {
  int lastEventMillsDelta = millis() - externalTriggerLastEventMillis;
  if(lastEventMillsDelta < TRIGGER_DEBOUNCE_MS) {
    String message = "externalTrigger debounce, last event " + String(lastEventMillsDelta) + "ms ago";
    Serial.println(message);
    return;
  }

  String message = "externalTrigger";
  Serial.println(message);
  toggleLED(externalLED);
}

void toggleLED(int pin) {
  int state = digitalRead(pin);
  if(state == LOW) {
    digitalWrite(pin, HIGH);  
  } else {
    digitalWrite(pin, LOW);
  }
}

int loop_counter = 0;
int loop_start = 0;
int loop_duration = 0;

void loop() {
  loop_start = millis();

  float temperature = temperatureRead();
  int hall = hallRead();
  int triggered = digitalRead(externalTrigger) == LOW;

  announceTemperature(temperature);
  String message = "Counter: " + String(loop_counter) + ", Since last: " + String(loop_duration) + ", Temperature: " + String(temperature, 2) + "C, Hall: " + String(hall) + ", button: " + String(triggered);
  Serial.println(message);
  if(triggered) {
    announceMotionEvent();
  }

  delay(3 * 1000);
  
  loop_counter++;
  loop_duration = millis() - loop_start;
}

void announceTemperature(float value) {
  String message = "Announcing temperature";
  Serial.println(message);

  bthome.resetMeasurement();
  bthome.addMeasurement(ID_TEMPERATURE_PRECISE, value);
  bthome.sendPacket();
  bthome.stop();
}

void announceMotionEvent() {
  int motion_state = 1;
  String message = "Announcing motion";
  Serial.println(message);

  bthome.resetMeasurement();
  bthome.addMeasurement_state(STATE_MOTION, motion_state);
  bthome.sendPacket();
  bthome.stop();
}

//Object ids by order
#if 0
#define ID_PACKET				0x00
#define ID_BATTERY				0x01
#define ID_TEMPERATURE_PRECISE 	0x02
#define ID_HUMIDITY_PRECISE 	0x03
#define ID_PRESSURE 			0x04
#define ID_ILLUMINANCE			0x05
#define ID_MASS					0x06
#define ID_MASSLB				0x07
#define ID_DEWPOINT				0x08
#define ID_COUNT				0x09
#define ID_ENERGY				0x0A
#define ID_POWER				0x0B
#define ID_VOLTAGE				0x0C
#define ID_PM25					0x0D
#define ID_PM10					0x0E
#define STATE_GENERIC_BOOLEAN	0x0F
#define STATE_POWER_ON			0x10
#define STATE_OPENING			0x11
#define ID_CO2					0x12
#define ID_TVOC					0x13
#define ID_MOISTURE_PRECISE		0x14
#define STATE_BATTERY_LOW		0x15
#define STATE_BATTERY_CHARHING	0x16
#define STATE_CO				0x17
#define STATE_COLD				0x18
#define STATE_CONNECTIVITY		0x19
#define STATE_DOOR				0x1A
#define STATE_GARAGE_DOOR		0x1B
#define STATE_GAS_DETECTED		0x1C
#define STATE_HEAT				0x1D
#define STATE_LIGHT				0x1E
#define STATE_LOCK				0x1F
#define STATE_MOISTURE			0x20
#define STATE_MOTION			0x21
#define STATE_MOVING			0x22
#define STATE_OCCUPANCY			0x23
#define STATE_PLUG				0x24
#define STATE_PRESENCE			0x25
#define STATE_PROBLEM			0x26
#define STATE_RUNNING			0x27
#define STATE_SAFETY			0x28
#define STATE_SMOKE				0x29
#define STATE_SOUND				0x2A
#define STATE_TAMPER			0x2B
#define STATE_VIBRATION			0x2C
#define STATE_WINDOW			0x2D
#define ID_HUMIDITY				0x2E
#define ID_MOISTURE				0x2F
#define EVENT_BUTTON			0x3A
#define EVENT_DIMMER			0x3C
#define ID_COUNT2				0x3D
#define ID_COUNT4				0x3E
#define ID_ROTATION				0x3F
#define ID_DISTANCE				0x40
#define ID_DISTANCEM			0x41
#define ID_DURATION				0x42
#define ID_CURRENT				0x43
#define ID_SPD					0x44
#define ID_TEMPERATURE			0x45
#define ID_UV					0x46
#define ID_VOLUME1				0x47
#define ID_VOLUME2				0x48
#define ID_VOLUMEFR				0x49
#define ID_VOLTAGE1				0x4A
#define ID_GAS					0x4B
#define ID_GAS4					0x4C
#define ID_ENERGY4				0x4D
#define ID_VOLUME				0x4E
#define ID_WATER				0x4F
#endif
