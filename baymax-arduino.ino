#include <EnableInterrupt.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "cmdconst.h"
#include "gpioconst.h"
//#include "interruptfunctions.h"
const int buttonPin = 5;
int buttonState = 0;
void registerInterrupts();

int currentValue = 0;
int voltageValue = 0;
bool doorDriverState = 0;
bool doorPassengerState = 0;
bool doorHoodState = 0;
bool doorTrunkState = 0; 
bool reserveState = 0;

#define ONE_WIRE_PIN 2

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);


void setup() {
  Serial.begin(9600);
  pinMode (DOOR_DRIVER_PIN, INPUT);
  pinMode (DOOR_PASSENGER_PIN, INPUT);
  registerInterrupts();
  //initTimer2();
  //enableInterrupt(DOOR_DRIVER_PIN, doorDriverStateChanged, CHANGE);
}

void loop() {  
  sensors.requestTemperatures(); // Send the command to get temperatures
  noInterrupts();
  Serial.write(2);
  Serial.write(TEMPERATURE);
  Serial.write(1);
  String buff = String(sensors.getTempCByIndex(0), 3);
  Serial.write(buff[0]);
  Serial.write(buff[1]);
  Serial.write(buff[2]);
  Serial.write(buff[3]);
  Serial.write(3);
  
  voltageChanged();
  currentChanged();
  interrupts();
  delay(1000);
}

void serialEvent() {
  while (Serial.available()) {
     
  }
}


void registerInterrupts() {
  //enableInterrupt(BATTERY_VOLTAGE_PIN, voltageChanged, CHANGE);
  //enableInterrupt(BATTERY_CURRENT_PIN, currentChanged, CHANGE);
  enableInterrupt(DOOR_DRIVER_PIN, doorDriverStateChanged, CHANGE);
  enableInterrupt(DOOR_PASSENGER_PIN, doorPassengerStateChanged, CHANGE);
  enableInterrupt(DOOR_HOOD_PIN, doorHoodStateChanged, CHANGE);
  enableInterrupt(DOOR_TRUNK_PIN, doorTrunkStateChanged, CHANGE);
  enableInterrupt(RESERVE_PIN, reserveStateChanged, CHANGE);
}


ISR(TIMER2_COMPA_vect) {
  stopTimer2();
  
  bool currentDoorDriverState = digitalRead(DOOR_DRIVER_PIN);
  bool currentDoorPassengerState = digitalRead(DOOR_PASSENGER_PIN);
  bool currentDoorHoodState = digitalRead(DOOR_HOOD_PIN);
  bool currentDoorTrunkState = digitalRead(DOOR_TRUNK_PIN);
  bool currentReserverState = digitalRead(RESERVE_PIN);
  if (currentDoorDriverState != doorDriverState) {
    doorDriverState = currentDoorDriverState;
    doorDriverStateChangedAction();
  }
  if (currentDoorPassengerState != doorPassengerState) {
    doorPassengerState = currentDoorPassengerState;
    doorPassengerStateChangedAction();
  }
  if (currentDoorHoodState != doorHoodState) {
    doorHoodState = currentDoorHoodState;
    doorHoodStateChangedAction();
  }
  if (currentDoorTrunkState != doorTrunkState) {
    doorTrunkState = currentDoorTrunkState;
    doorTrunkStateChangedAction();
  }
  if (currentReserverState != reserveState) {
    reserveState = currentReserverState;
    reserveStateChangedAction();
  }
  
}

void sendChangedCommand(unsigned char cmd, unsigned char value) {
    Serial.write(2);
    Serial.write(cmd);
    Serial.write(value);
    Serial.write(3);
}


void startTimer2() {
  TCNT2  = 0;
  TIMSK2 |= (1 << OCIE2A);  
}

void stopTimer2() {
  TIMSK2 &= ~(1 << OCIE2A); 
}

void initTimer2() {
    //set timer0 interrupt at 500Hz
  TCCR2A = 0;// set entire TCCR0A register to 0
  TCCR2B = 0;// same for TCCR0B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 500Hz increments
  OCR2A = 46;// = (16*10^6) / (500*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS02 for 256 prescaler
  TCCR2B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK2 &= ~(1 << OCIE2A);
}

void doorDriverStateChangedAction() {
    sendChangedCommand(DOOR_DRIVER_STATE_CHANGED, doorDriverState);
}

void doorPassengerStateChangedAction() {
    sendChangedCommand(DOOR_PASSENGER_STATE_CHANGED, doorPassengerState);
}

void doorHoodStateChangedAction() {
    sendChangedCommand(DOOR_HOOD_STATE_CHANGED, doorHoodState);
}

void doorTrunkStateChangedAction() {
    sendChangedCommand(DOOR_TRUNK_STATE_CHANGED, doorTrunkState);
}

void reserveStateChangedAction() {
    sendChangedCommand(RESERVE_STATE_CHANGED, reserveState);
}

void voltageChanged() {
    voltageValue = analogRead(BATTERY_VOLTAGE_PIN);
    String buff = String(voltageValue);
    Serial.write(2);
    Serial.write(BATTERY_VOLTAGE_CHANGED);
    Serial.write(voltageValue/4);
    //Serial.write(buff[1]);
    //Serial.write(buff[2]);
    Serial.write(3);
}

void currentChanged() {
    currentValue = analogRead(BATTERY_CURRENT_PIN);
    String buff = String(currentValue);
    Serial.write(2);
    Serial.write(BATTERY_CURRENT_CHANGED);
    Serial.write(currentValue/4);
    //Serial.write(buff[1]);
    //Serial.write(buff[2]);
    Serial.write(3);
}

void doorDriverStateChanged() {
  startTimer2();
}

void doorPassengerStateChanged() {
  startTimer2();
}

void doorHoodStateChanged() {
  startTimer2();  
}

void doorTrunkStateChanged() {
  startTimer2(); 
}

void reserveStateChanged() {
  startTimer2();  
}
