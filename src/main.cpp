// Libaries
#include <QTRSensors.h>     // For sensorn som blir levert
#include <Arduino.h>
#include "SimpleMotor.hpp"

// Pins
const int QTREmitterPin = 2;
// Sensor pin names reflect the number on the sensor chip
enum Sensor { s17 = A0, s15, s13, s11, s9 };
const int PWMB = 10;
const int Motor2_B02 = 3;
const int Motor2_B01 = 5;
const int Motor1_A02 = 8;
const int Motor1_A01 = 6;
const int PWMA = 11;
//const int onOffSwitch = 7;
// const int VM_Motor_Voltage = VIN     // Ikke i bruk akkurat no.
// A01 og B01 blir brukt for framover bevegelse / A02 og B02 er for backover.


QTRSensors qtr;
const uint8_t sensorCount = 5;
uint16_t sensorValues[sensorCount];

SimpleMotor motor;

// Setter opp pins og default motor verdier.
void setup()
{
    // Setup for Sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]) { s9, s11, s13, s15, s17 }, sensorCount);
    qtr.setEmitterPin(QTREmitterPin);

    // Setter Pin modes
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(Motor1_A01, OUTPUT);
    pinMode(Motor1_A02, OUTPUT);
    pinMode(Motor2_B01, OUTPUT);
    pinMode(Motor2_B02, OUTPUT);
    //pinMode(onOffSwitch, INPUT);

    // �pner port og setter data transfer rate til 9600
    Serial.begin(9600);

}

// Main loop
void loop()
{

}

