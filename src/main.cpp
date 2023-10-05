// Libaries
#include <QTRSensors.h>     // For sensorn som blir levert
#include <Arduino.h>
#include "SimpleMotor.hpp"

// Pins
const int QTREmitterPin = 2;
// Sensor pin names reflect the number on the sensor chip
enum Sensor { s17 = A0, s15, s13, s11, s9 };
const int PWMB = 6;
const int MotorLeft_B02 = 9;
const int MotorLeft_B01 = 10;
const int MotorRight_A02 = 11;
const int MotorRight_A01 = 3;
const int PWMA = 5;
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
    qtr.setSensorPins((const uint8_t[]) { s17, s15, s13, s11, s9 }, sensorCount);
    qtr.setEmitterPin(QTREmitterPin);

    // Default motor fart
    digitalWrite(+PWMA, HIGH);
    digitalWrite(PWMB, HIGH);

    // Setter Pin modes
    pinMode(PWMA, OUTPUT);
    pinMode(MotorRight_A01, OUTPUT);
    pinMode(MotorRight_A02, OUTPUT);
    pinMode(MotorLeft_B01, OUTPUT);
    pinMode(MotorLeft_B02, OUTPUT);
    //pinMode(onOffSwitch, INPUT);

    // Setter motorer til low for � unng� at dei starter p� program startup
    digitalWrite(MotorRight_A01, LOW);
    digitalWrite(MotorRight_A02, LOW);
    digitalWrite(MotorLeft_B01, LOW);
    digitalWrite(MotorLeft_B02, LOW);

    // �pner port og setter data transfer rate til 9600
    Serial.begin(9600);

    motor.setMotorPins(MotorLeft_B01, MotorLeft_B02, MotorRight_A01, MotorRight_A02);
    motor.setBaseSpeed(255);
}

// Main loop
void loop()
{
    // read raw sensor values
    qtr.read(sensorValues);

    // printer sensor values som en verdi mellom 0 til 2500 for 0 er maksimum reflesksjon aka hvit.
    /*for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();*/

    //input = Serial.parseInt();
    //Serial.println(input);

    motor.updateMotion(sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3], sensorValues[4]);
    motor.drive();

    /*for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }*/
    //delay(400);
    //input = 0;

    //delay(500);

    //delay(500);

    //MotorControl(Motion::Switchoff, MotorRight_A01, MotorRight_A02, MotorLeft_B01, MotorLeft_B02);
    //Serial.println("Switch OFF");
}

