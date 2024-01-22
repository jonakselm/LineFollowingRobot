#include <QTRSensors.h>
#include <Arduino.h>

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

// Setter opp pins og default motor verdier.
void setup()
{
    // Setup for Sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]) { s17, s15, s13, s11, s9 }, sensorCount);
    qtr.setEmitterPin(QTREmitterPin);

    Serial.begin(9600);
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

    /*for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }*/
    //delay(400);
    //input = 0;

    //delay(500);

    //delay(500);
}

