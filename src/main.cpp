// Libaries
#include <QTRSensors.h>     // For sensorn som blir levert
#include <Arduino.h>
#include "Motor.hpp"

// Declerations
const int PWMA = 13;
const int PWMB = 8;
const int Motor1_A01 = 12;
const int Motor1_A02 = 11;
const int Motor2_B01 = 10;
const int Motor2_B02 = 9;
//const int onOffSwitch = 7;
// const int VM_Motor_Voltage = VIN     // Ikke i bruk akkurat no.
// A01 og B01 blir brukt for framover bevegelse / A02 og B02 er for backover.
const int QTRSensorRight = 7;
const int QTRSensorLeft = 6;
const int QTREmitterPin = 2;

QTRSensors qtr;
const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

// Setter opp pins og default motor verdier.
void setup()
{
    // Setup for Sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){ 6, 7 }, SensorCount);
    qtr.setEmitterPin(QTREmitterPin);

    // Default motor fart
    digitalWrite(PWMA, HIGH);
    digitalWrite(PWMB, HIGH);

    // Setter Pin modes
    pinMode(PWMA, OUTPUT);
    pinMode(Motor1_A01, OUTPUT);
    pinMode(Motor1_A02, OUTPUT);
    pinMode(Motor2_B01, OUTPUT);
    pinMode(Motor2_B02, OUTPUT);
    //pinMode(onOffSwitch, INPUT);

    // Setter motorer til low for � unng� at dei starter p� program startup
    digitalWrite(Motor1_A01, LOW);
    digitalWrite(Motor1_A02, LOW);
    digitalWrite(Motor2_B01, LOW);
    digitalWrite(Motor2_B02, LOW);

    // �pner port og setter data transfer rate til 9600
    Serial.begin(9600);

    // User input tekst   // midlertidig for � teste motorer.
    Serial.println("select direction of movement");
    Serial.println("1. forward");
    Serial.println("2. left");
    Serial.println("3. right");
    Serial.println("4. backward");
    Serial.println("5. stop");
}

// Bruker input for � endre motor funksjon / midlertidig for testing.
int input, switchState;

// Main loop
void loop()
{
    // read raw sensor values
    qtr.read(sensorValues);

    // printer sensor values som en verdi mellom 0 til 2500 for 0 er maksimum reflesksjon aka hvit.
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();

    delay(500);

    while (switchState > 0)
    {
        input = Serial.parseInt();
        Serial.println(input);
        if(Serial.available() != EMPTY)
        {
            MotorControl(input, Motor1_A01, Motor1_A02, Motor2_B01, Motor2_B02);
            //delay(400);
            input = 0;
        }
        // looper for � printe sensor data
        for (uint8_t i = 0; i < SensorCount; i++)
        {
            Serial.print(sensorValues[i]);
            Serial.print('\t');
        }
        Serial.println();

        delay(500);
    }

    MotorControl(SWITCHOFF, Motor1_A01, Motor1_A02, Motor2_B01, Motor2_B02);
    //Serial.println("Switch OFF");
    delay(400);
}


// Tekk input i form av FORWARD, LEFT, RIGHT, BACKWARD, STOP og kj�rer motor funksjonene basert p� input.

// Sensor funksjon blir plassert her

