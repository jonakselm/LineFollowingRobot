// Libaries
#include <QTRSensors.h>     // For sensorn som blir levert
#include <Arduino.h>
#include "SimpleMotor.hpp"

// Pins
const int QTREmitterPin = 2;
// Sensor pin names reflect the number on the sensor chip
enum Sensor { s17 = A0, s15, s13, s11, s9 };
const int PWMB = 6;
const int Motor2_B02 = 9;
const int Motor2_B01 = 10;
const int Motor1_A02 = 11;
const int Motor1_A01 = 3;
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
    qtr.setSensorPins((const uint8_t[]) { s9, s11, s13, s15, s17 }, sensorCount);
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

    motor.setMotorPins(Motor2_B01, Motor2_B02, Motor1_A01, Motor1_A02);
    motor.setSpeed(135);
}

// Low values == whiteness, High values == blackness
SimpleMotor::Motion sensorToMotion(int s9, int s11, int s13, int s15, int s17)
{
    // TODO: Sensor logic
    const int sensorMin = 260;
    const int threshold = sensorMin + 60;
    bool is17Black = max(s17 - threshold, 0);
    bool is15Black = max(s15 - threshold, 0);
    bool is13Black = max(s13 - threshold, 0);
    bool is11Black = max(s11 - threshold, 0);
    bool is9Black = max(s9 - threshold, 0);

    int sensorsLeft = is17Black + is15Black;
    int sensorsMid = is13Black;
    int sensorsRight = is11Black + is9Black;

    /*Serial.print(is17Black);
    Serial.print(' ');
    Serial.print(is15Black);
    Serial.print(' ');
    Serial.print(is13Black);
    Serial.print(' ');
    Serial.print(is11Black);
    Serial.print(' ');
    Serial.print(is9Black);
    Serial.print(" \n");*/

    if (sensorsMid)
    {
        return SimpleMotor::Motion::Forward;
    }
    else if (motor.getMotion() == SimpleMotor::Motion::Left ||
        sensorsLeft > sensorsRight)
    {
        return SimpleMotor::Motion::Left;
    }
    else if (motor.getMotion() == SimpleMotor::Motion::Right ||
        sensorsRight > sensorsLeft)
    {
        return SimpleMotor::Motion::Right;
    }


    return SimpleMotor::Motion::Stop;
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
    // The rightmost values are the lowest
    SimpleMotor::Motion motion = sensorToMotion(sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3], sensorValues[4]);
    motor.setMotion(motion);
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

    //MotorControl(Motion::Switchoff, Motor1_A01, Motor1_A02, Motor2_B01, Motor2_B02);
    //Serial.println("Switch OFF");
}

