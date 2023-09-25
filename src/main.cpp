// Libaries
#include <QTRSensors.h>     // For sensorn som blir levert
#include <Arduino.h>
#include "SimpleMotor.hpp"

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

const int sensorMax = 400;
const int sensorMin = 200;

QTRSensors qtr;
const uint8_t sensorCount = 2;
uint16_t sensorValues[sensorCount];

SimpleMotor motor;

// Setter opp pins og default motor verdier.
void setup()
{
    // Setup for Sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]) { 6, 7 }, sensorCount);
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

    motor.setMotorPins(Motor2_B01, Motor2_B02, Motor1_A01, Motor1_A02);;
    motor.setSpeed(100);
}

// Bruker input for � endre motor funksjon / midlertidig for testing.
int input, switchState;

// Low values == whiteness, High values == blackness
/*
Motion sensorToMotion(const int sensorRight, const int sensorLeft)
{
    // TODO: Sensor logic
    const int threshold = 100;
    if (sensorLeft > sensorMin + threshold && sensorRight > sensorMin + threshold)
        return Motion::Forward;
    else if (sensorLeft > sensorMin + threshold && sensorRight < sensorMin + threshold)
        return Motion::Left;
    else if (sensorLeft < sensorMin + threshold && sensorRight > sensorMin + threshold)
        return Motion::Right;

    return Motion::Empty;
}
*/
SimpleMotor::Motion sensorToMotion(int sensorRight, int sensorLeft)
{
    // TODO: Sensor logic
    const int threshold = sensorMin + 50;
    sensorLeft = max(sensorLeft - threshold, 0);
    sensorRight = max(sensorRight - threshold, 0);
    /*Serial.print("Sensor Left: ");
    Serial.print(bool(sensorLeft));
    Serial.print(" ");
    Serial.print("Sensor Right: ");
    Serial.print(bool(sensorRight));
    Serial.print("\n");*/
    if (sensorLeft && sensorRight)
        return SimpleMotor::Motion::Forward;
    else if (sensorLeft && !sensorRight)
        return SimpleMotor::Motion::Left;
    else if (!sensorLeft && sensorRight)
        return SimpleMotor::Motion::Right;

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
    SimpleMotor::Motion motion = sensorToMotion(sensorValues[0], sensorValues[1]);
    /*for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }*/
    motor.MotorControl(motion);
    //delay(400);
    //input = 0;

    //delay(500);

    //delay(500);

    //MotorControl(Motion::Switchoff, Motor1_A01, Motor1_A02, Motor2_B01, Motor2_B02);
    //Serial.println("Switch OFF");
}

