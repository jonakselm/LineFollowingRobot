#include <Arduino.h>
#include "Sensor.hpp"

// Sensor pin names reflect the number on the sensor chip
enum SensorPin { s9 = A0, s11, s13, s15, s17, Count };
Sensor sensor((const uint8_t[]){ s9, s11, s13, s15, s17 }, SensorPin::Count);

void setup()
{
    Serial.begin(9600);
    sensor.loadPins((const uint8_t[]){ s9, s11, s13, s15, s17 }, SensorPin::Count)
}

// Main loop
void loop()
{
    /*int position = qtr.readLineBlack(sensorValues);

    Serial.print("Position: ");
    Serial.print(position);
    Serial.print('\n');*/
    int pos = qtr.readLineBlack(sensorValues);

    Serial.println(pos);

    // printer sensor values som en verdi mellom 0 til 2500 for 0 er maksimum reflesksjon aka hvit.
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();

    float sum = 0;
    float arr[sensorCount / 2] = { 0 };
    for (int i = 0; i < sensorCount / 2; i++)
    {
        Serial.print(i);
        Serial.print(": ");
        arr[i] = (sensorValues[sensorCount - 1 - i] - sensorValues[i]);
        arr[i] /= 400;
        sum += arr[i];
        Serial.print(arr[i]);
        Serial.print('\t');
    }
    sum /= sensorCount / 2;
    Serial.println(sum);

    //input = Serial.parseInt();
    //Serial.println(input);

    /*for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }*/
    delay(400);
    //input = 0;

    delay(500);
}

