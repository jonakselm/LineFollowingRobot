#include <Arduino.h>
#include "Sensor.hpp"

// Sensor pin names reflect the number on the sensor chip
enum SensorPin { s9 = A0, s11, s13, s15, s17 };
uint8_t sensorPins[] = { s9, s11, s13, s15, s17 };
Sensor sensor(sensorPins, 5);

void setup()
{
    Serial.begin(9600);
    /*sensor.calibrate(600);
    sensor.saveCalibration();*/
    sensor.loadCalibration();
}

// Main loop
void loop()
{
    Serial.println(sensor.readLine());

    delay(400);
    delay(500);
}

