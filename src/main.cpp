#include "Arduino.h"
#include "Pid.hpp"
#include "Sensor.hpp"
#include "Motor.hpp"
#include "Encoders.hpp"
#include <algorithm>

#define nitti

// Sensor setup
// Sensor pin names reflect the number on the pcb
enum SensorPin { s1 = 15, s2, s3, s4, s5, s6, s7, s8, s9 };
uint8_t sensorPins[] = {s1, s2, s3, s4, s5, s6, s7, s8, s9 };
constexpr int numSensorPins = 9;
Sensor sensor(sensorPins, numSensorPins);
const int switch1Pin = 12;
const int switch2Pin = 13;
const int switch3Pin = 14;
Encoders encoders(8, 9, 11, 10);

// Pid setup
double kp = 0.85;
double ki = 0.04;
double kd = 0.06;
double setpoint = 4000;
PIDController pid(kp, ki, kd,  setpoint);

uint64_t elapsedTime = 0;

// Motor setup
// A is left motor, B is right motor
constexpr int MB1 = 3, MB2 = 4, MA1 = 6, MA2 = 7, PWMB = 2, PWMA = 5;
/*constexpr int MA1 = 3,  MA2 = 4,
        MB1 = 6, MB2 = 7,
        PWMA = 2, PWMB = 5;*/

Motor motor(MA1, MA2, MB1, MB2, PWMA, PWMB, encoders);

void setup()
{
    // Sensor setup
    Serial.begin(9600);
    motor.autoCalibrate(sensor, 300);
    motor.stop();
    digitalWrite(MA1, 0);
    digitalWrite(MA2, 0);
    digitalWrite(MB1, 0);
    digitalWrite(MB2, 0);
}

void loop()
{
    encoders.update();
    if (encoders.getRelativeEncoderDiff())
    {
        Serial.println(encoders.getTotalEncoderDiff());
    }
    uint16_t pos = sensor.readLine();
    //Serial.print("Pos: ");
    //Serial.println(pos);

    double dt = double(millis() - elapsedTime) / 1000;
    elapsedTime = millis();

    // Compute PID output
    double pidOutput = pid.compute(pos, dt);
    //Serial.print("PID: ");
    //Serial.println(pidOutput);

#ifdef nitti
    auto sensorValues = sensor.getSensorValues();
    constexpr int numSensors = numSensorPins;
    constexpr int midIndex = numSensors / 2;
    constexpr int threshold = 600;  // Justert terskel for bedre følsomhet
    int countRight = 0;
    int countLeft = 0;
    static std::array<bool, numSensorPins> blackDetected = { true };
    static std::array<bool, numSensorPins> prevBlack = { true };

    // Analyser sensorverdier og oppdater svart-detekterte sensorer
    for (int i = 0; i < numSensors; i++) {
        blackDetected[i] = sensorValues[i] > threshold;
        Serial.print(blackDetected[i]);
        Serial.print(", ");
        /*if (blackDetected[i]) {
            if (i < midIndex) {
                countRight++;
            } else if (i > midIndex) {
                countLeft++;
            }
        }*/
    }
    Serial.println();

    bool anyBlack = std::any_of(blackDetected.begin(), blackDetected.end(), [](bool b)
    {
        return b;
    });
    if (anyBlack)
    {
        std::copy(blackDetected.begin(), blackDetected.end(), prevBlack.begin());

        uint8_t switchValue = digitalRead(switch1Pin);
        switchValue |= digitalRead(switch2Pin) << 1;
        switchValue |= digitalRead(switch3Pin) << 2;

        switch (switchValue)
        {
            case 0:
                motor.setSpeedScaler(0.5);
                break;
            case 1:
                motor.setSpeedScaler(0.55);
                break;
            case 2:
                motor.setSpeedScaler(0.60);
                break;
            case 3:
                motor.setSpeedScaler(0.65);
                break;
            case 4:
                motor.setSpeedScaler(0.70);
                break;
            case 5:
                motor.setSpeedScaler(0.75);
                break;
            case 6:
                motor.setSpeedScaler(0.80);
                break;
            case 7:
                motor.setSpeedScaler(0.85);
                break;
            default:
                motor.setSpeedScaler(0.9);
                break;
        }
        motor.updateOutput((long)pidOutput, -2000, 2000);
    }
    else
    {
        bool turnRight = std::any_of(prevBlack.begin(), prevBlack.begin() + 1, [](bool b)
        {
            Serial.print(b);
            Serial.print(", ");
            return b;
        });
        Serial.println();
        if (prevBlack[0])
        {
            motor.powerTurn(-45);
            Serial.println("Right");
        }
        else
        {
            Serial.println("Left");
            motor.powerTurn(45);
        }
    }

#endif
}



