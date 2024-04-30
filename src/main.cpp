#include "Arduino.h"
#include "Pid.hpp"
#include "Sensor.hpp"
#include "Motor.hpp"
#include "Encoders.hpp"



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
    constexpr int threshold = 1750;  // Justert terskel for bedre følsomhet
    int countRight = 0;
    int countLeft = 0;
    static bool blackDetected[numSensors] = {false};

    // Analyser sensorverdier og oppdater svart-detekterte sensorer
    for (int i = 0; i < numSensors; i++) {
        if (sensorValues[i] > threshold) {
            blackDetected[i] = true;
            if (i < midIndex) {
                countRight++;
            } else if (i > midIndex) {
                countLeft++;
            }
        }
    }

    constexpr int minSensorsForTurn = 4;
    static bool turnLeft = false;
    static bool shouldCheck = false;
    static bool shouldTurn = false;
    static uint64_t lastTurnTime = 0;
    constexpr int turnDelay = 50; // ms

    // Bestem retning basert på sensorlesninger
    if (countRight >= minSensorsForTurn && countLeft < minSensorsForTurn) {
        shouldCheck = true;
        shouldTurn = true;
        turnLeft = false;
        lastTurnTime = millis();
    } else if (countLeft >= minSensorsForTurn && countRight < minSensorsForTurn) {
        turnLeft = true;
        shouldCheck = true;
        shouldTurn = true;
        lastTurnTime = millis();
    }

    // Utfører sving logikk
    if (shouldCheck) {
        if ((turnLeft && blackDetected[numSensors - 1]) || (!turnLeft && blackDetected[0])) {
            shouldTurn = false;
            shouldCheck = false;
            for (int i = 0; i < numSensors; i++) {
                blackDetected[i] = false;
            }
        }
    }

    // Utfør sving hvis tidspunktet er riktig
    if (shouldTurn && millis() - turnDelay > lastTurnTime) {
        shouldCheck = false;
        shouldTurn = false;
        for (int i = 0; i < numSensors; i++) {
            blackDetected[i] = false;
        }
        int turnAngle = turnLeft ? 90 : -90;
        motor.powerTurn(turnAngle);
    }
#endif

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



