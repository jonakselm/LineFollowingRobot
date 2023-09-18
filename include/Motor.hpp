//
// Created by jonaksel on 9/18/23.
//

#ifndef LINEFOLLOWINGROBOT_MOTOR_HPP
#define LINEFOLLOWINGROBOT_MOTOR_HPP

///////////////////////
// Helper funksjoner //
///////////////////////

// Motion for switch for � lett kontrollere retning og setter default state til stop.
enum Motion { EMPTY, FORWARD, LEFT, RIGHT, BACKWARD, STOP, SWITCHOFF };

// Framover funksjon
void Forward(const int MA01, const int MA02, const int MB01, const int MB02)
{
    Serial.println("Going forward...");
    analogWrite(MA01, 255);
    analogWrite(MA02, 0);
    analogWrite(MB01, 255);
    analogWrite(MB02, 0);
}

// Backover funksjon
void Backward(const int MA01, const int MA02, const int MB01, const int MB02)
{
    Serial.println("Going backward...");
    analogWrite(MA01, 0);
    analogWrite(MA02, 255);
    analogWrite(MB01, 0);
    analogWrite(MB02, 255);
}

void Left(const int MA01, const int MA02, const int MB01, const int MB02)
{
    Serial.println("Going left...");
    analogWrite(MA01, 255);
    analogWrite(MA02, 0);
    analogWrite(MB01, 0);
    analogWrite(MB02, 255);
}

void Right(const int MA01, const int MA02, const int MB01, const int MB02)
{
    Serial.println("Going right...");
    analogWrite(MA01, 0);
    analogWrite(MA02, 255);
    analogWrite(MB01, 255);
    analogWrite(MB02, 0);
}

// Stop funksjon
void Stop(const int MA01, const int MA02, const int MB01, const int MB02)
{
    Serial.println("Stopping...");
    digitalWrite(MA01, 0);
    digitalWrite(MA02, 0);
    digitalWrite(MB01, 0);
    digitalWrite(MB02, 0);
}

void SwitchOff(const int MA01, const int MA02, const int MB01, const int MB02)
{
    digitalWrite(MA01, 0);
    digitalWrite(MA02, 0);
    digitalWrite(MB01, 0);
    digitalWrite(MB02, 0);
}

void MotorControl (Motion arg, const int MA01, const int MA02, const int MB01, const int MB02)
{
    // Bytter motor funksjon / Kanskje legge til speed control input ogs�?
    switch (arg)
    {
        case FORWARD:      // Forward
            Forward(MA01, MA02, MB01, MB02);
            break;
        case LEFT:         // Left
            Left(MA01, MA02, MB01, MB02);
            break;
        case RIGHT:        // Right
            Right(MA01, MA02, MB01, MB02);
            break;
        case BACKWARD:     // Backward
            Backward(MA01, MA02, MB01, MB02);
            break;
        case STOP:         // Stop
            Stop(MA01, MA02, MB01, MB02);
            break;
        case SWITCHOFF:    // For p�/av switch
            SwitchOff(MA01, MA02, MB01, MB02);
            break;
        default:           // Default stop
            Stop(MA01, MA02, MB01, MB02);
            break;
    }
}

#endif //LINEFOLLOWINGROBOT_MOTOR_HPP
