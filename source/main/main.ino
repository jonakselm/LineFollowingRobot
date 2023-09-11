// Libaries
#include <QTRSensors.h>     // For sensorn som blir levert

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


// motion for switch for å lett kontrollere retning og setter default state til stop.
enum motion { EMPTY, FORWARD, LEFT, RIGHT, BACKWARD, STOP, SWITCHOFF };

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
  pinMode(onOffSwitch, INPUT);

  // Setter motorer til low for å unngå at dei starter på program startup
  digitalWrite(Motor1_A01, LOW);
  digitalWrite(Motor1_A02, LOW);
  digitalWrite(Motor2_B01, LOW);
  digitalWrite(Motor2_B02, LOW);

  // Åpner port og setter data transfer rate til 9600
  Serial.begin(9600);

  // User input tekst   // midlertidig for å teste motorer.
  Serial.println("select direction of movement");
  Serial.println("1. forward");
  Serial.println("2. left");
  Serial.println("3. right");
  Serial.println("4. backward");
  Serial.println("5. stop");
}

// Bruker input for å endre motor funksjon / midlertidig for testing.
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
      MotorControl(input);
      //delay(400);
      input = 0;
    }
  // looper for å printe sensor data
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println();

    delay(500);
  }

  MotorControl(SWITCHOFF);
  //Serial.println("Switch OFF");
  delay(400);
}

///////////////////////
// Helper funksjoner //
///////////////////////

// Framover funksjon
void Forward() 
{       
  Serial.println("Going forward...");
  analogWrite(Motor1_A01, 255);
  analogWrite(Motor1_A02, 0);
  analogWrite(Motor2_B01, 255);
  analogWrite(Motor2_B02, 0);
}

// Backover funksjon
void Backward() 
{         
  Serial.println("Going backward...");
  analogWrite(Motor1_A01, 0);
  analogWrite(Motor1_A02, 255);
  analogWrite(Motor2_B01, 0);
  analogWrite(Motor2_B02, 255);
}

void Left()
{
  Serial.println("Going left...");
  analogWrite(Motor1_A01, 255);
  analogWrite(Motor1_A02, 0);
  analogWrite(Motor2_B01, 0);
  analogWrite(Motor2_B02, 255);
}

void Right()
{
  Serial.println("Going right...");
  analogWrite(Motor1_A01, 0);
  analogWrite(Motor1_A02, 255);
  analogWrite(Motor2_B01, 255);
  analogWrite(Motor2_B02, 0);
}

// Stop funksjon
void Stop() 
{            
  Serial.println("Stopping...");  
  digitalWrite(Motor1_A01, 0);
  digitalWrite(Motor1_A02, 0);
  digitalWrite(Motor2_B01, 0);
  digitalWrite(Motor2_B02, 0);
}

void SwitchOff() 
{             
  digitalWrite(Motor1_A01, 0);
  digitalWrite(Motor1_A02, 0);
  digitalWrite(Motor2_B01, 0);
  digitalWrite(Motor2_B02, 0);
}

// Tekk input i form av FORWARD, LEFT, RIGHT, BACKWARD, STOP og kjører motor funksjonene basert på input.
void MotorControl (motion arg)
{
  // Bytter motor funksjon / Kanskje legge til speed control input også?
  switch (arg) 
    {
    case FORWARD:      // Forward
      Forward();
      break;
    case LEFT:         // Left
      Left();
      break;
    case RIGHT:        // Right
      Right();
      break;
    case BACKWARD:     // Backward
      Backward();
      break;
    case STOP:         // Stop
      Stop();
      break;
    case SWITCHOFF:    // For på/av switch
      SwitchOff();
      break;
    default:           // Default stop
      Stop();
      break; 
    }
}

// Sensor funksjon blir plassert her

