#include <EEPROM.h>
#include <Arduino.h>
#include <Sensor.hpp>

Sensor::Sensor(uint8_t pins[], int amount)
    : m_numSensors(amount)
{
    m_sensor.setTypeRC();
    m_sensor.setSensorPins(pins, amount);
    m_sensorValues = new uint16_t[amount];
}

Sensor::~Sensor()
{
    if (m_sensorValues)
    {
        delete m_sensorValues;
        m_sensorValues = nullptr;
    }
}

void Sensor::loadPins(uint8_t *pins, int amount)
{

}

void Sensor::calibrate(int cycles)
{
    for (int i = 0; i < cycles; i++)
    {
        m_sensor.calibrate();
    }
}

void Sensor::loadCalibration()
{
    EEPROM.get(0, m_sensor.calibrationOn);
}

void Sensor::read()
{
    m_sensor.read(m_sensorValues);
}

uint16_t Sensor::readLine()
{
    return m_sensor.readLineBlack(m_sensorValues);
}

void Sensor::saveCalibration() const
{
    EEPROM.put(0, m_sensor.calibrationOn);
}

void Sensor::printValues() const
{
    for (int i = 0; i < m_numSensors; i++)
    {
        Serial.print(m_sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();
}