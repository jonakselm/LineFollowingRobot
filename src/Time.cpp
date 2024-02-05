#include "Time.hpp"
#include <Arduino.h>

Time::Time()
{
    m_now = millis();
    m_start = m_now;
}

void Time::restart()
{
    m_start = millis();
    m_now = m_start;
}

unsigned long long Time::getElapsedTime()
{
    m_now = millis();
    return m_now - m_start;
}