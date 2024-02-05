#ifndef LINEFOLLOWINGROBOT_SENSOR_HPP
#define LINEFOLLOWINGROBOT_SENSOR_HPP

#include <QTRSensors.h>

class Sensor
{
public:
    Sensor();
    ~Sensor();
    void loadPins(uint8_t pins[], int amount);
    void calibrate(int cycles);
    void loadCalibration();
    void read();
    uint16_t readLine();

    void saveCalibration() const;
    void printValues() const;

private:
    QTRSensors m_sensor;
    const int m_numSensors = 0;
    unsigned int *m_sensorValues = nullptr;
};

#endif //LINEFOLLOWINGROBOT_SENSOR_HPP
