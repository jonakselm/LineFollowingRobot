#ifndef LINEFOLLOWINGROBOT_SENSOR_HPP
#define LINEFOLLOWINGROBOT_SENSOR_HPP

#include <QTRSensors.h>
#include <vector>

class Sensor
{
public:
    Sensor(uint8_t pins[], int amount);
    Sensor(Sensor &) = delete;
    Sensor(const Sensor &) = delete;
    ~Sensor();

    Sensor operator=(Sensor &) = delete;
    Sensor operator=(const Sensor &) = delete;

    void calibrate(int cycles);
    void loadCalibration();
    void read();
    uint16_t readLine();

    void saveCalibration() const;
    void printValues() const;
    std::vector<int> getSensorValues() const;

private:
    QTRSensors m_sensor;
    const int m_numSensors = 0;
    uint16_t *m_sensorValues = nullptr;
};

#endif //LINEFOLLOWINGROBOT_SENSOR_HPP
