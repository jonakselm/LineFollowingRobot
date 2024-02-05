

#ifndef LINEFOLLOWINGROBOT_TIME_HPP
#define LINEFOLLOWINGROBOT_TIME_HPP

class Time
{
public:
    Time();

    void restart();

    unsigned long long getElapsedTime();

private:
    unsigned long long m_start;
    unsigned long long m_now;
};

#endif //LINEFOLLOWINGROBOT_TIME_HPP
