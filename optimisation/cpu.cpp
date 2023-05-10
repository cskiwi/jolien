#include "cpu.h"
#include <Arduino.h>

void printCpuInfo()
{
    printCpuInfo(true);
}

void printCpuInfo(bool lineEnd)
{
    uint32_t Freq = 0;

    Freq = getCpuFrequencyMhz();
    Serial.print("CPU Freq = ");
    Serial.print(Freq);
    Serial.println(" MHz");
    Freq = getXtalFrequencyMhz();
    Serial.print("XTAL Freq = ");
    Serial.print(Freq);
    Serial.println(" MHz");
    Freq = getApbFrequency();
    Serial.print("APB Freq = ");
    Serial.print(Freq);
    Serial.println(" Hz");

    if (lineEnd)
    {
        Serial.println(")");
    }
    else
    {
        Serial.print(')');
    }
}

void lowerCpuFrequency(int frequency)
{
    setCpuFrequencyMhz(frequency);
}
