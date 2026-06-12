#pragma once
#include <Arduino.h>
#include "JbdBleTransport.h"

class JbdBmsBle {
public:
    JbdBmsBle(JbdBleTransport* transport);

    bool readBasic();
    bool readCells();

    float getVoltage();
    float getCurrent();
    float getSoc();
    float getTemp1();
    float getTemp2();
	
	float cellVoltage[24];
	uint8_t cellCount = 0;

	bool readCells();
	uint8_t getCellCount();
	float getCellVoltage(uint8_t index);
	float getCellMin();
	float getCellMax();

private:
    JbdBleTransport* port;

    float voltage = 0;
    float current = 0;
    float soc = 0;
    float t1 = 0;
    float t2 = 0;

    uint8_t frame[64];
    uint8_t idx = 0;

    bool readFrame();
    void parseBasic();
};