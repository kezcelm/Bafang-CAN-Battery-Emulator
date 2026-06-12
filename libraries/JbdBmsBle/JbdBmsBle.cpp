#include "JbdBmsBle.h"

JbdBmsBle::JbdBmsBle(JbdBleTransport* transport) {
    port = transport;
}

// ================= SEND REQUEST =================

static const uint8_t CMD_BASIC[] = {
    0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77
};

static const uint8_t CMD_CELLS[] = {
    0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77
};

// ================= READ BASIC =================

bool JbdBmsBle::readBasic() {

    port->clear();

    // request wysyłasz z poziomu main (BLE)
    delay(50);

    return readFrame() && (frame[1] == 0x03);
}

// ================= FRAME READER =================

bool JbdBmsBle::readFrame() {

    idx = 0;
    unsigned long start = millis();

    while (millis() - start < 500) {

        while (port->available()) {

            uint8_t b = port->read();
            frame[idx++] = b;

            if (b == 0x77 && idx > 6) {
                parseBasic();
                return true;
            }

            if (idx >= 64) return false;
        }
    }

    return false;
}

// ================= PARSER =================

void JbdBmsBle::parseBasic() {

    if (frame[0] != 0xDD) return;

    uint16_t v = (frame[4] << 8) | frame[5];
    int16_t i = (frame[6] << 8) | frame[7];

    voltage = v / 100.0;
    current = i / 100.0;

    soc = frame[23];

    uint16_t tt1 = (frame[27] << 8) | frame[28];
    uint16_t tt2 = (frame[29] << 8) | frame[30];

    t1 = (tt1 - 2731) / 10.0;
    t2 = (tt2 - 2731) / 10.0;
}

bool JbdBmsBle::readCells() {

    port->clear();

    delay(50);

    if (!readFrame()) return false;

    if (frame[1] != 0x04) return false;

    cellCount = frame[3] / 2;

    uint8_t offset = 4;

    for (uint8_t i = 0; i < cellCount; i++) {

        uint16_t v =
            (frame[offset + i * 2] << 8) |
            frame[offset + i * 2 + 1];

        cellVoltage[i] = v / 1000.0; // JBD = mV
    }

    return true;
}

// ================= GETTERS =================

float JbdBmsBle::getVoltage() { return voltage; }
float JbdBmsBle::getCurrent() { return current; }
float JbdBmsBle::getSoc()     { return soc; }
float JbdBmsBle::getTemp1()   { return t1; }
float JbdBmsBle::getTemp2()   { return t2; }

uint8_t JbdBmsBle::getCellCount() {
    return cellCount;
}

float JbdBmsBle::getCellVoltage(uint8_t i) {
    if (i >= cellCount) return 0;
    return cellVoltage[i];
}

float JbdBmsBle::getCellMin() {
    float m = 100;
    for (int i = 0; i < cellCount; i++)
        if (cellVoltage[i] < m) m = cellVoltage[i];
    return m;
}

float JbdBmsBle::getCellMax() {
    float m = 0;
    for (int i = 0; i < cellCount; i++)
        if (cellVoltage[i] > m) m = cellVoltage[i];
    return m;
}