#include <SPI.h>
#include <mcp_can.h>
#include "JbdBms.h"

#define CAN_CS 10
#define BMS_RX 2
#define BMS_TX 3

MCP_CAN CAN(CAN_CS);
SoftwareSerial bmsSerial(BMS_RX, BMS_TX);
JbdBms bms(&bmsSerial);

// Timers
unsigned long t100 = 0;
unsigned long t200 = 0;
unsigned long t500 = 0;
unsigned long t1000 = 0;
unsigned long tBmsRead = 0;
#define BMS_READ_INTERVAL 500  // ms

// ==================================================
// BMS LIVE DATA
// ==================================================
float bmsVoltage = 0;       // V
float bmsCurrent = 0;       // mA
float bmsSoc = 0;           // %
uint16_t bmsCycle = 0;
uint16_t bmsRatedCap = 0;   // 10mAh units
uint16_t bmsResidualCap = 0; // 10mAh units
float bmsTemp1 = 0;         // °C
float bmsTemp2 = 0;         // °C
uint16_t bmsProtection = 0;
packCellInfoStruct bmsCells = {0};
bool bmsDataValid = false;

// ==================================================
// STATE MACHINE
// ==================================================
enum State {
    IDLE,
    WAIT_ACK_START,
    WAIT_ACK_DATA,
    WAIT_ACK_END
};

State state = IDLE;

// ==================================================
// CURRENT SESSION
// ==================================================
unsigned long activeCmd = 0;
unsigned long activeSource = 0;  // 0x03 = Display, 0x05 = Controller
int frameIndex = 0;
int totalDataFrames = 0;
unsigned long lastStateChange = 0;
#define ACK_TIMEOUT 500  // ms

// ==================================================
// MULTIFRAME DATA (0x6000 - 0x6003)
// ==================================================

// 0x6000: "C20010 4.3"
byte data6000_d[8] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30, 0x20, 0x34};
byte data6000_e[2] = {0x2E, 0x33};

// 0x6001: "C20010 1.5"
byte data6001_d[8] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30, 0x20, 0x31};
byte data6001_e[2] = {0x2E, 0x35};

// 0x6002: "C20010"
byte data6002_d[6] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30};

// 0x6003
byte data6003_d0[8] = {0x41, 0x4C, 0x49, 0x31, 0x30, 0x53, 0x32, 0x33};
byte data6003_d1[8] = {0x41, 0x4D, 0x30, 0x38, 0x37, 0x53, 0x43, 0x32};
byte data6003_d2[8] = {0x34, 0x30, 0x37, 0x31, 0x37, 0x43, 0x32, 0x30};
byte data6003_e[8]  = {0x30, 0x32, 0x31, 0x30, 0x30, 0x32, 0x33, 0x31};

// ==================================================
// SINGLE FRAME DATA (0x6400 - 0x6405)
// ==================================================
byte data6400[4] = {0x0A,  // Number of serial cell
                    0x07,  // Number of parallel cell
                    0x05,  // Cell Voltage difference
                    0x00};
byte data6401[6] = {0x02, 0x00,  // Charging cycle D0/D1
                    0x7B, 0x03,  // Max uncharged time M.L.T
                    0x7A, 0x02}; // Last uncharged time N.L.T
byte data6402[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E};
byte data6403[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E};
byte data6404[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0x00, 0x00, 0x00, 0x00};
byte data6405[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ==================================================
// BMS DATA REFRESH
// ==================================================
void updateBmsData()
{
    if (bms.readBmsData())
    {
        bmsVoltage = bms.getVoltage();
        bmsCurrent = bms.getCurrent();
        bmsSoc = bms.getChargePercentage();
        bmsCycle = bms.getCycle();
        bmsRatedCap = bms.getRatedCapacity();
        bmsResidualCap = bms.getResidualCapacity();
        bmsTemp1 = bms.getTemp1();
        bmsTemp2 = bms.getTemp2();
        bmsProtection = bms.getProtectionState();
        bmsDataValid = true;
    }

    if (bms.readPackData())
    {
        bmsCells = bms.getPackCellInfo();

        // Update data6400: [numSerial, numParallel, cellDiff_mV, 0x00]
        data6400[0] = bmsCells.NumOfCells;
        data6400[2] = (byte)(bmsCells.CellDiff & 0xFF);

        // Update data6401: [cycle_L, cycle_H, 0x00, 0x00, 0x00, 0x00]
        data6401[0] = (byte)(bmsCycle & 0xFF);
        data6401[1] = (byte)((bmsCycle >> 8) & 0xFF);

        // Update data6402-6405: cell voltages (2 bytes per cell, little-endian)
        // 6402: cells 1-4, 6403: cells 5-8, 6404: cells 9-12, 6405: cells 13-16
        for (int i = 0; i < 4 && i < bmsCells.NumOfCells; i++)
        {
            data6402[i * 2]     = (byte)(bmsCells.CellVoltage[i] & 0xFF);
            data6402[i * 2 + 1] = (byte)((bmsCells.CellVoltage[i] >> 8) & 0xFF);
        }
        for (int i = 0; i < 4 && (i + 4) < bmsCells.NumOfCells; i++)
        {
            data6403[i * 2]     = (byte)(bmsCells.CellVoltage[i + 4] & 0xFF);
            data6403[i * 2 + 1] = (byte)((bmsCells.CellVoltage[i + 4] >> 8) & 0xFF);
        }
        for (int i = 0; i < 4 && (i + 8) < bmsCells.NumOfCells; i++)
        {
            data6404[i * 2]     = (byte)(bmsCells.CellVoltage[i + 8] & 0xFF);
            data6404[i * 2 + 1] = (byte)((bmsCells.CellVoltage[i + 8] >> 8) & 0xFF);
        }
        for (int i = 0; i < 4 && (i + 12) < bmsCells.NumOfCells; i++)
        {
            data6405[i * 2]     = (byte)(bmsCells.CellVoltage[i + 12] & 0xFF);
            data6405[i * 2 + 1] = (byte)((bmsCells.CellVoltage[i + 12] >> 8) & 0xFF);
        }
    }
}

// ==================================================
// SEND START FRAME
// ==================================================
void sendStartFrame()
{
    unsigned int cmdLow = activeCmd & 0xFFFF;
    byte totalLen = 0;

    if (cmdLow == 0x6000) totalLen = 0x0A;
    else if (cmdLow == 0x6001) totalLen = 0x0A;
    else if (cmdLow == 0x6002) totalLen = 0x06;
    else if (cmdLow == 0x6003) totalLen = 0x20;

    byte data[1] = { totalLen };

    unsigned long baseId =
        (activeSource == 0x05) ? 0x042C0000 : 0x041C0000;

    unsigned long id = baseId | cmdLow;

    CAN.sendMsgBuf(id, 1, 1, data);

    state = WAIT_ACK_START;
    lastStateChange = millis();
}

// ==================================================
// SEND DATA FRAME
// ==================================================
void sendDataFrame()
{
    unsigned int cmdLow = activeCmd & 0xFFFF;

    unsigned long baseDataId =
        (activeSource == 0x05) ? 0x042D0000 : 0x041D0000;

    unsigned long canId = baseDataId | (unsigned long)frameIndex;

    if (cmdLow == 0x6000)
    {
        CAN.sendMsgBuf(canId, 1, 8, data6000_d);
        totalDataFrames = 1;
    }
    else if (cmdLow == 0x6001)
    {
        CAN.sendMsgBuf(canId, 1, 8, data6001_d);
        totalDataFrames = 1;
    }
    else if (cmdLow == 0x6002)
    {
        CAN.sendMsgBuf(canId, 1, 6, data6002_d);
        totalDataFrames = 1;
    }
    else if (cmdLow == 0x6003)
    {
        totalDataFrames = 3;

        if (frameIndex == 0)
            CAN.sendMsgBuf(canId, 1, 8, data6003_d0);
        else if (frameIndex == 1)
            CAN.sendMsgBuf(canId, 1, 8, data6003_d1);
        else if (frameIndex == 2)
            CAN.sendMsgBuf(canId, 1, 8, data6003_d2);
    }

    frameIndex++;
    state = WAIT_ACK_DATA;
    lastStateChange = millis();
}

// ==================================================
// SEND END FRAME
// ==================================================
void sendEndFrame()
{
    unsigned int cmdLow = activeCmd & 0xFFFF;

    unsigned long baseEndId =
        (activeSource == 0x05) ? 0x042E0001 : 0x041E0001;

    unsigned long canId = baseEndId;

    if (cmdLow == 0x6000)
        CAN.sendMsgBuf(canId, 1, 2, data6000_e);

    else if (cmdLow == 0x6001)
        CAN.sendMsgBuf(canId, 1, 2, data6001_e);

    else if (cmdLow == 0x6002)
        CAN.sendMsgBuf(canId, 1, 0, NULL);

    else if (cmdLow == 0x6003)
    {
        canId = (activeSource == 0x05) ? 0x042E0003 : 0x041E0003;
        CAN.sendMsgBuf(canId, 1, 8, data6003_e);
    }

    state = WAIT_ACK_END;
    lastStateChange = millis();
}

// ==================================================
// SINGLE FRAME HANDLER
// ==================================================
void sendSingleFrame(unsigned int cmdLow)
{
    unsigned long baseId =
        (activeSource == 0x05) ? 0x042A0000 : 0x041A0000;

    unsigned long canId = baseId | cmdLow;

    if (cmdLow == 0x6400)
        CAN.sendMsgBuf(canId, 1, 4, data6400);
    else if (cmdLow == 0x6401)
        CAN.sendMsgBuf(canId, 1, 6, data6401);
    else if (cmdLow == 0x6402)
        CAN.sendMsgBuf(canId, 1, 8, data6402);
    else if (cmdLow == 0x6403)
        CAN.sendMsgBuf(canId, 1, 8, data6403);
    else if (cmdLow == 0x6404)
        CAN.sendMsgBuf(canId, 1, 8, data6404);
    else if (cmdLow == 0x6405)
        CAN.sendMsgBuf(canId, 1, 8, data6405);
}

// ==================================================
// ACK HANDLER
// ==================================================
void handleAck(unsigned long rxId)
{
    unsigned int cmdLow = activeCmd & 0xFFFF;

    unsigned long ackBase =
        (activeSource == 0x05) ? 0x05220000 : 0x03220000;

    unsigned long expectedAck = ackBase | cmdLow;

    if (rxId != expectedAck)
        return;

    if (state == WAIT_ACK_START)
    {
        frameIndex = 0;
        sendDataFrame();
    }
    else if (state == WAIT_ACK_DATA)
    {
        if (frameIndex >= totalDataFrames)
            sendEndFrame();
        else
            sendDataFrame();
    }
    else if (state == WAIT_ACK_END)
    {
        state = IDLE;
    }
}

// ==================================================
// REQUEST HANDLER
// ==================================================
void handleRequest(unsigned long rxId)
{
    if (rxId >= 0x03216000 && rxId <= 0x03216003)
    {
        activeCmd = rxId;
        activeSource = 0x03;
        frameIndex = 0;
        sendStartFrame();
    }
    else if (rxId >= 0x05216000 && rxId <= 0x05216003)
    {
        activeCmd = rxId;
        activeSource = 0x05;
        frameIndex = 0;
        sendStartFrame();
    }
    else if (rxId >= 0x03216400 && rxId <= 0x03216405)
    {
        activeSource = 0x03;
        sendSingleFrame(rxId & 0xFFFF);
    }
    else if (rxId >= 0x05216400 && rxId <= 0x05216405)
    {
        activeSource = 0x05;
        sendSingleFrame(rxId & 0xFFFF);
    }
}

// ==================================================
// SETUP
// ==================================================
void setup()
{
    Serial.begin(9600);

    CAN.begin(CAN_250KBPS, MCP_8MHz);
    CAN.setMode(MODE_NORMAL);

    Serial.println("Bafang CAN emulator + JBD BMS started");

    // Initial BMS read
    updateBmsData();
}

void loop()
{
    unsigned long now = millis();

    // ==================================================
    // ACK TIMEOUT - reset state machine if no ACK received
    // ==================================================
    if (state != IDLE && (now - lastStateChange > ACK_TIMEOUT))
    {
        state = IDLE;
    }

    // ==================================================
    // REQ → RES HANDLING
    // ==================================================
    if (CAN.checkReceive())
    {
        unsigned long rxId;
        byte len = 0;
        byte buf[8];

        CAN.readMsgBuf(&len, buf);
        rxId = CAN.getCanId();

        if (state != IDLE)
        {
            handleAck(rxId);
        }
        else
        {
            handleRequest(rxId);
        }
    }

    // ==================================================
    // BMS READ (every 500ms)
    // ==================================================
    if (now - tBmsRead >= BMS_READ_INTERVAL)
    {
        tBmsRead = now;
        updateBmsData();
    }

    // ==================================================
    // 100 ms - Current, Voltage, Temperature
    // ==================================================
    if (now - t100 >= 100)
    {
        t100 = now;

        // Current: signed 16-bit mA (big-endian)
        int16_t current_raw = (int16_t)(bmsCurrent / 10);
        // Voltage: 16-bit in 10mV units (big-endian)
        uint16_t voltage_raw = (uint16_t)(bmsVoltage * 10);
        // Temperature: °F = °C * 9/5 + 32
        byte tempF = (byte)(bmsTemp1 * 9.0 / 5.0 + 32.0);

        byte data[5] = {(byte)((current_raw >> 8) & 0xFF),
                        (byte)(current_raw & 0xFF),
                        (byte)((voltage_raw >> 8) & 0xFF),
                        (byte)(voltage_raw & 0xFF),
                        tempF};
        CAN.sendMsgBuf(0x04F83401, 1, 5, data);
    }

    // ==================================================
    // 200 ms - Capacity, SOC, SOH
    // ==================================================
    if (now - t200 >= 200)
    {
        t200 = now;

        // Full charge capacity from BMS (10mAh units -> mAh)
        uint16_t fullCap = bmsRatedCap * 10;
        // Remaining capacity from BMS
        uint16_t remainCap = bmsResidualCap * 10;
        byte soc = (byte)bmsSoc;
        byte absSoc = soc;
        // SOH = (FCC / Design Capacity) * 100
        // FCC estimated from: residualCap / (soc/100)
        uint16_t designCap = bmsRatedCap;
        uint16_t fullChargeCap = (bmsSoc > 0) ? (uint16_t)((uint32_t)bmsResidualCap * 100 / (uint16_t)bmsSoc) : designCap;
        byte soh = (designCap > 0) ? (byte)((uint32_t)fullChargeCap * 100 / designCap) : 100;

        byte data[7] = {(byte)((fullCap >> 8) & 0xFF),
                        (byte)(fullCap & 0xFF),
                        (byte)((remainCap >> 8) & 0xFF),
                        (byte)(remainCap & 0xFF),
                        soc,
                        absSoc,
                        soh};
        CAN.sendMsgBuf(0x04F83400, 1, 7, data);
    }

    // ==================================================
    // 500 ms (BOTH FRAMES TOGETHER)
    // ==================================================
    if (now - t500 >= 500)
    {
        t500 = now;

        // byte ff1400[1] = {0x00};
        // CAN.sendMsgBuf(0x04FF1400, 1, 1, ff1400);

        byte f83402[1] = {0x00};
        CAN.sendMsgBuf(0x04F83402, 1, 1, f83402);
    }

    // ==================================================
    // 1000 ms
    // ==================================================
    if (now - t1000 >= 1000)
    {
        t1000 = now;

        byte data[6] = {0x49, 0x16, 0x01, 0x08, 0x11, 0x25};
        CAN.sendMsgBuf(0x04F83403, 1, 6, data);
    }
}