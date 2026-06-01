#include <SPI.h>
#include <mcp_can.h>
#include "JbdBms.h"

// ==================================================
// HARDWARE CONFIG
// ==================================================
#define CAN_CS      10
#define BMS_RX      2
#define BMS_TX      3
#define CAN_SPEED   CAN_250KBPS
#define CAN_CLOCK   MCP_8MHz

// ==================================================
// TIMING INTERVALS (ms)
// ==================================================
#define INTERVAL_BMS      500
#define INTERVAL_100MS    100
#define INTERVAL_200MS    200
#define INTERVAL_500MS    500
#define INTERVAL_1000MS   1000
#define ACK_TIMEOUT       500

// ==================================================
// CAN ID BASES (source: 0x03=Display, 0x05=Controller)
// ==================================================
#define CAN_BASE_START(src)   ((src) == 0x05 ? 0x042C0000UL : 0x041C0000UL)
#define CAN_BASE_DATA(src)    ((src) == 0x05 ? 0x042D0000UL : 0x041D0000UL)
#define CAN_BASE_END(src)     ((src) == 0x05 ? 0x042E0000UL : 0x041E0000UL)
#define CAN_BASE_SINGLE(src)  ((src) == 0x05 ? 0x042A0000UL : 0x041A0000UL)
#define CAN_BASE_ACK(src)     ((src) == 0x05 ? 0x05220000UL : 0x03220000UL)

#define CAN_ID_CURRENT_VOLTAGE  0x04F83401UL
#define CAN_ID_CAPACITY_SOC     0x04F83400UL
#define CAN_ID_HEARTBEAT        0x04F83402UL
#define CAN_ID_TIMESTAMP        0x04F83403UL

// ==================================================
// HARDWARE INSTANCES
// ==================================================
MCP_CAN CAN(CAN_CS);
SoftwareSerial bmsSerial(BMS_RX, BMS_TX);
JbdBms bms(&bmsSerial);

// ==================================================
// BMS LIVE DATA
// ==================================================
struct BmsData {
    float voltage;        // V
    float current;        // mA
    float soc;            // %
    uint16_t cycle;
    uint16_t ratedCap;    // 10mAh units
    uint16_t residualCap; // 10mAh units
    float temp1;          // °C (external probe)
    float temp2;          // °C (onboard)
    uint16_t protection;
    packCellInfoStruct cells;
    bool valid;
};

static BmsData bmsData = {};

// ==================================================
// MULTIFRAME PROTOCOL DATA (0x6000-0x6003)
// ==================================================

// 0x6000: HW version "C20010 4.3"
static const byte DATA_6000_D[] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30, 0x20, 0x34};
static const byte DATA_6000_E[] = {0x2E, 0x33};

// 0x6001: SW version "C20010 1.5"
static const byte DATA_6001_D[] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30, 0x20, 0x31};
static const byte DATA_6001_E[] = {0x2E, 0x35};

// 0x6002: Model "C20010"
static const byte DATA_6002_D[] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30};

// 0x6003: Serial number (3 data frames + end frame)
static const byte DATA_6003_D0[] = {0x41, 0x4C, 0x49, 0x31, 0x30, 0x53, 0x32, 0x33};
static const byte DATA_6003_D1[] = {0x41, 0x4D, 0x30, 0x38, 0x37, 0x53, 0x43, 0x32};
static const byte DATA_6003_D2[] = {0x34, 0x30, 0x37, 0x31, 0x37, 0x43, 0x32, 0x30};
static const byte DATA_6003_E[]  = {0x30, 0x32, 0x31, 0x30, 0x30, 0x32, 0x33, 0x31};

// Multiframe command descriptor
struct MultiframeCmd {
    uint16_t cmdId;
    const byte* dataFrames[3];
    byte dataFrameLens[3];
    byte numDataFrames;
    const byte* endFrame;
    byte endFrameLen;
    byte totalLen;
    uint16_t endIdOffset;  // added to end base ID
};

static const MultiframeCmd MULTIFRAME_CMDS[] = {
    {0x6000, {DATA_6000_D},                       {8},       1, DATA_6000_E, 2, 0x0A, 1},
    {0x6001, {DATA_6001_D},                       {8},       1, DATA_6001_E, 2, 0x0A, 1},
    {0x6002, {DATA_6002_D},                       {6},       1, NULL,        0, 0x06, 1},
    {0x6003, {DATA_6003_D0, DATA_6003_D1, DATA_6003_D2}, {8, 8, 8}, 3, DATA_6003_E, 8, 0x20, 3},
};
#define NUM_MULTIFRAME_CMDS 4

// ==================================================
// SINGLE FRAME DATA (0x6400-0x6405)
// ==================================================
static byte data6400[4] = {0x0A, 0x07, 0x05, 0x00};
static byte data6401[6] = {0x02, 0x00, 0x7B, 0x03, 0x7A, 0x02};
static byte dataCellVoltages[32] = {0};  // 16 cells * 2 bytes LE

// Lookup table for single frame commands
struct SingleFrameEntry {
    byte* data;
    byte len;
};

static const SingleFrameEntry SINGLE_FRAMES[] = {
    {data6400,               4},  // 0x6400
    {data6401,               6},  // 0x6401
    {&dataCellVoltages[0],   8},  // 0x6402: cells 1-4
    {&dataCellVoltages[8],   8},  // 0x6403: cells 5-8
    {&dataCellVoltages[16],  8},  // 0x6404: cells 9-12
    {&dataCellVoltages[24],  8},  // 0x6405: cells 13-16
};
#define NUM_SINGLE_FRAMES 6

// ==================================================
// STATE MACHINE
// ==================================================
enum State { IDLE, WAIT_ACK_START, WAIT_ACK_DATA, WAIT_ACK_END };

static State state = IDLE;
static byte activeSource = 0;
static const MultiframeCmd* activeCmd = NULL;
static byte frameIndex = 0;
static unsigned long lastStateChange = 0;

// Timers
static unsigned long tBmsRead = 0;
static unsigned long t100 = 0;
static unsigned long t200 = 0;
static unsigned long t500 = 0;
static unsigned long t1000 = 0;

// ==================================================
// HELPER: Find multiframe command by ID
// ==================================================
static const MultiframeCmd* findMultiframeCmd(uint16_t cmdId) {
    for (byte i = 0; i < NUM_MULTIFRAME_CMDS; i++) {
        if (MULTIFRAME_CMDS[i].cmdId == cmdId)
            return &MULTIFRAME_CMDS[i];
    }
    return NULL;
}

// ==================================================
// BMS DATA REFRESH
// ==================================================
static void updateBmsData() {
    // Hardcoded test values (BMS readout commented out)
    bmsData.voltage     = 39.0;    // V
    bmsData.current     = 0.0;     // mA
    bmsData.soc         = 85.0;    // %
    bmsData.cycle       = 2;
    bmsData.ratedCap    = 2010;    // 10mAh units (20.1 Ah)
    bmsData.residualCap = 1709;    // 10mAh units (17.09 Ah, ~85%)
    bmsData.temp1       = 22.0;    // °C
    bmsData.temp2       = 23.0;    // °C
    bmsData.protection  = 0;
    bmsData.valid       = true;

    // 10 cells @ ~3900 mV each
    bmsData.cells.NumOfCells = 10;
    for (byte i = 0; i < 10; i++)
        bmsData.cells.CellVoltage[i] = 3900 + i;  // slight variation
    bmsData.cells.CellLow  = 3900;
    bmsData.cells.CellHigh = 3909;
    bmsData.cells.CellDiff = 9;
    bmsData.cells.CellAvg  = 3904;

    // Update single frame data
    data6400[0] = bmsData.cells.NumOfCells;
    data6400[2] = (byte)(bmsData.cells.CellDiff & 0xFF);

    data6401[0] = (byte)(bmsData.cycle & 0xFF);
    data6401[1] = (byte)((bmsData.cycle >> 8) & 0xFF);

    for (byte i = 0; i < bmsData.cells.NumOfCells && i < 16; i++) {
        dataCellVoltages[i * 2]     = (byte)(bmsData.cells.CellVoltage[i] & 0xFF);
        dataCellVoltages[i * 2 + 1] = (byte)((bmsData.cells.CellVoltage[i] >> 8) & 0xFF);
    }

    /*
    // --- LIVE BMS READOUT (uncomment to restore) ---
    if (bms.readBmsData()) {
        bmsData.voltage    = bms.getVoltage();
        bmsData.current    = bms.getCurrent();
        bmsData.soc        = bms.getChargePercentage();
        bmsData.cycle      = bms.getCycle();
        bmsData.ratedCap   = bms.getRatedCapacity();
        bmsData.residualCap = bms.getResidualCapacity();
        bmsData.temp1      = bms.getTemp1();
        bmsData.temp2      = bms.getTemp2();
        bmsData.protection = bms.getProtectionState();
        bmsData.valid      = true;
    }

    if (bms.readPackData()) {
        bmsData.cells = bms.getPackCellInfo();

        data6400[0] = bmsData.cells.NumOfCells;
        data6400[2] = (byte)(bmsData.cells.CellDiff & 0xFF);

        data6401[0] = (byte)(bmsData.cycle & 0xFF);
        data6401[1] = (byte)((bmsData.cycle >> 8) & 0xFF);

        for (byte i = 0; i < bmsData.cells.NumOfCells && i < 16; i++) {
            dataCellVoltages[i * 2]     = (byte)(bmsData.cells.CellVoltage[i] & 0xFF);
            dataCellVoltages[i * 2 + 1] = (byte)((bmsData.cells.CellVoltage[i] >> 8) & 0xFF);
        }
    }
    */
}

// ==================================================
// MULTIFRAME PROTOCOL: Start / Data / End
// ==================================================
static void sendStartFrame() {
    byte payload[1] = { activeCmd->totalLen };
    unsigned long canId = CAN_BASE_START(activeSource) | activeCmd->cmdId;
    CAN.sendMsgBuf(canId, 1, 1, payload);

    state = WAIT_ACK_START;
    lastStateChange = millis();
}

static void sendDataFrame() {
    unsigned long canId = CAN_BASE_DATA(activeSource) | (unsigned long)frameIndex;

    byte len = activeCmd->dataFrameLens[frameIndex];
    // Cast away const for MCP_CAN API (it doesn't modify the buffer)
    CAN.sendMsgBuf(canId, 1, len, (byte*)activeCmd->dataFrames[frameIndex]);

    frameIndex++;
    state = WAIT_ACK_DATA;
    lastStateChange = millis();
}

static void sendEndFrame() {
    unsigned long canId = CAN_BASE_END(activeSource) | activeCmd->endIdOffset;

    if (activeCmd->endFrame != NULL && activeCmd->endFrameLen > 0) {
        CAN.sendMsgBuf(canId, 1, activeCmd->endFrameLen, (byte*)activeCmd->endFrame);
    } else {
        CAN.sendMsgBuf(canId, 1, 0, NULL);
    }

    state = WAIT_ACK_END;
    lastStateChange = millis();
}

// ==================================================
// SINGLE FRAME HANDLER
// ==================================================
static void sendSingleFrame(uint16_t cmdId) {
    uint16_t index = cmdId - 0x6400;
    if (index >= NUM_SINGLE_FRAMES) return;

    unsigned long canId = CAN_BASE_SINGLE(activeSource) | cmdId;
    CAN.sendMsgBuf(canId, 1, SINGLE_FRAMES[index].len, SINGLE_FRAMES[index].data);
}

// ==================================================
// ACK HANDLER
// ==================================================
static void handleAck(unsigned long rxId) {
    if (activeCmd == NULL) return;

    unsigned long expectedAck = CAN_BASE_ACK(activeSource) | activeCmd->cmdId;
    if (rxId != expectedAck) return;

    switch (state) {
        case WAIT_ACK_START:
            frameIndex = 0;
            sendDataFrame();
            break;
        case WAIT_ACK_DATA:
            if (frameIndex >= activeCmd->numDataFrames)
                sendEndFrame();
            else
                sendDataFrame();
            break;
        case WAIT_ACK_END:
            state = IDLE;
            activeCmd = NULL;
            break;
        default:
            break;
    }
}

// ==================================================
// REQUEST HANDLER
// ==================================================
static void handleRequest(unsigned long rxId) {
    uint16_t cmdLow = rxId & 0xFFFF;

    // Multiframe requests: 0x__216000 - 0x__216003
    if (cmdLow >= 0x6000 && cmdLow <= 0x6003) {
        byte src = ((rxId >> 24) & 0xFF);
        if (src != 0x03 && src != 0x05) return;

        const MultiframeCmd* cmd = findMultiframeCmd(cmdLow);
        if (cmd == NULL) return;

        activeSource = src;
        activeCmd = cmd;
        frameIndex = 0;
        sendStartFrame();
    }
    // Single frame requests: 0x__216400 - 0x__216405
    else if (cmdLow >= 0x6400 && cmdLow <= 0x6405) {
        byte src = ((rxId >> 24) & 0xFF);
        if (src != 0x03 && src != 0x05) return;

        activeSource = src;
        sendSingleFrame(cmdLow);
    }
}

// ==================================================
// PERIODIC CAN BROADCASTS
// ==================================================
static void sendCurrentVoltageTemp() {
    int16_t currentRaw = (int16_t)(bmsData.current / 10);
    uint16_t voltageRaw = (uint16_t)(bmsData.voltage * 10);
    byte tempF = (byte)(bmsData.temp1 * 9.0 / 5.0 + 32.0);

    byte data[5] = {
        (byte)((currentRaw >> 8) & 0xFF),
        (byte)(currentRaw & 0xFF),
        (byte)((voltageRaw >> 8) & 0xFF),
        (byte)(voltageRaw & 0xFF),
        tempF
    };
    CAN.sendMsgBuf(CAN_ID_CURRENT_VOLTAGE, 1, 5, data);
}

static void sendCapacitySoc() {
    uint16_t fullCap = bmsData.ratedCap * 10;
    uint16_t remainCap = bmsData.residualCap * 10;
    byte soc = (byte)bmsData.soc;

    // SOH estimation: FCC / design capacity * 100
    uint16_t designCap = bmsData.ratedCap;
    uint16_t fcc = (bmsData.soc > 0)
        ? (uint16_t)((uint32_t)bmsData.residualCap * 100 / (uint16_t)bmsData.soc)
        : designCap;
    byte soh = (designCap > 0)
        ? (byte)((uint32_t)fcc * 100 / designCap)
        : 100;

    byte data[7] = {
        (byte)((fullCap >> 8) & 0xFF),
        (byte)(fullCap & 0xFF),
        (byte)((remainCap >> 8) & 0xFF),
        (byte)(remainCap & 0xFF),
        soc,
        soc,   // absSoc = soc
        soh
    };
    CAN.sendMsgBuf(CAN_ID_CAPACITY_SOC, 1, 7, data);
}

static void sendHeartbeat() {
    byte data[1] = {0x00};
    CAN.sendMsgBuf(CAN_ID_HEARTBEAT, 1, 1, data);
}

static void sendTimestamp() {
    byte data[6] = {0x49, 0x11, 0x15, 0x01, 0x06, 0x26}; //Battery Time Information s/m/h d/m/y
    CAN.sendMsgBuf(CAN_ID_TIMESTAMP, 1, 6, data);
}

// ==================================================
// SETUP
// ==================================================
void setup() {
    Serial.begin(9600);
    CAN.begin(CAN_250KBPS, MCP_8MHz);
    CAN.setMode(MODE_NORMAL);

    Serial.println(F("Bafang CAN emulator + JBD BMS started"));
    updateBmsData();
}

// ==================================================
// MAIN LOOP
// ==================================================
void loop() {
    unsigned long now = millis();

    // Timeout: reset state machine if ACK not received
    if (state != IDLE && (now - lastStateChange > ACK_TIMEOUT)) {
        state = IDLE;
        activeCmd = NULL;
    }

    // Process incoming CAN messages
    if (CAN.checkReceive()) {
        byte len = 0;
        byte buf[8];
        CAN.readMsgBuf(&len, buf);
        unsigned long rxId = CAN.getCanId();

        if (state != IDLE)
            handleAck(rxId);
        else
            handleRequest(rxId);
    }

    // Periodic BMS read
    if (now - tBmsRead >= INTERVAL_BMS) {
        tBmsRead = now;
        updateBmsData();
    }

    // Periodic CAN broadcasts
    if (now - t100 >= INTERVAL_100MS) {
        t100 = now;
        sendCurrentVoltageTemp();
    }

    if (now - t200 >= INTERVAL_200MS) {
        t200 = now;
        sendCapacitySoc();
    }

    if (now - t500 >= INTERVAL_500MS) {
        t500 = now;
        sendHeartbeat();
    }

    if (now - t1000 >= INTERVAL_1000MS) {
        t1000 = now;
        sendTimestamp();
    }
}
