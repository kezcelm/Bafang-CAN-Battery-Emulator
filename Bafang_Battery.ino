#include "driver/twai.h"
#include "JbdBms.h"

struct MultiframeCmd;
struct SingleFrameEntry;

// ==================================================
// HARDWARE CONFIG (ESP32 + SN65HVD230)
// ==================================================
#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5
#define BMS_RX      16
#define BMS_TX      17

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
HardwareSerial bmsSerial(2);  // UART2
JbdBms bms(&bmsSerial);

// ==================================================
// CAN HELPER: send extended frame
// ==================================================
static bool canSend(uint32_t id, const uint8_t* data, uint8_t len) {
    twai_message_t msg = {};
    msg.identifier = id;
    msg.extd = 1;           // 29-bit extended ID
    msg.data_length_code = len;
    if (data && len > 0)
        memcpy(msg.data, data, len);
    return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
}

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
static const uint8_t DATA_6000_D[] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30, 0x20, 0x34};
static const uint8_t DATA_6000_E[] = {0x2E, 0x33};

// 0x6001: SW version "C20010 1.5"
static const uint8_t DATA_6001_D[] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30, 0x20, 0x31};
static const uint8_t DATA_6001_E[] = {0x2E, 0x35};

// 0x6002: Model "C20010"
static const uint8_t DATA_6002_D[] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30};

// 0x6003: Serial number (3 data frames + end frame)
static const uint8_t DATA_6003_D0[] = {0x41, 0x4C, 0x49, 0x31, 0x30, 0x53, 0x32, 0x33};
static const uint8_t DATA_6003_D1[] = {0x41, 0x4D, 0x30, 0x38, 0x37, 0x53, 0x43, 0x32};
static const uint8_t DATA_6003_D2[] = {0x34, 0x30, 0x37, 0x31, 0x37, 0x43, 0x32, 0x30};
static const uint8_t DATA_6003_E[]  = {0x30, 0x32, 0x31, 0x30, 0x30, 0x32, 0x33, 0x31};

// Multiframe command descriptor
struct MultiframeCmd {
    uint16_t cmdId;
    const uint8_t* dataFrames[3];
    uint8_t dataFrameLens[3];
    uint8_t numDataFrames;
    const uint8_t* endFrame;
    uint8_t endFrameLen;
    uint8_t totalLen;
    uint16_t endIdOffset;
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
static uint8_t data6400[4] = {0x0A, 0x07, 0x05, 0x00};
static uint8_t data6401[6] = {0x02, 0x00, 0x7B, 0x03, 0x7A, 0x02};
static uint8_t dataCellVoltages[32] = {0};  // 16 cells * 2 bytes LE

struct SingleFrameEntry {
    uint8_t* data;
    uint8_t len;
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
static uint8_t activeSource = 0;
static const MultiframeCmd* activeCmd = NULL;
static uint8_t frameIndex = 0;
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
    for (uint8_t i = 0; i < NUM_MULTIFRAME_CMDS; i++) {
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
    bmsData.voltage     = 39.0;
    bmsData.current     = 0.0;
    bmsData.soc         = 85.0;
    bmsData.cycle       = 2;
    bmsData.ratedCap    = 2010;    // 10mAh units (20.1 Ah)
    bmsData.residualCap = 1709;    // 10mAh units (17.09 Ah, ~85%)
    bmsData.temp1       = 22.0;
    bmsData.temp2       = 23.0;
    bmsData.protection  = 0;
    bmsData.valid       = true;

    // 10 cells @ ~3900 mV each
    bmsData.cells.NumOfCells = 10;
    for (uint8_t i = 0; i < 10; i++)
        bmsData.cells.CellVoltage[i] = 3900 + i;
    bmsData.cells.CellLow  = 3900;
    bmsData.cells.CellHigh = 3909;
    bmsData.cells.CellDiff = 9;
    bmsData.cells.CellAvg  = 3904;

    // Update single frame data
    data6400[0] = bmsData.cells.NumOfCells;
    data6400[2] = (uint8_t)(bmsData.cells.CellDiff & 0xFF);

    data6401[0] = (uint8_t)(bmsData.cycle & 0xFF);
    data6401[1] = (uint8_t)((bmsData.cycle >> 8) & 0xFF);

    for (uint8_t i = 0; i < bmsData.cells.NumOfCells && i < 16; i++) {
        dataCellVoltages[i * 2]     = (uint8_t)(bmsData.cells.CellVoltage[i] & 0xFF);
        dataCellVoltages[i * 2 + 1] = (uint8_t)((bmsData.cells.CellVoltage[i] >> 8) & 0xFF);
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
        data6400[2] = (uint8_t)(bmsData.cells.CellDiff & 0xFF);

        data6401[0] = (uint8_t)(bmsData.cycle & 0xFF);
        data6401[1] = (uint8_t)((bmsData.cycle >> 8) & 0xFF);

        for (uint8_t i = 0; i < bmsData.cells.NumOfCells && i < 16; i++) {
            dataCellVoltages[i * 2]     = (uint8_t)(bmsData.cells.CellVoltage[i] & 0xFF);
            dataCellVoltages[i * 2 + 1] = (uint8_t)((bmsData.cells.CellVoltage[i] >> 8) & 0xFF);
        }
    }
    */
}

// ==================================================
// MULTIFRAME PROTOCOL: Start / Data / End
// ==================================================
static void sendStartFrame() {
    uint8_t payload[1] = { activeCmd->totalLen };
    uint32_t canId = CAN_BASE_START(activeSource) | activeCmd->cmdId;
    canSend(canId, payload, 1);

    state = WAIT_ACK_START;
    lastStateChange = millis();
}

static void sendDataFrame() {
    uint32_t canId = CAN_BASE_DATA(activeSource) | (uint32_t)frameIndex;
    uint8_t len = activeCmd->dataFrameLens[frameIndex];
    canSend(canId, activeCmd->dataFrames[frameIndex], len);

    frameIndex++;
    state = WAIT_ACK_DATA;
    lastStateChange = millis();
}

static void sendEndFrame() {
    uint32_t canId = CAN_BASE_END(activeSource) | activeCmd->endIdOffset;
    canSend(canId, activeCmd->endFrame, activeCmd->endFrameLen);

    state = WAIT_ACK_END;
    lastStateChange = millis();
}

// ==================================================
// SINGLE FRAME HANDLER
// ==================================================
static void sendSingleFrame(uint16_t cmdId) {
    uint16_t index = cmdId - 0x6400;
    if (index >= NUM_SINGLE_FRAMES) return;

    uint32_t canId = CAN_BASE_SINGLE(activeSource) | cmdId;
    canSend(canId, SINGLE_FRAMES[index].data, SINGLE_FRAMES[index].len);
}

// ==================================================
// ACK HANDLER
// ==================================================
static void handleAck(uint32_t rxId) {
    if (activeCmd == NULL) return;

    uint32_t expectedAck = CAN_BASE_ACK(activeSource) | activeCmd->cmdId;
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
static void handleRequest(uint32_t rxId) {
    uint16_t cmdLow = rxId & 0xFFFF;

    if (cmdLow >= 0x6000 && cmdLow <= 0x6003) {
        uint8_t src = ((rxId >> 24) & 0xFF);
        if (src != 0x03 && src != 0x05) return;

        const MultiframeCmd* cmd = findMultiframeCmd(cmdLow);
        if (cmd == NULL) return;

        activeSource = src;
        activeCmd = cmd;
        frameIndex = 0;
        sendStartFrame();
    }
    else if (cmdLow >= 0x6400 && cmdLow <= 0x6405) {
        uint8_t src = ((rxId >> 24) & 0xFF);
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
    uint16_t voltageRaw = (uint16_t)(bmsData.voltage * 10);  // units: 100mV
    uint8_t tempF = (uint8_t)(bmsData.temp1 * 9.0 / 5.0 + 32.0);

    uint8_t data[5] = {
        (uint8_t)(currentRaw & 0xFF),
        (uint8_t)((currentRaw >> 8) & 0xFF),
        (uint8_t)(voltageRaw & 0xFF),
        (uint8_t)((voltageRaw >> 8) & 0xFF),
        tempF
    };
    canSend(CAN_ID_CURRENT_VOLTAGE, data, 5);
}

static void sendCapacitySoc() {
    uint16_t fullCap = bmsData.ratedCap * 10;
    uint16_t remainCap = bmsData.residualCap * 10;
    uint8_t soc = (uint8_t)bmsData.soc;

    uint16_t designCap = bmsData.ratedCap;
    uint16_t fcc = (bmsData.soc > 0)
        ? (uint16_t)((uint32_t)bmsData.residualCap * 100 / (uint16_t)bmsData.soc)
        : designCap;
    uint8_t soh = (designCap > 0)
        ? (uint8_t)((uint32_t)fcc * 100 / designCap)
        : 100;

    uint8_t data[7] = {
        (uint8_t)(fullCap & 0xFF),
        (uint8_t)((fullCap >> 8) & 0xFF),
        (uint8_t)(remainCap & 0xFF),
        (uint8_t)((remainCap >> 8) & 0xFF),
        soc,
        soc,   // absSoc
        soh
    };
    canSend(CAN_ID_CAPACITY_SOC, data, 7);
}

static void sendHeartbeat() {
    uint8_t data[1] = {0x00};
    canSend(CAN_ID_HEARTBEAT, data, 1);
}

static void sendTimestamp() {
    uint8_t data[6] = {0x49, 0x11, 0x15, 0x01, 0x06, 0x26};
    canSend(CAN_ID_TIMESTAMP, data, 6);
}

// ==================================================
// SETUP
// ==================================================
void setup() {
    Serial.begin(115200);

    // BMS UART
    bmsSerial.begin(9600, SERIAL_8N1, BMS_RX, BMS_TX);

    // TWAI (CAN) config: 250kbps, no filter
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        Serial.println("TWAI (CAN) started - SN65HVD230");
    } else {
        Serial.println("TWAI init FAILED");
    }

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
    twai_message_t rxMsg;
    if (twai_receive(&rxMsg, 0) == ESP_OK) {
        uint32_t rxId = rxMsg.identifier;

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
