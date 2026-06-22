#include "driver/twai.h"
#include <NimBLEDevice.h>

// Forward declarations
struct MultiframeCmd;
struct SingleFrameEntry;

// ==================================================
// HARDWARE CONFIG (ESP32 + SN65HVD230)
// ==================================================
#define CAN_TX_PIN  GPIO_NUM_5
#define CAN_RX_PIN  GPIO_NUM_4

// BMS BLE address
#define BMS_MAC "A5:C2:39:31:67:61"

// JBD BLE UUIDs
static NimBLEUUID SERVICE_UUID("0000FF00-0000-1000-8000-00805F9B34FB");
static NimBLEUUID RX_UUID("0000FF02-0000-1000-8000-00805F9B34FB");  // write commands here
static NimBLEUUID TX_UUID("0000FF01-0000-1000-8000-00805F9B34FB");  // notifications come here

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
// BLE INSTANCES
// ==================================================
static NimBLERemoteCharacteristic* rxChar = NULL;
static NimBLERemoteCharacteristic* txChar = NULL;
static bool bleConnected = false;

// ==================================================
// BMS LIVE DATA
// ==================================================
struct BmsData {
    float voltage;        // V
    float current;        // A
    float soc;            // %
    uint16_t cycle;
    uint16_t ratedCap;    // 10mAh units
    uint16_t residualCap; // 10mAh units
    float temp1;          // °C
    float temp2;          // °C
    uint16_t protection;
    uint8_t cellCount;
    float cellVoltage[24]; // V
    bool valid;
};

static BmsData bmsData = {};

// ==================================================
// BLE NOTIFY BUFFER & CALLBACK
// ==================================================
static std::vector<uint8_t> bleBuffer;

static void bleNotifyCB(
    NimBLERemoteCharacteristic* c,
    uint8_t* data,
    size_t len,
    bool isNotify)
{
    for (size_t i = 0; i < len; i++)
        bleBuffer.push_back(data[i]);

    // Frame complete when we see end byte 0x77
    if (!bleBuffer.empty() && bleBuffer.back() == 0x77) {

        // Basic info response (register 0x03)
        if (bleBuffer.size() > 30 && bleBuffer[0] == 0xDD && bleBuffer[1] == 0x03) {
            uint16_t v = (bleBuffer[4] << 8) | bleBuffer[5];
            int16_t i = (bleBuffer[6] << 8) | bleBuffer[7];

            bmsData.voltage = v / 100.0f;
            bmsData.current = i / 100.0f;

            bmsData.residualCap = (bleBuffer[8] << 8) | bleBuffer[9];
            bmsData.ratedCap    = (bleBuffer[10] << 8) | bleBuffer[11];
            bmsData.cycle       = (bleBuffer[12] << 8) | bleBuffer[13];

            bmsData.protection  = (bleBuffer[20] << 8) | bleBuffer[21];
            bmsData.soc         = bleBuffer[23];

            uint16_t tt1 = (bleBuffer[27] << 8) | bleBuffer[28];
            uint16_t tt2 = (bleBuffer[29] << 8) | bleBuffer[30];
            bmsData.temp1 = (tt1 - 2731) / 10.0f;
            bmsData.temp2 = (tt2 - 2731) / 10.0f;

            bmsData.valid = true;
        }

        // Cell voltages response (register 0x04)
        if (bleBuffer.size() > 4 && bleBuffer[0] == 0xDD && bleBuffer[1] == 0x04) {
            bmsData.cellCount = bleBuffer[3] / 2;
            uint8_t offset = 4;

            for (uint8_t i = 0; i < bmsData.cellCount && i < 24; i++) {
                uint16_t v = (bleBuffer[offset + i * 2] << 8) |
                              bleBuffer[offset + i * 2 + 1];
                bmsData.cellVoltage[i] = v / 1000.0f;  // mV -> V
            }
        }

        bleBuffer.clear();
    }
}

// ==================================================
// BLE CONNECT
// ==================================================
static bool connectBms() {
    NimBLEDevice::init("BafangBat");
    NimBLEClient* client = NimBLEDevice::createClient();

    NimBLEAddress addr(BMS_MAC, 0);
    if (!client->connect(addr)) {
        Serial.println("BLE: connection failed");
        return false;
    }

    auto* service = client->getService(SERVICE_UUID);
    if (!service) {
        Serial.println("BLE: service not found");
        return false;
    }

    rxChar = service->getCharacteristic(RX_UUID);
    txChar = service->getCharacteristic(TX_UUID);

    if (!rxChar || !txChar) {
        Serial.println("BLE: characteristics not found");
        return false;
    }

    txChar->subscribe(true, bleNotifyCB);
    bleConnected = true;
    Serial.println("BLE: connected to BMS");
    return true;
}

// ==================================================
// BLE REQUEST COMMANDS
// ==================================================
static void requestBmsBasic() {
    if (!bleConnected || !rxChar) return;
    uint8_t cmd[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
    rxChar->writeValue(cmd, sizeof(cmd), false);
}

static void requestBmsCells() {
    if (!bleConnected || !rxChar) return;
    uint8_t cmd[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
    rxChar->writeValue(cmd, sizeof(cmd), false);
}

// ==================================================
// CAN HELPER: send extended frame
// ==================================================
static bool canSend(uint32_t id, const uint8_t* data, uint8_t len) {
    twai_message_t msg = {};
    msg.identifier = id;
    msg.extd = 1;
    msg.data_length_code = len;
    if (data && len > 0)
        memcpy(msg.data, data, len);
    return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
}

// ==================================================
// MULTIFRAME PROTOCOL DATA (0x6000-0x6003)
// ==================================================

// static const uint8_t DATA_6000_D[] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30, 0x20, 0x34}; //C20010 4
// static const uint8_t DATA_6000_E[] = {0x2E, 0x33};                                     //.3

// static const uint8_t DATA_6001_D[] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30, 0x20, 0x31}; //C20010 1
// static const uint8_t DATA_6001_E[] = {0x2E, 0x35};                                     //.5

// static const uint8_t DATA_6002_D[] = {0x43, 0x32, 0x30, 0x30, 0x31, 0x30};

// static const uint8_t DATA_6003_D0[] = {0x41, 0x4C, 0x49, 0x31, 0x30, 0x53, 0x32, 0x33};
// static const uint8_t DATA_6003_D1[] = {0x41, 0x4D, 0x30, 0x38, 0x37, 0x53, 0x43, 0x32};
// static const uint8_t DATA_6003_D2[] = {0x34, 0x30, 0x37, 0x31, 0x37, 0x43, 0x32, 0x30};
// static const uint8_t DATA_6003_E[]  = {0x30, 0x32, 0x31, 0x30, 0x30, 0x32, 0x33, 0x31};
static const uint8_t DATA_6000_D[] = {0x45, 0x53, 0x50, 0x33, 0x32, 0x43, 0x20, 0x42}; // ESP32C B
static const uint8_t DATA_6000_E[] = {0x4C, 0x45};                                     // LE

static const uint8_t DATA_6001_D[] = {0x42, 0x41, 0x46, 0x45, 0x4D, 0x55, 0x20, 0x31}; // BAFEMU 1
static const uint8_t DATA_6001_E[] = {0x2E, 0x30};                                     // .0

static const uint8_t DATA_6002_D[] = {0x42, 0x41, 0x46, 0x45, 0x4D, 0x55};

static const uint8_t DATA_6003_D0[] = {0x42, 0x61, 0x66, 0x61, 0x6E, 0x67, 0x20, 0x43};  // "Bafang C"
static const uint8_t DATA_6003_D1[] = {0x41, 0x4E, 0x20, 0x42, 0x61, 0x74, 0x74, 0x65};  // "AN Batte"
static const uint8_t DATA_6003_D2[] = {0x72, 0x79, 0x20, 0x45, 0x6D, 0x75, 0x6C, 0x61};  // "ry Emula"
static const uint8_t DATA_6003_E[]  = {0x74, 0x6F, 0x72, 0x20, 0x20, 0x20, 0x20, 0x20};  // "tor     "

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
    {data6400,               4},
    {data6401,               6},
    {&dataCellVoltages[0],   8},
    {&dataCellVoltages[8],   8},
    {&dataCellVoltages[16],  8},
    {&dataCellVoltages[24],  8},
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
// UPDATE SINGLE FRAME DATA FROM BMS
// ==================================================
static void updateCanDataFromBms() {
    // data6400: [numCells, numParallel, cellDiff_mV, 0]
    data6400[0] = bmsData.cellCount;
    if (bmsData.cellCount > 0) {
        float cellMin = bmsData.cellVoltage[0];
        float cellMax = bmsData.cellVoltage[0];
        for (uint8_t i = 1; i < bmsData.cellCount; i++) {
            if (bmsData.cellVoltage[i] < cellMin) cellMin = bmsData.cellVoltage[i];
            if (bmsData.cellVoltage[i] > cellMax) cellMax = bmsData.cellVoltage[i];
        }
        data6400[2] = (uint8_t)((uint16_t)((cellMax - cellMin) * 1000) & 0xFF);
    }

    // data6401: [cycle_L, cycle_H, ...]
    data6401[0] = (uint8_t)(bmsData.cycle & 0xFF);
    data6401[1] = (uint8_t)((bmsData.cycle >> 8) & 0xFF);

    // Cell voltages in mV, little-endian
    for (uint8_t i = 0; i < bmsData.cellCount && i < 16; i++) {
        uint16_t mv = (uint16_t)(bmsData.cellVoltage[i] * 1000);
        dataCellVoltages[i * 2]     = (uint8_t)(mv & 0xFF);
        dataCellVoltages[i * 2 + 1] = (uint8_t)((mv >> 8) & 0xFF);
    }
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
    int16_t currentRaw = (int16_t)(bmsData.current * 10);  // 100mA units
    uint16_t voltageRaw = (uint16_t)(bmsData.voltage * 10); // 100mV units
    uint8_t tempC = (uint8_t)(bmsData.temp1);

    uint8_t data[5] = {
        (uint8_t)(currentRaw & 0xFF),
        (uint8_t)((currentRaw >> 8) & 0xFF),
        (uint8_t)(voltageRaw & 0xFF),
        (uint8_t)((voltageRaw >> 8) & 0xFF),
        tempC
    };
    canSend(CAN_ID_CURRENT_VOLTAGE, data, 5);
}

static void sendCapacitySoc() {
    uint16_t fullCap = bmsData.ratedCap * 10;     // -> mAh
    uint16_t remainCap = bmsData.residualCap * 10; // -> mAh
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
        soc,
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
    delay(1000);

    // TWAI (CAN) init: 250kbps via SN65HVD230
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        Serial.println("CAN: started (SN65HVD230, 250kbps)");
    } else {
        Serial.println("CAN: init FAILED");
    }

    // BLE connect to JBD BMS
    if (connectBms()) {
        // Initial BMS read & send single frames to CAN
        delay(200);
        requestBmsBasic();
        delay(200);
        requestBmsCells();
        delay(300);
        updateCanDataFromBms();

        for (uint16_t cmd = 0x6400; cmd <= 0x6405; cmd++) {
            uint32_t canId = CAN_BASE_SINGLE(0x03) | cmd;
            uint16_t index = cmd - 0x6400;
            if (index < NUM_SINGLE_FRAMES)
                canSend(canId, SINGLE_FRAMES[index].data, SINGLE_FRAMES[index].len);
        }
        Serial.println("CAN: initial single frames sent");
    } else {
        Serial.println("BLE: will retry in loop");
    }
}

// ==================================================
// MAIN LOOP
// ==================================================
void loop() {
    unsigned long now = millis();

    // Reconnect BLE if needed
    if (!bleConnected) {
        connectBms();
        delay(2000);
        return;
    }

    // Timeout: reset CAN state machine
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

    // Periodic BMS requests via BLE
    if (now - tBmsRead >= INTERVAL_BMS) {
        tBmsRead = now;
        requestBmsBasic();
        delay(100);
        requestBmsCells();
        delay(100);
        updateCanDataFromBms();
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
