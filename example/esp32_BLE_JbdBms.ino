#include <NimBLEDevice.h>

#define BMS_MAC "A5:C2:39:31:67:61"

// JBD UUID
static NimBLEUUID SERVICE_UUID("0000FF00-0000-1000-8000-00805F9B34FB");
static NimBLEUUID RX_UUID("0000FF02-0000-1000-8000-00805F9B34FB");
static NimBLEUUID TX_UUID("0000FF01-0000-1000-8000-00805F9B34FB");

NimBLERemoteCharacteristic* rxChar;
NimBLERemoteCharacteristic* txChar;

// ================== BUFFER ==================
std::vector<uint8_t> buffer;

// ================== STORAGE ==================
float voltage = 0;
float current = 0;
float soc = 0;
uint16_t cycle = 0;
float t1 = 0;
float t2 = 0;

float cell[24];
uint8_t cellCount = 0;

// ================== NOTIFY ==================
void notifyCB(
    NimBLERemoteCharacteristic* c,
    uint8_t* data,
    size_t len,
    bool isNotify)
{
    for (int i = 0; i < len; i++) {
        buffer.push_back(data[i]);
    }

    if (!buffer.empty() && buffer.back() == 0x77) {

        // ================= BASIC (0x03) =================
        if (buffer[0] == 0xDD && buffer[1] == 0x03) {

            uint16_t v = (buffer[4] << 8) | buffer[5];
            int16_t i = (buffer[6] << 8) | buffer[7];

            voltage = v / 100.0;
            current = i / 100.0;

            soc = buffer[23];

            cycle = (buffer[12] << 8) | buffer[13];

            uint16_t tt1 = (buffer[27] << 8) | buffer[28];
            uint16_t tt2 = (buffer[29] << 8) | buffer[30];

            t1 = (tt1 - 2731) / 10.0;
            t2 = (tt2 - 2731) / 10.0;
        }

        // ================= CELLS (0x04) =================
        if (buffer[1] == 0x04) {

            cellCount = buffer[3] / 2;
            int offset = 4;

            for (int i = 0; i < cellCount; i++) {
                uint16_t v =
                    (buffer[offset + i * 2] << 8) |
                    buffer[offset + i * 2 + 1];

                cell[i] = v / 1000.0;
            }
        }

        buffer.clear();
    }
}

// ================== CONNECT ==================
bool connectBMS() {

    NimBLEDevice::init("");
    NimBLEClient* client = NimBLEDevice::createClient();

    NimBLEAddress addr(BMS_MAC, 0);

    if (!client->connect(addr)) {
        Serial.println("Connection failed");
        return false;
    }

    auto service = client->getService(SERVICE_UUID);

    rxChar = service->getCharacteristic(RX_UUID);
    txChar = service->getCharacteristic(TX_UUID);

    txChar->subscribe(true, notifyCB);

    Serial.println("Connected to BMS");
    return true;
}

// ================== SEND COMMANDS ==================
void sendBasic() {
    uint8_t cmd[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
    rxChar->writeValue(cmd, sizeof(cmd), false);
}

void sendCells() {
    uint8_t cmd[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
    rxChar->writeValue(cmd, sizeof(cmd), false);
}

// ================== SETUP ==================
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("JBD BLE START");

    connectBMS();
}

// ================== LOOP ==================
void loop() {

    sendBasic();
    delay(200);

    sendCells();
    delay(500);

    // ================= PRINT =================
    Serial.println("============== BMS ==============");

    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println(" V");

    Serial.print("Current: ");
    Serial.print(current);
    Serial.println(" A");

    Serial.print("SOC: ");
    Serial.print(soc);
    Serial.println(" %");

    Serial.print("Cycles: ");
    Serial.println(cycle);

    Serial.print("Temp1: ");
    Serial.print(t1);
    Serial.println(" C");

    Serial.print("Temp2: ");
    Serial.print(t2);
    Serial.println(" C");

    Serial.println("--- Cells ---");

    for (int i = 0; i < cellCount; i++) {
        Serial.print("Cell ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(cell[i], 3);
        Serial.println(" V");
    }

    Serial.println("================================");

    delay(500);
}