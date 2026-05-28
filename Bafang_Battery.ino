#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS 10

MCP_CAN CAN(CAN_CS);

// Timers
unsigned long t100 = 0;
unsigned long t200 = 0;
unsigned long t500 = 0;
unsigned long t1000 = 0;

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
                    0x07,  // Number of paraler cell
                    0x05,  // Cell Voltage diference
                    0x00}; //
byte data6401[6] = {0x02, 0x00,  // Charging cycle D0/D1
                    0x7B, 0x03,  // Max uncharged time M.L.T
                    0x7A, 0x02}; // Last uncharged time N.L.T
byte data6402[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E};
byte data6403[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E};
byte data6404[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0x00, 0x00, 0x00, 0x00};
byte data6405[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

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

    Serial.println("Bafang CAN emulator started");
}

void loop()
{
    unsigned long now = millis();

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

        // ==================================================
        // Battery <-> BESST
        // 0x032164xx -> 0x041A64xx
        // ==================================================
        if (rxId == 0x05216400)
        {
            Serial.println(rxId, HEX);
            byte data[4] = {0x0A,  // Number of serial cell
                            0x07,  // Number of paraler cell
                            0x05,  // Cell Voltage diference
                            0x00}; //
            CAN.sendMsgBuf(0x042A6400, 1, 4, data);
        }  
        else if (rxId == 0x05216401)
        {
            Serial.println(rxId, HEX);
            byte data[6] = {0x02, 0x00,  // Charging cycle D0/D1
                            0x7B, 0x03,  // Max uncharged time M.L.T
                            0x7A, 0x02}; // Last uncharged time N.L.T
            CAN.sendMsgBuf(0x042A6401, 1, 6, data);
        }
        else if (rxId == 0x05216402)
        {
            Serial.println(rxId, HEX);
            byte data[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E};
            CAN.sendMsgBuf(0x042A6402, 1, 8, data);
        }
        else if (rxId == 0x05216403)
        {
            Serial.println(rxId, HEX);
            byte data[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E};
            CAN.sendMsgBuf(0x042A6403, 1, 8, data);
        }
        else if (rxId == 0x05216404)
        {
            Serial.println(rxId, HEX);
            byte data[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0x00, 0x00, 0x00, 0x00};
            CAN.sendMsgBuf(0x042A6404, 1, 8, data);
        }
        else if (rxId == 0x05216405)
        {
            Serial.println(rxId, HEX);
            byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            };
            CAN.sendMsgBuf(0x042A6405, 1, 8, data);
        }
        
        // ==================================================
        // Battery <-> Display
        // 0x032164xx -> 0x041A64xx
        // ==================================================

        if (rxId == 0x03216400)
        {
            Serial.println(rxId, HEX);
            byte data[4] = {0x0A,  // Number of serial cell
                            0x07,  // Number of paraler cell
                            0x05,  // cell Voltage diference
                            0x00}; //
            CAN.sendMsgBuf(0x041A6400, 1, 4, data);
        }

        else if (rxId == 0x03216401)
        {
            Serial.println(rxId, HEX);
            byte data[6] = {0x02, 0x00,  // Charging cycle D0/D1
                            0x7B, 0x03,  // Max uncharged time M.L.T
                            0x7A, 0x02}; // Last uncharged time N.L.T
            CAN.sendMsgBuf(0x041A6401, 1, 6, data);
        }

        else if (rxId == 0x03216402)
        {
            Serial.println(rxId, HEX);
            byte data[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E};
            CAN.sendMsgBuf(0x041A6402, 1, 8, data);
        }

        else if (rxId == 0x03216403)
        {
            Serial.println(rxId, HEX);
            byte data[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E, 0xA3, 0x0E};
            CAN.sendMsgBuf(0x041A6403, 1, 8, data);
        }

        else if (rxId == 0x03216404)
        {
            Serial.println(rxId, HEX);
            byte data[8] = {0xA3, 0x0E, 0xA3, 0x0E, 0x00, 0x00, 0x00, 0x00};
            CAN.sendMsgBuf(0x041A6404, 1, 8, data);
        }

        else if (rxId == 0x03216405)
        {
            Serial.println(rxId, HEX);
            byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            CAN.sendMsgBuf(0x041A6405, 1, 8, data);
        }
    }

    // ==================================================
    // 100 ms
    // ==================================================
    if (now - t100 >= 100)
    {
        t100 = now;

        byte data[5] = {0x00, 0x00, // Current
                        0x36, 0x10, //  Battery voltage
                        0x40};      // Temperature 'F
        CAN.sendMsgBuf(0x04F83401, 1, 5, data);
    }

    // ==================================================
    // 200 ms
    // ==================================================
    if (now - t200 >= 200)
    {
        t200 = now;

        byte data[7] = {0x84, 0x4E, // Full charge capacity mAh
                        0x2F, 0x2B, // Battery level mAh
                        0x37,       // SOC RelChargeState
                        0x38,       // AbsChargeState
                        0x64};      // SOH
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