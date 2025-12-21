#include <M5Core2.h>
#include <SPI.h>
#include <mcp_can.h>

// ===== CAN Pin Configuration (COMMU Module on Core2) =====
// According to M5-Bus: Position 21=G27 (CS), Position 23=G2 (INT)
#define CAN_CS_PIN   27   // Chip Select (M5-Bus position 21)
#define CAN_INT_PIN  2    // Interrupt (M5-Bus position 23)
#define CAN_MOSI     23   // SPI MOSI (M5-Bus position 7)
#define CAN_MISO     38   // SPI MISO (M5-Bus position 9)
#define CAN_SCK      18   // SPI SCK (M5-Bus position 11)

// ===== CAN Configuration =====
#define CAN_SPEED    CAN_500KBPS
#define CAN_CLOCK    MCP_8MHZ

// ===== OBD-II / Automotive Frames =====
struct CarFrame {
    unsigned long id;
    uint8_t len;
    uint8_t data[8];
    const char* name;
};

// Standard OBD-II frames (ID 0x7DF = broadcast, responses on 0x7E8-0x7EF)
const CarFrame carFrames[] = {
    // OBD-II Mode 01 requests (real-time data)
    {0x7DF, 8, {0x02, 0x01, 0x0C, 0x55, 0x55, 0x55, 0x55, 0x55}, "RPM"},           // Engine RPM
    {0x7DF, 8, {0x02, 0x01, 0x0D, 0x55, 0x55, 0x55, 0x55, 0x55}, "Speed"},         // Vehicle speed
    {0x7DF, 8, {0x02, 0x01, 0x05, 0x55, 0x55, 0x55, 0x55, 0x55}, "Coolant"},       // Coolant temperature
    {0x7DF, 8, {0x02, 0x01, 0x0F, 0x55, 0x55, 0x55, 0x55, 0x55}, "Intake"},        // Intake air temperature
    {0x7DF, 8, {0x02, 0x01, 0x11, 0x55, 0x55, 0x55, 0x55, 0x55}, "Throttle"},      // Throttle position
    {0x7DF, 8, {0x02, 0x01, 0x2F, 0x55, 0x55, 0x55, 0x55, 0x55}, "Fuel"},          // Fuel level
    {0x7DF, 8, {0x02, 0x01, 0x04, 0x55, 0x55, 0x55, 0x55, 0x55}, "Load"},          // Engine load
    {0x7DF, 8, {0x02, 0x01, 0x42, 0x55, 0x55, 0x55, 0x55, 0x55}, "Voltage"},       // Battery voltage
    
    // Mode 09 request (vehicle info)
    {0x7DF, 8, {0x02, 0x09, 0x02, 0x55, 0x55, 0x55, 0x55, 0x55}, "VIN"},           // VIN number
    
    // Common CAN frames (non-OBD)
    {0x316, 8, {0x05, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, "RPM BMW"},       // BMW E46 RPM
    {0x153, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, "Speed VAG"},     // VAG speed
    {0x280, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, "RPM PSA"},       // PSA RPM
};
const int numCarFrames = sizeof(carFrames) / sizeof(carFrames[0]);
int currentFrameIndex = 0;

// ===== Global Variables =====
MCP_CAN CAN(CAN_CS_PIN);

volatile bool canMsgReceived = false;
unsigned long txCount = 0;
unsigned long rxCount = 0;
unsigned long lastTxTime = 0;
unsigned long lastDisplayUpdate = 0;
bool debugSerial = true;
bool sendingEnabled = false;  // Paused at startup
bool lastSendingState = true; // To detect state change

// Buffer for received messages (log of last 5)
struct CanMessage {
    unsigned long id;
    uint8_t len;
    uint8_t data[8];
};
CanMessage rxLog[5];
int rxLogIndex = 0;

// Last sent frame
CanMessage lastTx;

// ===== Interface Colors =====
#define COLOR_BG        0x0821      // Very dark grey
#define COLOR_HEADER    0x1A3A      // Dark blue
#define COLOR_TEXT      TFT_WHITE
#define COLOR_OK        0x07E0      // Bright green
#define COLOR_ERROR     0xF800      // Red
#define COLOR_CHARGING  0xFFE0      // Yellow
#define COLOR_TX        0x07FF      // Cyan
#define COLOR_RX        0xFBE0      // Orange
#define COLOR_BORDER    0x4A69      // Border grey
#define COLOR_DIM       0x6B6D      // Secondary text grey

// ===== CAN Interrupt =====
void IRAM_ATTR canISR() {
    canMsgReceived = true;
}

// ===== Utility Functions =====
String formatHex(uint8_t* data, uint8_t len) {
    String result = "";
    for (int i = 0; i < len; i++) {
        if (data[i] < 0x10) result += "0";
        result += String(data[i], HEX);
        if (i < len - 1) result += " ";
    }
    result.toUpperCase();
    return result;
}

int getBatteryPercent() {
    float vbat = M5.Axp.GetBatVoltage();
    int percent = (int)((vbat - 3.2) / (4.2 - 3.2) * 100.0);
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    return percent;
}

// ===== Variables to prevent flickering =====
bool displayInitialized = false;
int lastBatPercent = -1;
bool lastCharging = false;
unsigned long lastTxCount = 0;
unsigned long lastRxCount = 0;
unsigned long lastRxLogIndex = 0;
bool lastDebugState = true;

// ===== Display Functions =====

void drawRoundRect(int x, int y, int w, int h, int r, uint16_t borderColor, uint16_t fillColor) {
    M5.Lcd.fillRoundRect(x, y, w, h, r, fillColor);
    M5.Lcd.drawRoundRect(x, y, w, h, r, borderColor);
}

void drawBatteryIcon(int x, int y, int percent, bool charging) {
    uint16_t color = (percent > 50) ? COLOR_OK : (percent > 20) ? COLOR_CHARGING : COLOR_ERROR;
    
    // Clear battery area
    M5.Lcd.fillRect(x, y, 28, 12, COLOR_HEADER);
    
    // Battery body
    M5.Lcd.drawRect(x, y, 22, 12, COLOR_TEXT);
    M5.Lcd.fillRect(x + 22, y + 3, 3, 6, COLOR_TEXT);
    
    // Charge level
    int barWidth = (percent * 18) / 100;
    if (barWidth > 0) {
        M5.Lcd.fillRect(x + 2, y + 2, barWidth, 8, color);
    }
    
    // Lightning if charging
    if (charging) {
        M5.Lcd.setTextColor(COLOR_BG);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(x + 6, y + 2);
        M5.Lcd.print("+");
    }
}

void drawStaticElements() {
    // Header background
    M5.Lcd.fillRect(0, 0, 320, 36, COLOR_HEADER);
    M5.Lcd.drawFastHLine(0, 36, 320, COLOR_BORDER);
    
    // CAN label
    M5.Lcd.setTextColor(COLOR_TEXT);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(32, 10);
    M5.Lcd.print("CAN");
    
    // TX frame
    drawRoundRect(5, 42, 310, 50, 4, COLOR_TX, COLOR_BG);
    M5.Lcd.fillRoundRect(10, 38, 30, 14, 3, COLOR_TX);
    M5.Lcd.setTextColor(COLOR_BG);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(15, 41);
    M5.Lcd.print("TX");
    
    // RX frame
    drawRoundRect(5, 98, 310, 100, 4, COLOR_RX, COLOR_BG);
    M5.Lcd.fillRoundRect(10, 94, 30, 14, 3, COLOR_RX);
    M5.Lcd.setTextColor(COLOR_BG);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(15, 97);
    M5.Lcd.print("RX");
}

void updateHeader(bool canOk) {
    // CAN LED
    int ledColor = canOk ? COLOR_OK : COLOR_ERROR;
    M5.Lcd.fillCircle(18, 18, 8, ledColor);
    M5.Lcd.drawCircle(18, 18, 8, COLOR_TEXT);
    
    // Battery (only if changed)
    int batPercent = getBatteryPercent();
    bool isCharging = M5.Axp.isCharging();
    
    if (batPercent != lastBatPercent || isCharging != lastCharging) {
        lastBatPercent = batPercent;
        lastCharging = isCharging;
        
        drawBatteryIcon(218, 12, batPercent, isCharging);
        
        // Percentage
        M5.Lcd.fillRect(248, 8, 60, 20, COLOR_HEADER);
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(250, 10);
        M5.Lcd.printf("%d%%", batPercent);
    }
}

void updateTxSection() {
    static int lastFrameIndex = -1;
    bool needUpdate = (txCount != lastTxCount) || (currentFrameIndex != lastFrameIndex);
    
    if (needUpdate) {
        lastTxCount = txCount;
        lastFrameIndex = currentFrameIndex;
        
        const CarFrame& frame = carFrames[currentFrameIndex];
        
        // Clear data area
        M5.Lcd.fillRect(15, 50, 295, 40, COLOR_BG);
        
        // Frame name + state
        M5.Lcd.setTextColor(sendingEnabled ? COLOR_OK : COLOR_DIM);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(45, 41);
        M5.Lcd.printf("%s %d/%d", sendingEnabled ? ">" : "||", currentFrameIndex + 1, numCarFrames);
        
        // Frame name
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(15, 52);
        M5.Lcd.print(frame.name);
        
        // ID
        M5.Lcd.setTextColor(COLOR_TX);
        M5.Lcd.setCursor(150, 52);
        M5.Lcd.printf("0x%03lX", frame.id);
        
        // Data
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(15, 75);
        M5.Lcd.print(formatHex((uint8_t*)frame.data, frame.len));
        
        // TX counter
        M5.Lcd.setTextColor(COLOR_DIM);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(260, 52);
        M5.Lcd.printf("#%lu", txCount);
    }
}

void updateRxLog() {
    // Only if changed
    if (rxCount == lastRxCount) return;
    lastRxCount = rxCount;
    
    // Clear log area
    M5.Lcd.fillRect(10, 105, 300, 88, COLOR_BG);
    
    // RX counter
    M5.Lcd.setTextColor(COLOR_DIM);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(270, 97);
    M5.Lcd.fillRect(265, 94, 45, 12, COLOR_RX);
    M5.Lcd.setTextColor(COLOR_BG);
    M5.Lcd.printf("#%lu", rxCount);
    
    // Message list
    int yPos = 108;
    int msgDisplayed = 0;
    
    for (int i = 0; i < 5 && msgDisplayed < 4; i++) {
        int idx = (rxLogIndex - 1 - i + 5) % 5;
        if (rxLog[idx].len > 0) {
            if (msgDisplayed % 2 == 0) {
                M5.Lcd.fillRect(10, yPos, 300, 20, 0x1082);
            }
            
            M5.Lcd.setTextColor(COLOR_RX);
            M5.Lcd.setCursor(15, yPos + 6);
            M5.Lcd.printf("0x%03lX", rxLog[idx].id);
            
            M5.Lcd.setTextColor(COLOR_TEXT);
            M5.Lcd.setCursor(70, yPos + 6);
            M5.Lcd.print(formatHex(rxLog[idx].data, rxLog[idx].len));
            
            yPos += 22;
            msgDisplayed++;
        }
    }
    
    if (msgDisplayed == 0) {
        M5.Lcd.setTextColor(COLOR_DIM);
        M5.Lcd.setCursor(100, 140);
        M5.Lcd.print("Waiting for frames...");
    }
}

void updateButtons() {
    static bool buttonsDrawn = false;
    bool sendChanged = (sendingEnabled != lastSendingState);
    bool debugChanged = (debugSerial != lastDebugState);
    
    int btnY = 205;
    int btnH = 30;
    int btnW = 100;
    int spacing = 5;
    
    // First time or PLAY/PAUSE state change
    if (!buttonsDrawn || sendChanged) {
        lastSendingState = sendingEnabled;
        
        // PLAY/PAUSE button
        uint16_t playColor = sendingEnabled ? COLOR_OK : COLOR_ERROR;
        drawRoundRect(spacing, btnY, btnW, btnH, 5, playColor, sendingEnabled ? 0x0320 : 0x4000);
        M5.Lcd.setTextColor(playColor);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(spacing + 15, btnY + 8);
        M5.Lcd.print(sendingEnabled ? "PAUSE" : " PLAY");
    }
    
    // NEXT button (static after first draw)
    if (!buttonsDrawn) {
        drawRoundRect(btnW + spacing * 2, btnY, btnW, btnH, 5, COLOR_CHARGING, 0x4200);
        M5.Lcd.setTextColor(COLOR_CHARGING);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(btnW + spacing * 2 + 20, btnY + 8);
        M5.Lcd.print("NEXT");
    }
    
    // DEBUG button
    if (!buttonsDrawn || debugChanged) {
        lastDebugState = debugSerial;
        
        uint16_t dbgColor = debugSerial ? COLOR_OK : COLOR_DIM;
        drawRoundRect(btnW * 2 + spacing * 3, btnY, btnW, btnH, 5, dbgColor, debugSerial ? 0x0320 : 0x2104);
        M5.Lcd.setTextColor(dbgColor);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(btnW * 2 + spacing * 3 + 12, btnY + 8);
        M5.Lcd.print("DEBUG");
    }
    
    buttonsDrawn = true;
}

void updateDisplay(bool canOk) {
    // First time: draw everything
    if (!displayInitialized) {
        M5.Lcd.fillScreen(COLOR_BG);
        drawStaticElements();
        displayInitialized = true;
        // Force update of everything
        lastBatPercent = -1;
        lastTxCount = 0;
        lastRxCount = 0;
        lastDebugState = !debugSerial;
        lastSendingState = !sendingEnabled;
    }
    
    // Partial updates only
    updateHeader(canOk);
    updateTxSection();
    updateRxLog();
    updateButtons();
}

// ===== CAN Transmission =====
void sendCanMessage() {
    const CarFrame& frame = carFrames[currentFrameIndex];
    
    byte result = CAN.sendMsgBuf(frame.id, 0, frame.len, (uint8_t*)frame.data);
    
    if (result == CAN_OK) {
        txCount++;
        lastTx.id = frame.id;
        lastTx.len = frame.len;
        memcpy(lastTx.data, frame.data, frame.len);
        
        if (debugSerial) {
            Serial.printf("[TX] %s ID: 0x%03lX Data: %s\n", 
                frame.name, frame.id, formatHex((uint8_t*)frame.data, frame.len).c_str());
        }
    } else {
        if (debugSerial) {
            Serial.println("[TX] CAN send error");
        }
    }
}

void nextFrame() {
    currentFrameIndex = (currentFrameIndex + 1) % numCarFrames;
    if (debugSerial) {
        Serial.printf("[FRAME] %d/%d: %s\n", currentFrameIndex + 1, numCarFrames, carFrames[currentFrameIndex].name);
    }
}

void prevFrame() {
    currentFrameIndex = (currentFrameIndex - 1 + numCarFrames) % numCarFrames;
    if (debugSerial) {
        Serial.printf("[FRAME] %d/%d: %s\n", currentFrameIndex + 1, numCarFrames, carFrames[currentFrameIndex].name);
    }
}

// ===== CAN Reception =====
void receiveCanMessages() {
    while (CAN.checkReceive() == CAN_MSGAVAIL) {
        unsigned long rxId;
        uint8_t len = 0;
        uint8_t rxBuf[8];
        
        CAN.readMsgBuf(&rxId, &len, rxBuf);
        
        // Store in log
        rxLog[rxLogIndex].id = rxId;
        rxLog[rxLogIndex].len = len;
        memcpy(rxLog[rxLogIndex].data, rxBuf, len);
        rxLogIndex = (rxLogIndex + 1) % 5;
        
        rxCount++;
        
        if (debugSerial) {
            Serial.printf("[RX] ID: 0x%03lX Data: %s\n", rxId, formatHex(rxBuf, len).c_str());
        }
    }
    canMsgReceived = false;
}

// ===== Setup =====
void setup() {
    // Initialize M5Core2 (LCD, SD, Serial, I2C)
    // Note: Speaker (I2S) disabled because GPIO2 is shared with CAN_INT
    M5.begin(true, false, true, true, kMBusModeOutput);
    
    M5.Lcd.fillScreen(COLOR_BG);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(COLOR_TEXT);
    M5.Lcd.setCursor(20, 20);
    M5.Lcd.println("M5Stack CAN Monitor");
    M5.Lcd.setCursor(20, 50);
    M5.Lcd.println("Initializing...");
    
    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== M5Stack Core2 + COMMU Module CAN ===");
    
    // Display used pins
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(20, 80);
    M5.Lcd.printf("CS: GPIO%d  INT: GPIO%d", CAN_CS_PIN, CAN_INT_PIN);
    M5.Lcd.setCursor(20, 95);
    M5.Lcd.printf("MOSI:%d MISO:%d SCK:%d", CAN_MOSI, CAN_MISO, CAN_SCK);
    
    Serial.printf("Pins: CS=%d, INT=%d, MOSI=%d, MISO=%d, SCK=%d\n", 
                  CAN_CS_PIN, CAN_INT_PIN, CAN_MOSI, CAN_MISO, CAN_SCK);
    
    // Configure SPI for MCP2515
    M5.Lcd.setCursor(20, 115);
    M5.Lcd.print("Init SPI...");
    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS_PIN);
    M5.Lcd.println(" OK");
    Serial.println("SPI initialized");
    
    // Initialize MCP2515
    M5.Lcd.setCursor(20, 130);
    M5.Lcd.print("Init MCP2515...");
    
    int canInitAttempts = 0;
    byte canResult;
    
    do {
        canResult = CAN.begin(MCP_ANY, CAN_SPEED, CAN_CLOCK);
        canInitAttempts++;
        
        Serial.printf("CAN.begin() attempt %d, result: %d\n", canInitAttempts, canResult);
        
        if (canResult != CAN_OK) {
            M5.Lcd.setTextColor(COLOR_ERROR);
            M5.Lcd.setCursor(20, 145);
            M5.Lcd.printf("Failed #%d (code:%d)", canInitAttempts, canResult);
            delay(500);
        }
    } while (canResult != CAN_OK && canInitAttempts < 5);
    
    if (canResult != CAN_OK) {
        Serial.println("Failed to initialize CAN!");
        M5.Lcd.fillScreen(COLOR_BG);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(COLOR_ERROR);
        M5.Lcd.setCursor(20, 80);
        M5.Lcd.println("CAN ERROR!");
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(20, 110);
        M5.Lcd.println("Check:");
        M5.Lcd.setCursor(20, 125);
        M5.Lcd.println("- COMMU module connected?");
        M5.Lcd.setCursor(20, 140);
        M5.Lcd.println("- Base removed from Core2?");
        M5.Lcd.setCursor(20, 155);
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.printf("Error code: %d", canResult);
        while(1) { 
            M5.update();
            delay(100); 
        }
    }
    
    M5.Lcd.setTextColor(COLOR_OK);
    M5.Lcd.println(" OK");
    
    // Set to normal mode
    CAN.setMode(MCP_NORMAL);
    Serial.println("CAN in NORMAL mode");
    
    // Configure interrupt (GPIO2)
    pinMode(CAN_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);
    
    M5.Lcd.setTextColor(COLOR_OK);
    M5.Lcd.setCursor(20, 160);
    M5.Lcd.println("CAN initialized @ 500 kbps!");
    
    Serial.println("CAN initialized at 500 kbps!");
    Serial.printf("Battery: %d%% %s\n", getBatteryPercent(), 
                  M5.Axp.isCharging() ? "(charging)" : "");
    
    delay(1000);  // Let message be visible
    
    // Initialize structures
    memset(&lastTx, 0, sizeof(lastTx));
    lastTx.id = 1234;
    lastTx.len = 8;
    memset(rxLog, 0, sizeof(rxLog));
    
    // Initial display (displayInitialized = false so everything will be drawn)
    displayInitialized = false;
    updateDisplay(true);
}

// ===== Loop =====
void loop() {
    M5.update();
    
    unsigned long now = millis();
    
    // CAN message reception
    if (canMsgReceived) {
        receiveCanMessages();
    }
    
    // Automatic send every second (only if enabled)
    if (sendingEnabled && (now - lastTxTime >= 1000)) {
        lastTxTime = now;
        sendCanMessage();
    }
    
    // Button A: Play/Pause automatic sending
    if (M5.BtnA.wasPressed()) {
        sendingEnabled = !sendingEnabled;
        if (debugSerial) {
            Serial.printf("[BTN] Auto send: %s\n", sendingEnabled ? "PLAY" : "PAUSE");
        }
        if (sendingEnabled) {
            lastTxTime = now;  // Reset timer for immediate send
        }
    }
    
    // Button B: Next frame
    if (M5.BtnB.wasPressed()) {
        nextFrame();
        // Send new frame immediately if in play mode
        if (sendingEnabled) {
            sendCanMessage();
            lastTxTime = now;
        }
    }
    
    // Button C: Toggle serial debug
    if (M5.BtnC.wasPressed()) {
        debugSerial = !debugSerial;
        Serial.printf("[BTN] Serial debug: %s\n", debugSerial ? "ON" : "OFF");
    }
    
    // Long press on screen: previous frame (left zone) or reset counters (right zone)
    if (M5.Touch.ispressed()) {
        auto t = M5.Touch.getPressPoint();
        static unsigned long touchStart = 0;
        static bool longPressHandled = false;
        
        if (touchStart == 0) {
            touchStart = now;
            longPressHandled = false;
        } else if (!longPressHandled && (now - touchStart > 800)) {
            // Long press
            if (t.x < 160) {
                // Left zone: previous frame
                prevFrame();
            } else {
                // Right zone: reset counters
                txCount = 0;
                rxCount = 0;
                memset(rxLog, 0, sizeof(rxLog));
                rxLogIndex = 0;
                if (debugSerial) Serial.println("[TOUCH] Counters reset");
            }
            longPressHandled = true;
        }
    } else {
        // Reset when released
        static unsigned long touchStart = 0;
        touchStart = 0;
    }
    
    // Update display every 100ms (only modified zones are redrawn)
    if (now - lastDisplayUpdate >= 100) {
        lastDisplayUpdate = now;
        updateDisplay(true);
    }
    
    delay(10);
}

