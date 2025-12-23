#include <M5Core2.h>
#include <SPI.h>
#include <Wire.h>
#include <FS.h>
#include <SD.h>
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

// ===== CAN Frame Configuration =====
// Single frame: Vehicle Speed (OBD-II Mode 01, PID 0x0D)
const unsigned long CAN_FRAME_ID = 0x7DF;
const uint8_t CAN_FRAME_DATA[8] = {0x02, 0x01, 0x0D, 0x55, 0x55, 0x55, 0x55, 0x55};
const uint8_t CAN_FRAME_LEN = 8;
const char* CAN_FRAME_NAME = "Speed";

// ===== Global Variables =====
MCP_CAN CAN(CAN_CS_PIN);

volatile bool canMsgReceived = false;
unsigned long txCount = 0;
unsigned long rxCount = 0;
unsigned long lastTxTime = 0;
unsigned long lastDisplayUpdate = 0;
bool sendingEnabled = false;  // Paused at startup
bool lastSendingState = true; // To detect state change

// ===== Power Management =====
#define BRIGHTNESS_DEFAULT   100    // Default brightness (0-100)
#define TIMEOUT_OFF_MS       15000  // 15s before screen off
#define CHARGE_CURRENT_FAST  0x0F   // ~780mA charge current (max safe)

uint8_t currentBrightness = BRIGHTNESS_DEFAULT;
bool screenOn = true;
unsigned long lastActivityTime = 0;
bool powerSaveMode = false;
bool displayInitialized = false;  // Declared early for power management functions

// ===== SD Card Logging =====
bool sdInitialized = false;
bool sdLoggingEnabled = false;
File logFile;
unsigned long logFileSize = 0;
#define MAX_LOG_FILE_SIZE  10485760  // 10MB max file size
#define LOG_FILENAME_PREFIX "/can_log_"
#define LOG_FILENAME_EXT   ".csv"

// ===== IMU Logging =====
File imuLogFile;
unsigned long imuLogFileSize = 0;
unsigned long lastImuLogTime = 0;
#define IMU_LOG_INTERVAL_MS  100  // 10 Hz
#define IMU_FILENAME_PREFIX "/imu_log_"

// ===== Session ID (shared between CAN and IMU logs) =====
String currentSessionId = "";

// Generate a unique random session ID (6 hex characters)
String generateSessionId() {
    uint32_t randNum = esp_random();  // Hardware RNG on ESP32
    char idStr[7];
    snprintf(idStr, sizeof(idStr), "%06X", (unsigned int)(randNum & 0xFFFFFF));
    return String(idStr);
}

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

// ===== Power Management Functions =====

// Turn screen on or off
void setScreenOn(bool on) {
    if (on) {
        M5.Axp.SetDCDC3(true);  // LCD power on
        M5.Axp.SetLcdVoltage(2800);  // Normal brightness
        screenOn = true;
        currentBrightness = BRIGHTNESS_DEFAULT;
        Serial.println("[PWR] Screen ON");
    } else {
        M5.Axp.SetDCDC3(false);  // LCD power off
        screenOn = false;
        currentBrightness = 0;
        Serial.println("[PWR] Screen OFF");
    }
}

// Wake up screen and reset activity timer
void wakeScreen() {
    lastActivityTime = millis();
    
    if (!screenOn) {
        setScreenOn(true);
        powerSaveMode = false;
        displayInitialized = false;  // Force redraw after wake
    }
}

// Configure fast charging
void setupFastCharging() {
    // AXP192 charge current settings:
    // 0x00 = 100mA, 0x01 = 190mA, 0x02 = 280mA, 0x03 = 360mA
    // 0x04 = 450mA, 0x05 = 550mA, 0x06 = 630mA, 0x07 = 700mA
    // 0x08 = 780mA, 0x09 = 880mA, 0x0A = 960mA, 0x0B = 1000mA
    // 0x0C = 1080mA, 0x0D = 1160mA, 0x0E = 1240mA, 0x0F = 1320mA (MAX)
    
    // Use 780mA for safe fast charging (higher can heat battery)
    // Access AXP192 register directly via I2C
    // Register 0x33: bit 7 = charge enable, bits 3-0 = current (0x08 = 780mA)
    Wire.beginTransmission(0x34);  // AXP192 I2C address
    Wire.write(0x33);  // Register address
    Wire.write(0xC8);  // 0xC8 = enable (bit 7) + 780mA (0x08)
    Wire.endTransmission();
    
    Serial.println("[PWR] Fast charging enabled (780mA)");
}

// Reduce power consumption
void setupPowerSaving() {
    // Disable unused peripherals to save power
    M5.Axp.SetLDO2(true);   // Keep LCD backlight enabled
    
    // Note: LDO3 (vibration motor) control may not be available in M5Core2 API
    // The vibration motor is typically controlled via GPIO, not AXP LDO
    
    // Set CPU frequency lower when idle (optional - can affect CAN timing)
    // setCpuFrequencyMhz(80);  // Reduce from 240MHz to 80MHz
    
    Serial.println("[PWR] Power saving configured");
}

// Check and update power saving state
void updatePowerSaving() {
    unsigned long now = millis();
    unsigned long idleTime = now - lastActivityTime;
    
    if (screenOn && idleTime > TIMEOUT_OFF_MS && !powerSaveMode) {
        // Screen OFF after inactivity
        setScreenOn(false);
        powerSaveMode = true;
    }
}

// Toggle screen on/off
void toggleScreen() {
    lastActivityTime = millis();
    
    if (screenOn) {
        setScreenOn(false);
        powerSaveMode = true;
    } else {
        setScreenOn(true);
        powerSaveMode = false;
        displayInitialized = false;  // Force redraw
    }
}

// ===== SD Card Functions =====

// Initialize SD card
bool initSD() {
    if (sdInitialized) return true;
    
    if (!SD.begin()) {
        Serial.println("[SD] Failed to initialize SD card");
        return false;
    }
    
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("[SD] No SD card found");
        return false;
    }
    
    sdInitialized = true;
    
    Serial.print("[SD] Card type: ");
    switch (cardType) {
        case CARD_MMC: Serial.println("MMC"); break;
        case CARD_SD: Serial.println("SDSC"); break;
        case CARD_SDHC: Serial.println("SDHC"); break;
        default: Serial.println("Unknown"); break;
    }
    Serial.printf("[SD] Card size: %.2f GB\n", (float)SD.cardSize() / (1024 * 1024 * 1024));
    
    return true;
}

// Create a new CAN log filename using session ID
String createLogFilename() {
    return String(LOG_FILENAME_PREFIX) + currentSessionId + String(LOG_FILENAME_EXT);
}

// Open log file for writing
bool openLogFile() {
    if (!sdInitialized) {
        if (!initSD()) {
            return false;
        }
    }
    
    // Close existing file if open
    if (logFile) {
        logFile.close();
    }
    
    String filename = createLogFilename();
    logFile = SD.open(filename, FILE_WRITE);
    
    if (!logFile) {
        Serial.printf("[SD] Failed to open file: %s\n", filename.c_str());
        return false;
    }
    
    logFileSize = 0;
    
    // Write CSV header
    logFile.println("timestamp_ms,type,id,length,data_hex");
    
    Serial.printf("[SD] Logging to: %s\n", filename.c_str());
    
    return true;
}

// Close log file
void closeLogFile() {
    if (logFile) {
        logFile.close();
        Serial.println("[SD] Log file closed");
    }
}

// Forward declarations for rotation
void closeImuLogFile();
bool openImuLogFile();

// Rotate all log files (CAN + IMU) with new session ID
bool rotateLogFiles() {
    closeLogFile();
    closeImuLogFile();
    
    currentSessionId = generateSessionId();
    Serial.printf("[SD] Rotating logs, new session ID: %s\n", currentSessionId.c_str());
    
    bool canOk = openLogFile();
    bool imuOk = openImuLogFile();
    
    if (!canOk || !imuOk) {
        if (canOk) closeLogFile();
        if (imuOk) closeImuLogFile();
        return false;
    }
    
    return true;
}

// Log a CAN frame to SD card
void logCanFrame(const char* type, unsigned long id, uint8_t len, uint8_t* data) {
    if (!sdLoggingEnabled || !logFile) {
        return;
    }
    
    // Check file size limit - rotate both files together
    if (logFileSize > MAX_LOG_FILE_SIZE || imuLogFileSize > MAX_LOG_FILE_SIZE) {
        if (!rotateLogFiles()) {
            sdLoggingEnabled = false;
            Serial.println("[SD] Failed to rotate log files, logging disabled");
            return;
        }
    }
    
    // Format: timestamp_ms,type,id,length,data_hex
    unsigned long timestamp = millis();
    logFile.printf("%lu,%s,0x%03lX,%d,", timestamp, type, id, len);
    
    // Write data as hex
    for (int i = 0; i < len; i++) {
        if (data[i] < 0x10) logFile.print("0");
        logFile.print(data[i], HEX);
        if (i < len - 1) logFile.print(" ");
    }
    logFile.println();
    
    logFileSize = logFile.size();
    
    // Flush periodically to prevent data loss
    static unsigned long lastFlush = 0;
    if (millis() - lastFlush > 5000) {  // Every 5 seconds
        logFile.flush();
        lastFlush = millis();
    }
}

// ===== IMU Log File Functions =====

// Create a new IMU log filename using session ID
String createImuLogFilename() {
    return String(IMU_FILENAME_PREFIX) + currentSessionId + String(LOG_FILENAME_EXT);
}

// Open IMU log file for writing
bool openImuLogFile() {
    if (!sdInitialized) {
        if (!initSD()) {
            return false;
        }
    }
    
    // Close existing file if open
    if (imuLogFile) {
        imuLogFile.close();
    }
    
    String filename = createImuLogFilename();
    imuLogFile = SD.open(filename, FILE_WRITE);
    
    if (!imuLogFile) {
        Serial.printf("[SD] Failed to open IMU file: %s\n", filename.c_str());
        return false;
    }
    
    imuLogFileSize = 0;
    
    // Write CSV header
    imuLogFile.println("timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");
    
    Serial.printf("[SD] IMU logging to: %s\n", filename.c_str());
    
    return true;
}

// Close IMU log file
void closeImuLogFile() {
    if (imuLogFile) {
        imuLogFile.close();
        Serial.println("[SD] IMU log file closed");
    }
}

// Log IMU data to SD card
void logImuData(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
    if (!sdLoggingEnabled || !imuLogFile) {
        return;
    }
    
    // Check file size limit - rotate both files together
    if (logFileSize > MAX_LOG_FILE_SIZE || imuLogFileSize > MAX_LOG_FILE_SIZE) {
        if (!rotateLogFiles()) {
            sdLoggingEnabled = false;
            Serial.println("[SD] Failed to rotate log files, logging disabled");
            return;
        }
    }
    
    // Format: timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
    unsigned long timestamp = millis();
    imuLogFile.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", 
                      timestamp, accX, accY, accZ, gyroX, gyroY, gyroZ);
    
    imuLogFileSize = imuLogFile.size();
    
    // Flush periodically to prevent data loss (sync with CAN log flush)
    static unsigned long lastImuFlush = 0;
    if (millis() - lastImuFlush > 5000) {  // Every 5 seconds
        imuLogFile.flush();
        lastImuFlush = millis();
    }
}

// Toggle SD logging (CAN + IMU)
void toggleSDLogging() {
    if (!sdLoggingEnabled) {
        // Generate unique session ID for both files
        currentSessionId = generateSessionId();
        Serial.printf("[SD] New session ID: %s\n", currentSessionId.c_str());
        
        // Enable logging - open both CAN and IMU files
        bool canOk = openLogFile();
        bool imuOk = openImuLogFile();
        
        if (canOk && imuOk) {
            sdLoggingEnabled = true;
            lastImuLogTime = millis();  // Reset IMU timing
            Serial.println("[SD] Logging enabled (CAN + IMU)");
        } else {
            // Close any opened file on partial failure
            if (canOk) closeLogFile();
            if (imuOk) closeImuLogFile();
            Serial.println("[SD] Failed to enable logging");
        }
    } else {
        // Disable logging - close both files
        closeLogFile();
        closeImuLogFile();
        sdLoggingEnabled = false;
        Serial.println("[SD] Logging disabled");
    }
}

// ===== Variables to prevent flickering =====
int lastBatPercent = -1;
bool lastCharging = false;
unsigned long lastTxCount = 0;
unsigned long lastRxCount = 0;
bool buttonsDrawn = false;  // Track if buttons have been drawn
bool lastSDLoggingState = false;  // Track SD logging state for button update

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

void drawSDIcon(int x, int y, bool enabled, bool initialized) {
    // SD card icon
    uint16_t color = initialized ? (enabled ? COLOR_OK : COLOR_DIM) : COLOR_ERROR;
    
    // Clear area
    M5.Lcd.fillRect(x, y, 18, 12, COLOR_HEADER);
    
    // SD card shape
    M5.Lcd.drawRect(x + 2, y + 1, 12, 10, color);
    M5.Lcd.fillRect(x + 3, y + 2, 2, 2, color);
    
    // Recording indicator (red dot when recording)
    if (enabled && initialized) {
        M5.Lcd.fillCircle(x + 14, y + 6, 2, COLOR_ERROR);
    }
}

void updateHeader(bool canOk) {
    // CAN LED
    int ledColor = canOk ? COLOR_OK : COLOR_ERROR;
    M5.Lcd.fillCircle(18, 18, 8, ledColor);
    M5.Lcd.drawCircle(18, 18, 8, COLOR_TEXT);
    
    // SD card icon
    static bool lastSDState = false;
    static bool lastSDInit = false;
    if (sdLoggingEnabled != lastSDState || sdInitialized != lastSDInit) {
        lastSDState = sdLoggingEnabled;
        lastSDInit = sdInitialized;
        drawSDIcon(110, 12, sdLoggingEnabled, sdInitialized);
    }
    
    // Fast charge indicator
    static bool lastFastCharge = false;
    bool isCharging = M5.Axp.isCharging();
    if (isCharging != lastFastCharge) {
        lastFastCharge = isCharging;
        M5.Lcd.fillRect(130, 10, 20, 16, COLOR_HEADER);
        if (isCharging) {
            // Lightning bolt for fast charging
            M5.Lcd.setTextColor(COLOR_CHARGING);
            M5.Lcd.setTextSize(2);
            M5.Lcd.setCursor(130, 10);
            M5.Lcd.print("^");
        }
    }
    
    // Battery (only if changed)
    int batPercent = getBatteryPercent();
    
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
    bool needUpdate = (txCount != lastTxCount);
    
    if (needUpdate) {
        lastTxCount = txCount;
        
        // Clear data area
        M5.Lcd.fillRect(15, 50, 295, 40, COLOR_BG);
        
        // State indicator
        M5.Lcd.setTextColor(sendingEnabled ? COLOR_OK : COLOR_DIM);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(45, 41);
        M5.Lcd.print(sendingEnabled ? ">" : "||");
        
        // Frame name
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(15, 52);
        M5.Lcd.print(CAN_FRAME_NAME);
        
        // ID
        M5.Lcd.setTextColor(COLOR_TX);
        M5.Lcd.setCursor(120, 52);
        M5.Lcd.printf("0x%03lX", CAN_FRAME_ID);
        
        // Data
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(15, 75);
        M5.Lcd.print(formatHex((uint8_t*)CAN_FRAME_DATA, CAN_FRAME_LEN));
        
        // TX counter
        M5.Lcd.setTextColor(COLOR_DIM);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(220, 52);
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
    bool sendChanged = (sendingEnabled != lastSendingState);
    bool sdChanged = (sdLoggingEnabled != lastSDLoggingState);
    
    int btnY = 205;
    int btnH = 30;
    int btnW = 100;
    int spacing = 5;
    
    // First time or PLAY/PAUSE state change
    if (!buttonsDrawn || sendChanged) {
        lastSendingState = sendingEnabled;
        
        // PLAY/PAUSE button (A)
        uint16_t playColor = sendingEnabled ? COLOR_OK : COLOR_ERROR;
        drawRoundRect(spacing, btnY, btnW, btnH, 5, playColor, sendingEnabled ? 0x0320 : 0x4000);
        M5.Lcd.setTextColor(playColor);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(spacing + 15, btnY + 8);
        M5.Lcd.print(sendingEnabled ? "PAUSE" : " PLAY");
    }
    
    // SCREEN button (B) - static after first draw
    if (!buttonsDrawn) {
        drawRoundRect(btnW + spacing * 2, btnY, btnW, btnH, 5, COLOR_CHARGING, 0x4200);
        M5.Lcd.setTextColor(COLOR_CHARGING);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(btnW + spacing * 2 + 8, btnY + 8);
        M5.Lcd.print("SCREEN");
    }
    
    // SD REC button (C)
    if (!buttonsDrawn || sdChanged) {
        lastSDLoggingState = sdLoggingEnabled;
        
        uint16_t sdColor = sdLoggingEnabled ? COLOR_OK : COLOR_DIM;
        drawRoundRect(btnW * 2 + spacing * 3, btnY, btnW, btnH, 5, sdColor, sdLoggingEnabled ? 0x0320 : 0x2104);
        M5.Lcd.setTextColor(sdColor);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(btnW * 2 + spacing * 3 + 8, btnY + 8);
        M5.Lcd.print("SD REC");
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
        lastSendingState = !sendingEnabled;
        lastSDLoggingState = !sdLoggingEnabled;
        buttonsDrawn = false;  // Force buttons redraw
    }
    
    // Partial updates only
    updateHeader(canOk);
    updateTxSection();
    updateRxLog();
    updateButtons();
}

// ===== CAN Transmission =====
void sendCanMessage() {
    byte result = CAN.sendMsgBuf(CAN_FRAME_ID, 0, CAN_FRAME_LEN, (uint8_t*)CAN_FRAME_DATA);
    
    if (result == CAN_OK) {
        txCount++;
        lastTx.id = CAN_FRAME_ID;
        lastTx.len = CAN_FRAME_LEN;
        memcpy(lastTx.data, CAN_FRAME_DATA, CAN_FRAME_LEN);
        
        // Log to SD card
        logCanFrame("TX", CAN_FRAME_ID, CAN_FRAME_LEN, (uint8_t*)CAN_FRAME_DATA);
        
        Serial.printf("[TX] %s ID: 0x%03lX Data: %s\n", 
            CAN_FRAME_NAME, CAN_FRAME_ID, formatHex((uint8_t*)CAN_FRAME_DATA, CAN_FRAME_LEN).c_str());
    } else {
        Serial.println("[TX] CAN send error");
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
        
        // Log to SD card
        logCanFrame("RX", rxId, len, rxBuf);
        
        Serial.printf("[RX] ID: 0x%03lX Data: %s\n", rxId, formatHex(rxBuf, len).c_str());
    }
    canMsgReceived = false;
}

// ===== Setup =====
void setup() {
    // Initialize M5Core2 (LCD, SD, Serial, I2C)
    // Note: Speaker (I2S) disabled because GPIO2 is shared with CAN_INT
    M5.begin(true, true, true, true, kMBusModeOutput);  // SD enabled
    
    // Initialize IMU (MPU6886)
    M5.IMU.Init();
    
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
    
    // Configure power management
    setupFastCharging();
    setupPowerSaving();
    setScreenOn(true);
    lastActivityTime = millis();
    
    // Initialize SD card
    M5.Lcd.setCursor(20, 175);
    M5.Lcd.print("Init SD...");
    if (initSD()) {
        M5.Lcd.setTextColor(COLOR_OK);
        M5.Lcd.println(" OK");
        Serial.println("SD card initialized");
    } else {
        M5.Lcd.setTextColor(COLOR_ERROR);
        M5.Lcd.println(" FAIL");
        Serial.println("SD card initialization failed");
    }
    
    delay(1000);  // Let message be visible
    
    // Initialize structures
    memset(&lastTx, 0, sizeof(lastTx));
    lastTx.id = 1234;
    lastTx.len = 8;
    memset(rxLog, 0, sizeof(rxLog));
    
    // Initial display (displayInitialized = false so everything will be drawn)
    displayInitialized = false;
    updateDisplay(true);
    
    // Force SD icon update
    drawSDIcon(110, 12, sdLoggingEnabled, sdInitialized);
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
    
    // IMU logging at 10 Hz (100ms interval)
    if (sdLoggingEnabled && (now - lastImuLogTime >= IMU_LOG_INTERVAL_MS)) {
        float accX, accY, accZ;
        float gyroX, gyroY, gyroZ;
        
        M5.IMU.getAccelData(&accX, &accY, &accZ);
        M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
        
        logImuData(accX, accY, accZ, gyroX, gyroY, gyroZ);
        lastImuLogTime = now;
    }
    
    // Button A: Play/Pause automatic sending
    if (M5.BtnA.wasPressed()) {
        wakeScreen();  // Wake screen on activity
        
        // If screen was off, first press just wakes up
        if (powerSaveMode) {
            powerSaveMode = false;
        } else {
            sendingEnabled = !sendingEnabled;
            Serial.printf("[BTN] Auto send: %s\n", sendingEnabled ? "PLAY" : "PAUSE");
            if (sendingEnabled) {
                lastTxTime = now;  // Reset timer for immediate send
            }
        }
    }
    
    // Button B: Toggle screen on/off
    if (M5.BtnB.wasPressed()) {
        toggleScreen();
    }
    
    // Button C: Toggle SD logging
    if (M5.BtnC.wasPressed()) {
        wakeScreen();  // Wake screen on activity
        
        if (!powerSaveMode) {
            toggleSDLogging();
        }
    }
    
    // Touch handling - only wake screen if off
    if (M5.Touch.ispressed()) {
        if (!screenOn) {
            wakeScreen();
        } else {
            lastActivityTime = now;  // Reset activity timer on any touch
        }
    }
    
    // Update power saving state
    updatePowerSaving();
    
    // Update display every 100ms (only modified zones are redrawn)
    if (screenOn && (now - lastDisplayUpdate >= 100)) {
        lastDisplayUpdate = now;
        updateDisplay(true);
    }
    
    delay(10);
}

