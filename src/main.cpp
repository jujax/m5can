#include <M5Core2.h>
#include <SPI.h>
#include <mcp_can.h>

// ===== Configuration des pins CAN (Module COMMU sur Core2) =====
// D'après le M5-Bus: Position 21=G27 (CS), Position 23=G2 (INT)
#define CAN_CS_PIN   27   // Chip Select (M5-Bus position 21)
#define CAN_INT_PIN  2    // Interrupt (M5-Bus position 23)
#define CAN_MOSI     23   // SPI MOSI (M5-Bus position 7)
#define CAN_MISO     38   // SPI MISO (M5-Bus position 9)
#define CAN_SCK      18   // SPI SCK (M5-Bus position 11)

// ===== Configuration CAN =====
#define CAN_SPEED    CAN_500KBPS
#define CAN_CLOCK    MCP_8MHZ
#define TX_CAN_ID    0x123

// ===== Variables globales =====
MCP_CAN CAN(CAN_CS_PIN);

volatile bool canMsgReceived = false;
unsigned long txCount = 0;
unsigned long rxCount = 0;
unsigned long lastTxTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastBatteryUpdate = 0;
bool debugSerial = true;
bool displayInitialized = false;
int lastBatteryPercent = -1;
bool lastChargingState = false;
unsigned long lastTxCount = 0;
unsigned long lastRxCount = 0;
int lastRxLogIndex = -1;

// Buffer pour les messages reçus (log des 5 derniers)
struct CanMessage {
    unsigned long id;
    uint8_t len;
    uint8_t data[8];
};
CanMessage rxLog[5];
int rxLogIndex = 0;

// Dernière trame envoyée
CanMessage lastTx;

// ===== Couleurs de l'interface =====
#define COLOR_BG        0x0821      // Gris très foncé
#define COLOR_HEADER    0x1A3A      // Bleu foncé
#define COLOR_TEXT      TFT_WHITE
#define COLOR_OK        0x07E0      // Vert vif
#define COLOR_ERROR     0xF800      // Rouge
#define COLOR_CHARGING  0xFFE0      // Jaune
#define COLOR_TX        0x07FF      // Cyan
#define COLOR_RX        0xFBE0      // Orange
#define COLOR_BORDER    0x4A69      // Gris bordure
#define COLOR_DIM       0x6B6D      // Gris texte secondaire

// ===== Interruption CAN =====
void IRAM_ATTR canISR() {
    canMsgReceived = true;
}

// ===== Fonctions utilitaires =====
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

// ===== Fonctions d'affichage =====

void drawRoundRect(int x, int y, int w, int h, int r, uint16_t borderColor, uint16_t fillColor) {
    M5.Lcd.fillRoundRect(x, y, w, h, r, fillColor);
    M5.Lcd.drawRoundRect(x, y, w, h, r, borderColor);
}

void drawBatteryIcon(int x, int y, int percent, bool charging) {
    // Icône batterie 24x12
    uint16_t color = (percent > 50) ? COLOR_OK : (percent > 20) ? COLOR_CHARGING : COLOR_ERROR;
    
    // Corps de la batterie
    M5.Lcd.drawRect(x, y, 22, 12, COLOR_TEXT);
    M5.Lcd.fillRect(x + 22, y + 3, 3, 6, COLOR_TEXT);
    
    // Niveau de charge
    int barWidth = (percent * 18) / 100;
    M5.Lcd.fillRect(x + 2, y + 2, barWidth, 8, color);
    
    // Éclair si en charge
    if (charging) {
        M5.Lcd.setTextColor(COLOR_CHARGING);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(x + 7, y + 2);
        M5.Lcd.print("~");
    }
}

void drawHeader(bool canOk, bool forceRedraw = false) {
    static bool lastCanOk = false;
    
    // Redessiner le header seulement si nécessaire
    if (forceRedraw || !displayInitialized || canOk != lastCanOk) {
        M5.Lcd.fillRect(0, 0, 320, 36, COLOR_HEADER);
        M5.Lcd.drawFastHLine(0, 36, 320, COLOR_BORDER);
        
        // Statut CAN avec indicateur LED
        int ledColor = canOk ? COLOR_OK : COLOR_ERROR;
        M5.Lcd.fillCircle(18, 18, 8, ledColor);
        M5.Lcd.drawCircle(18, 18, 8, COLOR_TEXT);
        
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(32, 10);
        M5.Lcd.print("CAN");
        
        lastCanOk = canOk;
    }
    
    // Mettre à jour la batterie seulement si elle a changé
    int batPercent = getBatteryPercent();
    bool isCharging = M5.Axp.isCharging();
    
    if (forceRedraw || !displayInitialized || 
        batPercent != lastBatteryPercent || isCharging != lastChargingState) {
        // Effacer l'ancienne zone batterie
        M5.Lcd.fillRect(220, 0, 100, 36, COLOR_HEADER);
        
        drawBatteryIcon(220, 12, batPercent, isCharging);
        
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(250, 10);
        M5.Lcd.printf("%d%%", batPercent);
        
        lastBatteryPercent = batPercent;
        lastChargingState = isCharging;
    }
}

void drawTxSection(bool forceRedraw = false) {
    static unsigned long lastDisplayedTxId = 0;
    static unsigned long lastDisplayedTxCount = 0;
    
    // Redessiner le cadre seulement si nécessaire
    if (forceRedraw || !displayInitialized) {
        drawRoundRect(5, 42, 310, 50, 4, COLOR_TX, COLOR_BG);
        
        // Label TX
        M5.Lcd.fillRoundRect(10, 38, 30, 14, 3, COLOR_TX);
        M5.Lcd.setTextColor(COLOR_BG);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(15, 41);
        M5.Lcd.print("TX");
    }
    
    // Mettre à jour ID et données seulement si changés
    if (forceRedraw || !displayInitialized || lastTx.id != lastDisplayedTxId) {
        // Effacer l'ancien ID
        M5.Lcd.fillRect(15, 52, 80, 20, COLOR_BG);
        
        M5.Lcd.setTextColor(COLOR_TX);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(15, 52);
        M5.Lcd.printf("0x%03lX", lastTx.id);
        
        // Effacer les anciennes données
        M5.Lcd.fillRect(15, 75, 200, 12, COLOR_BG);
        
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(15, 75);
        M5.Lcd.print(formatHex(lastTx.data, lastTx.len));
        
        lastDisplayedTxId = lastTx.id;
    }
    
    // Mettre à jour le compteur seulement s'il a changé
    if (forceRedraw || !displayInitialized || txCount != lastDisplayedTxCount) {
        M5.Lcd.fillRect(260, 52, 50, 20, COLOR_BG);
        M5.Lcd.setTextColor(COLOR_DIM);
        M5.Lcd.setCursor(260, 52);
        M5.Lcd.setTextSize(2);
        M5.Lcd.printf("#%lu", txCount);
        lastDisplayedTxCount = txCount;
    }
}

void drawRxLog(bool forceRedraw = false) {
    static unsigned long lastDisplayedRxCount = 0;
    static int lastRxLogIndex = -1;
    
    // Redessiner le cadre seulement si nécessaire
    if (forceRedraw || !displayInitialized) {
        drawRoundRect(5, 98, 310, 100, 4, COLOR_RX, COLOR_BG);
        
        // Label RX
        M5.Lcd.fillRoundRect(10, 94, 30, 14, 3, COLOR_RX);
        M5.Lcd.setTextColor(COLOR_BG);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(15, 97);
        M5.Lcd.print("RX");
    }
    
    // Mettre à jour le compteur RX seulement s'il a changé
    if (forceRedraw || !displayInitialized || rxCount != lastDisplayedRxCount) {
        M5.Lcd.fillRect(270, 97, 40, 10, COLOR_BG);
        M5.Lcd.setTextColor(COLOR_DIM);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(270, 97);
        M5.Lcd.printf("#%lu", rxCount);
        lastDisplayedRxCount = rxCount;
    }
    
    // Redessiner la liste seulement si un nouveau message est arrivé
    if (forceRedraw || !displayInitialized || rxLogIndex != lastRxLogIndex) {
        // Effacer toute la zone de log
        M5.Lcd.fillRect(10, 108, 300, 88, COLOR_BG);
        
        M5.Lcd.setTextSize(1);
        int yPos = 108;
        int msgDisplayed = 0;
        
        for (int i = 0; i < 5 && msgDisplayed < 4; i++) {
            int idx = (rxLogIndex - 1 - i + 5) % 5;
            if (rxLog[idx].len > 0) {
                // Alternance de couleur de fond
                if (msgDisplayed % 2 == 0) {
                    M5.Lcd.fillRect(10, yPos, 300, 20, 0x1082);
                }
                
                // ID en couleur
                M5.Lcd.setTextColor(COLOR_RX);
                M5.Lcd.setCursor(15, yPos + 6);
                M5.Lcd.printf("0x%03lX", rxLog[idx].id);
                
                // Données
                M5.Lcd.setTextColor(COLOR_TEXT);
                M5.Lcd.setCursor(70, yPos + 6);
                M5.Lcd.print(formatHex(rxLog[idx].data, rxLog[idx].len));
                
                yPos += 22;
                msgDisplayed++;
            }
        }
        
        // Message si vide
        if (msgDisplayed == 0) {
            M5.Lcd.setTextColor(COLOR_DIM);
            M5.Lcd.setTextSize(1);
            M5.Lcd.setCursor(100, 140);
            M5.Lcd.print("En attente de trames...");
        }
        
        lastRxLogIndex = rxLogIndex;
    }
}

void drawButtons(bool forceRedraw = false) {
    static bool lastDebugState = false;
    
    int btnY = 205;
    int btnH = 30;
    int btnW = 100;
    int spacing = 5;
    
    // Redessiner les boutons seulement si nécessaire
    if (forceRedraw || !displayInitialized || debugSerial != lastDebugState) {
        // Bouton SEND
        drawRoundRect(spacing, btnY, btnW, btnH, 5, COLOR_TX, 0x0A4A);
        M5.Lcd.setTextColor(COLOR_TX);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(25, btnY + 8);
        M5.Lcd.print("SEND");
        
        // Bouton CLEAR
        drawRoundRect(btnW + spacing * 2, btnY, btnW, btnH, 5, COLOR_CHARGING, 0x4200);
        M5.Lcd.setTextColor(COLOR_CHARGING);
        M5.Lcd.setCursor(btnW + spacing * 2 + 15, btnY + 8);
        M5.Lcd.print("CLEAR");
        
        // Bouton DEBUG (seulement si l'état a changé)
        uint16_t dbgColor = debugSerial ? COLOR_OK : COLOR_DIM;
        drawRoundRect(btnW * 2 + spacing * 3, btnY, btnW, btnH, 5, dbgColor, debugSerial ? 0x0320 : 0x2104);
        M5.Lcd.setTextColor(dbgColor);
        M5.Lcd.setCursor(btnW * 2 + spacing * 3 + 12, btnY + 8);
        M5.Lcd.print("DEBUG");
        
        lastDebugState = debugSerial;
    }
}

void updateDisplay(bool canOk, bool forceRedraw = false) {
    if (forceRedraw || !displayInitialized) {
        // Premier affichage : tout redessiner
        M5.Lcd.fillRect(0, 37, 320, 163, COLOR_BG);
        drawHeader(canOk, true);
        drawTxSection(true);
        drawRxLog(true);
        drawButtons(true);
        displayInitialized = true;
    } else {
        // Mise à jour partielle : seulement ce qui change
        drawHeader(canOk, false);
        drawTxSection(false);
        drawRxLog(false);
        drawButtons(false);
    }
}

// ===== Envoi CAN =====
void sendCanMessage() {
    // Données d'exemple (incrémentées à chaque envoi)
    static uint8_t counter = 0;
    uint8_t data[8] = {counter++, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, counter};
    
    byte result = CAN.sendMsgBuf(TX_CAN_ID, 0, 8, data);
    
    if (result == CAN_OK) {
        txCount++;
        lastTx.id = TX_CAN_ID;
        lastTx.len = 8;
        memcpy(lastTx.data, data, 8);
        
        if (debugSerial) {
            Serial.printf("[TX] ID: 0x%03X Data: %s\n", TX_CAN_ID, formatHex(data, 8).c_str());
        }
    } else {
        if (debugSerial) {
            Serial.println("[TX] Erreur d'envoi CAN");
        }
    }
}

// ===== Réception CAN =====
void receiveCanMessages() {
    while (CAN.checkReceive() == CAN_MSGAVAIL) {
        unsigned long rxId;
        uint8_t len = 0;
        uint8_t rxBuf[8];
        
        CAN.readMsgBuf(&rxId, &len, rxBuf);
        
        // Stocker dans le log
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
    // Initialisation M5Core2 (LCD, SD, Serial, I2C)
    // Note: On désactive le speaker (I2S) car GPIO2 est partagé avec CAN_INT
    M5.begin(true, false, true, true, kMBusModeOutput);
    
    M5.Lcd.fillScreen(COLOR_BG);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(COLOR_TEXT);
    M5.Lcd.setCursor(20, 20);
    M5.Lcd.println("M5Stack CAN Monitor");
    M5.Lcd.setCursor(20, 50);
    M5.Lcd.println("Initialisation...");
    
    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== M5Stack Core2 + Module COMMU CAN ===");
    
    // Afficher les pins utilisés
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(20, 80);
    M5.Lcd.printf("CS: GPIO%d  INT: GPIO%d", CAN_CS_PIN, CAN_INT_PIN);
    M5.Lcd.setCursor(20, 95);
    M5.Lcd.printf("MOSI:%d MISO:%d SCK:%d", CAN_MOSI, CAN_MISO, CAN_SCK);
    
    Serial.printf("Pins: CS=%d, INT=%d, MOSI=%d, MISO=%d, SCK=%d\n", 
                  CAN_CS_PIN, CAN_INT_PIN, CAN_MOSI, CAN_MISO, CAN_SCK);
    
    // Configuration SPI pour le MCP2515
    M5.Lcd.setCursor(20, 115);
    M5.Lcd.print("Init SPI...");
    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS_PIN);
    M5.Lcd.println(" OK");
    Serial.println("SPI initialise");
    
    // Initialisation MCP2515
    M5.Lcd.setCursor(20, 130);
    M5.Lcd.print("Init MCP2515...");
    
    int canInitAttempts = 0;
    byte canResult;
    
    do {
        canResult = CAN.begin(MCP_ANY, CAN_SPEED, CAN_CLOCK);
        canInitAttempts++;
        
        Serial.printf("CAN.begin() tentative %d, resultat: %d\n", canInitAttempts, canResult);
        
        if (canResult != CAN_OK) {
            M5.Lcd.setTextColor(COLOR_ERROR);
            M5.Lcd.setCursor(20, 145);
            M5.Lcd.printf("Echec #%d (code:%d)", canInitAttempts, canResult);
            delay(500);
        }
    } while (canResult != CAN_OK && canInitAttempts < 5);
    
    if (canResult != CAN_OK) {
        Serial.println("Impossible d'initialiser le CAN!");
        M5.Lcd.fillScreen(COLOR_BG);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(COLOR_ERROR);
        M5.Lcd.setCursor(20, 80);
        M5.Lcd.println("ERREUR CAN!");
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(20, 110);
        M5.Lcd.println("Verifiez:");
        M5.Lcd.setCursor(20, 125);
        M5.Lcd.println("- Module COMMU connecte?");
        M5.Lcd.setCursor(20, 140);
        M5.Lcd.println("- Base retiree du Core2?");
        M5.Lcd.setCursor(20, 155);
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.printf("Code erreur: %d", canResult);
        while(1) { 
            M5.update();
            delay(100); 
        }
    }
    
    M5.Lcd.setTextColor(COLOR_OK);
    M5.Lcd.println(" OK");
    
    // Passage en mode normal
    CAN.setMode(MCP_NORMAL);
    Serial.println("CAN en mode NORMAL");
    
    // Configuration de l'interruption (GPIO2)
    pinMode(CAN_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);
    
    M5.Lcd.setTextColor(COLOR_OK);
    M5.Lcd.setCursor(20, 160);
    M5.Lcd.println("CAN initialise @ 500 kbps!");
    
    Serial.println("CAN initialise a 500 kbps!");
    Serial.printf("Batterie: %d%% %s\n", getBatteryPercent(), 
                  M5.Axp.isCharging() ? "(en charge)" : "");
    
    delay(1500);  // Laisser voir le message
    
    // Initialiser les structures
    memset(&lastTx, 0, sizeof(lastTx));
    lastTx.id = TX_CAN_ID;
    lastTx.len = 8;
    memset(rxLog, 0, sizeof(rxLog));
    
    // Affichage initial
    M5.Lcd.fillScreen(COLOR_BG);
    updateDisplay(true, true);  // Force redraw pour l'init
}

// ===== Loop =====
void loop() {
    M5.update();
    
    unsigned long now = millis();
    
    // Réception des messages CAN
    if (canMsgReceived) {
        receiveCanMessages();
    }
    
    // Envoi automatique toutes les secondes
    if (now - lastTxTime >= 1000) {
        lastTxTime = now;
        sendCanMessage();
    }
    
    // Bouton A : Envoyer manuellement
    if (M5.BtnA.wasPressed()) {
        sendCanMessage();
        if (debugSerial) Serial.println("[BTN] Envoi manuel");
    }
    
    // Bouton B : Effacer le log
    if (M5.BtnB.wasPressed()) {
        memset(rxLog, 0, sizeof(rxLog));
        rxLogIndex = 0;
        rxCount = 0;
        txCount = 0;
        lastRxLogIndex = -1;  // Forcer redraw du log
        lastDisplayedRxCount = 0;
        lastDisplayedTxCount = 0;
        if (debugSerial) Serial.println("[BTN] Log efface");
        drawRxLog(true);  // Redessiner immédiatement
        drawTxSection(true);
    }
    
    // Bouton C : Toggle debug série
    if (M5.BtnC.wasPressed()) {
        debugSerial = !debugSerial;
        Serial.printf("[BTN] Debug serie: %s\n", debugSerial ? "ON" : "OFF");
    }
    
    // Mise à jour de l'affichage toutes les 500ms (moins fréquent pour éviter le clignotement)
    if (now - lastDisplayUpdate >= 500) {
        lastDisplayUpdate = now;
        updateDisplay(true, false);
    }
    
    // Mise à jour batterie toutes les 2 secondes (change moins souvent)
    if (now - lastBatteryUpdate >= 2000) {
        lastBatteryUpdate = now;
        // Forcer la mise à jour de la batterie
        lastBatteryPercent = -1;
    }
    
    delay(10);
}
