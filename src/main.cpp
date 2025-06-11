#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
using namespace websockets;

#define RX2_PIN 16
#define TX2_PIN 15
#define IMAGE_MAX_SIZE 100000
#define WIFI_SSID "iPhone"
#define WIFI_PASS "12345678"
const char* server_host = "145.24.223.53"; // Your server's IP
const uint16_t server_port = 8050;

uint8_t imageBuffer[IMAGE_MAX_SIZE];
volatile size_t imageLength = 0;
SemaphoreHandle_t imageReadySemaphore;
SemaphoreHandle_t sendCommandSemaphore;

WebsocketsClient wsClient;
volatile bool wsConnected = false;

void uploadImageWS(const uint8_t* data, size_t length) {
    if(wsClient.available()) {
        wsClient.sendBinary((const char*)data, length);
        Serial.printf("[UploadWS] Sent %d bytes via WebSocket!\n", length);
    } else {
        Serial.println("[UploadWS] WebSocket not connected!");
    }
}

// --- WebSocket events --- 
void onWebSocketEvent(WebsocketsEvent event, String data) {
    Serial.print("[WS EVENT] ");
    switch(event) {
        case WebsocketsEvent::ConnectionOpened:
            Serial.println("Connection Opened!");
            wsConnected = true;
            break;
        case WebsocketsEvent::ConnectionClosed:
            Serial.println("Connection Closed!");
            wsConnected = false;
            break;
        case WebsocketsEvent::GotPing:
            Serial.println("Got a Ping!");
            break;
        case WebsocketsEvent::GotPong:
            Serial.println("Got a Pong!");
            break;
        default:
            Serial.print("Unknown event: "); Serial.println((int)event);
            break;
    }
    Serial.print("Event code: "); Serial.println((int)event);
    Serial.print("Current wsConnected?: "); Serial.println(wsConnected ? "true" : "false");
    if (data.length() > 0) {
        Serial.print("Event Data: "); Serial.println(data);
    }
}
// ---------- Tasks ----------
void SendCommandTask(void *pvParameters) {
    while (true) {
        if (xSemaphoreTake(sendCommandSemaphore, portMAX_DELAY) == pdTRUE) {
            Serial1.write('C');  // send capture command
            Serial.println("[Sender] Sent capture command");
        }
    }
}
void ReceiveImageTask(void *pvParameters) {
    enum State { WAIT_HEADER_1, WAIT_HEADER_2, READ_SIZE, READ_DATA };
    State state = WAIT_HEADER_1;
    uint8_t sizeBuf[2];
    size_t sizeRead = 0;
    size_t bytesRead = 0;
    while (true) {
        if (Serial1.available()) {
            uint8_t byte = Serial1.read();
            switch (state) {
                case WAIT_HEADER_1:
                    if (byte == 0xA5) state = WAIT_HEADER_2;
                    break;
                case WAIT_HEADER_2:
                    if (byte == 0x5A) {
                        state = READ_SIZE;
                        sizeRead = 0;
                    } else {
                        state = WAIT_HEADER_1;
                    }
                    break;
                case READ_SIZE:
                    sizeBuf[sizeRead++] = byte;
                    if (sizeRead == 2) {
                        imageLength = sizeBuf[0] | (sizeBuf[1] << 8);
                        if (imageLength > IMAGE_MAX_SIZE) {
                            Serial.println("[Receiver] Image too large");
                            state = WAIT_HEADER_1;
                        } else {
                            bytesRead = 0;
                            state = READ_DATA;
                        }
                    }
                    break;
                case READ_DATA:
                    imageBuffer[bytesRead++] = byte;
                    if (bytesRead == imageLength) {
                        Serial.println("[Receiver] Complete image received. Semaphore given!");
                        xSemaphoreGive(imageReadySemaphore);
                        state = WAIT_HEADER_1;
                    }
                    break;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}
void HandleImageTask(void *pvParameters) {
    while (true) {
        Serial.println("[Handler] Waiting for imageReadySemaphore");
        if (xSemaphoreTake(imageReadySemaphore, portMAX_DELAY) == pdTRUE) {
            Serial.printf("[Handler] Image received: %d bytes\n", imageLength);
            uploadImageWS(imageBuffer, imageLength);
            xSemaphoreGive(sendCommandSemaphore);  // trigger next capture
        }
    }
}

// ---------- Setup ----------
void setup() {
    Serial.begin(115200);
    Serial1.begin(230400, SERIAL_8N1, RX2_PIN, TX2_PIN);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi!");

    // Setup semaphores
    imageReadySemaphore = xSemaphoreCreateBinary();
    sendCommandSemaphore = xSemaphoreCreateBinary();

    // Build ws://145.24.223.53:8050/ws
    String url = String("ws://") + server_host + ":" + String(server_port) + "/ws";
    wsClient.onEvent(onWebSocketEvent);
    wsClient.connect(url);

    // Wait until wsConnected is true (with a timeout/failsafe)
    Serial.print("Connecting to WebSocket");
    unsigned long ts = millis();
    while (!wsConnected && (millis() - ts < 10000)) { // 10s timeout
        wsClient.poll();
        delay(10);
        Serial.print(".");
    }
    Serial.println();

    if (!wsConnected) {
        Serial.println("ERROR: Could not connect to WebSocket server. Restart ESP or check network.");
        while(1) delay(1000); // don't proceed
    }

    // Only now, launch your tasks
    xTaskCreate(SendCommandTask, "SendCommand", 2048, NULL, 1, NULL);
    xTaskCreate(ReceiveImageTask, "ReceiveImage", 4096, NULL, 2, NULL);
    xTaskCreate(HandleImageTask, "HandleImage", 4096, NULL, 1, NULL);

    xSemaphoreGive(sendCommandSemaphore);  // start the first capture
    Serial.println("Image receiver started. Waiting for data...");
}

void loop() {
    wsClient.poll();  // Keep the WebSocket alive!
}