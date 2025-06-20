#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
using namespace websockets;
WebsocketsClient wsClient;

#define RX2_PIN 36
#define TX2_PIN 35
#define WIFI_SSID "iPhone"
#define WIFI_PASS "12345678"
const char* server_host = "145.24.223.53";
const uint16_t server_port = 8050;

// Detection parameters
const uint8_t DARK_THRESHOLD_R = 6;     // 6*8 = 48 (0-255 scale)
const uint8_t DARK_THRESHOLD_G = 12;    // 12*4 = 48
const uint8_t DARK_THRESHOLD_B = 6;     // 6*8 = 48
const uint16_t MIN_AREA = 500;          // Minimum detection area
const uint16_t CLOSE_AREA = 1000;       // Area threshold for "close" detection
const uint16_t BOX_COLOR = 0xF800;      // Red in RGB565 (0xF800)

// Image dimensions
const int IMG_WIDTH = 160;
const int IMG_HEIGHT = 120;
const int IMG_SIZE = IMG_WIDTH * IMG_HEIGHT * 2;  // 38400 bytes

bool wsConnected = false;
unsigned long lastReconnectAttempt = 0;
int retryCount = 0;
const int maxRetries = 5;
const unsigned long retryInterval = 500;

void set_pixel(uint8_t *image, uint16_t x, uint16_t y, uint16_t color, int width, int height) {
    if (x >= width || y >= height) return;
    int idx = (y * width + x) * 2;
    image[idx] = (color >> 8) & 0xFF;
    image[idx + 1] = color & 0xFF;
}

void draw_rectangle(uint8_t *image, uint16_t left, uint16_t top, uint16_t right, uint16_t bottom, int width, int height) {
    for (uint16_t x = left; x <= right; x++) {
        set_pixel(image, x, top, BOX_COLOR, width, height);
        set_pixel(image, x, bottom, BOX_COLOR, width, height);
    }
    for (uint16_t y = top; y <= bottom; y++) {
        set_pixel(image, left, y, BOX_COLOR, width, height);
        set_pixel(image, right, y, BOX_COLOR, width, height);
    }
}

void detect_and_draw_battery(uint8_t *image, int width, int height, int img_len) {
    uint16_t left = width, right = 0;
    uint16_t top = height, bottom = 0;
    bool found = false;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = (y * width + x) * 2;
            if (idx + 1 >= img_len) continue; // Prevent buffer overrun

            uint16_t pixel = (image[idx] << 8) | image[idx + 1];
            uint8_t r = (pixel >> 11) & 0x1F;
            uint8_t g = (pixel >> 5) & 0x3F;
            uint8_t b = pixel & 0x1F;

            if (r < DARK_THRESHOLD_R && g < DARK_THRESHOLD_G && b < DARK_THRESHOLD_B) {
                found = true;
                if (x < left) left = x;
                if (x > right) right = x;
                if (y < top) top = y;
                if (y > bottom) bottom = y;
            }
        }
    }

    if (found) {
        uint16_t box_w = right - left + 1;
        uint16_t box_h = bottom - top + 1;
        uint32_t area = box_w * box_h;

        if (area > MIN_AREA) {
            draw_rectangle(image, left, top, right, bottom, width, height);
            Serial.printf("Battery detected: [%d,%d]-[%d,%d] Area: %d ", left, top, right, bottom, area);
            Serial.println(area > CLOSE_AREA ? "CLOSE" : "FAR");
        }
    }
}
void onWebSocketEvent(WebsocketsEvent event, String data) {
    Serial.print("[WS EVENT] ");
    switch (event) {
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

void setup() {
    Serial.begin(115200);
    delay(50);
    Serial1.begin(4000000, SERIAL_8N1, TX2_PIN, RX2_PIN);
    delay(100);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi!");

    wsClient.onEvent(onWebSocketEvent);
    String url = String("ws://") + server_host + ":" + String(server_port) + "/ws";
    wsClient.connect(url);
    
    Serial.println("Image receiver started. Waiting for data...");
}

void loop() {
    wsClient.poll();
    if (!wsConnected && retryCount < maxRetries) {
        unsigned long now = millis();
        if (now - lastReconnectAttempt >= retryInterval) {
            Serial.printf("Reconnect attempt %d/%d\n", retryCount + 1, maxRetries);
            if (wsClient.connect("ws://145.24.223.53:8050")) {
                Serial.println("Reconnected!");
                wsConnected = true;
                retryCount = 0;
            } else {
                Serial.println("Reconnect failed.");
                retryCount++;
            }
            lastReconnectAttempt = now;
        }
    }

    Serial.println("Asking for image...");
    Serial1.write('C');

    // Wait for header
    while (Serial1.available() < 4);

    if (Serial1.read() == 0xA5 && Serial1.read() == 0x5A) {
        uint16_t len;
        Serial1.readBytes((char *)&len, 2);

        Serial.print("Receiving image of ");
        Serial.print(len);
        Serial.println(" bytes");

        uint8_t *image = (uint8_t *)malloc(len);
        if (!image) {
            Serial.println("Memory allocation failed");
            return;
        }

        Serial1.readBytes((char *)image, len);

        // Calculate width/height from length (assuming RGB565)
        int pixels = len / 2;
        int width = IMG_WIDTH;
        int height = pixels / width;
        if (width * height * 2 != len) {
            Serial.println("Warning: Image size does not match expected dimensions. Trying best effort.");
        }

        detect_and_draw_battery(image, width, height, len);

        if (wsConnected) {
            wsClient.sendBinary((const char *)image, len);
            Serial.println("Image sent over WebSocket.");
        } else {
            Serial.println("WebSocket not connected - image not sent.");
        }

        free(image);
    } else {
        Serial.println("Invalid header received");
        while (Serial1.available()) Serial1.read();
    }
}
