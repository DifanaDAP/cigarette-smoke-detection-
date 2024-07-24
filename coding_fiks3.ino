#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <FastLED.h>

// Wi-Fi credentials
const char* ssid = "4G-UFI-2277";
const char* password = "1234567890";
//const char* ssid = "ZTE-d45b3e";
//const char* password = "601888d4";

// Telegram BOT
String BOTtoken = "6709506289:AAEjnHuQYqhXO959FwirekUJFZegRO6h9rQ"; // Replace with your bot token
String CHAT_ID = "1402746701"; // Replace with your chat ID

bool sendPhoto = false;
WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

// Checks for new messages every 1 second.
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

const float RL = 1; // Load resistor value in ohms
const float m = -0.38608; // Slope of the curve
const float b = 1.42010; // Y-intercept of the curve
const float Ro = 0.35; // Clean air resistance in ohms

float vrl; // Variable to store the voltage across the load resistor
float rs; // Variable to store the sensor resistance
float ratio; // Variable to store the ratio of sensor resistance to clean air resistance
float ppm; // Variable to store the ppm value

// MQ-2 smoke sensor
#define MQ2_PIN 32
#define LDR_PIN 33
#define LED_PIN 14

// Number of LEDs in the WS2812B strip
#define NUM_LEDS 8
// LED brightness (0-255)
#define LED_BRIGHTNESS 30

CRGB leds[NUM_LEDS];

// Camera configuration
#define CAMERA_MODEL_WROVER_KIT
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  21
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    19
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    5
#define Y2_GPIO_NUM    4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

bool wifiConnected = false;
bool mq2Active = true;
bool mq2WasActive = true;

void configInitCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // init with high specs to pre-allocate larger buffers
    if (psramFound()) {
        config.frame_size = FRAMESIZE_UXGA;
        config.jpeg_quality = 10;  // 0-63 lower number means higher quality
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;  // 0-63 lower number means higher quality
        config.fb_count = 1;
    }

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        delay(1000);
        ESP.restart();
    }

    // Drop down frame size for higher initial frame rate
    sensor_t* s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
}

void handleNewMessages(int numNewMessages) {
    Serial.print("Handle New Messages: ");
    Serial.println(numNewMessages);

    for (int i = 0; i < numNewMessages; i++) {
        String chat_id = String(bot.messages[i].chat_id);
        if (chat_id != CHAT_ID) {
            bot.sendMessage(chat_id, "Unauthorized user", "");
            continue;
        }

        // Print the received message
        String text = bot.messages[i].text;
        Serial.println(text);

        String from_name = bot.messages[i].from_name;
        if (text == "/start") {
            String welcome = "Welcome, " + from_name + "\n";
            welcome += "Use the following commands to interact with the ESP32-CAM \n";
            welcome += "/photo : takes a new photo\n";
            bot.sendMessage(CHAT_ID, welcome, "");
        }
        if (text == "/photo") {
            sendPhoto = true;
            Serial.println("New photo request");
        }
    }
}

String sendPhotoTelegram(int numPhotos) {
    const char* myDomain = "api.telegram.org";
    String getAll = "";
    String getBody = "";

    for (int i = 0; i < numPhotos; i++) {
        camera_fb_t* fb = NULL;
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            delay(1000);
            ESP.restart();
            return "Camera capture failed";
        }

        Serial.println("Connect to " + String(myDomain));

        if (clientTCP.connect(myDomain, 443)) {
            Serial.println("Connection successful");

            String head = "--electroniclinic\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--electroniclinic\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
            String tail = "\r\n--electroniclinic--\r\n";

            uint16_t imageLen = fb->len;
            uint16_t extraLen = head.length() + tail.length();
            uint16_t totalLen = imageLen + extraLen;

            clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
            clientTCP.println("Host: " + String(myDomain));
            clientTCP.println("Content-Length: " + String(totalLen));
            clientTCP.println("Content-Type: multipart/form-data; boundary=electroniclinic");
            clientTCP.println();
            clientTCP.print(head);

            uint8_t* fbBuf = fb->buf;
            size_t fbLen = fb->len;
            for (size_t n = 0; n < fbLen; n = n + 1024) {
                if (n + 1024 < fbLen) {
                    clientTCP.write(fbBuf, 1024);
                    fbBuf += 1024;
                } else if (fbLen % 1024 > 0) {
                    size_t remainder = fbLen % 1024;
                    clientTCP.write(fbBuf, remainder);
                }
            }

            clientTCP.print(tail);

            esp_camera_fb_return(fb);

            int waitTime = 10000;   // timeout 10 seconds
            long startTimer = millis();
            boolean state = false;

            while ((startTimer + waitTime) > millis()) {
                Serial.print(".");
                delay(100);
                while (clientTCP.available()) {
                    char c = clientTCP.read();
                    if (state == true) getBody += String(c);
                    if (c == '\n') {
                        if (getAll.length() == 0) state = true;
                        getAll = "";
                    } else if (c != '\r') {
                        getAll += String(c);
                    }
                    startTimer = millis();
                }
                if (getBody.length() > 0) break;
            }
            clientTCP.stop();
            Serial.println(getBody);
        }

        delay(1000); // Delay between photo captures
    }
    
    return "Photo sent";
}

void sendNotification(String message) {
    if (wifiConnected) {
        bot.sendMessage(CHAT_ID, message, "");
    }
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

    Serial.begin(115200);
    pinMode(MQ2_PIN, INPUT);
    pinMode(LDR_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);

    // Initialize LED strip
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    wifiConnected = true;

    clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT);

    // Configure and initialize the camera
    configInitCamera();
    
    // Send initial connection message
    sendNotification("Device connected and ready.");
}

void loop() {
    // Check WiFi connection status
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiConnected) {
            wifiConnected = false;
            Serial.println("WiFi disconnected. Entering standby mode.");
        }
    } else {
        if (!wifiConnected) {
            wifiConnected = true;
            Serial.println("WiFi reconnected.");
            sendNotification("WiFi reconnected.");
        }
    }

    int mq2Value = analogRead(MQ2_PIN);
    int ldrValue = analogRead(LDR_PIN);

    // Check if MQ2 is active
    if (mq2Value == 0) {
        if (mq2Active) {
            mq2Active = false;
            sendNotification("MQ2 sensor tidak aktif.");
        }
    } else {
        if (!mq2Active) {
            mq2Active = true;
            sendNotification("MQ2 sensor aktif.");
        }
    }

    if (mq2Active) {
        vrl = analogRead(MQ2_PIN) * (5.0 / 4095.0);
        rs = (5.0 * (RL / vrl) - RL); // Calculate sensor resistance
        ratio = rs / Ro; // Calculate the ratio of sensor resistance to clean air resistance
        ppm = pow(10, (log10(ratio) - b) / m); // Calculate the ppm value using the given formula

        Serial.print("MQ2: ");
        Serial.print(ppm);
        Serial.print(" ppm, LDR: ");
        Serial.println(ldrValue);
        
        // New condition: notify when ppm is between 70 and 149
        if (ppm > 70 && ppm <= 149) {
            sendNotification("MQ2 mendeteksi ppm antara 70 and 149: " + String(ppm) + " ppm");
        }
        
        // Check conditions
        if (ppm >= 150) {
            if (ldrValue >= 2500) {
                // Condition 1: MQ2 detected and LDR detected
                Serial.println("MQ2 and LDR detected. Taking 3 photos with LED ON.");
                sendNotification("MQ2: " + String(ppm) + " ppm, LDR: " + String(ldrValue) + ". Mengambil 3 photo dengan LED Menyala.");

                // Activate LED
                for (int i = 0; i < NUM_LEDS; i++) {
                    leds[i] = CRGB::White;
                }
                FastLED.show();

                // Take and send 3 photos
                sendPhotoTelegram(3);

                // Reset LED
                for (int i = 0; i < NUM_LEDS; i++) {
                    leds[i] = CRGB::Black;
                }
                FastLED.show();
            } else {
                // Condition 2: MQ2 detected but LDR not detected
                Serial.println("MQ2 detected but LDR not detected. Taking 3 photos with LED OFF.");
                sendNotification("MQ2: " + String(ppm) + " ppm, LDR: " + String(ldrValue) + ". Mengambil 3 photo dengan LED mati.");

                // Take and send 3 photos
                sendPhotoTelegram(3);
            }
        } else {
            // Condition 3: MQ2 not detected
            Serial.println("MQ2 not detected. Standby mode.");
        }
    } else {
        Serial.println("MQ2 sensor inactive.");
    }
    //Check if there is a photo request
    if (sendPhoto) {
        sendPhotoTelegram(1);
        sendPhoto = false;
    }

    // Check for new Telegram messages
    if (wifiConnected && millis() > lastTimeBotRan + botRequestDelay) {
        int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
        while (numNewMessages) {
            Serial.println("Got response");
            handleNewMessages(numNewMessages);
            numNewMessages = bot.getUpdates(bot.last_message_received + 1);
        }
        lastTimeBotRan = millis();
    }

    // Delay to check sensors every 1 second
    delay(500);
}
