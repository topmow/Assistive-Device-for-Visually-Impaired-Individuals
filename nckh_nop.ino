#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <String.h>

// SIM và GPS pins
#define SIM_RX 12
#define SIM_TX 13
#define SS_RX 4
#define SS_TX 3
#define TRIG_PIN 9
#define ECHO_PIN 10
#define BUZZER_PIN 7

// Khởi tạo modules
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
SoftwareSerial SoftSerial(SS_RX, SS_TX);  // GPS
SoftwareSerial SIM800A(SIM_RX, SIM_TX);   // SIM800A

// Biến chung
const char phoneNum[] = "+84862878118";  // Số điện thoại quốc tế
char message[100];  // Buffer cho tin nhắn
bool pauseMPU = false;  // Cờ tạm dừng MPU6050
unsigned long lastGPSCheck = 0;
const unsigned long gpsCheckInterval = 5000;  // Kiểm tra GPS mỗi 5 giây

// Biến MPU6050
unsigned long lastWarningTime = 0;
unsigned long fallDetectedTime = 0;
unsigned long lastCallTime = 0;
bool fallDetected = false;
bool callActive = false;
const unsigned long callDelay = 30000;  // 3 phút
const unsigned long warningInterval = 60000;  // 1 phút

void setup() {
    Serial.begin(9600);
    SoftSerial.begin(9600);
    SIM800A.begin(9600);

    // Khởi tạo MPU6050
    Serial.println(F("[SETUP] Initializing MPU6050..."));
    if (!mpu.begin()) {
        Serial.println(F("[SETUP] MPU6050 not found! Halting..."));
        while (1) delay(10);
    }
    Serial.println(F("[SETUP] MPU6050 ready!"));
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    // Cấu hình SIM800A
    Serial.println(F("[SETUP] Initializing SIM800A..."));
    sendATCommand("AT", 500);
    sendATCommand("AT+IPR=9600", 500);
    sendATCommand("AT+CMGF=1", 500);  // Chế độ văn bản SMS
    sendATCommand("AT+CNMI=2,2,0,0,0", 500);  // Hiển thị tin nhắn mới
    Serial.println(F("[SETUP] SIM800A ready!"));
}

void loop() {
    // Đọc GPS định kỳ
    readUltrasonicSensor();
    if (millis() - lastGPSCheck >= gpsCheckInterval) {
        readGPS();
        lastGPSCheck = millis();
    }

    // Đọc MPU6050 nếu không bị tạm dừng
    if (!pauseMPU) {
        readMPU6050();
    } else {
        Serial.println(F("[MPU] Paused..."));
    }

    // Kiểm tra SMS
    checkSMSRequestLocation();

    delay(100);
}

void sendATCommand(const char* command, int delayTime) {
    Serial.print(F("[AT] Sending: "));
    Serial.println(command);
    SIM800A.println(command);
    delay(delayTime);
    Serial.println(F("[AT] Response:"));
    while (SIM800A.available()) {
        Serial.write(SIM800A.read());
    }
    Serial.flush();
    delay(50);
}

void sendMessage() {
    Serial.print(F("[SMS] Preparing to send: "));
    Serial.println(message);
    sendATCommand("AT+CMGF=1", 500);
    sendATCommand("AT+CMGS=\"+84862878118\"", 500);
    SIM800A.println(message);
    delay(500);
    SIM800A.println((char)26);  // Kết thúc SMS
    delay(1000);
    Serial.println(F("[SMS] Response:"));
    while (SIM800A.available()) {
        Serial.write(SIM800A.read());
    }
    Serial.print(F("[SMS] Sent to: "));
    Serial.println(phoneNum);
    Serial.flush();
}
void readUltrasonicSensor() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.034 / 2;

    Serial.print("Khoảng cách: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance < 60) {
        int buzzerIntensity = map(distance, 0, 60, 255, 50);
        analogWrite(BUZZER_PIN, buzzerIntensity);
    } else {
        digitalWrite(BUZZER_PIN, LOW);
    }
}

void readGPS() {
    while (SoftSerial.available() > 0) {
        if (gps.encode(SoftSerial.read())) {
            if (gps.location.isValid() && gps.satellites.value() >= 3) {
                Serial.print(F("[GPS] Valid - Lat: "));
                Serial.print(gps.location.lat(), 6);
                Serial.print(F(", Lon: "));
                Serial.print(gps.location.lng(), 6);
                Serial.print(F(", Satellites: "));
                Serial.println(gps.satellites.value());
            } else {
                Serial.print(F("[GPS] Invalid or satellites < 3, Satellites: "));
                Serial.println(gps.satellites.value());
            }
        }
    }
    if (millis() - gps.time.age() > 10000) {
        Serial.println(F("[GPS] No new signal!"));
    }
}

void checkSMSRequestLocation() {
    if (SIM800A.available()) {
        char smsData[100];
        int i = 0;
        while (SIM800A.available() && i < sizeof(smsData) - 1) {
            smsData[i++] = SIM800A.read();
        }
        smsData[i] = '\0';
        Serial.print(F("[SMS] Received: "));
        Serial.println(smsData);

        if (strstr(smsData, "+CMT:") != NULL) {
            char* startQuote = strchr(smsData, '"') + 1;
            char* endQuote = strchr(startQuote, '"');
            if (startQuote && endQuote) {
                *endQuote = '\0';
                Serial.print(F("[SMS] From: "));
                Serial.println(startQuote);

                if (strstr(startQuote, phoneNum) != NULL && strstr(smsData, "GET LOCATION") != NULL) {
                    Serial.println(F("[SMS] Valid GET LOCATION request!"));
                    pauseMPU = true;
                    sendLocationSMS();
                    pauseMPU = false;
                    Serial.println(F("[MPU] Resumed after SMS"));
                }
            }
        }
    }
}

void sendLocationSMS() {
    Serial.println(F("[SMS] Checking GPS for location..."));
    if (gps.location.isValid() && gps.satellites.value() >= 3 && millis() - gps.time.age() < 10000) {
        snprintf(message, sizeof(message), "Location: https://maps.google.com/?q=%.6f,%.6f", gps.location.lat(), gps.location.lng());
        Serial.print(F("[SMS] Valid GPS, sending: "));
        Serial.println(message);
    } else {
        strcpy(message, "Unable to get GPS location. Check satellite signal!");
        Serial.println(F("[SMS] Invalid GPS, sending error"));
    }
    sendMessage();
}

void readMPU6050() {
    Serial.println(F("[MPU] Reading..."));
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float acc_total = sqrt(a.acceleration.x * a.acceleration.x +
                           a.acceleration.y * a.acceleration.y +
                           a.acceleration.z * a.acceleration.z);

    float angleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    float angleY = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;

    Serial.print(F("[MPU] Total Acc: "));
    Serial.println(acc_total);
    Serial.print(F("[MPU] Angle X: "));
    Serial.print(angleX);
    Serial.print(F(", Y: "));
    Serial.println(angleY);

    if (acc_total < 5.0) {  // Phát hiện va chạm
        strcpy(message, "Collision detected!");
        analogWrite(BUZZER_PIN, HIGH);
        delay(5000);
        Serial.println(message);
        sendMessage();
        lastWarningTime = millis();

        if (abs(angleX) > 45 || abs(angleY) > 45) {  // Phát hiện ngã
            strcpy(message, "Fall detected!");
            Serial.println(message);
            sendMessage();
            lastWarningTime = millis();

            if (!fallDetected) {
                fallDetected = true;
                fallDetectedTime = millis();
                callActive = false;
            }
        }
    } else {
        if (fallDetected && acc_total > 1.0 && abs(angleX) < 15 && abs(angleY) < 15) {
            fallDetected = false;
            callActive = false;
            Serial.println(F("[MPU] Fall condition cleared."));
        }
    }

    if (fallDetected) {
        unsigned long currentTime = millis();
        if (!callActive && currentTime - fallDetectedTime >= callDelay) {
            callAcquaintance();
            lastCallTime = currentTime;
            callActive = true;
            Serial.println(F("[CALL] Initiated after fall."));
        }
        if (callActive && currentTime - lastCallTime >= warningInterval) {
            callAcquaintance();
            lastCallTime = currentTime;
            Serial.println(F("[CALL] Repeated call."));
        }
    }
}

void callAcquaintance() {
    Serial.println(F("[CALL] Initiating..."));
    sendATCommand("AT+CLIP=1", 500);
    sendATCommand("ATD+84862878118;", 500);
    Serial.print(F("[CALL] Calling "));
    Serial.println(phoneNum);
}