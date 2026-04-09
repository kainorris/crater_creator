#include "Gyro.h"

Gyro gyro;

#define IMPACT_THRESHOLD  0.25
#define COOLDOWN_MS       500

unsigned long lastImpact = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(1000);
    gyro.setup();
    Serial.println("Ready. Waiting for impacts...");
}

void loop() {
    Point3D accel = gyro.loop();
    float mag = accel.mag();

    if (millis() - lastImpact < COOLDOWN_MS) return;

    if (mag > IMPACT_THRESHOLD) {
        lastImpact = millis();
        Serial.print("A@E#F:");
        Serial.println(accel.y);
        Serial.print("X@E#F:");
        Serial.println(accel.x);
        Serial.print("Y@E#F:");
        Serial.println(accel.z); // This is intentional as z~=y

    }
}
