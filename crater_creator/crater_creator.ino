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
        Serial.print("IMPACT | mag: ");
        Serial.print(mag);
        Serial.print(" | X: ");
        Serial.print(accel.x);
        Serial.print(" Y: ");
        Serial.print(accel.y);
        Serial.print(" Z: ");
        Serial.println(accel.z);
    }
}
