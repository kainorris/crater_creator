#include "DistanceSensor.h"
#include "Gyro.h"
#define IMPACT_DELAY 100

DistanceSensor ds;
Gyro gyro;
int delay_lock = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  Serial.println("Start setup\n");
  delay(1000);
    // ds.setup();
    gyro.setup();
      Serial.println("End setup");

}

void loop() {
    // Point3D dist = ds.loop(); //QUESTION: How should we deal with null cases in distance
    Point3D accel = gyro.loop();
  // if (accel.x > 0.03 || accel.y > 0.03 || accel.z > 0.03) {
  //   Serial.print("AccelX: ");
  //   Serial.println(accel.x);
  //   Serial.print("AccelY: ");
  //   Serial.println(accel.y);
  //   Serial.print("AccelZ: ");
  //   Serial.println(accel.z);
  // }
  if (accel.z>=1.01 || accel.z<=0.99){ //A 0.01 difference in z acceleration usualy correlates to an impact.
  // Serial.print("A@E#F:");
  // Serial.print(accel.z);

// if (dist.x > 0 || dist.y > 0 || dist.z > 0) {
//     Serial.print("distX: ");
//     Serial.println(dist.x);
//     Serial.print("distY: ");
//     Serial.println(dist.y);
//     Serial.print("distZ: ");
//     Serial.println(dist.z);
//   }
//   if (dist.z>=1.01 || dist.z<=0.99){ //A 0.01 difference in z disteration usualy correlates to an impact.
//   Serial.print("A@E#F:");
//   Serial.print(dist.z);
  }

  // Heartbeat indicator but i made it cute
  // Serial.println("<3");
   Serial.print("AccelX: ");
  Serial.println(accel.x);
  Serial.print("AccelY: ");
  Serial.println(accel.y);
  Serial.print("AccelZ: ");
  Serial.println(accel.z);
  delay_lock++;
  if (accel.mag()>0.25 && delay_lock >= IMPACT_DELAY){
  Serial.print("A@E#F:");
  delay_lock = 0;
    Serial.println(accel.mag());
  Serial.print("AccelX: ");
  Serial.println(accel.x);
  Serial.print("AccelY: ");
  Serial.println(accel.y);
  Serial.print("AccelZ: ");
  Serial.println(accel.z);
  }

}
