#include "hal/gpio_types.h"
#include <assert.h>
#include <driver/gpio.h>

/**
   HC-SR04 Demo
   Demonstration of the HC-SR04 Ultrasonic Sensor
   Date: August 3, 2016

   Description:
    Connect the ultrasonic sensor to the Arduino as per the
    hardware connections below. Run the sketch and open a serial
    monitor. The distance read from the sensor will be displayed
    in centimeters and inches.

   Hardware Connections:
    Arduino | HC-SR04
    -------------------
      5V    |   VCC
      7     |   Trig
      8     |   Echo
      GND   |   GND

   License:
    Public Domain
*/

static QueueHandle_t gpio_evt_queue = NULL;

typedef struct {
  int trig_pin;
  int echo_pin;
} HCSR04;

// Anything over 400 cm (23200 us pulse) is "out of range"
#define HCSR04_MAX_DIST = 23200u;

void setup(HCSR04 *sensor) {
  gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);
  assert(GPIO_IS_VALID_OUTPUT_GPIO(sensor->trig_pin));
  assert(GPIO_IS_VALID_GPIO(sensor->echo_pin));

  // Trigger output conf
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << sensor->trig_pin),
      .pull_down_en = 0,
      .pull_up_en = 0,
  };

  gpio_config(&io_conf);

  // Echo input conf
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.intr_type =
      GPIO_INTR_POSEDGE; // Trigger on rising edge - allows us to instantly
                         // capture the time of the rising edge
  io_conf.pin_bit_mask = (1ULL << sensor->echo_pin);
  gpio_config(&io_conf);

  // create a queue to handle gpio event from isr
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Set Echo pin as input to measure the duration of
  // pulses coming back from the distance sensor
  pinMode(ECHO_PIN, INPUT);

  // We'll use the serial monitor to view the sensor output
  Serial.begin(9600);
}

void loop() {

  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  while (digitalRead(ECHO_PIN) == 0)
    ;

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1)
    ;
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  // of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    Serial.println("Out of range");
  } else {
    Serial.print(cm);
    Serial.print(" cm \t");
    Serial.print(inches);
    Serial.println(" in");
  }

  // Wait at least 60ms before next measurement
  delay(60);
}
