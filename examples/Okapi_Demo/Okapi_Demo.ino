/**
 * Okapi_Demo
 *
 * Minimal demonstration sketch for the Okapi data logger library.
 * The logger owns the loop: it reads its own on-board channels (RTC
 * temperature, pressure, rail voltages, solar current) and every sensor
 * watch() gave it, and writes one row per interval.
 *
 * Add a sensor with one more watch() call. It states the sensor, where it
 * is and its column order, and the logger writes the file's header and each
 * row from it: nothing here composes a string.
 */

#include <Okapi.h>

Okapi logger;

void setup() {
  Serial.begin(38400);
  // logger.watch(mySensor);   // one line per external sensor
  if (!logger.begin()) {
    Serial.println("begin() reported an error; check LED color for details.");
  }
}

void loop() {
  logger.run(60);   // seconds between readings
}
