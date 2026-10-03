# Okapi_Library

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3766504.svg)](https://doi.org/10.5281/zenodo.3766504)

An Arduino-compatible library for utilizing the basic and logging features of the [Okapi](https://github.com/NorthernWidget-Skunkworks/Project-Okapi) data logger.

Data-logger management. Basic operations, power management, on-board sensing, and links to external devices. Manages logging, power, telemetry, and on-board sensing for the Okapi data logger. Supports solar charging, Particle Boron telemetry, external interrupt counting, RTC-driven sleep/wake cycles, and SD-card data logging.

Since 2026-09-23 Okapi is built on [NW_Logger](https://github.com/NorthernWidget/NW_Logger), the logger core it shares with Margay. That gives it Margay's card layout (`/<SN>/log00001.csv` and `sta00001.csv`, `HWTest.txt`; the old `NW/<SN>/Logs/LogN.txt` is gone), the status file with `watch(sensor)`, `note(word)` and the Note column, its serial number from Page 0 when the board is provisioned (NW-Provision) and from the last 8 bytes of EEPROM otherwise, and the camelCase names (`run`, `addDataPoint`, `initLogFile`, `resetWDT`, `powerAuto`, `powerAux`, `i2cState`, `getVoltage`, `setVoltage`). The PascalCase names are gone rather than deprecated: nothing is deployed yet, so callers migrate forward. `begin()` still returns whether the self-tests passed.

**Installation:** included in [NorthernWidget-libraries](https://github.com/NorthernWidget/NorthernWidget-libraries).

```cpp
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
```

Add a sensor with one more `watch()` call. That line states the sensor, where it is and its column order, and the logger writes the file's header row and every data row from the sensors it holds, straight into the open file. No row is composed in RAM, and a sketch keeps no header and no `update()` function (see [LIBRARY-DESIGN.md](https://github.com/NorthernWidget/NW-Device-Specification/blob/master/LIBRARY-DESIGN.md) section 14).

See [examples/](examples/) for this demo.

**Full API reference:** https://docs.northernwidget.com/Okapi_Library/
