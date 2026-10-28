//Okapi library

#include <Okapi.h>
#include <Arduino.h>

Okapi::Okapi(board Model_, build Specs_)
  : _adcOb(0x48), _adcExt(0x49), _io(0x20) {
  _numAdrOb = 6;  //Clock, IO Expander, ADC_OB, ADC_Ext, DAC, BME
  uint8_t ob[6] = { 0x68, 0x20, 0x48, 0x49, 0x62, 0x77 };
  memcpy(_i2cAdrOb, ob, 6);
  // VSwitch_Pin = 3;
  // VSwitch_Pin = 12; //DEBUG!??

  // THESE ARE DIFFERENT NUMBERS THAN IN THE HEADER FILE!!!!!
  RTCInt = 2;
  LogInt = 27;

  FeatherRTS = 31;
  FeatherCTS = 30;
  FeatherGPIO = 29;
  CS_Ext = 24;

  // WDHold = 255; //Null pins
  // BatSwitch = 255; //Null pins

  //No watchdog ceiling is checked, because this board's watchdog is not being
  //fed. Its TPL5111's DONE line (net WD_DONE in Okapi.sch) lands on the I/O
  //expander's GPIOA3, not on an MCU pin, so resetWDT()'s pulse on WDHold - 23 by
  //default here, never overridden - reaches an unrelated pin. The period itself
  //is jumper-selected at JP8: R20 130k gives 3943 s (66 min) and R62 330k is
  //outside the part's 170k maximum. Set this once DONE is driven through the
  //expander (FIX!).
  wdtSeconds = 0;

  // BatteryDivider = 2.0;

  // if(Specs_ == Build_A) {
  // 	NumADR_OB = 6;
  // }

  // else if(Specs_ == Build_B) {
  // 	//NULL
  // }


  _model = Model_;  //Store model info locally
  _specs = Specs_;  //Store build info locally
}

bool Okapi::beginBoard(uint8_t* Vals, uint8_t NumVals) {
  pinMode(C0, OUTPUT);  //Allow for high power control
  pinMode(C1, OUTPUT);
  pinMode(I2C_SW, OUTPUT);

  pinMode(Sw_Bus_Prime, OUTPUT);
  pinMode(Sw_Bus_Sec, OUTPUT);
  digitalWrite(Sw_Bus_Prime, LOW);
  digitalWrite(Sw_Bus_Sec, LOW);
  powerAuto();  //Get main power running

  pinMode(AuxLED, OUTPUT);
  digitalWrite(AuxLED, LOW);  //Turn built in LED on
  delay(25);
  digitalWrite(Sw_Bus_Prime, HIGH);
  digitalWrite(Sw_Bus_Sec, HIGH);
  delay(500);

  acceptAddresses(Vals, NumVals);  //The watched sensors' addresses (bounded copy)

  i2cState(INTERNAL);
  _rtc.begin();       //Initalize RTC
  _rtc.clearAlarm();  //
  _adcOb.begin();
  _adcExt.begin();
  _dac.begin(0x62);
  _dac.SetRef(BUFFERED_VREF);  //Set buffer configuration //FIX! Make variable if need to set greater than 4.096v?
  Serial.begin(38400);         //Before the first thing that may print: the BME280 result below
  if (!bme280.begin(0x77)) {   //Initalize onboard temp/pressure/RH sensor (BME280)
    Serial.println("BME280 init: FAIL");
    _onBoardError = true;
    _bmeError = true;
    raise(NW_MSG_BME_MISSING);
  }

  Serial.print("Lib = ");
  Serial.println(libVersion);
  bool schema1 = readIdentity();  //Serial number and hardware version from Page 0 (Schema 1), else the last 8 bytes (Schema 0)
  //Schema 0: the model number the sketch declared
  if (!schema1) snprintf(_hwVersion, sizeof(_hwVersion), "%u", (unsigned)_model);
  serialTimeSet();               //A YYMMDDHHMMSS string waiting on Serial sets the clock; then the timestamp
  attachLoggerInterrupts(true);  //LED pins, SD chip select, file times, the alarm and the button (PCINT)
  attachExtInt();                //The external-interrupt counter, if setExtInt() named a pin

  I2Ctest();
  clockTest();
  SDtest();
  // BatTest();
  enviroStats();  //Only print out enviromental variables if BME is on board
  //FIX! Add Feather test??

  printReadyToLog();  //"Ready to Log" on Serial; the LED verdict waits for the first row, in run()
  //Pending Andy's decision on whether a logger keeps serving its own Report
  //register (QUEUE.md, item 1 step 3). The messages above are what the LED and
  //the Note column read; this chain is the served byte, and reports only the
  //first fault it meets.
  //The logger's own report at boot, for its first status row: the first fault the
  //self-tests found, else LoggingStarted (unit, kind 16).
  if (_sdCardMissing) _pages.latchFault(0x01);
  else if (_sdTestFailed) _pages.latchFault(0x05);
  else if (_clockError) _pages.latchFault(0x21);
  else if (_bmeError) _pages.latchFault(0x41);
  else if (_sensorError) _pages.latchFault(0x61);
  _pages.latchNotice(0xF0);
  raise(NW_MSG_LOG_STARTED);
  _bootReport = _pages.report();
  _pages.acknowledge();
  _newLog = true;  //Set flag to begin new log file

  LED_Color(OFF);
  return !(_onBoardError || _sensorError || _timeError || _sdCardMissing);
}

// The data file's header row: the on-board columns, then the sketch's Header,
// then Note. Note is always the last column and carries no comma after it.
size_t Okapi::printDataHeader(Print& out) {
  //The logger's own columns: the timestamp and the on-board sensors. The data
  //file's header row is this, then each watched sensor's, then Note. See
  //LIBRARY-DESIGN.md section 14.
  return out.print("Time [UTC], PresOB [mBar], RH_OB [%], TempOB [C], Temp RTC [C], VBeta [mV], VPrime [mV], ISolar [mA], IBeta [mA],");
}

size_t Okapi::printDataRow(Print& out) {
  //The values readOnBoard() left, in printDataHeader()'s order. This takes no
  //reading: a row written to the card and to the monitor must not read the
  //battery lines twice, and the two rows would otherwise differ.
  size_t n = 0;
  n += out.print(_logTimeDate);
  n += out.print(',');
  n += bme280.printDataRow(out);
  n += out.print(_rtcTemp);
  n += out.print(',');
  n += out.print(_vBetaMv);
  n += out.print(',');
  n += out.print(_vPrimeMv);
  n += out.print(',');
  n += out.print(_iSolarMa);
  n += out.print(',');
  n += out.print(_iBetaMa);
  n += out.print(',');
  return n;
}

void Okapi::enviroStats() {
  Serial.print("Temp = ");
  Serial.print(bme280.getTemperature());
  Serial.println("C");
  Serial.print("Pressure = ");
  Serial.print(bme280.getPressure());
  Serial.println(" mBar");
  Serial.print("RH = ");
  Serial.print(bme280.getHumidity());
  Serial.println("%");
}

//TEMPORARY thresholds (Andy, 2026-10-27: "it might not be perfect"), until the
//Li-ion curve and Page 1's battery block decide them. The rechargeable pack is
//three 18650 Li-ion cells in parallel (Andy, 2026-10-28), so V_LI is one cell's
//voltage: 4.2 V full, 3.7 V nominal. 3.9 V is about two thirds of charge at rest
//and 3.6 V about a tenth, so purple covers half the pack's capacity, which is
//the time to get there and replace it. The alkaline pack is judged as a whole,
//not per cell, because that is what is measured; its two numbers are a
//three-cell pack's.
static const uint16_t OKAPI_LIPO_ABSENT_MV = 2500;  //no Li-ion cell sits this low
static const uint16_t OKAPI_LIPO_LOW_MV = 3600;     //red: swap or charge before you leave
static const uint16_t OKAPI_LIPO_MID_MV = 3900;     //purple below this, blue above
static const uint16_t OKAPI_PRIME_LOW_MV = 3300;    //the whole alkaline pack, red below
static const uint16_t OKAPI_PRIME_MID_MV = 3750;    //the whole alkaline pack, purple below, blue above
static const uint16_t OKAPI_SOLAR_ONLINE_MV = 5000;  //the LT3652 charges from 4.95 V up; current waits on the shunt value

void Okapi::readOnBoard() {
  //Get onboard temp, RTC temp, and battery voltage, referance voltage
  // float VRef = analogRead(VRef_Pin);
  // float Vcc = 3.3; //(1.8/VRef)*3.3; //Compensate for Vcc using VRef
  // Serial.println(Vcc); //DEBUG!
  // float TempData = 0; //FIX!!! Dumb!

  // if(Model < Model_2v0) {  //For older thermistor models
  // 	float Val = float(analogRead(ThermSense_Pin));
  // 	float Comp = (1.8/3.3)*1024.0/analogRead(VRef_Pin);  //Find compensation value with VRef due to Vcc error
  // 	if(Model == 0) Comp = 1.0; //Overide comp calculation since many v0.0 models do not have ref equiped
  // 	Val = Val*Comp*(Vcc/1024.0); //Compensate for ref voltage error
  // 	//  float Vout = Vcc - Val;
  // 	//  Serial.println(Val); //DEBUG!
  // 	//  Serial.println(Vout);  //DEBUG!
  // 	TempData = TempConvert(Val, Vcc*Comp, 10000.0, A, B, C, D, 10000.0);
  // 	TempData = TempData - 273.15; //Get temp from on board thermistor
  // }

  // delay(10);
  // float BatVoltage = GetBatVoltage(); //Get battery voltage, Include voltage divider in math

  // Temp[3] = Clock.getTemperature(); //Get tempreture from RTC //FIX!
  // powerAuto(); //Turn on power  //FIX??
  pinMode(ADC_Sense_SW, OUTPUT);     //DEBUG!!!!!!!!!!!!!!!!!!
  digitalWrite(ADC_Sense_SW, HIGH);  //Enable reading of battery lines  //FIX! Shorten to reduce current draw!
  // float VBeta = 0;
  // float ISolar = 0;  //FIX! Adjust after changing gain of amp??
  // float VPrime = 0;
  // float IBeta = 0;

  float VBeta = _adcOb.readADC_SingleEnded(0);
  float VPrime = _adcOb.readADC_SingleEnded(2);
  digitalWrite(ADC_Sense_SW, LOW);  //Enable reading of battery lines
  VBeta = VBeta * 0.1875;
  VPrime = VPrime * 0.1875;


  float ISolar = _adcOb.readADC_SingleEnded(1) * 0.01875;  //FIX! Adjust after changing gain of amp??
  float IBeta = (_adcOb.readADC_SingleEnded(3) * 0.1875 - 2500) / 1.5;

  float RTCTemp = _rtc.getTemp();  //Get Temp from RTC
  getTime();                       //FIX!
  // if(Model< Model_2v0) return LogTimeDate + "," + String(RTCTemp) + "," + String(VBeta) + ",";
  _rtcTemp = RTCTemp;
  _vBetaMv = VBeta;
  _vPrimeMv = VPrime;
  _iSolarMa = ISolar;
  _iBetaMa = IBeta;

  //TEMPORARY (Andy, 2026-10-27): the batteries are on the PAC1934, not on the
  //ADS1115 at 0x48, which the v1.0 board does not carry. Its VBUS channels are
  //1 = V_PRIME (the alkaline backup), 2 = V_LI (the LiPo cell), 3 = V_BETA (the
  //LiPo rail), 4 = V_SOLAR (Okapi.sch, U32). Where the chip answers, its
  //readings replace the two the row carries; the current channels wait on the
  //shunt values, which the schematic does not give.
  if (_pacAddr == 0) _pacAddr = pacFind();
  if (_pacAddr != 0) {
    _vPrimeMv = pacVbusMv(1);
    _vLiMv = pacVbusMv(2);
    _vBetaMv = pacVbusMv(3);
    _vSolarMv = pacVbusMv(4);
    //The LiPo is absent when its terminal reads below any cell voltage, or the
    //bus could only be brought up from the backup (powerAuto() returned 1).
    //Either way the backup is carrying the logger: say so, in the file and in
    //the chirp's form.
    if (_vLiMv < OKAPI_LIPO_ABSENT_MV || _powerState == 1) raise(NW_MSG_ON_BACKUP_POWER);
  }
}

static uint32_t levelColour(float mv, uint16_t low, uint16_t mid) {
  if (mv < low) return RED;
  if (mv < mid) return PURPLE;
  return BLUE;
}

uint32_t Okapi::chirpColour() {
  //The LiPo's, every time (Andy, 2026-10-27): absent counts as low, because
  //there is nothing to run on. No reading at all says "all well", as a board
  //that cannot judge its battery does.
  if (_pacAddr == 0) return BLUE;
  return levelColour(_vLiMv, OKAPI_LIPO_LOW_MV, OKAPI_LIPO_MID_MV);
}

uint32_t Okapi::backupColour() {
  if (_pacAddr == 0) return BLUE;
  return levelColour(_vPrimeMv, OKAPI_PRIME_LOW_MV, OKAPI_PRIME_MID_MV);
}

void Okapi::chirp(uint8_t form) {
  //1. Margay's chirp, in the LiPo's colour (or the long-and-buzz when the card
  //   is out): what a Margay user already knows.
  NW_Logger::chirp(form);
  if (_pacAddr == 0) return;  //Nothing more to say without the power monitor
  //2. "ba bap, ba bap bap" in the alkaline backup's colour. ba = 200 ms, bap =
  //   100 ms, 100 ms dark inside a group, 300 ms at the comma; 500 ms before it
  //   so it reads as the next thing and not as part of the chirp.
  uint32_t backup = backupColour();
  delay(500);
  LED_Color(backup);
  delay(200);
  LED_Color(OFF);
  delay(100);
  LED_Color(backup);
  delay(100);
  LED_Color(OFF);
  delay(300);
  LED_Color(backup);
  delay(200);
  LED_Color(OFF);
  delay(100);
  LED_Color(backup);
  delay(100);
  LED_Color(OFF);
  delay(100);
  LED_Color(backup);
  delay(100);
  LED_Color(OFF);
  //3. If the panel is online, one slow swell: warm white brought up from dark
  //   to full and back over two seconds. Nothing else in the vocabulary ramps,
  //   every fault pattern switches hard, so a swell is its own thing and reads
  //   as calm rather than alarm; a trill at the flutter's rate read as a
  //   warning (Andy, 2026-10-27). White is daylight and belongs to no band.
  if (_vSolarMv < OKAPI_SOLAR_ONLINE_MV) return;
  delay(500);
  for (uint8_t step = 1; step <= 32; step++) {
    LED_Color(((uint32_t)(step * 8 - 1) << 24) | 0x00FFFFFFUL);  //luminosity byte up: 7, 15, ... 255
    delay(30);
  }
  for (uint8_t step = 32; step >= 1; step--) {
    LED_Color(((uint32_t)(step * 8 - 1) << 24) | 0x00FFFFFFUL);  //and down
    delay(30);
  }
  LED_Color(OFF);
}

//PAC1934 (Microchip DS20005850E): REFRESH_V at 0x1F latches fresh VBUS/VSENSE
//values without touching the accumulators and needs 1 ms (section 4.1.4);
//VBUS1-4 at 0x07-0x0A, two bytes MSB first, 16-bit unipolar over 32 V, so one
//count is 488 uV (section 1.0, VBUS_LSB); PRODUCT_ID at 0xFD reads 0x5B for the
//PAC1934; the address is one of 0x10-0x1F by the ADDR_SEL resistor (Table 5-1),
//whose value the schematic does not give, so the chip is found by asking.
static const uint8_t PAC_REFRESH_V = 0x1F;
static const uint8_t PAC_VBUS1 = 0x07;
static const uint8_t PAC_PRODUCT_ID = 0xFD;
static const uint8_t PAC1934_ID = 0x5B;

uint8_t Okapi::pacFind() {
  for (uint8_t addr = 0x10; addr <= 0x1F; addr++) {
    Wire.beginTransmission(addr);
    Wire.write(PAC_PRODUCT_ID);
    if (Wire.endTransmission(false) != 0) continue;
    if (Wire.requestFrom(addr, (uint8_t)1) != 1) continue;
    if (Wire.read() == PAC1934_ID) return addr;
  }
  return 0;
}

uint16_t Okapi::pacVbusMv(uint8_t channel) {
  if (_pacAddr == 0 || channel < 1 || channel > 4) return 0;
  Wire.beginTransmission(_pacAddr);
  Wire.write(PAC_REFRESH_V);
  if (Wire.endTransmission() != 0) return 0;
  delay(2);  //1 ms is the datasheet's minimum
  Wire.beginTransmission(_pacAddr);
  Wire.write(PAC_VBUS1 + channel - 1);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(_pacAddr, (uint8_t)2) != 2) return 0;
  uint16_t raw = (uint16_t)Wire.read() << 8;
  raw |= Wire.read();
  return (uint16_t)(((uint32_t)raw * 32000UL) >> 16);  //32 V over 65536 counts
}

size_t Okapi::printRow(Print& out, uint8_t lineIndex, uint32_t dataIndex) {
  _sd.chdir("/");  //The card's root
  _sd.chdir(_sn);  //Move into this logger's folder, named by its serial number
  File DataFile = _sd.open(_fileNameC, FILE_READ);
  if (!DataFile) return 0;

  DataFile.seek(dataIndex);                  //Run to starting location
  for (uint8_t i = 0; i < lineIndex; i++) {  //Read out previous lines
    int c;
    while ((c = DataFile.read()) >= 0 && c != '\n')
      ;
    if (c < 0) {  //The file ended before the row asked for
      DataFile.close();
      return 0;
    }
  }

  //The row itself, straight out: a character off the card is a character into
  //out, and nothing holds the row in between. The newline is where the row
  //ends and is not printed, which is what readStringUntil() did.
  size_t n = 0;
  int c;
  while ((c = DataFile.read()) >= 0 && c != '\n') n += out.write((uint8_t)c);
  DataFile.close();  //Nothing else closes it: this SdFat is built with
                     //DESTRUCTOR_CLOSES_FILE 0
  return n;
}


void Okapi::addDataPoint()  //The streaming row: this board's bus dance around it
{
  i2cState(EXTERNAL);
  readSensors();
  i2cState(INTERNAL);  //DEBUG!
  bme280.begin(0x77);  //DEBUG!
  if (logRow() != 0) {
    _pages.latchNotice(0xF2);  //RowNotWritten, the served byte
    raise(NW_MSG_ROW_NOT_WRITTEN);
  }
  _logCount++;   //FIX??
  fillPages();   //Okapi's reading of itself: Page 2, Page 3, Block 0
  reportRows();  //The status file: a row for the logger and every watched sensor with something to report
}

void Okapi::afterLogEvent()  //After an alarm-driven row: the backhaul
{
  if (_logCount >= _logCountPush && _powerState == 0) {    //If enough logs have been recorded and main battery power is available - backhaul //REPLACE WITH TIMER TEST!
                                                           //// Update conventions in MCP23018 library
    _io.digitalWrite(FeatherEN, HIGH, MCP23018::Port::B);  //Turn on Feather power
    ////IO.digitalWrite(FeatherEN, HIGH, MCP23018::Ports::B); //Turn on Feather power
    // for(int i = 0; i < 10; i++) {  //DEBUG!
    // 	Serial.println("START BACKHAUL"); //DEBUG!
    // 	delay(100);
    // }
    //delay(20);
    //Serial.end();
    //delay(20);
    //Serial.begin(38400); // Use different rate too?
    //delay(20);
    // Delay to make transmission work
    unsigned long Timeout = millis();
    while ((millis() - Timeout) < 1000)
      ;                                // Give transmission some time. (This must be substantial!)
    Serial.println("START BACKHAUL");  //DEBUG!
    //Serial.println("MID BACKHAUL"); //DEBUG!
    ///*
    for (int i = 0; i < _logCountPush; i++) {  //Print out SD values
      printRow(Serial, i, _lastSdIndex);
      Serial.println();
    }
    //*/
    _lastSdIndex = _sdIndex;  //copy new value over
    _logCount = 0;
    Serial.println("END BACKHAUL");  //DEBUG!
    Timeout = millis();
    while (digitalRead(FeatherGPIO) && (millis() - Timeout) < 180000)
      ;  //Wait for completerion or for timeout (180 seconds -- takes 2G/3G longer)
    // while((millis() - Timeout) < 59000); //DEBUG!
    // Give as much time as possible to complete the communications. Takes a while and can time out easily.
    pinMode(FeatherGPIO, INPUT);
  }
}

uint8_t Okapi::setVoltageRaw(uint16_t Val, bool Gain) {
  if (Val > 4095 || Val < 0) {
    Serial.println("BANG!");
    _dac.Sleep(ON);  //Make device output open to avoid issues //FIX??
    return 5;        //Return out of range error
  } else {
    // DAC.Sleep(OFF); //Make device is set to output
    _dac.SetGain(Gain);           //Set appropriate gain (default to 1x)
    return _dac.setVoltage(Val);  //Do not allow to set value to memory, return I2C status
  }
}

uint8_t Okapi::setVoltage(float Val)  //Interpolated nearest value from float
{
  uint16_t BitValue = 0;  //used to calculate the bit value to set the DAC to
  if (Val > 5.0 || Val < 0.0) {
    _dac.Sleep(ON);  //Make device output open to avoid issues //FIX??
    return 5;        //Return out of range error
  } else if (Val >= 2.5) {
    _dac.Sleep(OFF);                       //Make device is set to output
    BitValue = floor(Val * 819.2);         //Set for 2x single multiple
    if (BitValue > 4095) BitValue = 4095;  //FIX?? Prevent wrap around error due to float rounding
    _dac.SetGain(GAIN_2X);                 //Set 2x gain
    return _dac.setVoltage(BitValue);      //Return I2C status
  } else {
    _dac.Sleep(OFF);                       //Make device is set to output
    BitValue = floor(Val * 1638.4);        //Set for single multiple
    if (BitValue > 4095) BitValue = 4095;  //FIX?? Prevent wrap around error due to float rounding
    _dac.SetGain(GAIN_1X);                 //Set unity gain
    return _dac.setVoltage(BitValue);      //Return I2C status
  }
}

float Okapi::getVoltage(uint8_t Pin)  //Get voltage external ADC from specified pin
{
  i2cState(INTERNAL);
  float Val = _adcExt.readADC_SingleEnded(Pin) * 0.1875;
  i2cState(EXTERNAL);
  return Val;
}

uint8_t Okapi::powerAuto() {
  uint8_t DDR_Prev = DDRC;       //Read port state to be able to return
  DDRC = DDR_Prev | 0x0C;        //Set C1 and C0 as output
  DDRC = DDR_Prev & 0x7F;        //Make PC7 (EN_BUS_PRIME) and input
  uint8_t PortState = PORTC;     //Do manipulation locally, then push to port
  PortState = PortState & 0xF3;  //Clear C0 and C1
  PORTC = PortState;             //Turn rail off to ensure valid measurment //FIX??
  delay(2);                      //Cx rising edge to EN_BUS_PRIME rising (90%, ~3.0v) measured at 1.4ms, added ~50% margin of safety for robustness
  PortState = PortState & 0xF3;  //Clear C0 and C1
  PortState = PortState | 0x08;  //Set C1 HIGH, C0 LOW (Vbeta)
  // PORTC = PORTC & 0xF3; //Clear C0 and C1
  // PORTC = PORTC | 0x08; //Set C1 HIGH, C0 LOW (Vbeta)
  PORTC = PortState;  //Write values to port
  // digitalWrite(C1, HIGH); //Set for Vbeta initally
  // digitalWrite(C0, LOW);
  delay(2);                  //Cx rising edge to EN_BUS_PRIME rising (90%, ~3.0v) measured at 1.4ms, added ~50% margin of safety for robustness
  bool State = (PINC >> 7);  //Read state of PC7
  if (State) return 0;       //Good
  else {
    PORTC = PORTC & 0xF3;      //Clear C0 and C1
    PORTC = PORTC | 0x04;      //Set C1 LOW and C0 HIGH (Vprime)
    delay(2);                  //Cx rising edge to EN_BUS_PRIME rising (90%, ~3.0v) measured at 1.4ms, added ~50% margin of safety for robustness
    bool State = (PINC >> 7);  //Read state of PC7
    if (State) return 1;       //Good, aux
  }
  return 2;  //Error, power not good??
}

void Okapi::powerAux(uint8_t State)  //UPDATE! 0 or 3 = OFF, 1 = V_Prime, 2 = V_Beta
{
  State = State & 0b11;          //Restrict to lowest 2 bits
  uint8_t DDR_Prev = DDRC;       //Read port state to be able to return
  DDRC = DDR_Prev | 0x0C;        //Set C1 and C0 as output
  PORTC = PORTC & 0xF3;          //Clear C1 and C2
  PORTC = PORTC | (State << 2);  //Set C1 and C0 appropriately
                                 // Serial.println(PORTC); //DEBUG!
}

void Okapi::i2cState(bool State) {
  digitalWrite(I2C_SW, State);
  // uint8_t PortVal = PORTC; //Read status  //FIX??
  // PortVal = PortVal & 0xDF; //Clear C5
  // PortVal = PortVal | (State << 5); //Set C5 with appropriate value
  // PORTC = PortVal; //Set port
}

uint8_t Okapi::chipFaults() {
  uint8_t f = 0;
  if (_sdCardMissing || _sdTestFailed) f |= 0x01;
  if (_clockError) f |= 0x02;
  if (_bmeError) f |= 0x04;
  if (_sensorError) f |= 0x08;
  return f;
}

// Okapi's reading of itself, per the NW-Device-Specification Okapi appendix
// (hypothetical as of 2026-09-23). Block 1, power, waits on the power model
// (which of VBeta and VPrime is the LiPo, the solar scaling) and stays zero.
void Okapi::fillPages() {
  _pages.beginReading();
  if (!_bmeError) {
    _pages.put16(0x50, (uint16_t)(int16_t)(bme280.getTemperature() * 100.0));
    _pages.put16(0x52, (uint16_t)(bme280.getHumidity() * 100.0));
    _pages.put32(0x54, (uint32_t)(bme280.getPressure() * 100.0));
  }
  _pages.put32(0x58, clockUnix());  //Clock: Unix seconds
  _pages.put16(0x5C, (uint16_t)(int16_t)(_rtc.getTemp() * 100.0));
  _pages.put16(0x60, getExtIntCount(false));
  _pages.put16(0x62, _fileNum);
  _pages.put32(0x64, _logInterval);
  _pages.endReading(chipFaults());
}

static const char* const okapiChips[] = { "SDCard", "Clock", "BME280", "SensorBus", "Charger", "Backup" };
static const char* const okapiWords[] = { "LoggingStarted", "NewLogFile", "RowNotWritten" };  //unit kinds 16-18
static const char* const okapiChipWords[] = { "ClockSet" };                                   //kind 16 on Clock (0x30)

size_t Okapi::printStatus(Print& out, bool boot) {
  const NW_Report& r = boot ? _bootReport : _pages.report();
  const char* const* words = okapiWords;
  uint8_t n = 3;
  if (r.chip() == 1) {
    words = okapiChipWords;
    n = 1;
  }
  return _pages.printSnapshot(out, okapiChips, 6, libVersion, &r, words, n, OKAPI_LIBRARY_COMMIT, "", SKETCH_COMMIT);  //A logger: its library is its firmware; the sketch stands where a library would
}

void Okapi::sleepNow()  // here we put the arduino to sleep
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we
     * choose the according
     * sleep mode: SLEEP_MODE_PWR_DOWN
     *
     */
  // MCUCR = bit (BODS) | bit (BODSE);
  // MCUCR = bit (BODS);
  //  wdt_disable();  //DEBUG!??
  // power_adc_disable(); // ADC converter
  // // power_spi_disable(); // SPI
  // power_usart0_disable();// Serial (USART)
  // power_timer1_disable();// Timer 1
  // power_timer2_disable();// Timer 2
  // ADCSRA = 0;
  turnOffSDcard();
  // digitalWrite(Ext3v3Ctrl, HIGH); //Turn off extenral rail
  // SPI.end(); //Turn off SPI
  // digitalWrite(SD_CS, LOW);
  // pinMode(SD_CS, INPUT); //Disconnect SD chip slect pin
  // pinMode(5, INPUT); //Set all SPI pins as inputs, will be reversed be beginning SPI again
  // pinMode(6, INPUT);
  // pinMode(7, INPUT);
  // digitalWrite(VSwitch_Pin, LOW); //DEBUG!
  keep_ADCSRA = ADCSRA;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // sleep mode is set here
  cbi(ADCSRA, ADEN);
  sleep_enable();
  sleep_bod_disable();
  sei();

  sleep_cpu();
  sleep_disable();
  // pinMode(3, OUTPUT); //DEBUG!
  // detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
  //    ADCSRA = 1; //Turn ADC back on
  // digitalWrite(Ext3v3Ctrl, LOW); //turn external rail back on
  // digitalWrite(SD_CS, HIGH);
  //     SPI.begin();
  turnOnSDcard();
  ADCSRA = 135;  //DEBUG!
  Serial.begin(38400);
  // digitalWrite(VSwitch_Pin, HIGH);  //DEBUG!
  // pinMode(SD_CS, OUTPUT); //Disconnect SD chip slect pin
}

void Okapi::turnOffSDcard() {
  delay(6);
  // disable SPI
  // power_spi_disable();                     // disable SPI clock
  // DDRB &= ~((1<<DDB5) | (1<<DDB7) | (1<<DDB6) | (1<<DDB4));   // set All SPI pins to INPUT
  // pinMode(SD_CD, INPUT);
  // DDRC &= ~((1<<DDC0) | (1<<DDC1));
  // pinMode(31, OUTPUT); //DEBUG!
  // digitalWrite(31, LOW); //DEBUG!
  pinMode(Sw_Bus_Prime, INPUT);
  pinMode(Sw_Bus_Sec, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  // digitalWrite(8, LOW);
  // digitalWrite(9, LOW);
  Serial.end();
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  // digitalWrite(16, HIGH);
  // digitalWrite(17, HIGH);
  //digitalWrite(SD_CS, HIGH);
  // digitalWrite(5, LOW);
  // // Note: you must disconnect the LED on pin 13 or you’ll bleed current through the limit resistor
  // // LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF); // wait 1 second before pulling the plug!
  delay(6);
  // digitalWrite(Ext3v3Ctrl, HIGH); //MODEL <= v1
  // digitalWrite(Ext3v3Ctrl, LOW);  //turn off external 3v3 rail
  // digitalWrite(BatSwitch, LOW); //Turn off battery connection to sense divider
  // powerAux(OFF); //turn off external 3v3 rail
  // PowerOB(OFF); //Turn off battery connection to sense divider
  // digitalWrite(31, HIGH); //DEBUG!
  powerAux(OFF);  //Turn off power
  // digitalWrite(BatRailCtrl, HIGH);
  delay(1);
  digitalWrite(SD_CS, LOW);
  delay(20);
  // SPCR = SPCR & 0b11101111;
  SPCR = 0;
  power_spi_disable();
  // SPI.end();
  delay(10);
  // pinMode(5, OUTPUT);d
  // digitalWrite(5, LOW);
  // DDRB &= ~((1<<DDB5));
  // PORTB &= ~(1<<PORTB5); //Set port B5 (MOSI) LOW
  // DDRB &= ~((1<<DDB5) | (1<<DDB7) | (1<<DDB6) | (1<<DDB4));
  // PORTB |= ((1<<DDB5) | (1<<DDB7) | (1<<DDB6) | (1<<DDB4));     // set ALL SPI pins HIGH (~30k pullup)
  // digitalWrite(SD_CS, LOW);
  // pinMode(SD_CS, INPUT);
  delay(6);
}

void Okapi::turnOnSDcard() {
  // pinMode(SD_CS, OUTPUT);
  // SPI.begin();
  // sd.begin(SD_CS);
  // DDRB |= ((1<<DDB5));
  // digitalWrite(SD_CS, HIGH);
  // digitalWrite(Ext3v3Ctrl, HIGH);  //turn off external 3v3 rail
  // digitalWrite(BatSwitch, HIGH); //Turn off battery connection to sense divider
  _powerState = powerAuto();  //Fix??
  // PowerOB(ON); //Turn on battery connection to sense divider
  // powerAux(ON); //turn on external 3v3 rail
  delay(6);  // let the card settle
  // some cards will fail on power-up unless SS is pulled up  ( &  D0/MISO as well? )
  // DDRC = DDRC | ((1<<DDC0) | (1<<DDC1));
  // DDRB = DDRB | (1<<DDB7) | (1<<DDB5) | (1<<DDB4); // set SCLK(D13), MOSI(D11) & SS(D10) as OUTPUT
  // Note: | is an OR operation so  the other pins stay as they were.                (MISO stays as INPUT)
  // PORTB = PORTB & ~(1<<DDB7);  // disable pin 13 SCLK pull-up – leave pull-up in place on the other 3 lines
  power_spi_enable();  // enable the SPI clock
  SPCR = keep_SPCR;    // enable SPI peripheral
  // delay(20);
  // digitalWrite(BatRailCtrl, LOW);
  // digitalWrite(Ext3v3Ctrl, LOW); //MODEL <= v1
  delay(10);
  // digitalWrite(3, HIGH); //DEBUG!
  _sd.begin(SD_CS, SD_SCK_MHZ(8));
  // digitalWrite(3, LOW); //DEBUG!
}
