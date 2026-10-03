//Okapi library

#include <Okapi.h>
#include <Arduino.h>

Okapi::Okapi(board Model_, build Specs_) : _adcOb(0x48), _adcExt(0x49), _io(0x20)
{
	_numAdrOb = 6; //Clock, IO Expander, ADC_OB, ADC_Ext, DAC, BME
	uint8_t ob[6] = {0x68, 0x20, 0x48, 0x49, 0x62, 0x77};
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

	// BatteryDivider = 2.0;

	// if(Specs_ == Build_A) {
	// 	NumADR_OB = 6;
	// }

	// else if(Specs_ == Build_B) {
	// 	//NULL
	// }


	_model = Model_; //Store model info locally
	_specs = Specs_; //Store build info locally
}

bool Okapi::beginBoard(uint8_t *Vals, uint8_t NumVals)
{
	pinMode(C0, OUTPUT);  //Allow for high power control
	pinMode(C1, OUTPUT);
	pinMode(I2C_SW, OUTPUT);

	pinMode(Sw_Bus_Prime, OUTPUT);
	pinMode(Sw_Bus_Sec, OUTPUT);
	digitalWrite(Sw_Bus_Prime, LOW);
	digitalWrite(Sw_Bus_Sec, LOW);
	powerAuto(); //Get main power running

	pinMode(AuxLED, OUTPUT);
	digitalWrite(AuxLED, LOW); //Turn built in LED on
	delay(25);
	digitalWrite(Sw_Bus_Prime, HIGH);
	digitalWrite(Sw_Bus_Sec, HIGH);
	delay(500);

	acceptAddresses(Vals, NumVals); //The watched sensors' addresses (bounded copy)

	i2cState(INTERNAL);
	_rtc.begin(); //Initalize RTC
	_rtc.clearAlarm(); //
	_adcOb.begin();
	_adcExt.begin();
	_dac.begin(0x62);
	_dac.SetRef(BUFFERED_VREF); //Set buffer configuration //FIX! Make variable if need to set greater than 4.096v?
	if(!bme280.begin(0x77)) { //Initalize onboard temp/pressure/RH sensor (BME280)
		Serial.println("BME280 init: FAIL");
		_onBoardError = true;
		_bmeError = true;
	}

	Serial.begin(38400); //DEBUG!
	Serial.print("Lib = ");
	Serial.println(libVersion);
	bool schema1 = readIdentity(); //Serial number and hardware version from Page 0 (Schema 1), else the last 8 bytes (Schema 0)
	//Schema 0: the model number the sketch declared
	if(!schema1) snprintf(_hwVersion, sizeof(_hwVersion), "%u", (unsigned)_model);
	serialTimeSet(); //A YYMMDDHHMMSS string waiting on Serial sets the clock; then the timestamp
	attachLoggerInterrupts(true); //LED pins, SD chip select, file times, the alarm and the button (PCINT)
	attachExtInt(); //The external-interrupt counter, if setExtInt() named a pin

	I2Ctest();
	clockTest();
	SDtest();
	// BatTest();
	enviroStats();  //Only print out enviromental variables if BME is on board
	//FIX! Add Feather test??

	ledReport(); //The self-test results on the RGB LED, then "Ready to Log"
	//The logger's own report at boot, for its first status row: the first fault the
	//self-tests found, else LoggingStarted (unit, kind 16).
	if(_sdCardMissing) _pages.latchFault(0x01);
	else if(_sdTestFailed) _pages.latchFault(0x05);
	else if(_clockError) _pages.latchFault(0x21);
	else if(_bmeError) _pages.latchFault(0x41);
	else if(_sensorError) _pages.latchFault(0x61);
	_pages.latchNotice(0xF0);
	_bootReport = _pages.report();
	_pages.acknowledge();
	_newLog = true; //Set flag to begin new log file

	LED_Color(OFF);
	return !(_onBoardError || _sensorError || _timeError || _sdCardMissing);
}

// The data file's header row: the on-board columns, then the sketch's Header,
// then Note. Note is always the last column and carries no comma after it.
size_t Okapi::printDataHeader(Print& out)
{
	//The logger's own columns: the timestamp and the on-board sensors. The data
	//file's header row is this, then each watched sensor's, then Note. See
	//LIBRARY-DESIGN.md section 14.
	return out.print("Time [UTC], PresOB [mBar], RH_OB [%], TempOB [C], Temp RTC [C], VBeta [mV], VPrime [mV], ISolar [mA], IBeta [mA],");
}

size_t Okapi::printDataRow(Print& out)
{
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

void Okapi::enviroStats()
{
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

void Okapi::readOnBoard()
{
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
	pinMode(ADC_Sense_SW, OUTPUT); //DEBUG!!!!!!!!!!!!!!!!!!
	digitalWrite(ADC_Sense_SW, HIGH); //Enable reading of battery lines  //FIX! Shorten to reduce current draw!
	// float VBeta = 0;
	// float ISolar = 0;  //FIX! Adjust after changing gain of amp??
	// float VPrime = 0;
	// float IBeta = 0;

	float VBeta = _adcOb.readADC_SingleEnded(0);
	float VPrime = _adcOb.readADC_SingleEnded(2);
	digitalWrite(ADC_Sense_SW, LOW); //Enable reading of battery lines
	VBeta = VBeta*0.1875;
	VPrime = VPrime*0.1875;


	float ISolar = _adcOb.readADC_SingleEnded(1)*0.01875;  //FIX! Adjust after changing gain of amp??
	float IBeta = (_adcOb.readADC_SingleEnded(3)*0.1875 - 2500)/1.5;

	float RTCTemp = _rtc.getTemp();  //Get Temp from RTC
	getTime(); //FIX!
	// if(Model< Model_2v0) return LogTimeDate + "," + String(RTCTemp) + "," + String(VBeta) + ",";
	_rtcTemp = RTCTemp;
	_vBetaMv = VBeta;
	_vPrimeMv = VPrime;
	_iSolarMa = ISolar;
	_iBetaMa = IBeta;
}

String Okapi::readStr(uint8_t LineIndex, uint32_t DataIndex)  //Pass index (working backwards from most recent log)
{
	// Serial.println(Val); //Echo to serial monitor
	// SD.begin(SD_CS); //DEBUG!
	// SD.chdir("/"); //Return to root to define starting state
	// IO.PinMode(6, OUTPUT, A); //DEBUG!
	// IO.DigitalWrite(6, LOW, A); //DEBUG!
	_sd.chdir("/");  //The card's root
	_sd.chdir(_sn);  //Move into this logger's folder, named by its serial number
	File DataFile = _sd.open(_fileNameC, FILE_READ);

	// if the file is available, read from it:
	if (DataFile) {
		DataFile.seek(DataIndex); //Run to starting location
		for(int i = 0; i < LineIndex; i++) {
			DataFile.readStringUntil('\n'); //Read out previous lines
		}
		return DataFile.readStringUntil('\n'); //Return desired line
	   // return 0;
	}
	// if the file isn't open, pop up an error:
	else {
	   // return -1;
	}

	DataFile.close();

	// IO.PinMode(6, OUTPUT, A); //DEBUG!
	// IO.DigitalWrite(6, HIGH, A); //DEBUG!
}


void Okapi::addDataPoint() //The streaming row: this board's bus dance around it
{
	i2cState(EXTERNAL);
	readSensors();
	i2cState(INTERNAL);  //DEBUG!
	bme280.begin(0x77); //DEBUG!
	if(logRow() != 0) _pages.latchNotice(0xF2); //RowNotWritten
	_logCount++; //FIX??
	fillPages(); //Okapi's reading of itself: Page 2, Page 3, Block 0
	reportRows(); //The status file: a row for the logger and every watched sensor with something to report
}

void Okapi::afterLogEvent() //After an alarm-driven row: the backhaul
{
	if(_logCount >= _logCountPush && _powerState == 0) {  //If enough logs have been recorded and main battery power is available - backhaul //REPLACE WITH TIMER TEST!
	  //// Update conventions in MCP23018 library
		_io.digitalWrite(FeatherEN, HIGH, MCP23018::Port::B); //Turn on Feather power 
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
		while((millis() - Timeout) < 1000); // Give transmission some time. (This must be substantial!)
		Serial.println("START BACKHAUL"); //DEBUG!
		//Serial.println("MID BACKHAUL"); //DEBUG!
		///*
		for(int i = 0; i < _logCountPush; i++) { //Print out SD values
			Serial.println(readStr(i, _lastSdIndex));
		}
		//*/
		_lastSdIndex = _sdIndex; //copy new value over
		_logCount = 0;
		Serial.println("END BACKHAUL"); //DEBUG!
		Timeout = millis();
		while(digitalRead(FeatherGPIO) && (millis() - Timeout) < 180000); //Wait for completerion or for timeout (180 seconds -- takes 2G/3G longer)
		// while((millis() - Timeout) < 59000); //DEBUG!
		// Give as much time as possible to complete the communications. Takes a while and can time out easily.
		pinMode(FeatherGPIO, INPUT);
	}
}

uint8_t Okapi::setVoltageRaw(uint16_t Val, bool Gain)
{
	if(Val > 4095 || Val < 0) {
		Serial.println("BANG!");
		_dac.Sleep(ON); //Make device output open to avoid issues //FIX??
		return 5; //Return out of range error
	}
	else {
	// DAC.Sleep(OFF); //Make device is set to output
	_dac.SetGain(Gain); //Set appropriate gain (default to 1x)
	return _dac.setVoltage(Val); //Do not allow to set value to memory, return I2C status
	}
}

uint8_t Okapi::setVoltage(float Val)  //Interpolated nearest value from float
{
	uint16_t BitValue = 0; //used to calculate the bit value to set the DAC to
	if(Val > 5.0 || Val < 0.0) {
		_dac.Sleep(ON); //Make device output open to avoid issues //FIX??
		return 5; //Return out of range error
	}
	else if(Val >= 2.5) {
		_dac.Sleep(OFF); //Make device is set to output
		BitValue = floor(Val*819.2); //Set for 2x single multiple
		if(BitValue > 4095) BitValue = 4095; //FIX?? Prevent wrap around error due to float rounding
		_dac.SetGain(GAIN_2X); //Set 2x gain
		return _dac.setVoltage(BitValue); //Return I2C status
	}
	else {
		_dac.Sleep(OFF); //Make device is set to output
		BitValue = floor(Val*1638.4); //Set for single multiple
		if(BitValue > 4095) BitValue = 4095; //FIX?? Prevent wrap around error due to float rounding
		_dac.SetGain(GAIN_1X); //Set unity gain
		return _dac.setVoltage(BitValue); //Return I2C status
	}
}

float Okapi::getVoltage(uint8_t Pin)  //Get voltage external ADC from specified pin
{
	i2cState(INTERNAL);
	float Val = _adcExt.readADC_SingleEnded(Pin)*0.1875;
	i2cState(EXTERNAL);
	return Val;
}

uint8_t Okapi::powerAuto()
{
	uint8_t DDR_Prev = DDRC; //Read port state to be able to return
	DDRC = DDR_Prev | 0x0C; //Set C1 and C0 as output
	DDRC = DDR_Prev & 0x7F; //Make PC7 (EN_BUS_PRIME) and input
	uint8_t PortState = PORTC; //Do manipulation locally, then push to port
	PortState = PortState & 0xF3; //Clear C0 and C1
	PORTC = PortState; //Turn rail off to ensure valid measurment //FIX??
	delay(2); //Cx rising edge to EN_BUS_PRIME rising (90%, ~3.0v) measured at 1.4ms, added ~50% margin of safety for robustness
	PortState = PortState & 0xF3; //Clear C0 and C1
	PortState = PortState | 0x08; //Set C1 HIGH, C0 LOW (Vbeta)
	// PORTC = PORTC & 0xF3; //Clear C0 and C1
	// PORTC = PORTC | 0x08; //Set C1 HIGH, C0 LOW (Vbeta)
	PORTC = PortState; //Write values to port
	// digitalWrite(C1, HIGH); //Set for Vbeta initally
	// digitalWrite(C0, LOW);
	delay(2); //Cx rising edge to EN_BUS_PRIME rising (90%, ~3.0v) measured at 1.4ms, added ~50% margin of safety for robustness
	bool State = (PINC >> 7); //Read state of PC7
	if(State) return 0; //Good
	else {
		PORTC = PORTC & 0xF3; //Clear C0 and C1
		PORTC = PORTC | 0x04; //Set C1 LOW and C0 HIGH (Vprime)
		delay(2); //Cx rising edge to EN_BUS_PRIME rising (90%, ~3.0v) measured at 1.4ms, added ~50% margin of safety for robustness
		bool State = (PINC >> 7); //Read state of PC7
		if(State) return 1; //Good, aux
	}
	return 2; //Error, power not good??
}

void Okapi::powerAux(uint8_t State)  //UPDATE! 0 or 3 = OFF, 1 = V_Prime, 2 = V_Beta
{
	State = State & 0b11; //Restrict to lowest 2 bits
	uint8_t DDR_Prev = DDRC; //Read port state to be able to return
	DDRC = DDR_Prev | 0x0C; //Set C1 and C0 as output
	PORTC = PORTC & 0xF3; //Clear C1 and C2
	PORTC = PORTC | (State << 2); //Set C1 and C0 appropriately
	// Serial.println(PORTC); //DEBUG!
}

void Okapi::i2cState(bool State)
{
	digitalWrite(I2C_SW, State);
	// uint8_t PortVal = PORTC; //Read status  //FIX??
	// PortVal = PortVal & 0xDF; //Clear C5
	// PortVal = PortVal | (State << 5); //Set C5 with appropriate value
	// PORTC = PortVal; //Set port
}

uint8_t Okapi::chipFaults()
{
	uint8_t f = 0;
	if(_sdCardMissing || _sdTestFailed) f |= 0x01;
	if(_clockError) f |= 0x02;
	if(_bmeError) f |= 0x04;
	if(_sensorError) f |= 0x08;
	return f;
}

// Okapi's reading of itself, per the NW-Device-Specification Okapi appendix
// (hypothetical as of 2026-09-23). Block 1, power, waits on the power model
// (which of VBeta and VPrime is the LiPo, the solar scaling) and stays zero.
void Okapi::fillPages()
{
	_pages.beginReading();
	if(!_bmeError) {
		_pages.put16(0x50, (uint16_t)(int16_t)(bme280.getTemperature() * 100.0));
		_pages.put16(0x52, (uint16_t)(bme280.getHumidity() * 100.0));
		_pages.put32(0x54, (uint32_t)(bme280.getPressure() * 100.0));
	}
	_pages.put32(0x58, clockUnix()); //Clock: Unix seconds
	_pages.put16(0x5C, (uint16_t)(int16_t)(_rtc.getTemp() * 100.0));
	_pages.put16(0x60, getExtIntCount(false));
	_pages.put16(0x62, _fileNum);
	_pages.put32(0x64, _logInterval);
	_pages.endReading(chipFaults());
}

static const char* const okapiChips[] = {"SDCard", "Clock", "BME280", "SensorBus", "Charger", "Backup"};
static const char* const okapiWords[] = {"LoggingStarted", "NewLogFile", "RowNotWritten"};   //unit kinds 16-18
static const char* const okapiChipWords[] = {"ClockSet"};   //kind 16 on Clock (0x30)

size_t Okapi::printStatus(Print& out, bool boot)
{
	const NW_Report& r = boot ? _bootReport : _pages.report();
	const char* const* words = okapiWords; uint8_t n = 3;
	if(r.chip() == 1) { words = okapiChipWords; n = 1; }
	return _pages.printSnapshot(out, okapiChips, 6, libVersion, &r, words, n, OKAPI_LIBRARY_COMMIT, "", SKETCH_COMMIT); //A logger: its library is its firmware; the sketch stands where a library would
}

void Okapi::sleepNow()         // here we put the arduino to sleep
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
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
	cbi(ADCSRA,ADEN);
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
	ADCSRA = 135; //DEBUG!
	Serial.begin(38400);
	// digitalWrite(VSwitch_Pin, HIGH);  //DEBUG!
	// pinMode(SD_CS, OUTPUT); //Disconnect SD chip slect pin
}

void Okapi::turnOffSDcard()
{
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
	powerAux(OFF); //Turn off power
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

void Okapi::turnOnSDcard()
{
	// pinMode(SD_CS, OUTPUT);
	// SPI.begin();
	// sd.begin(SD_CS);
	// DDRB |= ((1<<DDB5));
	// digitalWrite(SD_CS, HIGH);
	// digitalWrite(Ext3v3Ctrl, HIGH);  //turn off external 3v3 rail
	// digitalWrite(BatSwitch, HIGH); //Turn off battery connection to sense divider
	_powerState = powerAuto(); //Fix??
	// PowerOB(ON); //Turn on battery connection to sense divider
	// powerAux(ON); //turn on external 3v3 rail
	delay(6);                                            // let the card settle
	// some cards will fail on power-up unless SS is pulled up  ( &  D0/MISO as well? )
	// DDRC = DDRC | ((1<<DDC0) | (1<<DDC1));
	// DDRB = DDRB | (1<<DDB7) | (1<<DDB5) | (1<<DDB4); // set SCLK(D13), MOSI(D11) & SS(D10) as OUTPUT
	// Note: | is an OR operation so  the other pins stay as they were.                (MISO stays as INPUT)
	// PORTB = PORTB & ~(1<<DDB7);  // disable pin 13 SCLK pull-up – leave pull-up in place on the other 3 lines
	power_spi_enable();                      // enable the SPI clock
	SPCR=keep_SPCR;                          // enable SPI peripheral
	// delay(20);
	// digitalWrite(BatRailCtrl, LOW);
	// digitalWrite(Ext3v3Ctrl, LOW); //MODEL <= v1
	delay(10);
	// digitalWrite(3, HIGH); //DEBUG!
	_sd.begin(SD_CS, SD_SCK_MHZ(8));
	// digitalWrite(3, LOW); //DEBUG!
}
