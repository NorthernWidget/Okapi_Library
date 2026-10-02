#ifndef Okapi_h
#define Okapi_h


#include <Arduino.h>
#include <Wire.h>
#include <NW_Logger.h>   // the logger core that Okapi and Margay share (NW_Sensor and NW_Pages through it)
#include <Adafruit_ADS1015.h> //Include ADC interface
#include <MCP4725.h>  //Include custom DAC library
#include <MCP23018.h>

// Build identity: this library's version (held equal to library.properties by
// NW-Tests/version_check.py) and its build commit, set by the NW-Build wrapper from
// git and blank in an Arduino IDE build; the sketch's commit the same way.
#define OKAPI_LIBRARY_VERSION "0.7.0"
#ifndef OKAPI_LIBRARY_COMMIT
#define OKAPI_LIBRARY_COMMIT ""
#endif
#ifndef SKETCH_COMMIT
#define SKETCH_COMMIT ""
#endif

#define VPRIME 1
#define VBETA 2


#define INTERNAL 0
#define EXTERNAL 1

#define MODEL_1v0
#define MODEL_0v0

enum board
{
    Model_0v0 = 0,
    Model_1v0 = 1,
    Model_2v0 = 2
};

enum build
{
	Build_A = 0,
	Build_B = 1,
	Build_C = 2
};

enum temp_val
{
	Therm_Val = 0,
	RTC_Val = 1
};

/**
 * @brief The Okapi data logger: what is Okapi's about it, on the NW_Logger core.
 * @details Everything a logger does (the card layout, the data and status
 * files, run(), the interrupts, the self-tests, the LED, the serial number
 * from Page 0) comes from NW_Logger. Okapi adds its two power rails with
 * their arbitration, its on-board ADS1115 pair and MCP4725 DAC, the MCP23018
 * port expander that switches the Feather, and the backhaul over Serial
 * after every LogCountPush rows. Okapi is a Schema 1 device like Margay: its
 * Pages 0 from EEPROM, Pages 2 and 3 a reading of itself (the appendix's
 * Block 1, power, waits on the power model and stays zero).
 *
 * Names are camelCase as of 2026-09-23; the PascalCase names stay as
 * deprecated forwarders for one release.
 */
class Okapi : public NW_Logger
{

	public:
    /**
     * @brief
     * Instantiate the Okapi data-logger class
     *
     * @param[in] Model_: Not currently used for Okapi
     *
     * @param[in] Specs_: Defines the number of on-board I2C device addresses.
     *                      **Build_A**: **DEFAULT** 6 on-board I2C devices.
     *                      **Build_B**: 0 on-board I2C devices.
     *
    */
		Okapi(board Model_ = Model_0v0, build Specs_ = Build_A); //Use Build_A by default
    /**
     * @brief Begin with a list of attached I2C devices
     *
     * @param[in] *Vals: List of I2C addresses for external sensors
     * @param[in] NumVals: The length of the *Vals list
     * @param[in] Header_: A header string for the data file
     * @return true when the self-tests found nothing wrong
     */
		bool begin(uint8_t *Vals, uint8_t NumVals, String header_) override;
		using NW_Logger::begin; ///< begin(header) with no external sensors
    /**
     * @brief Read one row back from the data file, for the backhaul
     *
     * @param[in] LineIndex: The desired line number, counted from DataIndex
     * @param[in] DataIndex: The byte at which to start in the file
     */
		String readStr(uint8_t LineIndex, uint32_t DataIndex);
    /**
     * @brief Read voltage from the external 16-bit ADC
     * @param[in] Pin (range 0-3) -- which pin to read?
     */
		float getVoltage(uint8_t Pin); //Read ADC
    /**
     * @brief Set voltage on the 12-bit digital-to-analog converter
     * @param[in] Val: 0 to 4095, which scales from 0 to Vbus
     * @param[in] Gain: GAIN_1X or GAIN_2X
     */
		uint8_t setVoltageRaw(uint16_t Val, bool Gain = GAIN_1X); //Set DAC with raw input, default to 1x gain
    /**
     * @brief Set voltage on the 12-bit digital-to-analog converter
     * @param[in] Val: Desired voltage value, in volts.
     */
		uint8_t setVoltage(float Val); //Set DAC to nearest interpolated value
    /**
     * @brief Writes date/time, on-board sensors, and external string to SD
     * @param *Update: External update function from Arduino sketch
     */
		void addDataPoint(String (*Update)(void)) override;
    /**
     * @brief Obtain date/time, P/T/RH, temperatures, and voltages from board
     */
		/** @brief Read the on-board channels into _rtcTemp, _vBetaMv, _vPrimeMv, _iSolarMa and _iBetaMa (and the timestamp). */
		void readOnBoard() override;

		/** @brief Print the logger's own data columns: the timestamp and the on-board sensors. */
		size_t printDataHeader(Print& out) override;

		/** @brief Print those columns' values, as readOnBoard() left them. Takes no reading. */
		size_t printDataRow(Print& out) override;
    /**
     * @brief Determine which input has power and set up power path from that
     */
		uint8_t powerAuto();
    /**
     * @brief Power from Main, backup, or off
     * @param[in] State: 0 or 3 = OFF, 1 = V_Prime, 2 = V_Beta
     */
		void powerAux(uint8_t State);
    /**
     * @brief Use the on-board (INTERNAL) or external (EXTERNAL) I2C bus
     */
		void i2cState(bool State);

		// --- NW_Sensor: Okapi is a Schema 1 device and watches itself ---
		const char* name() const override { return "Okapi"; }
		size_t printStatus(Print& out, bool boot = false) override;

		// --- PascalCase names, deprecated 2026-09-23: forwarders for one release ---
		[[deprecated("Use logStr()")]] int LogStr(String Val) { return logStr(Val); }
		[[deprecated("Use readStr()")]] String ReadStr(uint8_t LineIndex, uint32_t DataIndex) { return readStr(LineIndex, DataIndex); }
		[[deprecated("Use run()")]] void Run(String (*Update)(void), unsigned long _logInterval) { run(Update, _logInterval); }
		[[deprecated("Use getVoltage()")]] float GetVoltage(uint8_t Pin) { return getVoltage(Pin); }
		[[deprecated("Use setVoltageRaw()")]] uint8_t SetVoltageRaw(uint16_t Val, bool Gain = GAIN_1X) { return setVoltageRaw(Val, Gain); }
		[[deprecated("Use setVoltage()")]] uint8_t SetVoltage(float Val) { return setVoltage(Val); }
		[[deprecated("Use addDataPoint()")]] void AddDataPoint(String (*Update)(void)) { addDataPoint(Update); }
		[[deprecated("Use getOnBoardVals()")]] String GetOnBoardVals() { return getOnBoardVals(); }
		[[deprecated("Use initLogFile()")]] void InitLogFile() { initLogFile(); }
		[[deprecated("Use resetWDT()")]] void ResetWD() { resetWDT(); }
		[[deprecated("Use powerAuto()")]] uint8_t PowerAuto() { return powerAuto(); }
		[[deprecated("Use powerAux()")]] void PowerAux(uint8_t State) { powerAux(State); }
		[[deprecated("Use i2cState()")]] void I2CState(bool State) { i2cState(State); }

    /// USART Transmit
		uint8_t TX = 11; //ADD TO DOCUMENTATION!
    /// USART Receive
		uint8_t RX = 10; //ADD TO DOCUMENTATION!
    /// BSCHULZ1701: WHAT ARE THESE??
		uint8_t C0 = 18;
    /// BSCHULZ1701: WHAT ARE THESE??
		uint8_t C1 = 19;
    /// Primary bus switch pin (BSCHULZ1701: is this in addition to the physical switch?)
		uint8_t Sw_Bus_Prime = 23;
    /// Secondary bus switch pin (BSCHULZ1701: is this in addition to the physical switch?)
		uint8_t Sw_Bus_Sec = 22;
    /// IO Exp PORT B (BSCHULZ1701: what is this in plain English / purpose?)
		uint8_t PG_3v3_Core = 1;
    /// IO Exp PORT B (BSCHULZ1701: turn Feather on if True, I guess?)
		uint8_t FeatherEN = 7;
    /// GPIO pin D0 **Arduino Pin 12**
		uint8_t D0 = 12;
    /// GPIO pin D1 **Arduino Pin 25**
		uint8_t D1 = 25;
    /// GPIO pin D2 **Arduino Pin 3**
		uint8_t D2 = 3;
    /// GPIO pin D3 **Arduino Pin 26**
		uint8_t D3 = 26;
    /// Switch the Analog-Digital Converter on (true) or off (false) **Pin 0**
		uint8_t ADC_Sense_SW = 0;
    /// Feather pin: should this be public or private?
		uint8_t FeatherRTS = 31;
    /// Feather pin: should this be public or private?
		uint8_t FeatherCTS = 30;
    /// Feather pin: should this be public or private?
		uint8_t FeatherGPIO = 29;
    /// Feather pin: should this be public or private?
		uint8_t CS_Ext = 24;
    /// WHAT IS THIS?
		uint8_t GlobalInt = 28;
    /// Okapi data logger library version
		float _rtcTemp = NW_ERROR;    ///< RTC die temperature from the last readOnBoard() [C]
		float _vBetaMv = NW_ERROR;    ///< Beta rail voltage from the last readOnBoard() [mV]
		float _vPrimeMv = NW_ERROR;   ///< Prime rail voltage from the last readOnBoard() [mV]
		float _iSolarMa = NW_ERROR;   ///< Solar current from the last readOnBoard() [mA]
		float _iBetaMa = NW_ERROR;    ///< Beta current from the last readOnBoard() [mA]
		const String libVersion = OKAPI_LIBRARY_VERSION;

	protected:
		void sleepNow() override;
		void afterLogEvent() override; //The backhaul: after LogCountPush rows on main power, hand them to the Feather
		void turnOffSDcard();
		void turnOnSDcard();
		void enviroStats();
		uint8_t chipFaults();    // Okapi's chip-fault bits for Block 0: SDCard, Clock, BME280, SensorBus, Charger, Backup
		void fillPages();        // Page 2 and 3 from the logger's own readings, then endReading()

		MCP4725 _dac; //Instatiate DAC
		Adafruit_ADS1115 _adcOb; //Initialize on board (power moitoring) ADC
		Adafruit_ADS1115 _adcExt;  //Initialize external (sensor) ADC
		MCP23018 _io;

		float _powerState = 0; //Keep track of what power mode the system is using when waking from sleep

		board _model;
		build _specs;

		uint16_t _logCountPush = 5; //Number of logs to take before sending data off
		uint16_t _logCount = 0; //Number of logs since last data write
		uint16_t _index = 0; //Index of data entry USE???? FIX!
		uint32_t _lastSdIndex = 0; //Index as last data dump
};

#endif
