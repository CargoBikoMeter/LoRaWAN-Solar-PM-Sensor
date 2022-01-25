#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <ArduinoLog.h>
//#include <Wire.h>

#include "SdsDustSensor.h" // https://github.com/lewapek/sds-dust-sensors-arduino-library
// Heltec Cubecell HTCC AB01: https://heltec.org/project/htcc-ab01/
#include <softSerial.h> // for SDS011 sensor
#include "CubeCell_NeoPixel.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "device_config.h"

#define _LORA_APP_DATA_SIZE 12
#define _HAS_RGB 1

uint8_t LEDR = 0, LEDG = 0, LEDB = 0;
bool LEDON = false;
bool DRAIN_BATTERY = false;
bool PIXELS_INITIALIZED = false;
CubeCell_NeoPixel pixels(1, RGB, NEO_GRB + NEO_KHZ800);

bool MEASUREMENT_FINISHED = false;
bool DO_MEASUREMENT = true;
bool REMOTE_NO_MEASUREMENT = false;
uint16_t BATTERY_VOLTAGE = 0;
uint16_t LAST_BATTERY_VOLTAGE = 0;
uint16_t MEASURE_SHUTDOWN_VOLTAGE = _MEASURE_SHUTDOWN_VOLTAGE; // system shutdown below 3.5 Volt
uint16_t MEASURE_RESTART_VOLTAGE  = _MEASURE_RESTART_VOLTAGE;  // no measurement below 3.6 Volt
uint32_t DEEP_SLEEP_TIME = _DEEP_SLEEP_TIME;
uint32_t DEEP_SLEEP_CYCLE = _DEEP_SLEEP_CYCLE;


/************** sub functions *******************/

// switch external power ON
void VextON()
{
  Log.notice(F("VextON"));
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
  delay(500);
}

// switch external power OFF
void VextOFF() //Vext default OFF
{
  Log.notice(F("VextOFF"));
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
  delay(500);
}

void print_timer_values() {
  Log.verbose(F("DEEP_SLEEP_CYCLE = %d"),DEEP_SLEEP_CYCLE);
  Log.verbose(F("DEEP_SLEEP_TIME = %d"),DEEP_SLEEP_TIME);
  Log.verbose(F("MEASURE_SHUTDOWN_VOLTAGE = %d"),MEASURE_SHUTDOWN_VOLTAGE);
  Log.verbose(F("MEASURE_RESTART_VOLTAGE = %d"),MEASURE_RESTART_VOLTAGE);
  Log.verbose(F("Duty cycle: %d s"),int(appTxDutyCycle / 1000));
}

void set_led(uint8_t r, uint8_t g, uint8_t b) {
  Log.notice(F("set_led"));
  // switch on power
#if _HAS_RGB
  VextON();
  
  if (!PIXELS_INITIALIZED) {
    pixels.begin();
    PIXELS_INITIALIZED = true;
  }

  if (r == 0 && g == 0 && b == 0) {
    pixels.clear();
    pixels.show();
  } else {
    pixels.setPixelColor(0, pixels.Color(r,g,b));
    pixels.show();
    // delay(10*1000);
  }
#endif
}


// *****   BEGIN sensor stuff   *****

void reset_measurement_data() {
  Log.notice(F("reset_measurement_data - reset appData fields"));
  // initialize the entire appData field, even if not used later
  appData[0] = 0;  // BATTERY_VOLTAGE
  appData[1] = 0;  // BATTERY_VOLTAGE
  appData[2] = 0;  // temperature
  appData[3] = 0;  // temperature
  appData[4] = 0;  // humidity
  appData[5] = 0;  // humidity
  appData[6] = 0;  // pressure
  appData[7] = 0;  // pressure
  appData[8] = 0;  // p25
  appData[9] = 0;  // p25
  appData[10] = 0; // p10
  appData[11] = 0; // p10
}

/********************************************/
/* battery check and deep sleep mode switch */
/********************************************/

void set_measurement_status(bool on) {
 Log.notice(F("set_measurement_status"));
 if (on) {
   if (!DO_MEASUREMENT) {
     DO_MEASUREMENT=true;
     Log.verbose(F("DO_MEASUREMENT set to true: %d"),DO_MEASUREMENT);
     set_led(0,0,0);
   } else {
     Log.verbose(F("DO_MEASUREMENT already set to true: %d"),DO_MEASUREMENT);
   }
 } else {
   if (DO_MEASUREMENT) {
     DO_MEASUREMENT=false;
     Log.verbose(F("DO_MEASUREMENT set to false: %d"),DO_MEASUREMENT);
   } else {
     Log.verbose(F("DO_MEASUREMENT already set to false: %d"),DO_MEASUREMENT);
   }
 }
}

void check_battery() {
  // get first time battery level for sending via LoRaWAN initialization
  Log.notice(F("check_battery"));

  BATTERY_VOLTAGE = getBatteryVoltage();
  Log.verbose(F("BATTERY_VOLTAGE in millivolt: %d"),BATTERY_VOLTAGE);

  if ( DO_MEASUREMENT && BATTERY_VOLTAGE <= MEASURE_SHUTDOWN_VOLTAGE ) {
    set_measurement_status(false);
    Log.error(F("battery level to low, DO_MEASUREMENT: %d"),DO_MEASUREMENT);
    LAST_BATTERY_VOLTAGE = BATTERY_VOLTAGE;
  }

  if ( !DO_MEASUREMENT && BATTERY_VOLTAGE > MEASURE_RESTART_VOLTAGE ) {
    set_measurement_status(true);
    Log.verbose(F("battery level good, DO_MEASUREMENT: %d"),DO_MEASUREMENT);
  }

  if (!DO_MEASUREMENT) {
    if ( BATTERY_VOLTAGE < LAST_BATTERY_VOLTAGE) {
      appTxDutyCycle += DEEP_SLEEP_TIME;
    }
    if ( BATTERY_VOLTAGE > LAST_BATTERY_VOLTAGE) {
      appTxDutyCycle = DEEP_SLEEP_TIME;
    }
    LAST_BATTERY_VOLTAGE = BATTERY_VOLTAGE;
    set_led(0,0,0);
  } else {
    // reset appTxDutyCycle to default cycle time
    appTxDutyCycle = DEEP_SLEEP_CYCLE;
  }
  
  // prepare LoRaWAN TX buffer
  appData[0] = BATTERY_VOLTAGE;
  appData[1] = BATTERY_VOLTAGE>>8;
  
}

/*****************/
/* BME280 sensor */
/*****************/
// Adafruit BME280 settings
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
bool bme_status;

void init_BME280() {
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  Log.notice(F("init_BME280"));
  
  bme_status = bme.begin(0x76);  
  if (!bme_status) {
    Log.error(F("Could not find a valid BME280 sensor, check wiring!"));
  }
  delay(500);
}

void get_BME280() {
  Log.notice(F("get_BME280"));
  int16_t temperature = 0; 
  int16_t humidity = 0; 
  int16_t pressure = 0;

  if ( bme_status ) {
  temperature = bme.readTemperature() * 100;
  Log.verbose(F("Temperature: %d"),temperature);
      
  // Convert temperature to Fahrenheit
  //Log.verbose(F("Temperature in Fahrenheit: %d"), (1.8 * bme.readTemperature() + 32));
    
  pressure = bme.readPressure() / 100.0F;
  Log.verbose(F("Pressure: %d"),pressure);

  humidity = bme.readHumidity() * 100;
  Log.verbose(F("Humidity: %d"),humidity);
  
  appData[2] = temperature;
  appData[3] = temperature >> 8;
  appData[4] = humidity;
  appData[5] = humidity >> 8;
  appData[6] = pressure;
  appData[7] = pressure >> 8;
  }
}

/*****************/
/* SDS011 sensor */
/*****************/
#define FAN_SPINUP 30000 // fan 'clean' time before measurements in milliseconds; 30000 (default)

// softSerial based communication for Heltec cubecell boards
softSerial softwareSerial(_BoardTxPin,_BoardRxPin); // sds(_BoardTxPin->SDS011-RXD, _BoardRxPin->SDS011-TXD)
SdsDustSensor  sds(softwareSerial);  

bool SDS_OK = false; // false: not OK; true: OK (no communication error)

void init_SDS011(){
  String FIRMWARE_VERSION;

  pinMode(_BoardRxPin,INPUT_PULLDOWN);
  // SDS011 init
  Log.notice(F("init_SDS011"));

  Log.notice(F(" initialize sensor"));
  sds.begin();

  Log.notice(F(" wakeup sensor"));
  sds.wakeup();

  if ( sds.setQueryReportingMode().toString().compareTo("Mode: query") == 0 ) {
    SDS_OK = true;
    Log.verbose(F(" SDS_OK: %d"),SDS_OK);
    Log.verbose(F(" query firmware version ..."));
    if ( _DEBUG) {
      Serial.println(sds.queryFirmwareVersion().toString());
    }
  } else {
    SDS_OK = false;
    Log.error(F(" ERROR: SDS_OK: %d"),SDS_OK);
  }
}  

void get_SDS011() {
  String PM_STATUS;

  if ( SDS_OK) {
    Log.notice(F("get_SDS011"));
    Log.verbose(F(" waiting for fan cleaning in %d seconds"),FAN_SPINUP/1000);
    delay(FAN_SPINUP);

    // get SDS011 sensor data
    Log.notice(F(" getting PM25 an PM10 data"));
    PmResult pm = sds.queryPm();  // DON'T USE function sds.readPm() !!!
        
    if (pm.isOk()) {
      Log.notice(F(" got PM25 and PM10 data, now  prepare appData fields"));
      PM_STATUS = pm.statusToString();

      Log.verbose(F(" PM status ..."));
      if ( _DEBUG) {
        Serial.print(PM_STATUS);
        Serial.println("");
      }

      uint16_t tmp = (uint16_t)(pm.pm25 * 100);
      Log.verbose(F(" PM25 assigned to tmp for TTN appData[8,9]:: %d"),tmp);
  
      appData[8] = tmp;
      appData[9] = tmp>>8;
      tmp = (uint16_t)(pm.pm10 * 100);
      Log.verbose(F(" PM10 assigned to tmp for TTN appData[10,11]:: %d"),tmp);
      appData[10] = tmp;
      appData[11] = tmp>>8; 
    }

    // during SDS011 reading voltage on Vext with step-up modul is to low
    // for BME280 sensor (2.8 Volt, after sleep 3.2 Volt)
    Log.notice(F(" set SDS011 sensor to sleep"));
    sds.sleep();
  }
  Log.verbose(F(" exit function get_SDS011"));
}

void measure() {
  Log.notice(F("measure"));
  // reset and get measurement data
  reset_measurement_data();
  check_battery();

  if ( DO_MEASUREMENT && ! REMOTE_NO_MEASUREMENT ) {
    // initialize and read PM data from SDS011
    // switch power ON for SDS011 and BME280 sensor
    VextON();
   
    init_SDS011();
    get_SDS011();
  
    // Init I2C and serial communication
    // Wire.end() required, see: http://community.heltec.cn/t/cubecell-vext-with-i2c-sensors/886
    // ensures that the I2C work correctly, maybe a problem of the Wire library
    // without that, the BME280 initialization fails in most cases

    Wire.end();
    Wire.begin(); 
    // measure temperature, humidity and pressure
    init_BME280();
    get_BME280();
    Wire.end();

    // switch OFF Vext to save power during sleep period
    if ( !DRAIN_BATTERY ) {
      detachInterrupt(_BoardRxPin); // prevents system hang after switching power off
      VextOFF();
    }
  } else {
    Log.verbose(F(" Measurement disabled by: DO_MEASUREMENT=%d AND REMOTE_NO_MEASUREMENT=%d"),
    DO_MEASUREMENT,REMOTE_NO_MEASUREMENT);
  }
}

// *****   END sensor stuff   ***** 


/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = DEEP_SLEEP_CYCLE;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = _TTN_APP_PORT;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
  Log.notice(F("prepareTxFrame"));

	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/

  appDataSize = _LORA_APP_DATA_SIZE;
      
  measure(); // perform all measurement functions

  Log.notice(F("prepareTxFrame finished"));
}

void  setup_check_battery() {
  // get first time battery level 
  Log.notice(F("Logmessage notice setup_check_battery"));
  BATTERY_VOLTAGE = getBatteryVoltage();
  Log.verbose(F("BATTERY_VOLTAGE in millivolt:: %d"),BATTERY_VOLTAGE);

  if ( BATTERY_VOLTAGE <= MEASURE_SHUTDOWN_VOLTAGE ) {
    Log.error(F("ERROR: battery level to low for measurement"));
    set_measurement_status(false);
  }
}

// Logging helper routines
void printTimestamp(Print* _logOutput, int logLevel) {
  static char c[12];
  // sprintf(c, "%l ", TimerGetCurrentTime());
  sprintf(c, "%d ", millis());
  _logOutput->print(c);
}

void printNewline(Print* _logOutput, int logLevel) {
  _logOutput->print('\n');
}

void setup_logging() {
  Log.verbose(F("setup(): Logging started"));
  Log.begin(LOGLEVEL, &Serial);
  Log.setPrefix(printTimestamp);
  Log.setSuffix(printNewline);
  Log.verbose("Logging has started");
}

void setup() {
  boardInitMcu();
  if (_DEBUG) {
	  Serial.begin(115200);
    delay(1000);
  }

  setup_logging();
  
  setup_check_battery();

#if(AT_SUPPORT)
	enableAt();
#endif
	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}

// -------------- Command Processing -----------------
void process_system_power_command(unsigned char len, unsigned char *buffer) {
  // downlink payload command: 00 01 AND switch case number
  if (len == 0) {
    Log.error(F("Zero length power command"));
  } else {
    Log.verbose(F("Processing power command"));
  }

  switch (buffer[0]) {
    case 0x01:
      REMOTE_NO_MEASUREMENT = true;
      Log.verbose(F("REMOTE_NO_MEASUREMENT: ON"));
      break;
    case 0x02:
      REMOTE_NO_MEASUREMENT = false;
      Log.verbose(F("REMOTE_NO_MEASUREMENT: OFF"));
      break;
    case 0x03:
      DRAIN_BATTERY = true;
      Log.verbose(F("DRAIN_BATTERY: ON"));
      break;
    case 0x04:
      DRAIN_BATTERY = false;
      Log.verbose(F("DRAIN_BATTERY: OFF"));
      VextOFF();
      break;
    default:
      Log.error(F("Unknown power command %d"),buffer[0]);
      break;
  }
}

void set_default_timers() {
  DEEP_SLEEP_CYCLE = _DEEP_SLEEP_CYCLE;
  DEEP_SLEEP_TIME  = _DEEP_SLEEP_TIME;
  MEASURE_RESTART_VOLTAGE = _MEASURE_RESTART_VOLTAGE;
  MEASURE_SHUTDOWN_VOLTAGE = _MEASURE_SHUTDOWN_VOLTAGE;
}

void process_system_delay_command(unsigned char len, unsigned char *buffer) {
  // downlink payload command: 00 02 AND switch case number
  if (len != 1) {
    Log.error(F("Len of delay command != 1"));
  } else {
    Log.verbose(F("Processing delay command"));
  }

  if (buffer[0] != 0) {
     DEEP_SLEEP_CYCLE = buffer[0] * 1000 * 5;
     appTxDutyCycle = DEEP_SLEEP_CYCLE;
    Log.verbose(F("appTxDutyCycle %d seconds"), appTxDutyCycle / 1000);
  }
}

void process_system_led_command(unsigned char len, unsigned char *buffer) {
  // downlink payload command: 00 03 AND switch case number
  if (len == 0) {
    Log.error(F("Zero length LED command"));
    return;
  } else {
    Log.verbose(F("Processing LED command"));
  }

  switch (buffer[0]) {
    case 0x00:
      LEDR = 0;
      LEDG = 0;
      LEDB = 0;
      set_led(0,0,0);
      break;
    case 0x01:
      LEDR = 255;
      LEDG = 255;
      LEDB = 255;
      set_led(255,255,255);
      break;
#if _HAS_RGB
    case 0x02:
      if (len == 4) {
        // do rgb magic
        LEDR = buffer[1];
        LEDG = buffer[2];
        LEDB = buffer[3];
        set_led(LEDR,LEDG,LEDB);
      } else {
        Log.error(F("Missing RGB values for LED. Len = %d"),len);
      }
      break;
#endif
    default:
      Log.error(F("Unknown LED command %d"), buffer[0]);
      break;
  }
}

void process_system_timer_command(unsigned char len, unsigned char *buffer) {
  // downlink payload command: 00 04 AND switch case number
  if (len <= 1) {
    Log.error(F("Len of timer command <= 1"));
  } else {
    Log.verbose(F("Processing timer command"));
  }

  switch(buffer[0]){
    case 0x00:
      set_default_timers();
      break;
    case 0x01:
      DEEP_SLEEP_CYCLE = 60*1000*buffer[1];
      appTxDutyCycle = DEEP_SLEEP_CYCLE;
      break;
    case 0x02:
      DEEP_SLEEP_TIME = 60*1000*buffer[1];
      break;
    case 0x03:
      MEASURE_SHUTDOWN_VOLTAGE = 100 * buffer[1];
      break;
    case 0x04:
      MEASURE_RESTART_VOLTAGE = 100 * buffer[1];
      break;
    case 0xff:
      if (len == 4) {
        DEEP_SLEEP_CYCLE = 60 * 1000 * buffer[1];
        DEEP_SLEEP_TIME  = 60 * 1000 * buffer[2];
        MEASURE_SHUTDOWN_VOLTAGE = 100 * buffer[3];
        MEASURE_RESTART_VOLTAGE  = 100 * buffer[4];
      } else {
        Log.error(F("Only %d arguments, needed 4"),len);
      }
      break;
    default:
      Log.error(F("Unknown timer command %X"),buffer[0]);
  }
  // sanity Check
  if (
        MEASURE_SHUTDOWN_VOLTAGE > MEASURE_RESTART_VOLTAGE) {
      Log.error(F("Sanity check failed, loading defaults"));
      set_default_timers();
    }
  print_timer_values();
}

void process_system_command(unsigned char len, unsigned char *buffer) {
  // downlink payload command: 00 AND switch case number
  if (len == 0) {
    Log.error(F("Zero length system command"));
    return;
  } else {
    Log.verbose(F("Processing system command"));
  }
  switch (buffer[0]) {
    case 0x01:
      process_system_power_command(len-1,buffer+1);
      break;
    case 0x02:
      process_system_delay_command(len-1,buffer+1);
      break;
    case 0x03:
      process_system_led_command(len-1,buffer+1);
      break;
    case 0x04:
      process_system_timer_command(len-1,buffer+1);
      break;
    case 0xff:
      // Reboots
      Log.notice(F("Executing reboot command"));
      delay(100);
      HW_Reset(0);
    default:
      Log.error(F("Unknown system command %d"),buffer[0]);
      break;
  }
}

void process_sensor_bme280(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length bme280 command"));
    return;
  }

}

void process_sensor_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length sensor command"));
    return;
  }
  switch (buffer[0]) {
    case 0x11:
      process_sensor_bme280(len-1,buffer+1);
      break;
    default:
      Log.error(F("Unknown sensor command %d"),buffer[0]);
      break;
  }
}

void process_received_lora(unsigned char len, unsigned char *buffer) {
  if (len == 0)
    return;

  Log.verbose(F("Processing %d bytes of received data"),len);
  switch (buffer[0]) {
    case 0:
      process_system_command(len-1,buffer+1);
      break;
    case 1:
      process_sensor_command(len-1,buffer+1);
      break;
    default:
      Log.error(F("Unknown command %d"),buffer[0]);
      break;
  }
}

void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Log.verbose(F("+REV DATA:%s,RXSIZE %d,PORT %d\r\n"),
    mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",
    mcpsIndication->BufferSize,
    mcpsIndication->Port);
  process_received_lora(mcpsIndication->BufferSize, mcpsIndication->Buffer);
}

void loop()
{
  //Log.verbose(F("loop entry point: deviceState: %d"),deviceState);
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
#if(LORAWAN_DEVEUI_AUTO)
    LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
      //Log.verbose(F("getDevParam ..."));
      getDevParam();
#endif
      //Log.verbose(F("printDevParam ..."));
      printDevParam();

      LoRaWAN.init(loraWanClass,loraWanRegion);

      // set data rate and spreading factor
      // source: https://www.thethingsnetwork.org/forum/t/example-ttn-code-for-heltec-htcc-ab02a-with-gps-in-us915/46093/6
      /* NOTE: setDataRateForNoADR
       *  
       *  #ifdef REGION_US915
       *  int8_t defaultDrForNoAdr = 1;
       * 
       *  0 LoRa: SF9 / 125 kHz
       *  1 LoRa: SF9 / 125 kHz 1760 1 28 dBm
       *  2 LoRa: SF8 / 125 kHz 3125 2 26 dBm
       *  3 LoRa: SF7 / 125 kHz 5470 3 : 9 ….
       *  4 LoRa: SF7 / 125 kHz
       *  
       */
      LoRaWAN.setDataRateForNoADR(1);
      
      deviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      prepareTxFrame( appPort );
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
      Log.verbose(F("DEVICE_STATE_CYCLE: appTxDutyCycle: %d"), int(appTxDutyCycle/1000));
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      Log.verbose(F("going into deep sleep"));
      LoRaWAN.sleep();
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}