#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

/* OTAA parameters*/
// device: YOUR-DEVICE_NAME
uint8_t devEui[] = { TODO };
uint8_t appEui[] = { TODO };
uint8_t appKey[] = { TODO };

/* ABP para*/
uint8_t nwkSKey[] = { TODO };
uint8_t appSKey[] = { TODO };
uint32_t devAddr =  ( uint32_t )0xTODO;

#define _TTN_APP_PORT 1

/* define board pins */
// GPIO3: Cubecell AB01 RX - connect with SDS011 TXD
// GPIO2: Cubecell AB02 TX - connect with SDS011 RXD
#define _BoardRxPin GPIO3
#define _BoardTxPin GPIO2

//#define _DEEP_SLEEP_TIME 2*60*1000 // 2 minutes
#define _DEEP_SLEEP_TIME 120*60*1000 // 120 minutes
#define _DEEP_SLEEP_CYCLE  15*60*1000 // 15 minutes: 15*60*1000
//#define _DEEP_SLEEP_CYCLE  0.5*60*1000 // 30 seconds for DEBUG
#define _MEASURE_SHUTDOWN_VOLTAGE 3500 // 3.5 V
#define _MEASURE_RESTART_VOLTAGE 3600 // 3.6 V

#define _DEBUG 1 // 1: serial console on, 0: serial console off 

#define LOGLEVEL LOG_LEVEL_VERBOSE

#endif