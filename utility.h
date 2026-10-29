/*
 * utility.h
 *
 *  Created on: Dec. 30, 2020
 *      Author: joet
 */

#ifndef UTILITY_H_
#define UTILITY_H_

//#include "mbedtls_config_autogen.h"
#include "config-device-acceleration.h"
#include "cryptoacc_management.h"
//#include "em_common.h"
#include "sha256_alt.h"
//#include "entropy_rail.h"
#include "sl_bt_api.h"
#include "app-config.h"
#include "string.h"

#define buffersize 80

#define DEVICE_SKU_PS 0x4000
#define DEVICE_REG_PS 0x4002                   // 0 = unregistered  1 = registered
#define DEVICE_MASTERKEY_PS 0x4001
#define DEVICE_TIMESTAMP_PS 0x4003
#define DEVICE_CONFIGURATION_PS 0x4004         // this is sensor threshold
//#define DEVICE_MODE_PS 0x4005
#define DEVICE_SITEKEY_PS 0x4006
#define DEVICE_INTIALIZED_PS 0x4007     // the device was previously intialized by app  0x77 = intialized
//#define DEVICE_INTIALGEN_PS  0x4008     // device never generated a random masterkey and no settings load defaults    0xA9 = generated/intial settings done
#define LAST_SET_TIME       0x4008
//#define TX_PWR  0

#define DEVICE_USEDKEY0_PS 0x4009
#define DEVICE_USEDKEY1_PS 0x400A
#define DEVICE_USEDKEY2_PS 0x400B
#define DEVICE_USEDKEY3_PS 0x400C
#define DEVICE_USEDKEY4_PS 0x400D
#define DEVICE_USEDKEY5_PS 0x400E
#define DEVICE_USEDKEY6_PS 0x400F
#define DEVICE_USEDKEY7_PS 0x4010
#define DEVICE_USEDKEY8_PS 0x4011
#define DEVICE_USEDKEY9_PS 0x4012
#define DEVICE_USEDKEY10_PS 0x4013
#define DEVICE_USEDKEY11_PS 0x4014
#define DEVICE_USEDKEY12_PS 0x4015
#define DEVICE_USEDKEY13_PS 0x4016
#define DEVICE_USEDKEY14_PS 0x4017
#define DEVICE_USEDKEY15_PS 0x4018
#define DEVICE_USEDKEY16_PS 0x4019
#define DEVICE_USEDKEY17_PS 0x401A
#define DEVICE_USEDKEY18_PS 0x401B
#define DEVICE_USEDKEY19_PS 0x401C


#define SETTINGS    0x4020
//#define ADV_PER     0x4020
//#define TX_PWR      0x4021
//#define SIG_STS     0x4022
//#define SENS_ON     0x4023
//#define SENS_OFF    0x4024
//#define BATT_MODE   0x4025
//#define DBG_MODE    0x4026
#define CUSTOM_FLAG 0x4021
#define UNIX_TIME   0x4022
#define UP_TIME     0x4023
#define PAYLOAD     0x4024

//LORA RELATED KEYS


#define BATT_SAMP         10
#define SAMP_AVG        3
#define SENS_SAMP        600


#define USERDATA  ((uint32_t *) USERDATA_BASE)
#define MANUFAC_DATE      0
#define HW_VERSION        1
#define DEVICE_EUI0				2
#define DEVICE_EUI1				3
#define DEVICE_EUI2				4
#define DEVICE_EUI3				5

#define APP_KEY0 				6
#define APP_KEY1 				7
#define APP_KEY2				8
#define APP_KEY3 				9
#define APP_KEY4 				10
#define APP_KEY5 				11
#define APP_KEY6 				12
#define APP_KEY7 				13

#define SERIAL0        14
#define SERIAL1        15
#define SERIAL2        16
#define SERIAL3        17
#define SERIAL4        18
#define SERIAL5        19
#define SERIAL6        20
#define SERIAL7        21


#define NEW_HW

#ifdef NEW_HW
#define MAGSENSE_PORT 1
#else
#define MAGSENSE_PORT 2
#endif
#define MAGSENSE_PIN  0



#define GEN2
#define LORA

//#define PASSTHROUGH


//#define ECOLAB_DEV



//Structure for sensor state
typedef enum {

    factorysleep = 0,   // no advertising  wait for interrupt to go to factorywake
    factorywake,        // fast advertising for 1 minute then go back to  factorysleep
    unregsleep,         // no advertising  wait for interrupt  to go to unregwake
    unregwake,          // fast advertising for 1 minute then go back to unregsleep
    registered,         // slow advertising security enabled
    development         // slow advertising no security

}deviceMODEs;
typedef struct {
  uint8_t data;
  uint8_t min;
  uint8_t hour;
  uint8_t flag;
}sens_time;

typedef struct {
  uint8_t lora_en;
  uint8_t rejoin;
  uint8_t region;
  uint8_t payload;
  uint8_t retry;
  uint8_t ack;
  uint8_t ack_fail;
  uint8_t tx_interval;
  uint8_t join_status;
  uint8_t lora_rssi_max;
  uint8_t lora_rssi_min;
  uint8_t lora_rssi_lvl;
  uint8_t lora_rssi_last;
  uint8_t lora_snr_max;
  uint8_t lora_snr_min;
  uint8_t lora_snr_lvl;
  uint8_t lora_snr_last;
  uint8_t devEUI[8];
  uint8_t appEUI[8];
  uint8_t appKEY[16];
  uint8_t data_rate;
  uint8_t sub_channel;
  uint8_t tx_pwr;
  uint8_t join_test;
  uint16_t join_attempts;
  uint16_t join_failed;
  uint8_t pkt_sent;
  uint8_t pkt_fail;
  uint8_t join_delay_flag;
  uint16_t join_delay_counter;
  uint16_t join_delay;
  uint8_t rejoin_attempt;
  uint8_t ping;
  uint8_t ping_result;
  uint8_t join_begin;
  uint8_t event_trigger;
  uint8_t network_delay;
  uint16_t network_delay_tm;
  uint8_t manual_join_delay;
  uint16_t manual_join_delay_tm;
  uint8_t initial_join;
  uint32_t heartbeat;
  uint8_t install_join;
  uint8_t unreg_flag;
  uint8_t lora_alive_flag;
  uint8_t lora_alive_status;
  uint8_t serial[16];
  uint8_t reg_payload_sent;
}lora_settings;


//Structure for storing device data
typedef struct {

    deviceMODEs operatingMode;
    uint32_t    wakeTimer;               // wake timer for factory mode
    uint8_t     production_mode;         // runs once after reset blocking sleep mode  activated only after power on reset
    uint32_t    production_mode_timer;   // timer for production mode
    uint32_t    sampleTime;              // battery and temperature sampling interval timer
    uint8_t     registration;            // registration status  0x01 = registered  0x00 = not registered
    uint16_t    advertising_period;      // current advertising period
    uint16_t    advertising_period_provisional;  // new advertising interval request subject to security verification
    int16_t     tx_pwr;
    uint8_t     sku;
    uint8_t     sku_tmp;
    uint8_t     detection_threshold;
    uint8_t     authSTATUS;
    sens_time     sensON;
    sens_time     sensOFF;
    uint16_t sensSAMP;
    uint8_t     sensFLAG;
    uint8_t     sigSTS;
    uint8_t     dbgMODE;
    uint8_t     battMODE;
    uint8_t     wakeFlag;
    uint8_t     fw_ver;
    lora_settings lora;

}deviceDATA;

//Structure for storing system data
typedef struct{

  uint32_t LocalTimeCounter;
  uint32_t advertiserate;
  uint32_t realtimecounter;
  uint32_t realtimedifference;
  uint32_t energyconsumed;
//  volatile double  batteryvolts;  //battery voltage as a float
//  uint8_t  batterylvl;
//  uint8_t  fuelguage;
  int8_t   temperaturecurrent;
//  uint16_t batt_read;   //battery voltage in increments of 100mV
//  uint32_t battSUM;
//  uint16_t battAVG;

}MTsystem_t;


//Structure for storing device configuration
typedef struct{

    uint8_t device_sku;
    deviceMODEs mode;

  }deviceConfiguration;

  //Structure for Battery Measurement data
  typedef struct {

  uint16_t battADCsamples[BATTERYSAMPLES];
  uint8_t  battADCindex;
  uint32_t battTIMER;
  uint8_t  battDISP;
  uint32_t battSUM;
  uint16_t battAVG;
  volatile double  batteryvolts;  //battery voltage as a float
  uint8_t  batterylvl;
//  uint8_t  fuelguage;
  uint16_t batt_read;   //battery voltage in increments of 100mV

  }battData;


  /*Structurex for Fuel Gauge Implementation (can be cusotmized for each application)
   * The structure will have a variable that will
   * indicate the total capacity of the battery (derated)
   * Next, variables will be added which represent the the various tasks that will
   * consume energy (MCU wake, advertisements, adc readings, etc) and each varialbe
   * will have an accumulating counter to count the total number of these tasks
   * that have been done
   * Each task will also have a value that dictates how much energy this task consumes
   * Each task will be a new fuelConsumer struct
   * The number of fuelConsumer structs will be dictated by the applicatino
   */

  typedef struct {

//  uint16_t battADCsamples[BATTERYSAMPLES];
//  uint8_t  battADCindex;
//  uint32_t battTIMER;
//  volatile double  batteryvolts;  //battery voltage as a float

    uint32_t counter;       //value that stores the total number of the particular task
    uint16_t consumption;   //value that stores the energy consumption of performing the task once

  }fuelConsumer;

  typedef struct {

//  uint16_t battADCsamples[BATTERYSAMPLES];
//  uint8_t  battADCindex;
//  uint32_t battTIMER;
//  volatile double  batteryvolts;  //battery voltage as a float

    uint32_t      total_capacity;   //value that stores the total battery capacity for calculations
    fuelConsumer  mcu_wake;
    fuelConsumer  ble_adv;
    fuelConsumer  temp_meas;
    fuelConsumer  batt_meas;
    fuelConsumer  ble_connection;
    fuelConsumer  mcu_bootup;
    fuelConsumer  sensor_act;
    fuelConsumer  sensor_mov;

  }fuelGauge;

  typedef struct {

//  uint16_t battADCsamples[BATTERYSAMPLES];
//  uint8_t  battADCindex;
//  uint32_t battTIMER;
//  volatile double  batteryvolts;  //battery voltage as a float

    uint8_t resetCounter;
    uint8_t resetTemp;
    uint8_t resetBatt;
    uint32_t localtimer;
  }resetData;

  typedef struct _last_set_time_info{
    uint8_t local[2];
    uint8_t time[8];
  }last_set_time_t;

typedef struct{
  uint32_t tm_sec;
  uint32_t tm_min;
  uint32_t tm_hour;
  uint32_t tm_mday;
  uint32_t tm_mon;
  uint32_t tm_year;
  uint32_t tm_wday;
  uint32_t tm_yday;
}timeINFO;

struct safe_c_str
{
    char *buf_p;
    uint8_t max_chars; // this includes room for the last '\0' character of a valid C-String
    uint8_t num_chars;
};
#define SAFE_C_STR_INIT(c_str, buf) safe_c_str_init(c_str, &buf[0], sizeof(buf))

void safe_c_str_init(struct safe_c_str *c_str_p, char *buf_p, uint8_t max_chars);
const char *safe_c_str_get(struct safe_c_str *c_str_p);
void safe_c_str_append(struct safe_c_str *c_str_p, const char *src_p, uint8_t num_chars);

#define MIN(a,b) (((a)<(b))?(a):(b))
//  uint32_t battSUM;
//  uint32_t battAVG;

uint8_t genRANDOM(void);
uint8_t AuthenticateRegisterUser(uint8array *writeValue, deviceDATA *device);
uint8_t AuthenticateSecureUser(uint8array *writeValue, deviceDATA *device);
uint8_t AuthenticateFactoryUser(uint8array *writeValue, deviceDATA *device);
uint8_t AuthenticateLockedUser(uint8array *writeValue, deviceDATA *device);
uint8_t AuthenticateInstallationUser(uint8array *writeValue, deviceDATA *device);
void ps_load(uint16_t key, uint8_t *pValue, uint8_t size);
void ps_save(uint16_t key, uint8_t *pValue, uint8_t size);
void checkHALL(void);
uint8_t checkKEY(void);
uint8_t NewKeyVerify(uint8array *writeValue);
uint8_t NewKeySave(void);
void KeyStorageInit(void);
//uint16_t summation(uint16_t *ptrArrNumbers, uint16_t *ptrSum, uint8_t pos, uint16_t nextNum);
void BATTcheck(void);
void BATTcheckinit(void);
void TempMeasure(void);
uint32_t BattMeasure(void);
void deviceLOADx(void);
void updateDevice(void);
void deviceClear(void);
void defaultsLOAD(void);
uint16_t measureBatt(void);
void initIADC (void);
//void measureBatt(void);
void configPINS(void);
void sendVER(void);
void sendMAC(void);
void sendBatt(uint32_t adcValue);
void sendSENS(uint8_t count);
void sendIO1(void);
void sendIO2(void);
void sendLOCAL();
void sendUNIX();
void sendDevEui(void);
void sendAppk(void);
void sendmfg(void);
void sendhwv(void);
void sendhb(void);
void load_devEUI(void);
void load_appKEY(void);
void char_send(uint8_t dum, uint8_t type);
void wakeDevice(void);
void Pin1IntHandler(uint8_t intNo);
void Pin2IntHandler(uint8_t intNo);
void Pin3IntHandler(uint8_t intNo);
void GPIOIntHandler(uint8_t intNo);
//GPIOINT_IrqCallbackPtr_t Pin3IntHandler(uint8_t intNo);
//GPIOINT_IrqCallbackPtr_t
void hex_to_ascii_byte(uint8_t hex_byte, uint8_t *ascii_val);
void sendsnr(void);
void sendrssi(void);
void sendLoraJoinState(void);
void sendSKU(void);
void sendLORAflag(void);
void sendLORAstatus(void);
void sendSerial(void);
void load_serial(void);
void sendAppEui(void);
void sendREGION(void);
void send_string(uint8_t *str);
//void Pin1IntHandler(void);
//void Pin2IntHandler(void);


#endif /* UTILITY_H_ */
