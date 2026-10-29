/*
 * utility.c
 *
 *  Created on: Dec. 30, 2020
 *      Author: joet
 */


#include "utility.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "sl_bt_api.h"
#include "collection.h"
#include "swmodbeacon.h"
#include "em_iadc.h"
#include "tempdrv.h"
#include "uartdrv.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "em_wdog.h"
#include "mbedtls/entropy.h"
#include "mbedtls/entropy_poll.h"
#include "sl_uartdrv_instances.h"
#include "time.h"
#include "app.h"
#include "lora_api_internal_join.h"
#include "lora_xdot_radio.h"
//#include "stdlib.h"

//#include "global.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
//#define   CANNOTCHANGEN




#ifndef ECOLAB_DEV
#define SUPPRESS_TIME  (32768 * 1800)  // 30 minutes
#else
#define SUPPRESS_TIME  (32768 * 1)  // 30 minutes
#endif


#define MAXUSEDKEYS 20

// Set CLK_ADC to 10MHz
#define CLK_SRC_ADC_FREQ          10000000 // CLK_SRC_ADC
#define CLK_ADC_FREQ              10000000 // CLK_ADC - 10MHz max in normal mode

// When changing GPIO port/pins below, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_0_BUS          CDBUSALLOC
#define IADC_INPUT_0_BUSALLOC     GPIO_CDBUSALLOC_CDEVEN0_ADC0
#define IADC_INPUT_1_BUS          CDBUSALLOC
#define IADC_INPUT_1_BUSALLOC     GPIO_CDBUSALLOC_CDODD0_ADC0



#define BIT_MOVEMENT      2   //port pin for MOV signal(PB2) -> PA1 in Gen1
#define BIT_ACTIVATION    1   //port pin for ACT signal(PC1) -> PA0 in Gen1


//#ifdef TREX
//  const uint8_t firmware_version[4] = {"2.09"};//{"2.14"} {"2.01"};//{"1.25"};
//  const uint8_t hardware_version[4] = {"1.10"};
//#else
//  const uint8_t firmware_version[4] = {"1.29"};//{"1.27"};{"1.28"};
//  const uint8_t hardware_version[4] = {"1.25"};
//#endif

/*******************************************************************************
 ***************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/

//unsigned char p[32];


uint8_t msg_to_hash[16];

//CRYPTO_SHA256_Digest_TypeDef hashz;
uint8_t key[32];


//External decelerations
extern collectionType     coll;
extern timeINFO           time_saved;
extern uint8_t            conbuffer[MAX_CONTIG_SIZE];
extern uint16_t           currentindex;  // keep track of how many entries we have
extern uint8_t            *conpoint;
extern EventsType         MyEvents;
extern uint8_t            SuppressDetect;
extern const uint8_t      firmware_version[];  // = {"2.00"};
extern volatile uint32_t  rx_data_ready;
//extern uint8_t            authSTATUS;
//extern uint8_t            EventsCounter;
extern uint8_t            CollMemoryFULL;
extern uint8_t            local_saved[];
extern uint8_t            unix_saved[];
extern uint8_t            CollMemoryFULL;
extern UARTDRV_HandleData_t sl_uartdrv_usart_testUART_handle_data;
//extern UARTDRV_Handle_t sl_uartdrv_usart_testUART_handle = &sl_uartdrv_usart_testUART_handle_data;
extern UARTDRV_InitUart_t sl_uartdrv_usart_init_testUART;
extern uint32_t lora_hb_cnt;

//Structure declarations
sl_status_t sc;
deviceDATA  MYdevice;
MTsystem_t  MTsystem;
battData DeviceBatt;
resetData resetColl[8];
last_set_time_t last_set_time;

//Variable declarations
uint8_t     CollMemoryFULL = 0;
uint32_t    resetCause;
uint8_t     BattVolts;
uint8_t     dispTemp;
//uint8_t     EventsCounter = 0;
//uint16_t    DetectionsCounter = 0;
uint8_t     readval;
uint8_t     minTemp = 228;
uint8_t     maxTemp = 60;
uint32_t    intMovCntr;
uint32_t    intActCntr;
uint32_t    timestamp = 0;
uint32_t    local_time = 0;
//uint8_t     authSTATUS = 0;
uint8_t     newKey = 0;
uint32_t keyValidTimer = 0;   //  save after max time reached
uint32_t    intFLAGSodd;
uint32_t    intFLAGSeven;
//uint32_t    intActCntr;
//uint32_t    intMovCntr;
bool        unlocked;
uint8_t keyHold[36] = {0};
//uint8_t newKey = 0;
//uint32_t keyValidTimer = 0;   //  save after max time reached
uint8_t UsedKeyIndex = 0;     // next location to save to
uint8_t reg_flag=0;
extern uint8_t ping_cnt;



uint16_t UsedKeyLocations[MAXUSEDKEYS] = {DEVICE_USEDKEY0_PS,DEVICE_USEDKEY1_PS,DEVICE_USEDKEY2_PS,DEVICE_USEDKEY3_PS,DEVICE_USEDKEY4_PS,
                                          DEVICE_USEDKEY5_PS,DEVICE_USEDKEY6_PS,DEVICE_USEDKEY7_PS,DEVICE_USEDKEY8_PS,DEVICE_USEDKEY9_PS,
                      DEVICE_USEDKEY10_PS,DEVICE_USEDKEY11_PS,DEVICE_USEDKEY12_PS,DEVICE_USEDKEY13_PS,DEVICE_USEDKEY14_PS,
                      DEVICE_USEDKEY15_PS,DEVICE_USEDKEY16_PS,DEVICE_USEDKEY17_PS,DEVICE_USEDKEY18_PS,DEVICE_USEDKEY19_PS};

static volatile int32_t sample;
//static uint8_t advertising_set_handle = 0xff;

const char hexasctable[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
//extern uint8_t test_mode_timer;
const uint8_t manufacturer[] = "SmartWave";
const uint8_t serial_number[] = "1234567890";
volatile uint32_t debugtimestamp;
const uint8_t openKEY[32] = {0x43,0x55,0xEF,0x88,0xD8,0x63,0x03,0x01,0xCD,0xA0,0xA9,0x23,0x77,0xB5,0x45,0x9A,       // pmp ap use for registration
                             0x15,0x30,0xE8,0x38,0xF0,0x70,0x69,0x59,0xBB,0xA4,0x49,0x17,0x39,0xC5,0xFF,0x03};

const uint8_t factoryKEY[32] = {0x4A,0xe5,0x86,0x2E,0x66,0x6C,0xD4,0xE6,0xE4,0x9D,0x46,0x14,0x6f,0xC4,0x72,
                            0x90,0xCB,0x03,0x28,0x50,0xE3,0x14,0x71,0xA5,0x0C,0x95,0xAA,0x51,0x78,0x90,0xAD,0xFE};

const uint8_t genericKEY[32] = {0x77,0x30,0x4E,0x25,0xB0,0xAF,0x30,0x25,0x5F,0xF9,0x48,0x4D,0x3D,0x97,0x53,0x11,
                               0x36,0x26,0x7c,0x3c,0x37,0x91,0xF1,0x14,0x06,0x98,0xB0,0xD3,0xC9,0xF8,0x7B,0xD9};

const uint8_t staticsiteKEY[16] = {0x76,0x2f,0x6c,0x38,0x80,0x99,0x8c,0x71,0x1b,0x47,0x2c,0xb9,0x05,0x5e,0x2f,0x67};

const uint8_t factoryInstallKEY[32] = {0x81,0xCF,0xA4,0x2E,0xE5,0xBF,0x8F,0x28,0x19,0xF2,0x9A,0xFA,0x5B,0x4C,0x23,
                            0xEE,0x51,0xD2,0x61,0x7B,0xD2,0x71,0x5B,0x32,0x71,0x1E,0x9D,0x46,0xBE,0x00,0xB7,0x40};

const uint8_t staticInstallKEY[16] = {0xCC,0x00,0x5D,0xA0,0x03,0xE3,0xD5,0x89,0x18,0x8B,0xC1,0x8E,0x74,0x57,0x87,0x2D};

//const uint8_t staticsiteKEY[16] = {0x76,0x2f,0x6c,0x38,0x80,0x99,0x8c,0x71,0x1b,0x47,0x2c,0xb9,0x05,0x5e,0x2f,0x67};

const uint8_t staticserverKEY[16] = {0xff,0x14,0xa4,0x8b,0x7d,0x46,0xf7,0xa3,0xad,0xf4,0x94,0xa3,0x7d,0xc0,0xfc,0x0f};










/*
 *     session key [kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkttttcTr]    k = received session key  t = time  c = sensor config T = type r = registered
 *     0x4001 random      [rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr]         master key
 *     0x4003 timestamp   [tttt]                                     timestamp of last good session key
 *
 *     kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkktttt (36 bytes)
 *     SHA256 kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkktttt -> sessionkey
 *     compare sessionkey with received key
 *
 */

void hex_to_ascii_byte(uint8_t hex_byte, uint8_t *ascii_val)
{
  uint8_t upper, lower;

  upper = (uint8_t)((hex_byte >> 4) & 0x0f);
  lower = (uint8_t)(hex_byte & 0x0f);

  if(upper < 0x0a)
    ascii_val[0] = (uint8_t)(upper + 0x30);
  else
    ascii_val[0] = (uint8_t)(upper + 0x37);

  if(lower < 0x0a)
    ascii_val[1] = (uint8_t)(lower + 0x30);
  else
    ascii_val[1] = (uint8_t)(lower + 0x37);
}

//UART DIAGNOSTIC CODE PORTED FROM v1.29
void sendVER(void)
{
    uint8_t i = 0;
    char string[] = "VERSION: ";

    for (i = 0 ; string[i] != 0; i++)
    {
        USART_Tx(USART1, string[i]);
    }

    for (i = 0 ; i < 5; i++)
    {
        USART_Tx(USART1, firmware_version[i]);
    }
    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');
}

void sendmfg(void)
{
    uint8_t i = 0;
    char string[] = "Manufacturing date: ";
    uint32_t manufact_date = USERDATA[MANUFAC_DATE];

    uint8_t manufacturing_date[6] = {0};
//    manufacturing_date[5] = (uint8_t) ((manufact_date & 0x0000000F));   //D0
//    manufacturing_date[4] = (uint8_t) (((manufact_date & 0x000000F0) >> 4));  //D1
//    manufacturing_date[3] = (uint8_t) (((manufact_date & 0x00000F00) >> 8));  //M0
//    manufacturing_date[2] = (uint8_t) (((manufact_date & 0x0000F000) >> 12)); //M1
//    manufacturing_date[1] = (uint8_t) (((manufact_date & 0x000F0000) >> 16));   //Y0
//    manufacturing_date[0] = (uint8_t) (((manufact_date & 0x00F00000) >> 20)); //Y1

    manufacturing_date[5] = (uint8_t) ((manufact_date & 0x0000000F)         + 48);  //D0
    manufacturing_date[4] = (uint8_t) (((manufact_date & 0x000000F0) >> 4)  + 48);  //D1
    manufacturing_date[3] = (uint8_t) (((manufact_date & 0x00000F00) >> 8)  + 48);  //M0
    manufacturing_date[2] = (uint8_t) (((manufact_date & 0x0000F000) >> 12) + 48);  //M1
    manufacturing_date[1] = (uint8_t) (((manufact_date & 0x000F0000) >> 16) + 48);  //Y0
    manufacturing_date[0] = (uint8_t) (((manufact_date & 0x00F00000) >> 20) + 48);  //Y1




    for (i = 0 ; string[i] != 0; i++)
    {
        USART_Tx(USART1, string[i]);
    }

    for (i = 0 ; i < 6; i++)
    {
        USART_Tx(USART1, manufacturing_date[i]);
    }

    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');
}

void sendhwv(void)
{
    uint8_t i = 0;
    char string[] = "HW version: ";

    uint32_t hw_ver  = USERDATA[HW_VERSION];
    uint8_t hwv[4] = {0};

    int len = sizeof(hwv) / sizeof(uint8_t);

    hwv[3] = (uint8_t) ((hw_ver  &  0x000000FF));   //
    hwv[2] = (uint8_t) (((hw_ver  & 0x0000FF00) >> 8)); //
    hwv[1] = (uint8_t) (((hw_ver  & 0x00FF0000) >> 16));  //
    hwv[0] = (uint8_t) (((hw_ver  & 0xFF000000) >> 24));  //

    for (i = 0 ; string[i] != 0; i++)
    {
        USART_Tx(USART1, string[i]);
    }

    for (i = 0 ; i < len; i++)
    {
        USART_Tx(USART1, hwv[i]);
    }

    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');
}

void sendhb(void)
{
    uint8_t i = 0;
    char string[] = "Heartbeat: ";

    uint32_t hw_ver  = USERDATA[HW_VERSION];
    uint8_t hwv[4] = {0};

    int len = sizeof(hwv) / sizeof(uint8_t);

    hwv[1] = (uint8_t) ((lora_hb_cnt  &  0x00FF));   //
    hwv[0] = (uint8_t) (((lora_hb_cnt  & 0xFF00) >> 8)); //
//    hwv[1] = (uint8_t) (((hw_ver  & 0x00FF0000) >> 16));  //
//    hwv[0] = (uint8_t) (((hw_ver  & 0xFF000000) >> 24));  //

    for (i = 0 ; string[i] != 0; i++)
    {
        USART_Tx(USART1, string[i]);
    }

    for (i = 0 ; i < 2; i++)
    {
        USART_Tx(USART1, hexasctable[((hwv[i] & 0xF0) >> 4)]);
        USART_Tx(USART1, hexasctable[(hwv[i] & 0x0F)]);
//        USART_Tx(USART1, hwv[i]);
    }

    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');
}

void sendsnr(void)
{
    uint8_t i = 0;
    char string[] = "SNR: ";

//    uint32_t hw_ver  = USERDATA[HW_VERSION];
//    uint8_t hwv[4] = {0};
//
//    int len = sizeof(hwv) / sizeof(uint8_t);

//    hwv[1] = (uint8_t) ((MYdevice.lora.lora_snr_last &  0x00FF));   //
//    hwv[0] = (uint8_t) (((lora_hb_cnt  & 0xFF00) >> 8)); //
//    hwv[1] = (uint8_t) (((hw_ver  & 0x00FF0000) >> 16));  //
//    hwv[0] = (uint8_t) (((hw_ver  & 0xFF000000) >> 24));  //

    for (i = 0 ; string[i] != 0; i++)
    {
        USART_Tx(USART1, string[i]);
    }

//    for (i = 0 ; i < 2; i++)
    {
        USART_Tx(USART1, hexasctable[((MYdevice.lora.lora_snr_last & 0xF0) >> 4)]);
        USART_Tx(USART1, hexasctable[(MYdevice.lora.lora_snr_last & 0x0F)]);
//        USART_Tx(USART1, hwv[i]);
    }

    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');
}
void sendrssi(void)
{
    uint8_t i = 0;
    char string[] = "RSSI: ";

//    uint32_t hw_ver  = USERDATA[HW_VERSION];
//    uint8_t hwv[4] = {0};
//
//    int len = sizeof(hwv) / sizeof(uint8_t);

//    hwv[1] = (uint8_t) ((MYdevice.lora.lora_snr_last &  0x00FF));   //
//    hwv[0] = (uint8_t) (((lora_hb_cnt  & 0xFF00) >> 8)); //
//    hwv[1] = (uint8_t) (((hw_ver  & 0x00FF0000) >> 16));  //
//    hwv[0] = (uint8_t) (((hw_ver  & 0xFF000000) >> 24));  //

    for (i = 0 ; string[i] != 0; i++)
    {
        USART_Tx(USART1, string[i]);
    }

//    for (i = 0 ; i < 2; i++)
    {
        USART_Tx(USART1, hexasctable[((MYdevice.lora.lora_rssi_last & 0xF0) >> 4)]);
        USART_Tx(USART1, hexasctable[(MYdevice.lora.lora_rssi_last & 0x0F)]);
//        USART_Tx(USART1, hwv[i]);
    }

    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');
}

void load_devEUI(void)
{
  uint32_t dev_eui0  = USERDATA[DEVICE_EUI0];
  uint32_t dev_eui1  = USERDATA[DEVICE_EUI1];
  uint32_t dev_eui2  = USERDATA[DEVICE_EUI2];
  uint32_t dev_eui3  = USERDATA[DEVICE_EUI3];

  uint8_t dev_id0[4] = {0};
  uint8_t dev_id1[4] = {0};
  uint8_t dev_id2[4] = {0};
  uint8_t dev_id3[4] = {0};

  uint8_t i=0;

  dev_id0[0] = (uint8_t) ((dev_eui0  &  0x000000FF));   //
  dev_id0[1] = (uint8_t) (((dev_eui0  & 0x0000FF00) >> 8)); //
  dev_id0[2] = (uint8_t) (((dev_eui0  & 0x00FF0000) >> 16));  //
  dev_id0[3] = (uint8_t) (((dev_eui0  & 0xFF000000) >> 24));  //

  dev_id1[0] = (uint8_t) ((dev_eui1  &  0x000000FF));   //
  dev_id1[1] = (uint8_t) (((dev_eui1  & 0x0000FF00) >> 8)); //
  dev_id1[2] = (uint8_t) (((dev_eui1  & 0x00FF0000) >> 16));  //
  dev_id1[3] = (uint8_t) (((dev_eui1  & 0xFF000000) >> 24));  //

  dev_id2[0] = (uint8_t) ((dev_eui2  &  0x000000FF));   //
  dev_id2[1] = (uint8_t) (((dev_eui2  & 0x0000FF00) >> 8)); //
  dev_id2[2] = (uint8_t) (((dev_eui2  & 0x00FF0000) >> 16));  //
  dev_id2[3] = (uint8_t) (((dev_eui2  & 0xFF000000) >> 24));  //

  dev_id3[0] = (uint8_t) ((dev_eui3  &  0x000000FF));   //
  dev_id3[1] = (uint8_t) (((dev_eui3  & 0x0000FF00) >> 8)); //
  dev_id3[2] = (uint8_t) (((dev_eui3  & 0x00FF0000) >> 16));  //
  dev_id3[3] = (uint8_t) (((dev_eui3  & 0xFF000000) >> 24));  //


//  MYdevice.lora.devEUI[i++] = ascii_byte_to_hex( (uint8_t *)(& dev_id0[0]));
//  MYdevice.lora.devEUI[i++] = ascii_byte_to_hex( (uint8_t *)(& dev_id0[2]));
//  MYdevice.lora.devEUI[i++] = ascii_byte_to_hex( (uint8_t *)(& dev_id1[0]));
//  MYdevice.lora.devEUI[i++] = ascii_byte_to_hex( (uint8_t *)(& dev_id1[2]));
//  MYdevice.lora.devEUI[i++] = ascii_byte_to_hex( (uint8_t *)(& dev_id2[0]));
//  MYdevice.lora.devEUI[i++] = ascii_byte_to_hex( (uint8_t *)(& dev_id2[2]));
//  MYdevice.lora.devEUI[i++] = ascii_byte_to_hex( (uint8_t *)(& dev_id3[0]));
//  MYdevice.lora.devEUI[i++] = ascii_byte_to_hex( (uint8_t *)(& dev_id3[2]));

  MYdevice.lora.devEUI[i++] = 0x11;
  MYdevice.lora.devEUI[i++] = 0x22;
  MYdevice.lora.devEUI[i++] = 0x33;
  MYdevice.lora.devEUI[i++] = 0x44;
  MYdevice.lora.devEUI[i++] = 0x55;
  MYdevice.lora.devEUI[i++] = 0xCC;
  MYdevice.lora.devEUI[i++] = 0xAA;
  MYdevice.lora.devEUI[i++] = 0xBB;

  ps_save(EEPROM_DEVICE_ID_OFFSET,MYdevice.lora.devEUI, EEPROM_DEVICE_ID_LENGTH);


}

void load_appKEY(void)
{
  uint8_t i = 0;

  uint32_t application_key0   = USERDATA[APP_KEY0];
  uint32_t application_key1   = USERDATA[APP_KEY1];
  uint32_t application_key2   = USERDATA[APP_KEY2];
  uint32_t application_key3   = USERDATA[APP_KEY3];
  uint32_t application_key4   = USERDATA[APP_KEY4];
  uint32_t application_key5   = USERDATA[APP_KEY5];
  uint32_t application_key6   = USERDATA[APP_KEY6];
  uint32_t application_key7   = USERDATA[APP_KEY7];

  uint8_t app_key0[4] = {0};
  uint8_t app_key1[4] = {0};
  uint8_t app_key2[4] = {0};
  uint8_t app_key3[4] = {0};
  uint8_t app_key4[4] = {0};
  uint8_t app_key5[4] = {0};
  uint8_t app_key6[4] = {0};
  uint8_t app_key7[4] = {0};


  app_key0[0] = (uint8_t) (( application_key0  & 0x000000FF));  //
  app_key0[1] = (uint8_t) (((application_key0  & 0x0000FF00) >> 8));  //
  app_key0[2] = (uint8_t) (((application_key0  & 0x00FF0000) >> 16));   //
  app_key0[3] = (uint8_t) (((application_key0  & 0xFF000000) >> 24)); //

  app_key1[0] = (uint8_t) (( application_key1  & 0x000000FF));  //
  app_key1[1] = (uint8_t) (((application_key1  & 0x0000FF00) >> 8));  //
  app_key1[2] = (uint8_t) (((application_key1  & 0x00FF0000) >> 16));   //
  app_key1[3] = (uint8_t) (((application_key1  & 0xFF000000) >> 24)); //

  app_key2[0] = (uint8_t) (( application_key2  & 0x000000FF));  //
  app_key2[1] = (uint8_t) (((application_key2  & 0x0000FF00) >> 8));  //
  app_key2[2] = (uint8_t) (((application_key2  & 0x00FF0000) >> 16));   //
  app_key2[3] = (uint8_t) (((application_key2  & 0xFF000000) >> 24)); //

  app_key3[0] = (uint8_t) (( application_key3  & 0x000000FF));  //
  app_key3[1] = (uint8_t) (((application_key3  & 0x0000FF00) >> 8));  //
  app_key3[2] = (uint8_t) (((application_key3  & 0x00FF0000) >> 16));   //
  app_key3[3] = (uint8_t) (((application_key3  & 0xFF000000) >> 24)); //

  app_key4[0] = (uint8_t) (( application_key4  & 0x000000FF));  //
  app_key4[1] = (uint8_t) (((application_key4  & 0x0000FF00) >> 8));  //
  app_key4[2] = (uint8_t) (((application_key4  & 0x00FF0000) >> 16));   //
  app_key4[3] = (uint8_t) (((application_key4  & 0xFF000000) >> 24)); //

  app_key5[0] = (uint8_t) (( application_key5  & 0x000000FF));  //
  app_key5[1] = (uint8_t) (((application_key5  & 0x0000FF00) >> 8));  //
  app_key5[2] = (uint8_t) (((application_key5  & 0x00FF0000) >> 16));   //
  app_key5[3] = (uint8_t) (((application_key5  & 0xFF000000) >> 24)); //

  app_key6[0] = (uint8_t) (( application_key6  & 0x000000FF));  //
  app_key6[1] = (uint8_t) (((application_key6  & 0x0000FF00) >> 8));  //
  app_key6[2] = (uint8_t) (((application_key6  & 0x00FF0000) >> 16));   //
  app_key6[3] = (uint8_t) (((application_key6  & 0xFF000000) >> 24)); //

  app_key7[0] = (uint8_t) (( application_key7  & 0x000000FF));  //
  app_key7[1] = (uint8_t) (((application_key7  & 0x0000FF00) >> 8));  //
  app_key7[2] = (uint8_t) (((application_key7  & 0x00FF0000) >> 16));   //
  app_key7[3] = (uint8_t) (((application_key7  & 0xFF000000) >> 24)); //
//
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key0[0]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key0[2]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key1[0]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key1[2]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key2[0]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key2[2]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key3[0]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key3[2]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key4[0]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key4[2]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key5[0]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key5[2]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key6[0]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key6[2]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key7[0]));
//  MYdevice.lora.appKEY[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key7[2]));


  MYdevice.lora.appKEY[i++] = 0x88;
  MYdevice.lora.appKEY[i++] = 0x99;
  MYdevice.lora.appKEY[i++] = 0xAA;
  MYdevice.lora.appKEY[i++] = 0xBB;
  MYdevice.lora.appKEY[i++] = 0xCC;
  MYdevice.lora.appKEY[i++] = 0xDD;
  MYdevice.lora.appKEY[i++] = 0xEE;
  MYdevice.lora.appKEY[i++] = 0xFF;
  MYdevice.lora.appKEY[i++] = 0x00;
  MYdevice.lora.appKEY[i++] = 0x11;
  MYdevice.lora.appKEY[i++] = 0x22;
  MYdevice.lora.appKEY[i++] = 0x33;
  MYdevice.lora.appKEY[i++] = 0x44;
  MYdevice.lora.appKEY[i++] = 0x55;
  MYdevice.lora.appKEY[i++] = 0x66;
  MYdevice.lora.appKEY[i++] = 0x77;

//  ps_save(EEPROM_LORA_APP_KEY_OFFSET,MYdevice.lora.appKEY, EEPROM_LORA_APP_KEY_LENGTH);

  ps_save(EEPROM_LORA_APP_KEY_OFFSET,MYdevice.lora.appKEY, EEPROM_LORA_APP_KEY_LENGTH);

}


void load_serial(void)
{
  uint8_t i = 0;

  uint32_t serial0   = USERDATA[SERIAL0];
  uint32_t serial1   = USERDATA[SERIAL1];
  uint32_t serial2   = USERDATA[SERIAL2];
  uint32_t serial3   = USERDATA[SERIAL3];
  uint32_t serial4   = USERDATA[SERIAL4];
  uint32_t serial5   = USERDATA[SERIAL5];
  uint32_t serial6   = USERDATA[SERIAL6];
  uint32_t serial7   = USERDATA[SERIAL7];

  uint8_t app_key0[4] = {0};
  uint8_t app_key1[4] = {0};
  uint8_t app_key2[4] = {0};
  uint8_t app_key3[4] = {0};
  uint8_t app_key4[4] = {0};
  uint8_t app_key5[4] = {0};
  uint8_t app_key6[4] = {0};
  uint8_t app_key7[4] = {0};


  app_key0[0] = (uint8_t) (( serial0  & 0x000000FF));  //
  app_key0[1] = (uint8_t) (((serial0  & 0x0000FF00) >> 8));  //
  app_key0[2] = (uint8_t) (((serial0  & 0x00FF0000) >> 16));   //
  app_key0[3] = (uint8_t) (((serial0  & 0xFF000000) >> 24)); //

  app_key1[0] = (uint8_t) (( serial1  & 0x000000FF));  //
  app_key1[1] = (uint8_t) (((serial1  & 0x0000FF00) >> 8));  //
  app_key1[2] = (uint8_t) (((serial1  & 0x00FF0000) >> 16));   //
  app_key1[3] = (uint8_t) (((serial1  & 0xFF000000) >> 24)); //

  app_key2[0] = (uint8_t) (( serial2  & 0x000000FF));  //
  app_key2[1] = (uint8_t) (((serial2  & 0x0000FF00) >> 8));  //
  app_key2[2] = (uint8_t) (((serial2  & 0x00FF0000) >> 16));   //
  app_key2[3] = (uint8_t) (((serial2  & 0xFF000000) >> 24)); //

  app_key3[0] = (uint8_t) (( serial3  & 0x000000FF));  //
  app_key3[1] = (uint8_t) (((serial3  & 0x0000FF00) >> 8));  //
  app_key3[2] = (uint8_t) (((serial3  & 0x00FF0000) >> 16));   //
  app_key3[3] = (uint8_t) (((serial3  & 0xFF000000) >> 24)); //

  app_key4[0] = (uint8_t) (( serial4  & 0x000000FF));  //
  app_key4[1] = (uint8_t) (((serial4  & 0x0000FF00) >> 8));  //
  app_key4[2] = (uint8_t) (((serial4  & 0x00FF0000) >> 16));   //
  app_key4[3] = (uint8_t) (((serial4  & 0xFF000000) >> 24)); //

  app_key5[0] = (uint8_t) (( serial5  & 0x000000FF));  //
  app_key5[1] = (uint8_t) (((serial5  & 0x0000FF00) >> 8));  //
  app_key5[2] = (uint8_t) (((serial5  & 0x00FF0000) >> 16));   //
  app_key5[3] = (uint8_t) (((serial5  & 0xFF000000) >> 24)); //

  app_key6[0] = (uint8_t) (( serial6  & 0x000000FF));  //
  app_key6[1] = (uint8_t) (((serial6  & 0x0000FF00) >> 8));  //
  app_key6[2] = (uint8_t) (((serial6  & 0x00FF0000) >> 16));   //
  app_key6[3] = (uint8_t) (((serial6  & 0xFF000000) >> 24)); //

  app_key7[0] = (uint8_t) (( serial7  & 0x000000FF));  //
  app_key7[1] = (uint8_t) (((serial7  & 0x0000FF00) >> 8));  //
  app_key7[2] = (uint8_t) (((serial7  & 0x00FF0000) >> 16));   //
  app_key7[3] = (uint8_t) (((serial7  & 0xFF000000) >> 24)); //

  MYdevice.lora.serial[i++] = (uint8_t) (( serial0  & 0x000000FF));  //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial0  & 0x0000FF00) >> 8));  //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial0  & 0x00FF0000) >> 16));   //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial0  & 0xFF000000) >> 24)); //

  MYdevice.lora.serial[i++] = (uint8_t) (( serial1  & 0x000000FF));  //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial1  & 0x0000FF00) >> 8));  //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial1  & 0x00FF0000) >> 16));   //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial1  & 0xFF000000) >> 24)); //

  MYdevice.lora.serial[i++] = (uint8_t) (( serial2  & 0x000000FF));  //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial2  & 0x0000FF00) >> 8));  //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial2  & 0x00FF0000) >> 16));   //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial2  & 0xFF000000) >> 24)); //

  MYdevice.lora.serial[i++] = (uint8_t) (( serial3  & 0x000000FF));  //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial3  & 0x0000FF00) >> 8));  //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial3  & 0x00FF0000) >> 16));   //
  MYdevice.lora.serial[i++] = (uint8_t) (((serial3  & 0xFF000000) >> 24)); //

//  app_key4[0] = (uint8_t) (( serial4  & 0x000000FF));  //
//  app_key4[1] = (uint8_t) (((serial4  & 0x0000FF00) >> 8));  //
//  app_key4[2] = (uint8_t) (((serial4  & 0x00FF0000) >> 16));   //
//  app_key4[3] = (uint8_t) (((serial4  & 0xFF000000) >> 24)); //
//
//  app_key5[0] = (uint8_t) (( serial5  & 0x000000FF));  //
//  app_key5[1] = (uint8_t) (((serial5  & 0x0000FF00) >> 8));  //
//  app_key5[2] = (uint8_t) (((serial5  & 0x00FF0000) >> 16));   //
//  app_key5[3] = (uint8_t) (((serial5  & 0xFF000000) >> 24)); //
//
//  app_key6[0] = (uint8_t) (( serial6  & 0x000000FF));  //
//  app_key6[1] = (uint8_t) (((serial6  & 0x0000FF00) >> 8));  //
//  app_key6[2] = (uint8_t) (((serial6  & 0x00FF0000) >> 16));   //
//  app_key6[3] = (uint8_t) (((serial6  & 0xFF000000) >> 24)); //
//
//  app_key7[0] = (uint8_t) (( serial7  & 0x000000FF));  //
//  app_key7[1] = (uint8_t) (((serial7  & 0x0000FF00) >> 8));  //
//  app_key7[2] = (uint8_t) (((serial7  & 0x00FF0000) >> 16));   //
//  app_key7[3] = (uint8_t) (((serial7  & 0xFF000000) >> 24)); //

//  app_key0[3] = (uint8_t) (( serial0  & 0x000000FF));  //
//  app_key0[2] = (uint8_t) (((serial0  & 0x0000FF00) >> 8));  //
//  app_key0[1] = (uint8_t) (((serial0  & 0x00FF0000) >> 16));   //
//  app_key0[0] = (uint8_t) (((serial0  & 0xFF000000) >> 24)); //
//
//  app_key1[3] = (uint8_t) (( serial1  & 0x000000FF));  //
//  app_key1[2] = (uint8_t) (((serial1  & 0x0000FF00) >> 8));  //
//  app_key1[1] = (uint8_t) (((serial1  & 0x00FF0000) >> 16));   //
//  app_key1[0] = (uint8_t) (((serial1  & 0xFF000000) >> 24)); //
//
//  app_key2[0] = (uint8_t) (( serial2  & 0x000000FF));  //
//  app_key2[1] = (uint8_t) (((serial2  & 0x0000FF00) >> 8));  //
//  app_key2[2] = (uint8_t) (((serial2  & 0x00FF0000) >> 16));   //
//  app_key2[3] = (uint8_t) (((serial2  & 0xFF000000) >> 24)); //
//
//  app_key3[0] = (uint8_t) (( serial3  & 0x000000FF));  //
//  app_key3[1] = (uint8_t) (((serial3  & 0x0000FF00) >> 8));  //
//  app_key3[2] = (uint8_t) (((serial3  & 0x00FF0000) >> 16));   //
//  app_key3[3] = (uint8_t) (((serial3  & 0xFF000000) >> 24)); //
//
//  app_key4[0] = (uint8_t) (( serial4  & 0x000000FF));  //
//  app_key4[1] = (uint8_t) (((serial4  & 0x0000FF00) >> 8));  //
//  app_key4[2] = (uint8_t) (((serial4  & 0x00FF0000) >> 16));   //
//  app_key4[3] = (uint8_t) (((serial4  & 0xFF000000) >> 24)); //
//
//  app_key5[0] = (uint8_t) (( serial5  & 0x000000FF));  //
//  app_key5[1] = (uint8_t) (((serial5  & 0x0000FF00) >> 8));  //
//  app_key5[2] = (uint8_t) (((serial5  & 0x00FF0000) >> 16));   //
//  app_key5[3] = (uint8_t) (((serial5  & 0xFF000000) >> 24)); //
//
//  app_key6[0] = (uint8_t) (( serial6  & 0x000000FF));  //
//  app_key6[1] = (uint8_t) (((serial6  & 0x0000FF00) >> 8));  //
//  app_key6[2] = (uint8_t) (((serial6  & 0x00FF0000) >> 16));   //
//  app_key6[3] = (uint8_t) (((serial6  & 0xFF000000) >> 24)); //
//
//  app_key7[0] = (uint8_t) (( serial7  & 0x000000FF));  //
//  app_key7[1] = (uint8_t) (((serial7  & 0x0000FF00) >> 8));  //
//  app_key7[2] = (uint8_t) (((serial7  & 0x00FF0000) >> 16));   //
//  app_key7[3] = (uint8_t) (((serial7  & 0xFF000000) >> 24)); //


//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key0[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key0[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key1[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key1[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key2[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key2[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key3[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key3[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key4[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key4[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key5[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key5[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key6[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key6[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key7[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key7[2]));

//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key0[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key0[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key1[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key1[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key2[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key2[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key3[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key3[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key4[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key4[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key5[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key5[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key6[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key6[0]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key7[2]));
//  MYdevice.lora.serial[i++] = ascii_byte_to_hex( (uint8_t *)(& app_key7[0]));

//  ps_save(EEPROM_LORA_APP_KEY_OFFSET,MYdevice.lora.appKEY, EEPROM_LORA_APP_KEY_LENGTH);

}
void sendDevEui(void)
{
    uint8_t i = 0;
    char string[] = "Dev EUI: ";

   for (i = 0 ; string[i] != 0; i++)
   {
       USART_Tx(USART1, string[i]);
   }

   for(i=0; i<8; i++)
   {
       USART_Tx(USART1, hexasctable[((MYdevice.lora.devEUI[i] & 0xF0) >> 4)]);
       USART_Tx(USART1, hexasctable[(MYdevice.lora.devEUI[i] & 0x0F)]);
   }
    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');
}

void sendAppEui(void)
{
    uint8_t i = 0;
    char string[] = "App EUI: ";

   for (i = 0 ; string[i] != 0; i++)
   {
       USART_Tx(USART1, string[i]);
   }

   for(i=0; i<8; i++)
   {
       USART_Tx(USART1, hexasctable[((MYdevice.lora.appEUI[i] & 0xF0) >> 4)]);
       USART_Tx(USART1, hexasctable[(MYdevice.lora.appEUI[i] & 0x0F)]);
   }
    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');
}

void sendAppk(void)
{
    uint8_t i = 0;
    char string[] = "App Key: ";

    for (i = 0 ; string[i] != 0; i++)
    {
        USART_Tx(USART1, string[i]);
    }

   for(i=0; i<16; i++)
   {
       USART_Tx(USART1, hexasctable[((MYdevice.lora.appKEY[i] & 0xF0) >> 4)]);
       USART_Tx(USART1, hexasctable[(MYdevice.lora.appKEY[i] & 0x0F)]);
   }
    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');

}

void sendSerial(void)
{
    uint8_t i = 0;
    char string[] = "Serial #: ";

    for (i = 0 ; string[i] != 0; i++)
    {
        USART_Tx(USART1, string[i]);
    }

   for(i=0; i<16; i++)
   {
//       USART_Tx(USART1, hexasctable[((MYdevice.lora.serial[i] & 0xF0) >> 4)]);
//       USART_Tx(USART1, hexasctable[(MYdevice.lora.serial[i] & 0x0F)]);
       USART_Tx(USART1, MYdevice.lora.serial[i]);
   }
    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');

}

void sendLoraJoinState(void)
{
    uint8_t i = 0;
    char string1[] = "LORA JOIN STATUS: ";

   for (i = 0 ; string1[i] != 0; i++)
   {
       USART_Tx(USART1, string1[i]);
   }

//   for(i=0; i<8; i++)
   if(MYdevice.lora.join_status == 2)
   {
       char string4[] = "IN PROGRESS ";
       for (i = 0 ; string4[i] != 0; i++)
       {
           USART_Tx(USART1, string4[i]);
       }
   }
   else
   {
       USART_Tx(USART1, hexasctable[((MYdevice.lora.join_status & 0xF0) >> 4)]);
       USART_Tx(USART1, hexasctable[(MYdevice.lora.join_status & 0x0F)]);
   }
    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');

    char string2[] = "LORA RSSI: ";

   for (i = 0 ; string2[i] != 0; i++)
   {
       USART_Tx(USART1, string2[i]);
   }

//   for(i=0; i<8; i++)
   if(MYdevice.lora.join_status != 2)
   {
       USART_Tx(USART1, hexasctable[((MYdevice.lora.lora_rssi_last & 0xF0) >> 4)]);
       USART_Tx(USART1, hexasctable[(MYdevice.lora.lora_rssi_last & 0x0F)]);
   }
   else
     {
       USART_Tx(USART1,'N');
       USART_Tx(USART1,'A');
     }
    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');

    char string3[] = "LORA SNR: ";

   for (i = 0 ; string3[i] != 0; i++)
   {
       USART_Tx(USART1, string3[i]);
   }

//   for(i=0; i<8; i++)
   if(MYdevice.lora.join_status != 2)
   {
       USART_Tx(USART1, hexasctable[((MYdevice.lora.lora_snr_last & 0xF0) >> 4)]);
       USART_Tx(USART1, hexasctable[(MYdevice.lora.lora_snr_last & 0x0F)]);
   }
   else
     {
       USART_Tx(USART1,'N');
       USART_Tx(USART1,'A');
     }
    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');
}

void char_send(uint8_t dum, uint8_t type)
{
//  UARTDRV_InitUart(sl_uartdrv_usart_testUART_handle, &sl_uartdrv_usart_init_testUART);


  USART_Tx(USART1,'\r');
  USART_Tx(USART1,'\n');
if(type)
  {
  USART_Tx(USART1, hexasctable[((dum & 0xF0) >> 4)]);
  USART_Tx(USART1, hexasctable[(dum & 0x0F)]);
  }
else
  USART_Tx(USART1,dum);



  USART_Tx(USART1,'\r');
  USART_Tx(USART1,'\n');
//  UARTDRV_DeInit(sl_uartdrv_usart_testUART_handle);
}

void send_string(uint8_t *str)
{
  for (uint8_t i = 0 ; str[i] != 0; i++)
      {
        USART_Tx(USART1, str[i]);
      }
  USART_Tx(USART1,'\r');
  USART_Tx(USART1,'\n');
}

void char_send1(uint8_t dum, uint8_t type)
{
//  UARTDRV_InitUart(sl_uartdrv_usart_testUART_handle, &sl_uartdrv_usart_init_testUART);


//  USART_Tx(USART1,'\r');
//  USART_Tx(USART1,'\n');
if(type)
  {
  USART_Tx(USART1, hexasctable[((dum & 0xF0) >> 4)]);
  USART_Tx(USART1, hexasctable[(dum & 0x0F)]);
  }
else
  USART_Tx(USART1,dum);



//  USART_Tx(USART1,'\r');
//  USART_Tx(USART1,'\n');
//  UARTDRV_DeInit(sl_uartdrv_usart_testUART_handle);
}

void sendMAC(void)
{
      uint8_t i = 0;
      bd_addr address;
      uint8_t address_type;


      char string[] = "MAC ADDRESS: ";
//      struct gecko_msg_system_get_bt_address_rsp_t *myMACaddr;
      sc = sl_bt_system_get_identity_address(&address, &address_type);

//      myMACaddr = gecko_cmd_system_get_bt_address();

      for (i = 0 ; string[i] != 0; i++)
         {
           USART_Tx(USART1, string[i]);
         }

      i = 5;
      while(i != 0)
      {
          USART_Tx(USART1, hexasctable[((address.addr[i] & 0xF0) >> 4)]);
          USART_Tx(USART1, hexasctable[(address.addr[i] & 0x0F)]);
          USART_Tx(USART1, ':');
          i--;
      }
      USART_Tx(USART1, hexasctable[((address.addr[0] & 0xF0) >> 4)]);
      USART_Tx(USART1, hexasctable[(address.addr[0] & 0x0F)]);
      USART_Tx(USART1,'\r');
      USART_Tx(USART1,'\n');

}

void sendSKU(void)
{
      uint8_t i = 0;
      char string[] = "SKU: ";
      for (i = 0 ; string[i] != 0; i++)
         {
           USART_Tx(USART1, string[i]);
         }

      i = 5;
      USART_Tx(USART1, hexasctable[((MYdevice.sku & 0xF0) >> 4)]);
      USART_Tx(USART1, hexasctable[(MYdevice.sku & 0x0F)]);
      USART_Tx(USART1,'\r');
      USART_Tx(USART1,'\n');

}
void sendLORAflag(void)
{
      uint8_t i = 0;
      char string[] = "LoRa FLAG: ";

      for (i = 0 ; string[i] != 0; i++)
         {
           USART_Tx(USART1, string[i]);
         }

      i = 5;
      USART_Tx(USART1, hexasctable[((MYdevice.lora.lora_alive_flag & 0xF0) >> 4)]);
      USART_Tx(USART1, hexasctable[(MYdevice.lora.lora_alive_flag)]);
      USART_Tx(USART1,'\r');
      USART_Tx(USART1,'\n');

}
void sendLORAstatus(void)
{
      uint8_t i = 0;
      char string[] = "LoRa STATUS: ";
//      struct gecko_msg_system_get_bt_address_rsp_t *myMACaddr;

//      myMACaddr = gecko_cmd_system_get_bt_address();

      for (i = 0 ; string[i] != 0; i++)
         {
           USART_Tx(USART1, string[i]);
         }

      i = 5;
      USART_Tx(USART1, hexasctable[((MYdevice.lora.lora_alive_status & 0xF0) >> 4)]);
      USART_Tx(USART1, hexasctable[(MYdevice.lora.lora_alive_status & 0x0F)]);
      USART_Tx(USART1,'\r');
      USART_Tx(USART1,'\n');

}

void sendREGION(void)
{
      uint8_t i = 0;
      char string[] = "REGION: ";
//      struct gecko_msg_system_get_bt_address_rsp_t *myMACaddr;

//      myMACaddr = gecko_cmd_system_get_bt_address();

      for (i = 0 ; string[i] != 0; i++)
         {
           USART_Tx(USART1, string[i]);
         }

      switch(MYdevice.lora.region)
      {
        case 1:
          USART_Tx(USART1, 'U');
          USART_Tx(USART1, 'S');
          break;
        case 2:
          USART_Tx(USART1, 'E');
          USART_Tx(USART1, 'U');
          break;
        case 3:
          USART_Tx(USART1, 'A');
          USART_Tx(USART1, 'Z');
          break;
        default:
          USART_Tx(USART1, 'E');
          USART_Tx(USART1, 'R');
          USART_Tx(USART1, 'R');
          break;
      }

      USART_Tx(USART1,'\r');
      USART_Tx(USART1,'\n');

}

void sendSENS(uint8_t count)
{
    uint8_t i = 0;
    char string[] = "SENSOR: ";
    uint8_t hundreds = 0;
    uint8_t tens = 0;
    uint8_t ones = 0;

    hundreds = (count / 100) + 0x30;
    tens = ((count % 100) / 10) + 0x30;
    ones = ((count % 100) % 10) + 0x30;

    for (i = 0 ; string[i] != 0; i++)
    {
        USART_Tx(USART0, string[i]);
    }

    USART_Tx(USART1, hundreds);
    USART_Tx(USART1, tens);
    USART_Tx(USART1, ones);
    USART_Tx(USART1,'\r');
    USART_Tx(USART1,'\n');
//    USART_Tx(USART0,0x00);
//    USART_Tx(USART0,0x00);
//    USART_Tx(USART0,0x00);

}

void sendHALL(uint8_t position)
{
//    uint8_t i = 0;
//    char string[] = "SENSOR:";
//    char stringCLOSE[] = "CLOSE";
//    char stringOPEN[] = "OPEN";
//
//    for (i = 0 ; string[i] != 0; i++)
//    {
//        USART_Tx(USART0, string[i]);
//    }
//
//    if(position == 1)
//    {
//        for (i = 0 ; stringOPEN[i] != 0; i++)
//        {
//            USART_Tx(USART0, stringOPEN[i]);
//        }
//    }
//    else
//    {
//        for (i = 0 ; stringCLOSE[i] != 0; i++)
//        {
//            USART_Tx(USART0, stringCLOSE[i]);
//        }
//
//    }
//    USART_Tx(USART0,'\r');
//    USART_Tx(USART0,'\n');

}


void sendRSSI(int8_t value)
{
//    uint8_t i = 0;
//    char string[] = "RSSI: ";
//    uint8_t hundreds = 0;
//    uint8_t tens = 0;
//    uint8_t ones = 0;
//
//    for (i = 0 ; string[i] != 0; i++)
//    {
//        USART_Tx(USART0, string[i]);
//    }
//
//    if(value & 0x80)
//    {
//        USART_Tx(USART0,'-');
//        value = -value;
//    }
//
//    hundreds = (value / 100) + 0x30;
//    tens = ((value % 100) / 10) + 0x30;
//    ones = ((value % 100) % 10) + 0x30;
//
//    USART_Tx(USART0, hundreds);
//    USART_Tx(USART0, tens);
//    USART_Tx(USART0, ones);
//    USART_Tx(USART0,'\r');
//    USART_Tx(USART0,'\n');

}

void sendBatt(uint32_t adcValue)
{
//
//    uint8_t i = 0;
//    char string[] = "BATT: ";
//    uint8_t thousands = 0;
//    uint8_t hundreds = 0;
//    uint8_t tens = 0;
//    uint8_t ones = 0;
//
//    for (i = 0 ; string[i] != 0; i++)
//    {
//        USART_Tx(USART0, string[i]);
//    }
//        thousands = (adcValue / 1000) + 0x30;
//        hundreds = ((adcValue % 1000) / 100) + 0x30;
//        tens = (((adcValue % 1000) % 100) /10) + 0x30;
//        ones = (((adcValue % 1000) % 100) % 10) + 0x30;
//
//        USART_Tx(USART0, thousands);
//        USART_Tx(USART0, hundreds);
//        USART_Tx(USART0, tens);
//        USART_Tx(USART0, ones);
//        USART_Tx(USART0,'\r');
//        USART_Tx(USART0,'\n');
}

void sendIO1(void)
{
//    uint8_t i = 0;
//    char string[] = "IO1 ";
//
//    for (i = 0 ; string[i] != 0; i++)
//    {
//        USART_Tx(USART0, string[i]);
//    }
//    USART_Tx(USART0,'\r');
//    USART_Tx(USART0,'\n');
}
void sendIO2(void)
{
//    uint8_t i = 0;
//    char string[] = "IO2 ";
//
//    for (i = 0 ; string[i] != 0; i++)
//    {
//        USART_Tx(USART0, string[i]);
//    }
//    USART_Tx(USART0,'\r');
//    USART_Tx(USART0,'\n');
}


void sendLOCAL(void)
{
//    uint8_t i = 0;
//    char string[] = "LOCAL COUNTER: ";
//
//    for (i = 0 ; string[i] != 0; i++)
//    {
//        USART_Tx(USART0, string[i]);
//    }
//
//    for (i = 0 ; i < 4; i++)
//    {
////        USART_Tx(USART0, local_saved[i]);
//        USART_Tx(USART0, hexasctable[((local_saved[i] & 0xF0) >> 4)]);
//        USART_Tx(USART0, hexasctable[(local_saved[i] & 0x0F)]);
//    }
//    USART_Tx(USART0,'\r');
//    USART_Tx(USART0,'\n');
}

void sendUNIX(void)
{
//    uint8_t i = 0;
//    char string[] = "UNIX TIME: ";
//
//    for (i = 0 ; string[i] != 0; i++)
//    {
//        USART_Tx(USART0, string[i]);
//    }
//
//    for (i = 0 ; i < 4; i++)
//    {
//        USART_Tx(USART0, hexasctable[((unix_saved[i] & 0xF0) >> 4)]);
//        USART_Tx(USART0, hexasctable[(unix_saved[i] & 0x0F)]);
//    }
//    USART_Tx(USART0,'\r');
//    USART_Tx(USART0,'\n');
}
//void sendUNIX_date(void)
//{
//    uint8_t i = 0;
//    char string[] = "LAST SAVED DATE:";
//
//    for (i = 0 ; string[i] != 0; i++)
//    {
//        USART_Tx(USART0, string[i]);
//    }
//
//    USART_Tx(USART0,'\r');
//    USART_Tx(USART0,'\n');
//    char string[] = "MONTH: ";
//    USART_Tx(USART0, time_saved.tm_mon);
//    USART_Tx(USART0,'\r');
//    USART_Tx(USART0,'\n');
//    char string[] = "DAY: ";
//    USART_Tx(USART0, time_saved.tm_mday);
//    USART_Tx(USART0,'\r');
//    USART_Tx(USART0,'\n');
//    char string[] = "YEAR: ";
//    USART_Tx(USART0, time_saved.tm_mon);
//
//    USART_Tx(USART0,'\r');
//    USART_Tx(USART0,'\n');
//}


void decode(volatile char *buffer)
{
//    volatile char *p;
//    p = buffer;
//    uint8_t i = 0;
//    bool m = 0;
//    if(rx_data_ready)
//    {
//        rx_data_ready = 0;
//
//        while(buffer[i] != '\0') // find leading char
//        {
//            if(buffer[i] == '#') m = 1;
//            p++;  // pointer to where message starts
//            if(i++ >= buffersize - 2) return;
//        }
//
//        if(m)
//        {
//            switch(buffer[i-1])
//            {
//            case '1':
//
//                sendMAC();
//                MYdevice.production_mode_timer = 0;  // stay in testmode
//
//                break;
//
//            case '2':
//
//                sendVER();
//                MYdevice.production_mode_timer = 0;  // stay in testmode
//
//                break;
//
//            case '3':
//
//                break;
//
//            case '4':
//
//                break;
//            default:
//
//                break;
//
//            }
//        }
//    }
}
//END OF PORTED UART DIAGNOSTIC CODE






void ps_save(uint16_t key, uint8_t *pValue, uint8_t size)
{
//    uint16_t resultcode = 0xFFFF;
    uint8_t data = 0xFF;
    data =  *pValue;
    sl_status_t sc;

    sc = sl_bt_nvm_save(key, size, pValue);
//    sl_app_assert(sc == SL_STATUS_OK,
//                  "[E: 0x%04x] Failed to set advertiser data\n",
//                  (int)sc);
}

void ps_load(uint16_t key, uint8_t *pValue, uint8_t size)
{
  sl_status_t sc;

    sc = sl_bt_nvm_load(key, size, size, pValue);
//    sl_app_assert(sc == SL_STATUS_OK,
//                  "[E: 0x%04x] Failed to set advertiser data\n",
//                  (int)sc);

}

void set_last_set_time_info(void) {

  ps_save(LAST_SET_TIME, &last_set_time, sizeof(last_set_time));
//  return ret;
}

uint8_t checkKEY(void)
{
    uint8_t random[32] = {0};
    uint8_t i = 0;
    uint8_t n = 0;
    ps_load(0x4001, random, 32);  // if not registered send random key

//    char_send(random, 1);
    for(i = 0; i < 32; i++)
    {
//        char_send(random[i], 1);
        if (random[i] == 0) n++;
    }

    if (n > 26)
    {
        return 1;
    }
    return 0;
}


void updateDevice(void)
{
    if((MYdevice.sku & 0x0F) == 1)
    {
#ifndef ECOLAB_DEV
        coll.FilterTime = (32768*1800); // 30 minutes express
#else
        coll.FilterTime = (32768*60); // 1 minutes express for EcoLab development
#endif
    }
    else if((MYdevice.sku & 0x0F) == 3) // 4 hour trapper
    {
#ifndef ECOLAB_DEV
        coll.FilterTime = (32768*14400); // 4 hours trapper
#else
        coll.FilterTime = (32768*60); // 1 minutes express for EcoLab development
#endif
    }
    else if((MYdevice.sku & 0x0F) == 2) // 30 mins ambushr
    {
#ifndef ECOLAB_DEV
        coll.FilterTime = (32768*1800); // 30 mins ambush
#else
        coll.FilterTime = (32768*60); // 1 minutes express for EcoLab development
#endif
    }
    else if((MYdevice.sku & 0x0F) == 4) // 30 mins TREX
    {
#ifndef ECOLAB_DEV
        coll.FilterTime = (32768*1800); // 30 mins ambush
#else
        coll.FilterTime = (32768*60); // 1 minutes express for EcoLab development
#endif
    }
    else coll.FilterTime = 0;

    //if(MYdevice.sku != 4)
    //    GPIO_PinModeSet(gpioPortA, 0, gpioModeInputPull, 1);     // make input for all other traps
    //else
    //    GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);      // make output for TREX
    //    GPIO_PinModeSet(gpioPortA, 1, gpioModeInput, 1);         // Movement

}

void deviceLOADx(void)
{
  uint8_t data1 = 0;
  uint8_t data2 = 0;
  uint8_t data3 = 0;

    ps_load(DEVICE_SKU_PS,&data1,1);  // check to see what type this device is
//    data1 = 0x01; //EXPRESS
//    data1 = 0x53; //LoRa EXPRESS
    MYdevice.sku = data1;  // update shadow variable


    ps_load(DEVICE_CONFIGURATION_PS, &data2, 1);
//    data2 = 0x01;
    MYdevice.detection_threshold = data2;

    ps_load(DEVICE_REG_PS, &data3, 1);  // 1 = registration   0 = unregistered  2 = development  set to unregistered by default if never powered on
//    data3 = 0x01;
    MYdevice.registration = data3;  //update the shadow variable
//    char_send(MYdevice.registration, 1);
//    if(MYdevice.sku==0xFF)
//      data3=1;
    ps_load(CONFIG_LORA_ENABLE_OFFSET, &MYdevice.lora.lora_en, 1);  // check to see what type this device is
//    MYdevice.lora.lora_en = 1;
    switch(data3)
    {
        case 0x00:
            MYdevice.operatingMode = unregsleep;      // taken out of service mode
          //  gecko_cmd_hardware_set_lazy_soft_timer(0,0,TIMER3_HANDLE,0); // turn off timer this is to turn off timer after production mode is finished
            break;

        case 0x01:
            MYdevice.operatingMode = registered;      // in service mode
//            MYdevice.lora.lora_en = 1;
//            reg_flag = 1;
//            MYdevice.lora.manual_join_delay_tm = 1500;
            if(MYdevice.lora.lora_en == 1)
                reg_flag = 1;

            break;

        case 0x02:
            MYdevice.operatingMode = development;    // backward compatability mode
            break;

        case 0x03:
          MYdevice.operatingMode = factorysleep;   // intial operating mode
          break;
        default:
          break;

           // gecko_cmd_hardware_set_lazy_soft_timer(0,0,TIMER3_HANDLE,0); // turn off timer  this is to turn off timer after production mode is finished

    }
    updateDevice();

}

void deviceClear(void)
{
    MyEvents.EventIndex = 0;  // all Events were read by client reset buffer

    conpoint = conbuffer; // reset pointer
    currentindex = 0;
    minTemp = 228;   // clear min temperature
    maxTemp = 60;    // clear max temperature
    if(MYdevice.registration == 1)
      {
        SuppressDetect = 1;
      }
        CollMemoryFULL = 0;

        sc = sl_bt_system_set_lazy_soft_timer(SUPPRESS_TIME, SUPPRESS_TIME/10,  TIMER7_HANDLE, 1);
#ifndef ECOLAB_DEV
        sc = sl_bt_system_set_lazy_soft_timer((32768 * 5), 327,  TIMER6_HANDLE, 1);
#else
        sc = sl_bt_system_set_lazy_soft_timer((32768), 327,  TIMER6_HANDLE, 1);
#endif
//    }
    coll.DetectionsCounter = 0;
    coll.EventsCounter = 0;
    coll.Filter = 0;  // stop filtering
//    swSetupAdvBeaconing();
    advertise_stop();
}

void defaultsLOAD(void)
{
    uint8_t data = 0;


#ifdef FACTORY

    ps_save(DEVICE_INTIALIZED_PS, (uint8_t *)0xFF, 1);  // write 0xFF to return to factory mode

#endif

    if(checkKEY()== 1)
    {
        data = 0xFF;
        ps_save(DEVICE_INTIALIZED_PS, &data, 1);  // write 0xFF to return to factory mode
    }
    MYdevice.tx_pwr = 0;
    ps_load(DEVICE_INTIALIZED_PS, &data, 1);  // saved flag to know if device was intialized
    if(data != 0xA9)  // generate a random masterkey if none generated in past
        {
            genRANDOM();
            data = 0xA9;
            ps_save(DEVICE_INTIALIZED_PS, &data, 1);  // set flag for next time  so we don't intialize again
            data = 0x03;
            ps_save(DEVICE_REG_PS, &data, 1);  // 1 = registration   0 = unregistered  3 = factory  set to unregistered by default if never powered on
            MYdevice.registration  = 0x03;  //update the shadow variable                         2 = development
            //MYdevice.operatingMode = factorysleep;   //
//            MYdevice.detection_threshold = THRESHOLD_N;
            //ps_save(DEVICE_CONFIGURATION_PS, &MYdevice.detection_threshold, 1);
            //MYdevice.sku = DEVICE_SKU;  // define type here
            //ps_save(DEVICE_SKU_PS,&MYdevice.sku,1);  //define a default type
        }
#ifdef TREX
    ps_load(DEVICE_SKU_PS,&MYdevice.sku,1);  //define a default type
    MYdevice.sku &= 0xF0;  // define TREX here
    MYdevice.sku |= 0x04;  // define TREX here
    ps_save(DEVICE_SKU_PS,&MYdevice.sku,1);  //define a default type

#endif

#ifdef DEVELOPMENT    //  development mode for compatability

    data = 0x02;
    ps_save(DEVICE_REG_PS, &data, 1);  // 1 = registration   0 = unregistered  2 = development  3 = factory set to unregistered by default if never powered on
    MYdevice.registration = 0x02;  //update the shadow variable
    MYdevice.operatingMode = development;

#endif
    }


void wakeDevice(void)
{
//  sl_status_t sc;
//  sl_bt_advertiser_create_set(&advertising_set_handle);
    if(MYdevice.operatingMode == factorysleep)
    {
        MYdevice.operatingMode = factorywake;
        sc = sl_bt_system_set_lazy_soft_timer((49152),4915,TIMER3_HANDLE,0); // restart timer for time repeating
//        swSetupAdvBeaconing();
//        MYdevice.lora.join_test=1;
//        char_send('T',1);
    }
    else if(MYdevice.operatingMode == unregsleep)
    {
        MYdevice.operatingMode = unregwake;
        sl_bt_system_set_lazy_soft_timer((49152),4915,TIMER3_HANDLE,0); // restart timer for time repeating
        MYdevice.lora.unreg_flag = 1;
//        swSetupAdvBeaconing();
//        MYdevice.lora.join_test=1;
//        char_send('U',1);
    }
    WDOG_Enable(true);
    WDOG_Feed();

//MYdevice.lora.lora_alive_flag = 1;
//MYdevice.lora.lora_alive_status = 3;
    MYdevice.wakeTimer = 0;  // reset wake timer
//    MYdevice.wakeFlag = 1;
    intActCntr = 0;
    intMovCntr = 0;
}

//void genRANDOM(void)
//{
//    uint8_t n[32];
//    uint8_t random[32] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x010,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x020};
//    uint8_t timestamp[4] = {0x00,0x00,0x00,0x00};  //starting time stamp when reset
//    uint8_t i = 0;
//    bd_addr address;
//    uint8_t address_type;
//
//    sc = sl_bt_system_get_identity_address(&address, &address_type);
//
//    adcrndsetup();
//
//    while(i < 32)
//    {
//        n[i] += adcrnd();
//        i++;
//    }
//    CMU_ClockEnable(cmuClock_CRYPTO, true); // enable the clock to the crypto accelerator
//    CRYPTO_SHA_256(CRYPTO, n, 32, random);  // whitening
//    ps_save(DEVICE_MASTERKEY_PS, random, 32);   // save to use in authentication
//
//    if(checkKEY()== 1)
//    {
//        random[0] = address.addr[0];
//        random[1] = address.addr[1];
//        random[2] = address.addr[2];
//        random[3] = address.addr[3];
//        random[4] = address.addr[4];
//        random[5] = address.addr[5];
//        ps_save(DEVICE_MASTERKEY_PS, random, 32);   // save to use in authentication
//    }
//
//    ps_save(DEVICE_TIMESTAMP_PS, timestamp, 4);   // intial time stamp value
//}

uint8_t genRANDOM(void)
{
    mbedtls_entropy_context MYrandomnumber;
    unsigned char outputrandbuff[32] = {0};
    uint8_t i;
//    size_t actualsize = 0;
    int result;
    sl_status_t nvmstatus;

    mbedtls_entropy_init( &MYrandomnumber );
//    result = mbedtls_entropy_poll( &MYrandomnumber, outputrandbuff, 32, &actualsize );
    result = mbedtls_entropy_func( &MYrandomnumber, outputrandbuff, 32);

//    mbedtls_entropy_rail_init( &MYrandomnumber );
//    result = mbedtls_entropy_rail_poll( &MYrandomnumber, outputrandbuff, 32, &actualsize );

    if(result == 0)// && actualsize == 32)
    {
       nvmstatus = sl_bt_nvm_save(DEVICE_MASTERKEY_PS, 32, outputrandbuff);
//       for(i=0; i<32; i++)
//         char_send(outputrandbuff[i], 1);
       if(nvmstatus == SL_STATUS_OK) return 0;
       else return 2;
    }
    else return 1;
//  return 1;
}




uint8_t AuthenticateRegisterUser(uint8array *writeValue, deviceDATA *device)
{
    uint8_t scratch52[52] = {0};           // temporary working array for openkey
    uint8_t mysessionkeyarray[32] = {0};   // SHA256 hash of my open session key
    uint8_t siteKEY[16] = {0};
    uint8_t result = 0xFF;           // result of encryption
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t device_sku = 0;
    uint8_t prev_reg = 0;
    size_t valLEN = 0;
    mbedtls_sha256_context MYsha256ctx;
    volatile int shaRESULT;
    sl_status_t nvmstatus;

    prev_reg = device->registration;
    device->authSTATUS = 1;     //  set to fail until authentication is

    mbedtls_sha256_init( &MYsha256ctx );

    sl_bt_nvm_load( DEVICE_SITEKEY_PS, 16, &valLEN, siteKEY );


    i = 0;
    while(i < 16)    // 36 bytes  openkey  [0]...[31]  unix time [32][33][34][35]
    {
        scratch52[i] = siteKEY[i];   // site key known to test app and device
        i++;
    }
    i = 16;
    j = 0;
    while(i < 48)    // 36 bytes  openkey  [0]...[31]  unix time [32][33][34][35]
    {
        scratch52[i] = openKEY[j];   // factory key known to test app and device
        i++;
        j++;
    }

     scratch52[19] |= writeValue->data[32];  //unix time MSB
     scratch52[23] &= writeValue->data[33];  //unix time
     scratch52[27] ^= writeValue->data[34]; //unix time
     scratch52[37] &= writeValue->data[35]; //unix time LSB

    i = 48;
    j = 32;
    while(i < 52)    // 36 bytes of session key
    {
        scratch52[i] = writeValue->data[j];   // unix time at [32][33][34][35]
        i++;
        j++;
    }

    shaRESULT =  mbedtls_sha256_ret( scratch52, (size_t)52, mysessionkeyarray, 0 );
    result = 0;
    i = 0;
    while(i < 32)    // 32 bytes of session key  is this a session key?
    {
        if(mysessionkeyarray[i] != writeValue->data[i]) result = 1;  // no match
        i++;
    }

    if(result == 0 && shaRESULT == 0)
    {
        device->authSTATUS = 2;  // authentication pass
//        device_sku = writeValue->data[37];
//        ps_save(DEVICE_SKU_PS, &device_sku, 1); // save SKU
//        updateDevice();
        if(writeValue->data[54] == 0xFE && writeValue->data[37] == 0xFE)
          {
            MYdevice.lora.lora_alive_flag = 1;
            MYdevice.lora.lora_alive_status = 3;
          }
        else
          {
#ifdef GEN2
#ifndef LORA
        device->registration = writeValue->data[44];
#else
        if(writeValue->data[54] == 0 || writeValue->data[54] == 1 )
          device->registration = writeValue->data[54];

#endif
#else
        device->registration = writeValue->data[38];
#endif
        nvmstatus = sl_bt_nvm_save(DEVICE_REG_PS, 1, &device->registration); // save configuration
        if(nvmstatus != SL_STATUS_OK)return 0xF0;
        device->detection_threshold = writeValue->data[36];
        nvmstatus = sl_bt_nvm_save(DEVICE_CONFIGURATION_PS, 1, &device->detection_threshold); // save configuration
        if(nvmstatus != SL_STATUS_OK)return 0xF1;
        if(device->registration == 0x01)
          {
            device->operatingMode = registered;  // we are in registered mode
//            if(prev_reg != device->registration)
              reg_flag = 1;
            lora_hb_cnt=0;
//            char_send('A',0);
//            char_send(device->registration, 1);
//            if(writeValue->data[54]!=0xFF)
//              device->lora.manual_join_delay_tm = 1500;
          }
        else if(device->registration == 0x00)
        {
            device->operatingMode = unregwake;   // we are unregistered
            device->wakeTimer = 0;  // reset timer so we can stay awake until app is finished
            uint8_t site_tmp[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            ps_save(DEVICE_SITEKEY_PS, site_tmp, 16);
            load_default_settings();
            unreg_lora();
            MYdevice.lora.unreg_flag = 1;
            MYdevice.lora.reg_payload_sent = 0;
//            char_send('B',0);
//            char_send(device->registration, 1);
        }
        if(writeValue->data[54]!=0xFF)
          deviceClear();  // tbd
    }
    }
    return result; // match
}

uint8_t AuthenticateSecureUser(uint8array *writeValue, deviceDATA *device)
{
    uint8_t scratch52[52] = {0};           // temporary working array for openkey
    uint8_t mysessionkeyarray[32] = {0};   // SHA256 hash of my open session key
    uint8_t masterKEY[32] = {0};
    uint8_t siteKEY[16] = {0};
    uint8_t result = 0xFF;           // result of encryption
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t prev_reg = 0;
//    size_t valLEN = 0;
    mbedtls_sha256_context MYsha256ctx;
    volatile int shaRESULT;
    sl_status_t nvmstatus;
//    authSTATUS = 1;     //  set to fail until authentication is
//    char_send('a',0);
    device->authSTATUS = 1;     //  set to fail until authentication is
    mbedtls_sha256_init( &MYsha256ctx );
    prev_reg = device->registration;
    if(NewKeyVerify(writeValue) == 1)
    {
        device->authSTATUS = 3; // this is a repeated key
//        char_send('X',0);
        return 1;  // make sure it's not an old key
    }

//    sl_bt_nvm_load( DEVICE_MASTERKEY_PS, 32, &valLEN, masterKEY );
//    sl_bt_nvm_load( DEVICE_SITEKEY_PS, 16, &valLEN, siteKEY );
    (void)ps_load(DEVICE_MASTERKEY_PS, masterKEY, 32);   // load masterkey that was randomly generated
    (void)ps_load(DEVICE_SITEKEY_PS, siteKEY, 16);

    siteKEY[3] |= staticserverKEY[1];
    siteKEY[5] &= staticserverKEY[6];
    siteKEY[9] &= staticserverKEY[7];
    siteKEY[10] ^= staticserverKEY[11];
    siteKEY[12] |= staticserverKEY[3];
    siteKEY[15] ^= staticserverKEY[4];

    i = 0;
    while(i < 16)    // 36 bytes  openkey  [0]...[31]  unix time [32][33][34][35]
    {
        scratch52[i] = siteKEY[i];   // site key known to server and device
        i++;
    }
    i = 16;
    j = 0;
    while(i < 48)    // 36 bytes  openkey  [0]...[31]  unix time [32][33][34][35]
    {
        scratch52[i] = masterKEY[j];   // masterkey known to server and device
        i++;
        j++;
    }

     scratch52[19] |= writeValue->data[32];  //unix time MSB
     scratch52[23] &= writeValue->data[33];  //unix time
     scratch52[27] ^= writeValue->data[34]; //unix time
     scratch52[37] &= writeValue->data[35]; //unix time LSB

    i = 48;
    j = 32;
    while(i < 52)    // 36 bytes of session key
    {
        scratch52[i] = writeValue->data[j];   // unix time at [32][33][34][35]
        i++;
        j++;
    }

    shaRESULT =  mbedtls_sha256_ret( scratch52, (size_t)52, mysessionkeyarray, 0 );

    result = 0;
    i = 0;
//    char_send('b',0);
    while(i < 32)    // 32 bytes of session key  is this a session key?
    {
        if(mysessionkeyarray[i] != writeValue->data[i]) result = 1;  // no match
        i++;
    }
//    char_send('c',0);
    if(result == 0 && shaRESULT == 0)
    {
        //added from 1.29, not sure if still needed.
//        char_send('D',0);
        i = 0;
        if(writeValue->data[50]==0)
          {
            while(i < 36)    // 36 bytes of session key
            {
                keyHold[i] = writeValue->data[i];   // hold key for 24 hours then save
                i++;
            }
            newKey = 1;
            keyValidTimer = 0;
            //end of added section
          }


        device->authSTATUS = 2;  // authentication pass
        //device_sku = writeValue->data[37];
        //updateDevice();
        //ps_save(DEVICE_SKU_PS, &device_sku, 1); // save SKU
        device->detection_threshold = writeValue->data[36];
        nvmstatus = sl_bt_nvm_save(DEVICE_CONFIGURATION_PS, 1, &device->detection_threshold); // save configuration
        if(nvmstatus != SL_STATUS_OK)return 0xF1;
#ifdef GEN2
#ifndef LORA
        device->registration = writeValue->data[44];
#else
//        device->registration = writeValue->data[51];
        if(writeValue->data[54] == 0 || writeValue->data[54] == 1 )
          device->registration = writeValue->data[54];
#endif
#else
        device->registration = writeValue->data[38];
#endif

//        if(&device->registration==0)
//          {
//            load_defualt_settings();
//          }
        nvmstatus = sl_bt_nvm_save(DEVICE_REG_PS, 1, &device->registration); // save configuration
        if(nvmstatus != SL_STATUS_OK)return 0xF0;
        if(device->registration == 0x01)
          {
            device->operatingMode = registered;  // we are in registered mode
//            if(prev_reg != device->registration)
//              reg_flag = 1;
//            lora_hb_cnt=0;
//            if(writeValue->data[54]!=0xFF)
//              device->lora.manual_join_delay_tm = 1500;
          }
        else if(device->registration == 0x00)
        {
            device->operatingMode = unregwake;   // we are unregistered
            device->wakeTimer = 0;  // reset timer so we can stay awake until app is finished
            uint8_t site_tmp[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            ps_save(DEVICE_SITEKEY_PS, site_tmp, 16);
            load_default_settings();
            unreg_lora();
            MYdevice.lora.unreg_flag = 1;
            MYdevice.lora.reg_payload_sent = 0;
        }
        if(writeValue->data[54]!=0xFF)
          deviceClear();  // tbd
    }

    return result; // match
}


uint8_t AuthenticateFactoryUser(uint8array *writeValue, deviceDATA *device)
{
    uint8_t scratch52[52] = {0};           // temporary working array for openkey
    uint8_t mysessionkeyarray[32] = {0};   // SHA256 hash of my open session key
    //uint8_t siteKEY[16] = {0};     // not needed
    uint8_t result = 0xFF;           // result of encryption
    uint8_t i = 0;
    uint8_t j = 0;
//    size_t valLEN = 0;
    mbedtls_sha256_context MYsha256ctx;
    volatile int shaRESULT;
    sl_status_t nvmstatus;

    device->authSTATUS = 1;     //  set to fail until authentication is
    mbedtls_sha256_init( &MYsha256ctx );


    i = 0;
    while(i < 16)    // 16 bytes  staticsiteey  [0]...[15]  unix time [32][33][34][35]
    {
        scratch52[i] = staticsiteKEY[i];   //  static site key known to test app and device
        i++;
    }
    i = 16;
    j = 0;
    while(i < 48)    // 36 bytes  factorykey  [0]...[31]  unix time [32][33][34][35]
    {
        scratch52[i] = factoryKEY[j];   // factory key known to test app and device
        i++;
        j++;
    }

     scratch52[19] |= writeValue->data[32];  //unix time MSB
     scratch52[23] &= writeValue->data[33];  //unix time
     scratch52[27] ^= writeValue->data[34]; //unix time
     scratch52[37] &= writeValue->data[35]; //unix time LSB

    i = 48;
    j = 32;
    while(i < 52)    // 36 bytes of session key
    {
        scratch52[i] = writeValue->data[j];   // unix time at [32][33][34][35]
        i++;
        j++;
    }

    shaRESULT =  mbedtls_sha256_ret( scratch52, (size_t)52, mysessionkeyarray, 0 );

    result = 0;
    i = 0;
    while(i < 32)    // 32 bytes of session key  is this a session key?
    {
        if(mysessionkeyarray[i] != writeValue->data[i]) result = 1;  // no match
        i++;
    }

    if(result == 0 && shaRESULT == 0)
    {
        device->authSTATUS = 2;  // authentication pass
        MYdevice.wakeTimer = 0;
        if(writeValue->data[37] == 0xFE)
        {
                MYdevice.lora.lora_alive_flag = 1;
                MYdevice.lora.lora_alive_status = 3;
        }
        else
        {
  //        if(writeValue->data[37] !=0xFF)
//          {
             device->sku = writeValue->data[37];
             nvmstatus = sl_bt_nvm_save(DEVICE_SKU_PS, 1, &device->sku); // save configuration
               updateDevice(); // tbd
//          }
          if(writeValue->data[36] != 0xFF)
          {
             device->detection_threshold = writeValue->data[36];
             nvmstatus = sl_bt_nvm_save(DEVICE_CONFIGURATION_PS, 1, &device->detection_threshold); // save configuration
          }
          deviceClear();  // tbd
  //        if((writeValue->data[52]==1)&&(device->registration==0x03))
          if((writeValue->data[52]==1)&&(writeValue->data[54]==0x03))
            {
              MYdevice.lora.join_test=1;
              MYdevice.lora.join_status = 2;
              MYdevice.lora.ping_result = 3;
              ping_cnt = 0;
  //            char_send('A',0);
            }
         }
        // device cannot be registered in factory mode
    }
    return result; // match
}

uint8_t AuthenticateInstallationUser(uint8array *writeValue, deviceDATA *device)
{
    uint8_t scratch52[52] = {0};           // temporary working array for openkey
    uint8_t mysessionkeyarray[32] = {0};   // SHA256 hash of my open session key
    uint8_t unix[4] = {0x63, 0x86, 0x1F, 0x76};
    //uint8_t siteKEY[16] = {0};     // not needed
    uint8_t result = 0xFF;           // result of encryption
    uint8_t i = 0;
    uint8_t j = 0;
//    size_t valLEN = 0;
    mbedtls_sha256_context MYsha256ctx;
    volatile int shaRESULT;
    sl_status_t nvmstatus;
    device->authSTATUS = 1;     //  set to fail until authentication is
    mbedtls_sha256_init( &MYsha256ctx );

//char_send('A',0);
//    factoryInstallKEY[27] |= writeValue->data[32];  //unix time MSB
//    factoryInstallKEY[4] &= writeValue->data[33];  //unix time
//    factoryInstallKEY[16] ^= writeValue->data[34]; //unix time
//    factoryInstallKEY[1] &= writeValue->data[35]; //unix time LSB

    i = 0;
    while(i < 16)    // 16 bytes  staticsiteey  [0]...[15]  unix time [32][33][34][35]
    {
        scratch52[i] = staticInstallKEY[i];   //  static site key known to test app and device
        i++;
    }
    i = 16;
    j = 0;
    while(i < 48)    // 36 bytes  factorykey  [0]...[31]  unix time [32][33][34][35]
    {
        scratch52[i] = factoryInstallKEY[j];   // factory key known to test app and device
        i++;
        j++;
    }

     scratch52[43] |= writeValue->data[32];  //unix time MSB
     scratch52[20] &= writeValue->data[33];  //unix time
     scratch52[32] ^= writeValue->data[34]; //unix time
     scratch52[17] &= writeValue->data[35]; //unix time LSB
//    scratch52[43] |= unix[0];  //unix time MSB
//    scratch52[20] &= unix[1];  //unix time
//    scratch52[32] ^= unix[2]; //unix time
//    scratch52[17] &= unix[3]; //unix time LSB

    i = 48;
    j = 32;
    while(i < 52)    // 36 bytes of session key
    {
        scratch52[i] = writeValue->data[j];   // unix time at [32][33][34][35]
//        scratch52[i] = unix[j];   // unix time at [32][33][34][35]
        i++;
        j++;
    }
    shaRESULT =  mbedtls_sha256_ret( scratch52, (size_t)52, mysessionkeyarray, 0 );
    result = 0;
    i = 0;
    while(i < 32)    // 32 bytes of session key  is this a session key?
    {
        if(mysessionkeyarray[i] != writeValue->data[i]) result = 1;  // no match
        i++;
    }
    if(result == 0 && shaRESULT == 0)
    {
//        char_send('B',0);
        device->authSTATUS = 2;  // authentication pass
        MYdevice.wakeTimer = 0;
//        if(writeValue->data[37] == 0xFF && writeValue->data[54] == 0xFF)
        if(writeValue->data[54] == 0xFF && device->sku == 0xFF)
        {
            if(writeValue->data[52]==1)
              {
                MYdevice.lora.join_test=1;
                MYdevice.lora.region = writeValue->data[44];
//                lora_api_internal_join__restart();
                MYdevice.lora.join_status = 2;
                MYdevice.lora.install_join = 1;
                MYdevice.lora.ping_result = 3;
                ping_cnt = 0;
//                char_send('A',0);
              }
//           device->sku = writeValue->data[37];
//           nvmstatus = sl_bt_nvm_save(DEVICE_SKU_PS, 1, &device->sku); // save configuration
//             updateDevice(); // tbd
        }
        else if(writeValue->data[54] == 0xFE && writeValue->data[37] == 0xFE)
        {
                MYdevice.lora.lora_alive_flag = 1;
                MYdevice.lora.lora_alive_status = 3;
//                char_send('A',0);
//           device->sku = writeValue->data[37];
//           nvmstatus = sl_bt_nvm_save(DEVICE_SKU_PS, 1, &device->sku); // save configuration
//             updateDevice(); // tbd
        }
//        if(writeValue->data[36] != 0xFF)
//        {
//           device->detection_threshold = writeValue->data[36];
//           nvmstatus = sl_bt_nvm_save(DEVICE_CONFIGURATION_PS, 1, &device->detection_threshold); // save configuration
//        }
        deviceClear();  // tbd

        // device cannot be registered in factory mode
    }
    return result; // match
}

uint8_t AuthenticateLockedUser(uint8array *writeValue, deviceDATA *device)
{
    uint8_t scratch52[52] = {0};           // temporary working array for openkey
    uint8_t mysessionkeyarray[32] = {0};   // SHA256 hash of my open session key
//    uint8_t siteKEY[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};     // not needed
    uint8_t siteKEY[16] = {};     // not needed
    uint8_t result = 0xFF;           // result of encryption
    uint8_t i = 0;
    uint8_t j = 0;
//    size_t valLEN = 0;
    mbedtls_sha256_context MYsha256ctx;
    volatile int shaRESULT;
    sl_status_t nvmstatus;
    device->authSTATUS = 1;     //  set to fail until authentication is
    mbedtls_sha256_init( &MYsha256ctx );
    (void)ps_load(DEVICE_SITEKEY_PS, siteKEY, 16);

    i = 0;
    while(i < 16)    // 16 bytes  staticsiteey  [0]...[15]  unix time [32][33][34][35]
    {
        scratch52[i] = siteKEY[i];   //  static site key known to test app and device
        i++;
    }
    i = 16;
    j = 0;
    while(i < 48)    // 36 bytes  factorykey  [0]...[31]  unix time [32][33][34][35]
    {
        scratch52[i] = genericKEY[j];   // factory key known to test app and device
        i++;
        j++;
    }

     scratch52[19] |= writeValue->data[32];  //unix time MSB
     scratch52[23] &= writeValue->data[33];  //unix time
     scratch52[27] ^= writeValue->data[34]; //unix time
     scratch52[37] &= writeValue->data[35]; //unix time LSB

    i = 48;
    j = 32;
    while(i < 52)    // 36 bytes of session key
    {
        scratch52[i] = writeValue->data[j];   // unix time at [32][33][34][35]
        i++;
        j++;
    }

    shaRESULT =  mbedtls_sha256_ret( scratch52, (size_t)52, mysessionkeyarray, 0 );

    result = 0;
    i = 0;
    while(i < 32)    // 32 bytes of session key  is this a session key?
    {
        if(mysessionkeyarray[i] != writeValue->data[i]) result = 1;  // no match
        i++;
    }

    if(result == 0 && shaRESULT == 0)
    {
        if(writeValue->data[54] == 0x03)
          device->authSTATUS = 5;  //recovery of master key
        else
          device->authSTATUS = 1;

//        if(writeValue->data[37] !=0xFF)
//        {
//           device->sku = writeValue->data[37];
//           nvmstatus = sl_bt_nvm_save(DEVICE_SKU_PS, 1, &device->sku); // save configuration
//             updateDevice(); // tbd
//        }
//        if(writeValue->data[36] != 0xFF)
//        {
//           device->detection_threshold = writeValue->data[36];
//           nvmstatus = sl_bt_nvm_save(DEVICE_CONFIGURATION_PS, 1, &device->detection_threshold); // save configuration
//        }
        deviceClear();  // tbd
        // device cannot be registered in factory mode
    }
//    char_send(MYdevice.authSTATUS,0);
    return result; // match
}



//all the following code was copies from v1.29
uint8_t NewKeyVerify(uint8array *writeValue)
{
    uint8_t scratch36[36] = {0};
    uint8_t i = 0, result = 0;
    uint8_t j = 0;

    while(j < MAXUSEDKEYS )
    {
        (void)ps_load(UsedKeyLocations[j], scratch36, 36);
        i = 0;
        result = 0;
        while(i < 36)    // 36 bytes of session key  is this a session key used in past?
        {
            if(scratch36[i] != writeValue->data[i]) result = 1;  // no match
            i++;
        }
        if(result == 0) return 1;  // soon as  a key found return
        j++;
    }
    return 0; // no key found
}

uint8_t NewKeySave(void)
{
    ps_save(UsedKeyLocations[UsedKeyIndex], keyHold, 36);  // save current key
    newKey = 0;  // flag new key needed
    if(++UsedKeyIndex >= MAXUSEDKEYS)UsedKeyIndex = 0;    // start over
    return 0;
}

void KeyStorageInit(void)
{
    uint8_t scratch36[36] = {0};    // call once when initializing device
    uint8_t i = 0;

    while(i < MAXUSEDKEYS)
    {
        ps_save(UsedKeyLocations[i], scratch36, 36);  // save current key
        i++;
    }
}



void checkHALL(void)
{
    GPIO_PinOutSet(gpioPortC, 1);   // power up Hall Sensor

    if(GPIO_PinInGet(gpioPortB, 2))
    {
        coll.EventsCounter = 0;  // trap is unsnapped leave power on
    }
    else
    {
        GPIO_PinOutClear(gpioPortC, 1);  // power off Hall Sensor trap has snapped
        coll.EventsCounter = 1;
    }

}

/**************************************************************************//**
 * @brief  Initialize IADC function
 *****************************************************************************/
void initIADC (void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  // Enable IADC0 and GPIO clock branches
  CMU_ClockEnable(cmuClock_IADC0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Reset IADC to reset configuration in case it has been modified by
  // other code
  IADC_reset(IADC0);

  // Select clock for IADC
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);  // FSRCO - 20MHz

  // Modify init structs and initialize
  init.warmup = iadcWarmupKeepWarm;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  // Configuration 0 is used by both scan and single conversions by default
  // Use unbuffered AVDD as reference
  initAllConfigs.configs[0].reference =  iadcCfgReferenceInt1V2; //iadcCfgReferenceVddx;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                             CLK_ADC_FREQ,
                                             0,
                                             iadcCfgModeNormal,
                                             init.srcClkPrescale);

  // Assign pins to positive and negative inputs in differential mode
  initSingleInput.posInput   = iadcPosInputAvdd; // iadcPosInputPortCPin4;   // PC04 -> P25 on BRD4001 J102
  initSingleInput.negInput   = iadcNegInputGnd;  //iadcNegInputPortCPin5;   // PC05 -> P27 on BRD4001 J102

  // Initialize the IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize the Single conversion inputs
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
  GPIO->IADC_INPUT_1_BUS |= IADC_INPUT_1_BUSALLOC;
}
//in BG22, the temp measurement is not from internal ADC source, it will use TEMPDRV
void TempMeasure(void)
{
  TEMPDRV_Init();
  TEMPDRV_Enable(1);
  MTsystem.temperaturecurrent = TEMPDRV_GetTemp() + 128;
  TEMPDRV_Enable(0);
  TEMPDRV_DeInit();
}
uint16_t measureBatt(void)
{
  // Start IADC conversion
  IADC_command(IADC0, iadcCmdStartSingle);

  // Wait for conversion to be complete
  while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
              | _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively

  // Get ADC result
  sample = IADC_pullSingleFifoResult(IADC0).data;

  // adjust to a percentage
  DeviceBatt.batterylvl = ((sample - 1700) / 8);
  // if too low just say 5 percent
  if(sample < 1700) DeviceBatt.batterylvl = 5;
  // if above 3 volts limit at 100 percent
  else if(sample > 2500) DeviceBatt.batterylvl = 100;

  //battery voltage in increments of 100mV
  DeviceBatt.batt_read = (((sample * 48) / 0xFFF)&0xFFFF);
  //battery voltage as float
  DeviceBatt.batteryvolts = (sample * 4.8) / 0xFFF;

  return DeviceBatt.batt_read;

}

//batt array initialization to fill array on power up
void BATTcheckinit(void)
{
  uint8_t i = 0;
  initIADC();
  //Initialize fucntion to fill bat measurement array with 8 readings
  for(i = 0; i < BATTERYSAMPLES; i++)
  {
    //ready battery level
    DeviceBatt.battADCsamples[i] = measureBatt();

    //add to battSUM
    DeviceBatt.battSUM += DeviceBatt.battADCsamples[i];
  }
  IADC_reset(IADC0);
  DeviceBatt.battAVG = DeviceBatt.battSUM>>3;
  DeviceBatt.battTIMER = BATTSAMPLEPERIOD;
  DeviceBatt.battADCindex = 0;
  DeviceBatt.battDISP = 100;
}


void BATTcheck(void)
{
//  uint8_t i = 0;



//  {

      initIADC();
    /*
     * battADCsample array is already full after initialization therefore in order to maintain battAVG
     * we just need to remove the battMeasure value that is being updated from battSUM
     * and add/replace the new value
    */

    //remove battMeasure value from battSUM that is being updated
      DeviceBatt.battSUM -= DeviceBatt.battADCsamples[DeviceBatt.battADCindex];

    //update the array with the new measurement
      DeviceBatt.battADCsamples[DeviceBatt.battADCindex] = measureBatt();
      IADC_reset(IADC0);
    //add the new value to battSUM
      DeviceBatt.battSUM += DeviceBatt.battADCsamples[DeviceBatt.battADCindex];

    //reset index if greater that BATTERYSAMPLES
    if(DeviceBatt.battADCindex++ >= BATTERYSAMPLES)
      DeviceBatt.battADCindex = 0;


//    for(i = 0; i < BATTERYSAMPLES; i++)
//    {
//      battSUM += battery->battADCsamples[i];
//    }

//    battAVG = battSUM/BATTERYSAMPLES;

    //since BATTERYSAMPLES is 8, we just need to divide by 8 (left shift 3) to get battAVG
    DeviceBatt.battAVG = DeviceBatt.battSUM>>3;
    // adjust to a percentage
    DeviceBatt.battDISP = ((DeviceBatt.battAVG - 1700) / 8);
    // if too low just say 5 percent
    if(DeviceBatt.battAVG < 1700)
      DeviceBatt.battDISP = 5;
    // if above 3 volts limit at 100 percent
    else if(DeviceBatt.battAVG > 2500)
      DeviceBatt.battDISP = 100;  // if above 3 volts limit at 100 percent

    //save bat measurement to NVM for retrival on powerup
//    ps_save(BATT_SAVE, DeviceBatt.battAVG , 2);

//  }

}


//BIT ACTIVATION (IO1) (PC01)
//void Pin1IntHandler(void)
void Pin1IntHandler(uint8_t intnumber)
{
  intFLAGSeven = GPIO_IntGet();   // Activation interrupt PA0

#ifndef TREX
  if(intActCntr++ > 2)wakeDevice();    // count interrupts
#endif

#ifndef TREX
  //if(MYdevice.sku != 4)
  //{
#ifdef NEW_HW
  if(GPIO_PinInGet(gpioPortB, 1))
#else
  if(GPIO_PinInGet(gpioPortC, 1))
#endif
          SignalActivation(1);
      else
          SignalActivation(0); // falling edge so lets measure the pause
#endif
 // }
  //else  SignalTOT();

  GPIO_IntClear(GPIO_IntGet());

}


//BIT_MOVEMENT (IO2) (PB02)
//void Pin2IntHandler(void)
void Pin2IntHandler(uint8_t intnumber)
{
  intFLAGSodd = GPIO_IntGet();   // PA1 movement

//#ifdef TREX
//  if(intMovCntr++ > 2)wakeDevice();    // count interrupts
//#endif

#ifndef TREX
  //if(MYdevice.sku != 4)
  //{
      if(GPIO_PinInGet(gpioPortB, BIT_MOVEMENT))
          SignalMovement(1);
      else
          SignalMovement(0);
  //}
#else
      //if(TREXarmed)
      //{
      //    TREXarmed = 0;
      //    SignalTOT();
      //}
#endif

  GPIO_IntClear(GPIO_IntGet());
}


//MAGSENSE Input
void Pin3IntHandler(uint8_t intNo)
{
//  if(SuppressDetect==0)
  uint8_t tmp;
//  pin3flag = 1;
//  pin3cnt++;
  tmp = intNo;
  tmp = GPIO_IntGet();   // PA1 movement
//  pin3state = GPIO_PinInGet(gpioPortC, 0);
  GPIO_IntClear(GPIO_IntGet());
//  send_string("MAG");
//  char_send('M',1);

}
//extern UARTDRV_Handle_t sl_uartdrv_usart_testUART_handle;

void configPINS(void)
{
  GPIO_PinModeSet( gpioPortB, BIT_MOVEMENT, gpioModeInputPull, 1 );  //IO2
  GPIO_PinModeSet( gpioPortC, BIT_ACTIVATION, gpioModeInputPull, 1 );  //IO1
  GPIO_PinModeSet( gpioPortC, 3, gpioModeInputPull, 0 );  //TP5
  GPIO_PinModeSet( gpioPortC, 5, gpioModeInputPull, 0 );  //TP4
  GPIO_PinModeSet( gpioPortB, 0, gpioModePushPull, 1 );   // PWRCAP
  GPIO_PinOutSet( gpioPortB, 0 );  // PWRCAP   power up sensor

//  GPIO_PinModeSet( gpioPortC, 0, gpioModeInputPull, 1 );  //TP4
//  GPIO_PinModeSet( gpioPortC, 2, gpioModeInputPull, 1 );  //TP4
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIOINT_Init();

  GPIOINT_CallbackRegister( 1, Pin1IntHandler);
  GPIOINT_CallbackRegister( 2, Pin2IntHandler);

  GPIO_IntConfig( gpioPortB, BIT_MOVEMENT, 1, 1, 1);
  GPIO_IntConfig( gpioPortC, BIT_ACTIVATION, 1, 1, 1);

//  UARTDRV_DeInit(sl_uartdrv_usart_testUART_handle);
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------
void safe_c_str_init(struct safe_c_str *c_str_p, char *buf_p, uint8_t max_chars)
{
    c_str_p->buf_p = buf_p;
    c_str_p->max_chars = max_chars; // includes space for last '\0' char.
    c_str_p->num_chars = 0;
    c_str_p->buf_p[0] = '\0';
}
//-----------------------------------------------------------------
const char *safe_c_str_get(struct safe_c_str *c_str_p)
{
    return (const char *)c_str_p->buf_p;
}
//-----------------------------------------------------------------
void safe_c_str_append(struct safe_c_str *c_str_p, const char *src_p, uint8_t num_chars)
{
    uint8_t remaining_bytes = (c_str_p->max_chars - 1) - c_str_p->num_chars;
    uint8_t max_bytes_to_copy = MIN(remaining_bytes, num_chars);

    for (uint8_t i = 0; i < max_bytes_to_copy; i++)
    {
        uint8_t src = src_p[i];
        if ((src != '\r') && (src != '\n') && (src != '\0'))
        {
            c_str_p->buf_p[c_str_p->num_chars++] = src;
        }
        else
        {
            // ensure last byte is null (a valid c-string)
            c_str_p->buf_p[c_str_p->num_chars] = '\0';
            return;
        }
    }
    // ensure last byte is null (a valid c-string)
    c_str_p->buf_p[c_str_p->num_chars] = '\0';
}
