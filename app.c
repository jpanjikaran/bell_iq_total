/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "em_iadc.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "utility.h"
#include "thandles.h"
#include "collection.h"
#include "sl_status.h"
#include "swmodbeacon.h"
#include "em_wdog.h"
#include "sleep.h"
#include "tempdrv.h"
#include "uartdrv.h"
#include "time.h"
#include "sl_uartdrv_instances.h"
#include "em_msc.h"
#include "gpiointerrupt.h"
#include "sl_power_manager.h"
#include "lora_xdot_radio.h"
#include "api_eusart_radio.h"
#include "radio_common.h"
//#include "lora_api_internal_data.h"


/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
//#define BIT_MOVEMENT      2   //port pin for MOV signal(PB2) -> PA1 in Gen1
//#define BIT_ACTIVATION    1   //port pin for ACT signal(PC1) -> PA0 in Gen1
//#define CAP_PWR           0
//
//#define TP4               5   //port pin for TP4(PC5) -> PB11 in Gen1
//#define TP5               3   //port pin for TP5(PC3) -> PB13 in Gen1

#define BIT_MOVEMENT      2   //port pin for MOV signal(PB2) -> PA1 in Gen1
#define BIT_ACTIVATION    1   //port pin for ACT signal(PC1) -> PA0 in Gen1
#define CAP_PWR           0


#define BIT_MOVEMENT_P      1   //port pin for MOV signal(PB2) -> PA1 in Gen1

#ifdef NEW_HW
#define BIT_ACTIVATION_P    1   //port pin for ACT signal(PC1) -> PA0 in Gen1
#define CAP_PWR_P           2
#else
#define BIT_ACTIVATION_P    2   //port pin for ACT signal(PC1) -> PA0 in Gen1
#define CAP_PWR_P           1
#endif
#define TP4               5   //port pin for TP4(PC5) -> PB11 in Gen1
#define TP5               3   //port pin for TP5(PC3) -> PB13 in Gen1

#define UINT16_TO_BYTES(n)            ((uint8_t) (n)), ((uint8_t)((n) >> 8))
#define UINT16_TO_BYTE0(n)            ((uint8_t) (n))
#define UINT16_TO_BYTE1(n)            ((uint8_t) ((n) >> 8))


//Adervertising rates defines
#define MS_100  160
#define MS_500  800
#define MS_750  1200
#define MS_1000 1600
#define MS_1250 2000
#define MS_1500 2400

#define TMR_500  16384
#define TMR_750  24576
#define TMR_1000 32768
#define TMR_1250 40960
#define TMR_1500 49152


//#define RESET_FACTORY

//#define LORA

//#define GEN2

//#define NA  0x01
//#define ANZ 0x02
//#define EU  0x03

//Region must be defined before compiling
//#define REGION  EU




#define USART_INITASYNC_DEFAULT2 {                                                                                               \
    usartEnable,           /* Enable RX/TX when init completed. */                                \
    0,                     /* Use current configured reference clock for configuring baudrate. */ \
    9600,                /* 115200 bits/s. */                                                   \
    usartOVS16,            /* 16x oversampling. */                                                \
    usartDatabits8,        /* 8 databits. */                                                      \
    usartNoParity,         /* No parity. */                                                       \
    usartStopbits1,        /* 1 stopbit. */                                                       \
    false,                 /* Do not disable majority vote. */                                    \
    false,                 /* Not USART PRS input mode. */                                        \
    usartPrsRxCh0,         /* PRS channel 0. */                                                   \
    false,                 /* Auto CS functionality enable/disable switch */                      \
    0,                     /* Auto CS Hold cycles */                                              \
    0,                     /* Auto CS Setup cycles */                                             \
    usartHwFlowControlNone /* No HW flow control */                                               \
  }



#define LETIMER_INIT_DEFAULTx                                                  \
  {                                                                           \
    true,             /* Enable timer when init complete. */                  \
    false,            /* Stop counter during debug halt. */                   \
    false,            /* Do not load COMP0 into CNT on underflow. */          \
    false,            /* Do not load COMP1 into COMP0 when REP0 reaches 0. */ \
    0,                /* Idle value 0 for output 0. */                        \
    0,                /* Idle value 0 for output 1. */                        \
    letimerUFOANone,  /* No action on underflow on output 0. */               \
    letimerUFOANone,  /* No action on underflow on output 1. */               \
    letimerRepeatBuffered /* Count until stopped by SW. */                        \
  }

#ifdef TREX
#ifdef GEN2
  const uint8_t firmware_version[4] = {"4.28"};//{"1.29"};{"1.27"};{"1.28"};
#else
  const uint8_t firmware_version[4] = {"2.09"};//{"2.14"} {"2.01"};//{"1.25"};
  const uint8_t hardware_version[4] = {"1.10"};
#endif
#else
#ifdef GEN2
#ifndef ECOLAB_DEV
  const uint8_t firmware_version[5] = {"159.4"};//{"1.29"};{"1.27"};{"1.28"};
#else
  const uint8_t firmware_version[5] = {"250.2"};
#endif
#else
  const uint8_t firmware_version[4] = {"1.30"};//{"1.29"};{"1.27"};{"1.28"};
#endif
  const uint8_t hardware_version[4] = {"1.26"};
#endif


/*******************************************************************************
 ***************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/
//External declarations
extern uint8_t    BattVolts;
extern uint8_t    dispTemp;
extern uint8_t    readval;
//extern uint8_t    EventsCounter;
extern uint8_t    newKey;
extern uint32_t   keyValidTimer;   //  save after max time reached
//
extern deviceDATA   MYdevice;
extern MTsystem_t   MTsystem;
extern battData     DeviceBatt;
extern smartwavebcn     myBeacon;
extern collectionType   coll;
extern resetData resetColl[8];
extern last_set_time_t last_set_time;

extern uint32_t    intFLAGSodd;
extern uint32_t    intFLAGSeven;
extern uint32_t    intActCntr;
extern uint32_t    intMovCntr;
extern uint8_t     SuppressDetect;
extern uint8_t     minTemp;
extern uint8_t     maxTemp;
//extern uint16_t    DetectionsCounter;
extern uint16_t   currentindex;  // keep track of how many entries we have
extern char eusartTxBuffer[64];
extern LORA_STATE lora_state;
extern bool radio_is_sleeping;
extern uint8_t reg_flag;
extern uint8_t letmr_flag;


// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static uint8_t boot_to_dfu = 0;

//Variable declarations
uint8_t Firstsig0[1] = {0};
uint8_t     Toggle = 0;
uint8_t     Twake = 0;
uint8_t     TREXarmed = 0;
uint8_t     SendTREXstatus = 0; // Initial state should reflect hall open no magnet
uint8_t     HallPower = 1;
uint8_t     HallCount = 1;
//uint32_t    intMovCntr;
//uint32_t    intActCntr;
int8_t      currentMode;
uint8_t     value = 0;
uint16_t      mtuSize = 0;
uint16_t    EventRequested = 0;
uint32_t    EventsSinceReset = 0;
uint8_t     connectionState = 0;
uint16_t    indicationscnt = 0;
uint8_t     data_parse[6];
uint8_t     lora_parse[10];
uint8_t     data_tmp[4], local_saved[4], unix_saved[4];
uint8_t     tmr3_samp_time;
timeINFO    time_i, time_saved;
int16_t ret_max, ret_min;
uint8_t prev_senseFLAG = 0;
bool send_lora_pkt = false;
bool send_lora_heartbeat=false;
bool send_lora_bell_msg=false;
uint8_t lora_hb_payload = 0;
//uint8_t
uint32_t lora_hb_cnt = 0;
bool my_sys_is_ready_to_sleep = false;

//uint8_t data[185] = {0};
uint8_t lora_dev_settings[182] = {0};
uint8_t first_event_flag = 0;
uint16_t first_event_cnt = 0;
uint8_t first_event_tx = 0;
uint8_t first_event_tx_cnt = 0;
uint8_t ind_flag;
uint8_t join_ctr = 0;
uint8_t ping_cnt=0;
uint8_t join_cnt=0;
uint8_t test_mode_active = 0;
char hexasctable1[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

unsigned char tmp_masterkey[32] = {0};


uint8_t tmp_test = 250;

uint32_t  myUNIXtime = 0, saved_unix_time = 0;;  // Global for UNIX time
volatile uint8_t mode;

WDOG_Init_TypeDef init =
{
  .enable = true,                       /* Start watchdog when initialization is done. */
  .debugRun = false,                    /* WDOG is not counting during debug halt. */
  .em2Run = true,                      /* WDOG is not counting when in EM2. */
  .em3Run = true,                      /* WDOG is not counting when in EM3. */
  .em4Block = false,                    /* EM4 can be entered. */
  .lock = false,                        /* Do not lock WDOG configuration. */
  .perSel = wdogPeriod_256k,            /* Set longest possible timeout period. */
  .warnSel = wdogWarnDisable,           /* Disable warning interrupt. */
  .winSel = wdogIllegalWindowDisable,   /* Disable illegal window interrupt. */
  .resetDisable = false                 /* Do not disable reset. */
};


WDOG_Init_TypeDef WDOG_init_default =

  {                                                                             \
    true,                     /* Start watchdog when initialization is done. */ \
    false,                    /* WDOG is not counting during debug halt. */     \
    false,                    /* WDOG is not counting when in EM2. */           \
    false,                    /* WDOG is not counting when in EM3. */           \
    false,                    /* EM4 can be entered. */                         \
    false,                    /* Do not lock WDOG configuration. */             \
    wdogPeriod_256k,          /* Set longest possible timeout period. */        \
    wdogWarnDisable,          /* Disable warning interrupt. */                  \
    wdogIllegalWindowDisable, /* Disable illegal window interrupt. */           \
    false                     /* Do not disable reset. */                       \
  };




void settings_parse(void)
{
  uint8_t i;

//  for(i=0; i<6; i++)
//    {
//    char_send(data_parse[i],1);
//    }
  MYdevice.advertising_period_provisional = data_parse[2];

  MYdevice.battMODE = data_parse[5];
  MYdevice.dbgMODE = data_parse[5];

  //Advertising rate
  if((MYdevice.advertising_period_provisional>49)&&(MYdevice.advertising_period_provisional<150))
    {
      MYdevice.advertising_period = MYdevice.advertising_period_provisional*16;
    }
  else
    MYdevice.advertising_period = MS_1500;



//  char_send(data_parse[0],1);

  if((data_parse[0]>127)&&(data_parse[0]<135))
    {
      MYdevice.tx_pwr = (data_parse[0]-128)*10;
//      char_send(MYdevice.tx_pwr,1);
    }
  else
    MYdevice.tx_pwr = 0;

//  char_send(MYdevice.tx_pwr,1);

  MYdevice.sigSTS = data_parse[1];
  MYdevice.sensOFF.data = data_parse[3];
  MYdevice.sensON.data = data_parse[4];
//  MYdevice.sensOFF.data = 86;
//  MYdevice.sensON.data = 41;
  if((MYdevice.sensOFF.data<96) && (MYdevice.sensON.data<96))
    {
      MYdevice.sensOFF.hour = MYdevice.sensOFF.data/4;
      MYdevice.sensOFF.min = (MYdevice.sensOFF.data%4)*15;
      MYdevice.sensON.hour = MYdevice.sensON.data/4;
      MYdevice.sensON.min = (MYdevice.sensON.data%4)*15;
//      MYdevice.sensOFF.hour = MYdevice.sensOFF.data>>2;
//      MYdevice.sensOFF.min = (MYdevice.sensOFF.data&0x02)*15;
//      MYdevice.sensON.hour = MYdevice.sensON.data>>2;
//      MYdevice.sensON.min = (MYdevice.sensON.data&0x02)*15;
    }
  else
    {
      MYdevice.sensOFF.data = 0xFF;
      MYdevice.sensON.data = 0xFF;
      GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1);   // power for pull up resistors on I/O  only on express trapper ambush
      GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 1);   // power for pull up resistors on I/O
      GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeWiredOr, 1);
      GPIO_IntConfig( gpioPortB, BIT_MOVEMENT, 1, 1, 1);
      GPIO_IntConfig( gpioPortC, BIT_ACTIVATION, 1, 1, 1);
    }

  MYdevice.battMODE = (MYdevice.battMODE & 0x80)>>7;
  MYdevice.dbgMODE = (MYdevice.dbgMODE & 0x40)>>6;
  MYdevice.sensSAMP = 600;
  MYdevice.sensFLAG = 0;
}

void calcDate(timeINFO *tm)
{
  uint32_t seconds, minutes, hours, days, year, month;
  uint32_t dayOfWeek;
  seconds = myUNIXtime;

  /* calculate minutes */
  minutes  = seconds / 60;
  seconds -= minutes * 60;
  /* calculate hours */
  hours    = minutes / 60;
  minutes -= hours   * 60;
  /* calculate days */
  days     = hours   / 24;
  hours   -= days    * 24;

  /* Unix time starts in 1970 on a Thursday */
  year      = 1970;
  dayOfWeek = 4;

  while(1)
  {
    bool     leapYear   = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    uint16_t daysInYear = leapYear ? 366 : 365;
    if (days >= daysInYear)
    {
      dayOfWeek += leapYear ? 2 : 1;
      days      -= daysInYear;
      if (dayOfWeek >= 7)
        dayOfWeek -= 7;
      ++year;
    }
    else
    {
      tm->tm_yday = days;
      dayOfWeek  += days;
      dayOfWeek  %= 7;

      /* calculate the month and day */
      static const uint8_t daysInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      for(month = 0; month < 12; ++month)
      {
        uint8_t dim = daysInMonth[month];

        /* add a day to feburary if this is a leap year */
        if (month == 1 && leapYear)
          ++dim;

        if (days >= dim)
          days -= dim;
        else
          break;
      }
      break;
    }
  }

  tm->tm_sec  = seconds;
  tm->tm_min  = minutes;
  tm->tm_hour = hours;
  tm->tm_mday = days + 1;
  tm->tm_mon  = month+1;
  tm->tm_year = year;
  tm->tm_wday = dayOfWeek;
}



void load_default_settings(void)
{
  uint8_t data;
  data = 0xFF;
  ps_save(CUSTOM_FLAG, &data, 1);
  MYdevice.advertising_period = MS_1500;
  MYdevice.tx_pwr = 0;
  MYdevice.sigSTS = 2;
  MYdevice.sensOFF.data = 0xFF;
  MYdevice.sensON.data = 0xFF;
  MYdevice.sensFLAG = 0;
  MYdevice.battMODE = 1;
  MYdevice.dbgMODE = 1;
}

void lora_settings_parse(void)
{
  if((lora_parse[0] != 0)&&(MYdevice.lora.unreg_flag = 1))
    {
      if(MYdevice.sku != 0xFF)
        MYdevice.lora.unreg_flag = 0;
      MYdevice.lora.region = lora_parse[0];
      ps_save(CONFIG_ZONE_OFFSET, &MYdevice.lora.region, 1);  // check to see what type this device is
    }

  MYdevice.lora.payload = lora_parse[1];
  ps_save(PAYLOAD, &MYdevice.lora.payload, 1);  // check to see what type this device is

  MYdevice.lora.event_trigger = lora_parse[2];

  MYdevice.lora.manual_join_delay = lora_parse[3];

  if(lora_parse[4] != 0)
    MYdevice.lora.tx_interval = lora_parse[4];
  else
    MYdevice.lora.tx_interval = lora_parse[4];

  MYdevice.lora.heartbeat = MYdevice.lora.tx_interval*3600;

  MYdevice.lora.lora_en = lora_parse[5];
  ps_save(CONFIG_LORA_ENABLE_OFFSET, &MYdevice.lora.lora_en, 1);  // check to see what type this device is
  ps_save(CONFIG_LORA_HB_OFFSET, &MYdevice.lora.tx_interval, 1);  // check to see what type this device is

  MYdevice.lora.rejoin = lora_parse[8];

  MYdevice.lora.ping = lora_parse[9];

  if(MYdevice.lora.manual_join_delay<6)
    MYdevice.lora.manual_join_delay = 5;
//    MYdevice.lora.manual_join_delay = 1;

  if(MYdevice.lora.rejoin==1)
    {
//      char_send('B',0);
      reg_flag=0;
      if(MYdevice.lora.rejoin_attempt<4)
        MYdevice.lora.rejoin=0;
      else
        {
          if(MYdevice.lora.manual_join_delay_tm==0)
            {
              MYdevice.lora.manual_join_delay_tm = MYdevice.lora.manual_join_delay * 30;
              MYdevice.lora.join_status = 2;
              MYdevice.lora.join_test = 1;
              MYdevice.lora.ping_result = 3;
              ping_cnt = 0;
//              char_send('C',0);
//              MYdevice.lora.tx_interval = 1;
//              MYdevice.lora.heartbeat = MYdevice.lora.tx_interval*3600;
//              lora_hb_cnt = MYdevice.lora.heartbeat-30;
//              radio_transmit_hex_data_12(0x07, 0xCD, myBeacon.devMACadd[0], myBeacon.devMACadd[1], myBeacon.devMACadd[2], myBeacon.devMACadd[3], myBeacon.devMACadd[4], myBeacon.devMACadd[5],0x04, coll.EventsCounter,DeviceBatt.batterylvl,false, true);
            }
          else
            {
              MYdevice.lora.rejoin=0;
            }

        }
    }
//  if(MYdevice.lora.rejoin==1&&MYdevice.lora.manual_join_delay_tm==0)
//    {
//      reg_flag=0;
//      MYdevice.lora.manual_join_delay_tm = MYdevice.lora.manual_join_delay * 30;
//    }
  if(MYdevice.lora.ping==1)
    {
      reg_flag=0;
    }
  if(MYdevice.lora.lora_en==0)
    reg_flag = 0;
}

void save_settings(void)
{
  uint8_t data;

  ps_save(SETTINGS, data_parse, 6);

  data = 1;
  ps_save(CUSTOM_FLAG, &data, 1);
}
void save_settings2(void)
{
//  uint8_t tmp1[2], data;
//  uint16_t adv;

////  adv = MYdevice.advertising_period;
//  tmp1[0] = 0x09;
//  tmp1[1] = 0x60;
//  ps_save(ADV_PER, tmp1, 2);
//
//  data = 60;
//  ps_save(TX_PWR, &data, 1);
//
//  data =  0x02;
//  ps_save(SIG_STS, &data, 1);
//
//  data = 0xA6;
//  ps_save(SENS_ON, &data, 1);
//
//  data = 0x7D;
//  ps_save(SENS_OFF, &data, 1);
//
//  data = 1;
//  ps_save(BATT_MODE, &data, 1);
//
//  data = 1;
//  ps_save(DBG_MODE, &data, 1);
//
//  data = 1;
//  ps_save(CUSTOM_FLAG, &data, 1);
}

void load_settings(void)
{
  uint8_t custom_flag = 0;

  ps_load(CUSTOM_FLAG, &custom_flag, 1);
//  custom_flag = 1;

  if(custom_flag==1)
  {
    ps_load(SETTINGS, data_parse, 6);
    settings_parse();
    MYdevice.sensFLAG = 0;
    ps_load(UP_TIME, local_saved, 4);
    ps_load(UNIX_TIME, unix_saved, 4);

    //for testing sending out over uart
//    unix_saved[0] = 0x60;
//    unix_saved[1] = 0xE3;
//    unix_saved[2] = 0x3E;
//    unix_saved[3] = 0x6E;

//    local_saved[0] = 0x00;
//    local_saved[1] = 0x5B;
//    local_saved[2] = 0x5B;
//    local_saved[3] = 0x5C;
//    sendLOCAL();
//    sendUNIX();
  }
  else
  {
    load_default_settings();
  }
  ps_load(PAYLOAD, &MYdevice.lora.payload, 1);
  if(MYdevice.lora.payload==0||MYdevice.lora.payload>0xE0)
    MYdevice.lora.payload = 0x01;
//  MYdevice.advertising_period = MS_1000;
//  MYdevice.tx_pwr = 0;
}


void backdoor_access (void) {
    //REFER to https://www.rapidtables.com/code/text/ascii-table.html
    uint32_t *userDataPage  = (uint32_t *) USERDATA_BASE;
    uint32_t userData[] = {
        0x00140412,    //manufacturing_date in hex
        0x322e3039 ,    //hardware_version in hex

//        0x30303830,    //2:0x0,0x0,0x8,0x0
//        0x30303030,    //3:0x0,0x0,0x0,0x0
//        0x30303030,    //4:0x0,0x0,0x0,0x0
//        0x46303130,    //5:0xF,0x0,0x1,0x0
//
//        0x32413931,    //6:0x2,0xA,0x9,0x1
//        0x38373433,    //7:0x8,0x7,0x4,0x3
//        0x32413731,    //8:0x2,0xA,0x7,0x1
//        0x38424444,    //9:0x8,0xB,0xD,0xD
//        0x39364330,    //10:0x9,0x6,0xC,0x0
//        0x39354432,    //11:0x9,0x5,0xD,0x2
//        0x41343331,    //12:0xA,0x4,0x3,0x1
//        0x41383535     //13:0xA,0x8,0x5,0x5

        0x30303830,    //2:0x0,0x0,0x8,0x0
        0x30303030,    //3:0x0,0x0,0x0,0x0
        0x30383030,    //4:0x0,0x8,0x0,0x0
        0x30303030,    //5:0x0,0x0,0x0,0x0

        0x38453230,    //6:0x8,0xE,0x2,0x0
        0x31343331,    //7:0x1,0x4,0x3,0x1
        0x36384232,    //8:0x6,0x8,0xB,0x2
        0x46333242,    //9:0xF,0x3,0x2,0xB
        0x42323345,    //10:0xB,0x2,0x3,0xE
        0x39324642,    //11:0x9,0x2,0xF,0xB
        0x30294632,    //12:0x0,0x9,0xF,0x2
        0x30454433,     //13:0x0,0xE,0xD,0x3

        0x34313331,    //6:0x4,0x1,0x3,0x1
        0x34323332,    //7:0x4,0x2,0x3,0x2
        0x34353339,    //8:0x4,0x5,0x3,0x9
        0x34363338,    //9:0x4,0x6,0x3,0x8
        0x35413333,    //10:0x5,0xA,0x3,0x3
        0x35343334,    //11:0x5,0x4,0x3,0x4
        0x34413330,    //12:0x4,0xA,0x3,0x0
        0x35303335     //13:0x5,0x0,0x3,0x5





    };


    MSC_Init();
    MSC_ErasePage(userDataPage);
    MSC_WriteWord(userDataPage, userData, sizeof(userData));
    MSC_Deinit();
}

/* TestPortScan - reads condition of test port and returns as a test mode 0 - 3 */
uint8_t TestPortScan(void)
{
    volatile uint8_t mode = 0;//

    tpSetup();

    if(GPIO_PinInGet(gpioPortC,5))
      mode = mode | 0x01;
    else
      mode = mode & 0xFE;

    if(GPIO_PinInGet(gpioPortC,3))
      mode = mode | 0x02;
    else
      mode = mode & 0xFD;


    //    if(mode>0)
    //      {
            GPIO_PinModeSet(gpioPortC, TP4, gpioModeDisabled, 0);  // TP4
            GPIO_PinModeSet(gpioPortC, TP5, gpioModeDisabled, 0);  // TP5
    //      }
//    mode = 1;
//            char_send(mode,1);
    return mode;
}

void adv_set_data(void)
{
//  sl_status_t sc;
//  uint8_t  TXpwr = 0;
  bd_addr address;
  uint8_t address_type;
  uint8_t dev_mode, snr, rssi;

  sl_bt_system_get_identity_address(&address, &address_type);
//  sl_app_assert(sc == SL_STATUS_OK,
//                "[E: 0x%04x] Failed to set advertiser data\n",
//                (int)sc);0



//  MYdevice.lora.join_status = 0;
//  MYdevice.lora.lora_en = 1;
//  MYdevice.lora.lora_rssi_min = 0;
//  MYdevice.lora.lora_rssi_max = 0;
//  MYdevice.lora.lora_rssi_lvl = 200;
  if(MYdevice.registration==1)
    {
      if(MYdevice.lora.lora_en == 1)
        {
          if(MYdevice.lora.join_status==1)
            {
              snr=0;
              rssi=0;
              MYdevice.lora.lora_rssi_lvl = 100;
//              if((MYdevice.lora.lora_rssi_last>156)||(MYdevice.lora.lora_rssi_last<128))
//                rssi=1;
//              else
//                rssi=0;
//              if(MYdevice.lora.lora_snr_last<128||MYdevice.lora.lora_snr_last>241)
//                snr=1;
//              else
//                snr=0;
//              if(rssi && snr)
//                MYdevice.lora.lora_rssi_lvl = 2;
//              else if (rssi || snr)
//                MYdevice.lora.lora_rssi_lvl = 1;
//              else
//                MYdevice.lora.lora_rssi_lvl = 0;

              if(MYdevice.lora.lora_rssi_last>155&&(MYdevice.lora.lora_snr_last>241||MYdevice.lora.lora_snr_last<128))
                MYdevice.lora.lora_rssi_lvl = 2;
              else if(MYdevice.lora.lora_rssi_last<142||(MYdevice.lora.lora_snr_last<242&&MYdevice.lora.lora_snr_last>235))
                MYdevice.lora.lora_rssi_lvl = 0;
              else
                MYdevice.lora.lora_rssi_lvl = 1;


            }
          else
            MYdevice.lora.lora_rssi_lvl = 0;
        }
      else
        {
          MYdevice.lora.lora_rssi_lvl = 0;
          MYdevice.lora.join_status = 0;
//          char_send('C',0);
        }
    }
  else if (MYdevice.operatingMode==factorywake || MYdevice.operatingMode==unregwake)
    {
          if(MYdevice.lora.join_status==1)
            {
              snr=0;
              rssi=0;
              MYdevice.lora.lora_rssi_lvl = 100;
              if((MYdevice.lora.lora_rssi_last>156)||(MYdevice.lora.lora_rssi_last<128))
                rssi=1;
              else
                rssi=0;
              if(MYdevice.lora.lora_snr_last<128)
                snr=1;
              else
                snr=0;
              if(rssi && snr)
                MYdevice.lora.lora_rssi_lvl = 2;
              else if (rssi || snr)
                MYdevice.lora.lora_rssi_lvl = 1;
              else
                MYdevice.lora.lora_rssi_lvl = 0;
            }
          else
            MYdevice.lora.lora_rssi_lvl = 0;
    }
  else
    {
      MYdevice.lora.lora_rssi_lvl = 0;
    }
  if(MYdevice.registration != 1|| MYdevice.lora.lora_en==0)
    {
      MYdevice.lora.join_delay_flag = 0;
      MYdevice.lora.network_delay = 0;
      MYdevice.lora.lora_rssi_lvl=0;
      if(MYdevice.lora.join_status!=2)
        {
          MYdevice.lora.join_status=0;
//          char_send('D',0);
        }
      MYdevice.lora.lora_en=0;
    }
//  if(MYdevice.lora.network_delay)
//    MYdevice.lora.lora_rssi_lvl = 0xAA;
//  MYdevice.registration = 1;

  myBeacon.flagsLen = 2;            // Length of field.
  myBeacon.flagsType = 0x01;         // Type of field.
  myBeacon.flags = 0x04 | 0x02;  // Flags: LE General Discoverable Mode, BR/EDR is disabled.
  myBeacon.namelen = 3;            // Name Length
  myBeacon.nametype = 9;            // Name Type
  myBeacon.localname0 = 'B';          // First Char
  myBeacon.localname1 = 'G';           // Second Char
  // Manufacturer specific data.
  myBeacon.mandataLen = 22;   // Length of field.
  myBeacon.mandataType = 0xFF; // Type of field.

  // The first two data octets shall contain a company identifier code from
  // the Assigned Numbers - Company Identifiers document.
  // 0x004C = Apple
  myBeacon.compId[1] = 0x07;
  myBeacon.compId[0] = 0xCD;  //Smartwave Company ID
  myBeacon.commscheme = 0x01;  // Comm Scheme
  myBeacon.devtype = MYdevice.sku;
  myBeacon.devMACadd[5] = address.addr[5];
  myBeacon.devMACadd[4] = address.addr[4];
  myBeacon.devMACadd[3] = address.addr[3];
  myBeacon.devMACadd[2] = address.addr[2];
  myBeacon.devMACadd[1] = address.addr[1];
  myBeacon.devMACadd[0] = address.addr[0];
  myBeacon.txPwr = (uint8_t)MYdevice.tx_pwr + 0x80;      // TX PWR
  dev_mode = (MYdevice.lora.lora_en<<7);
  dev_mode |= (MYdevice.lora.join_status<<6);
  dev_mode |= (MYdevice.lora.lora_rssi_lvl<<4);
  dev_mode |= (MYdevice.lora.join_delay_flag)<<3;
  dev_mode |= (MYdevice.lora.network_delay)<<2;
  dev_mode |= MYdevice.registration;
  myBeacon.RRSSI = dev_mode;        // REG
//  myBeacon.RRSSI = MYdevice.registration;        // REG
  myBeacon.tempRead = MTsystem.temperaturecurrent;       //  Temperature
  myBeacon.battLvl = DeviceBatt.batterylvl;       //  Battery
  myBeacon.status = coll.EventsCounter;
  myBeacon.numEvents[1] = UINT16_TO_BYTE1(coll.DetectionsCounter);
  myBeacon.numEvents[0] = UINT16_TO_BYTE0(coll.DetectionsCounter);;
  myBeacon.LocalTime[3] = (uint8_t)(MTsystem.LocalTimeCounter >> 24);
  myBeacon.LocalTime[2] = (uint8_t)((MTsystem.LocalTimeCounter & 0x00FF0000)>> 16);
  myBeacon.LocalTime[1] = (uint8_t)((MTsystem.LocalTimeCounter & 0x0000FF00)>> 8);
  myBeacon.LocalTime[0] = (uint8_t)MTsystem.LocalTimeCounter;



  /* Set custom advertising data */
  uint8_t len = sizeof(myBeacon);
  const uint8_t *pData = (uint8_t*)(&myBeacon);

   sl_bt_advertiser_set_data(advertising_set_handle,
                                  0,
                                  len,
                                  pData);
//   sl_app_assert(sc == SL_STATUS_OK,
//                 "[E: 0x%04x] Failed to set advertiser data\n",
//                 (int)sc);

//   sl_bt_system_set_max_tx_power(MYdevice.tx_pwr,MYdevice.tx_pwr);
   sl_bt_advertiser_set_channel_map(advertising_set_handle,7);
//   sl_bt_advertiser_set_channel_map(advertising_set_handle,2);
//   MYdevice.tx_pwr = 30;


   if(MYdevice.operatingMode == factorywake || MYdevice.operatingMode == unregwake)  // in factory wake and unreg wake we want advertise quick
   {
//       MYdevice.advertising_period = MS_100;
//       sl_bt_system_set_max_tx_power(0,0);
//              sl_bt_system_set_tx_power(0,0,&ret_min,&ret_max);
//              sl_bt_system_set_tx_power(MYdevice.tx_pwr,MYdevice.tx_pwr,&ret_min,&ret_max);
              sl_bt_advertiser_set_tx_power(advertising_set_handle, -30, &ret_max);
       sl_bt_advertiser_set_timing(
           advertising_set_handle, // advertising set handle
           MS_100, // min. adv. interval (milliseconds * 1.6)
           MS_100, // max. adv. interval (milliseconds * 1.6)
           0,   // adv. duration
           0);  // max. num. adv. events

   }
   else
   {
//       sl_bt_system_set_tx_power(MYdevice.tx_pwr,MYdevice.tx_pwr,&ret_min,&ret_max);
//       sl_bt_advertiser_set_tx_power(advertising_set_handle, MYdevice.tx_pwr, &ret_max);
       sl_bt_advertiser_set_tx_power(advertising_set_handle, -30, &ret_max);
//       sl_bt_system_set_tx_power(-5,-5,&ret_min,&ret_max);
       sl_bt_advertiser_set_timing(
           advertising_set_handle, // advertising set handle
           MYdevice.advertising_period, // min. adv. interval (milliseconds * 1.6)
           MYdevice.advertising_period, // max. adv. interval (milliseconds * 1.6)
           0,   // adv. duration
           0);  // max. num. adv. events


   }


}

void advertise_start(void)
{
  sl_status_t sc;

  // Start user defined advertising and enable connections
  sc = sl_bt_advertiser_start(advertising_set_handle,
                              advertiser_user_data,
                              advertiser_connectable_scannable);
//  sl_app_assert(sc == SL_STATUS_OK,
//                "[E: 0x%04x] Failed to start advertising\n",
//                (int)sc);

}

void advertise_stop(void)
{
  sl_status_t sc;

  // Stop advertising
  sc = sl_bt_advertiser_stop(advertising_set_handle);
//  sl_app_assert(sc == SL_STATUS_OK,
//                "[E: 0x%04x] Failed to stop advertising\n",
//                (int)sc);

}


void TestState(void)
//{
//    //volatile uint8_t mode;
//    volatile static uint8_t txon=0;
////    mode = TestPortScan();
////    mode = 2;
////    if(mode>0)
////      {
////        GPIO_PinModeSet(gpioPortC, TP4, gpioModeDisabled, 0);  // TP4
////        GPIO_PinModeSet(gpioPortC, TP5, gpioModeDisabled, 0);  // TP5
////      }
////    currentMode = mode;
////    char_send(currentMode,1);
//    if(currentMode==2)
//        test_mode_active = 1;
//    else
//        test_mode_active = 0;
//
//    GPIO_PinModeSet(gpioPortC, 0, gpioModeInput , 0);   // power for pull up resistors on I/O
//
//  #ifdef TREX
//        if (currentMode == 2) {
//          GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 1);
//          GPIO_PinModeSet(gpioPortB, 2, gpioModeInputPull, 1);         // set pull up on hall output open drain
//
//        }
//        else {
//          GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 1);   //set input if not used
//          GPIO_PinModeSet(gpioPortB, 2, gpioModeDisabled, 0);  //disable the pullup
//        }
//  #endif
//
//  #ifndef TREX
//        if (currentMode == 2) {
//            GPIO_PinModeSet(gpioPortB, BIT_ACTIVATION, gpioModeDisabled, 1);   // power for pull up resistors on I/O  only on express trapper ambush
//            GPIO_PinModeSet(gpioPortB, BIT_MOVEMENT, gpioModeDisabled, 1);   // power for pull up resistors on I/O
//            GPIO_PinModeSet(gpioPortC, CAP_PWR, gpioModeWiredOr, 1);
//        }
//        else {
//
//            /*setting with no external pull ups*/
////            GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeWiredOr, 0);
////            GPIO_PinModeSet(gpioPortC, BIT_ACTIVATION, gpioModeDisabled, 1);   // power for pull up resistors on I/O  only on express trapper ambush
////            GPIO_PinModeSet(gpioPortB, BIT_MOVEMENT, gpioModeDisabled, 1);
//            /*end of no external settings*/
//
//            /*setting with no external pull ups - 11152023*/
//            GPIO_PinModeSet(gpioPortC, CAP_PWR, gpioModeWiredOr, 1);
//            GPIO_PinModeSet(gpioPortB, BIT_ACTIVATION, gpioModePushPull, 1);   // power for pull up resistors on I/O  only on express trapper ambush
//            GPIO_PinModeSet(gpioPortB, BIT_MOVEMENT, gpioModePushPull, 1);
//            /*end of no external settings - 11152023*/
//
//            /*feb 0702023 settings*/
////            GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModePushPull, 1);
////            GPIO_PinModeSet(gpioPortC, BIT_ACTIVATION, gpioModeDisabled, 0);     // make input for all other traps
////            GPIO_PinModeSet(gpioPortB, BIT_MOVEMENT, gpioModeDisabled, 0);
//            /*end of feb 07 settings*/
//
//
//
//
//        }
//  #endif
//    if (currentMode == 1) {
//        GPIO_DbgSWDIOEnable(false);
////        GPIO_DbgSWDClkEnable(false);
////        GPIO_PinModeSet(gpioPortC,0, gpioModeInput, 0 );
////        GPIO_PinModeSet(gpioPortC,2, gpioModeInput, 0 );
////        GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeWiredOr, 0);
////        GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModePushPull, 0);
////        GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeWiredOr, 0);
////        GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeDisabled, 0);
//    }
//    else {
//        GPIO_DbgSWDIOEnable(true);
////        GPIO_DbgSWDClkEnable(true);
//    }
//    if(currentMode != 3 && txon == 1)
//    {
//       sl_bt_test_dtm_end();
////       char_send("X",0);
//        txon = 0;
//    }
//    switch(currentMode)
//    {
//        case 1:
//          if(radio_is_sleeping)
//            {
//              SLEEP_SleepBlockEnd(sleepEM3);
//              SLEEP_SleepBlockEnd(sleepEM2);  // resume normal power  management
//              my_sys_is_ready_to_sleep = true;
////              GPIO_PinModeSet(gpioPortA, 5, gpioModeDisabled, 1);   //added for sleep current test - JP - 11242023
////              GPIO_PinModeSet(gpioPortA, 6, gpioModeDisabled, 1);
//              GPIO_PinModeSet(gpioPortA, 5, gpioModeInput, 0);   //added for sleep current test - JP - 11242023
//              SLEEP_Sleep();
////              UARTDRV_DeInit(sl_uartdrv_usart_debug_handle);
//            }
////            sl_bt_test_dtm_tx_v4(test_pkt_10101010, 255, 1, 1, 6);
//        break;
//        case 2:
//            SLEEP_SleepBlockEnd(sleepEM3);
////            if((MYdevice.lora.join_status ==0)&&(join_ctr<2))
////              {
////                MYdevice.lora.join_status = 2;
////                MYdevice.lora.join_test = 1;
////                join_ctr++;
////              }
//        break;
//        case 3:
//            SLEEP_SleepBlockEnd(sleepEM3);
//            if(txon == 0)
//              {
//
//                    MYdevice.tx_pwr = 60;
//                    sl_bt_system_set_tx_power(MYdevice.tx_pwr,MYdevice.tx_pwr,&ret_min,&ret_max);
////                    sl_bt_test_dtm_tx_v4(test_pkt_carrier, 255, 1, 1, 0);  //high frequency, channel 39 (2480MHz), 6dBm output, modulated 0xAA continuously sent
////                    sl_bt_test_dtm_tx_v4(test_pkt_pn9, 255, 1, 1, 0);
//                    sl_bt_test_dtm_tx_v4(test_pkt_10101010, 255, 1, 1, 6);
////                    sl_bt_test_dtm_tx_v4(test_pkt_11111111, 255, 1, 1, 0);
//
//                    txon = 1;
//                    UARTDRV_DeInit(sl_uartdrv_usart_testUART_handle);
//                    GPIO_PinModeSet(gpioPortA, 5, gpioModePushPull, 0);   //added for sleep current test - JP - 11242023
////                    while(1);
////                    char_send('B', 0);
////                    UARTDRV_DeInit(sl_uartdrv_usart_debug_handle);
//
////                    GPIO_PinModeSet(gpioPortA, 6, gpioModePushPull , 0);   //added for sleep current test - JP - 11242023
//                    GPIO_PinModeSet(LORA_RST_PORT, LORA_RST_PIN, gpioModeInput     , 0);
////                    while(1);
//            }
//        break;
//        default:
//          SLEEP_SleepBlockEnd(sleepEM3);
//        break;
//    }
//}
{
    //volatile uint8_t mode;
    volatile static uint8_t txon=0;
//    mode = TestPortScan();
//    mode = 3;
//    if(mode>0)
//      {
//        GPIO_PinModeSet(gpioPortC, TP4, gpioModeDisabled, 0);  // TP4
//        GPIO_PinModeSet(gpioPortC, TP5, gpioModeDisabled, 0);  // TP5
//      }
//    currentMode = mode;
//    char_send(currentMode,1);
    if(currentMode==2)
        test_mode_active = 1;
    else
        test_mode_active = 0;

    GPIO_PinModeSet(MAGSENSE_PORT, MAGSENSE_PIN, gpioModeInput , 0);   // power for pull up resistors on I/O

  #ifdef TREX
        if (currentMode == 2) {
          GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 1);
          GPIO_PinModeSet(gpioPortB, 2, gpioModeInputPull, 1);         // set pull up on hall output open drain

        }
        else {
          GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 1);   //set input if not used
          GPIO_PinModeSet(gpioPortB, 2, gpioModeDisabled, 0);  //disable the pullup
        }
  #endif

  #ifndef TREX
        if (currentMode == 2) {
            GPIO_PinModeSet(BIT_ACTIVATION_P, BIT_ACTIVATION, gpioModeDisabled, 1);   // power for pull up resistors on I/O  only on express trapper ambush
            GPIO_PinModeSet(BIT_MOVEMENT_P, BIT_MOVEMENT, gpioModeDisabled, 1);   // power for pull up resistors on I/O
            GPIO_PinModeSet(CAP_PWR_P, CAP_PWR, gpioModeWiredOr, 1);
        }
        else {

            /*setting with no external pull ups*/
//            GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeWiredOr, 0);
//            GPIO_PinModeSet(gpioPortC, BIT_ACTIVATION, gpioModeDisabled, 1);   // power for pull up resistors on I/O  only on express trapper ambush
//            GPIO_PinModeSet(gpioPortB, BIT_MOVEMENT, gpioModeDisabled, 1);
            /*end of no external settings*/

            /*setting with no external pull ups - 11152023*/
            GPIO_PinModeSet(CAP_PWR_P, CAP_PWR, gpioModeWiredOr, 1);
            GPIO_PinModeSet(BIT_ACTIVATION_P, BIT_ACTIVATION, gpioModePushPull, 1);   // power for pull up resistors on I/O  only on express trapper ambush
            GPIO_PinModeSet(BIT_MOVEMENT_P, BIT_MOVEMENT, gpioModePushPull, 1);
            /*end of no external settings - 11152023*/

            /*feb 0702023 settings*/
//            GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModePushPull, 1);
//            GPIO_PinModeSet(gpioPortC, BIT_ACTIVATION, gpioModeDisabled, 0);     // make input for all other traps
//            GPIO_PinModeSet(gpioPortB, BIT_MOVEMENT, gpioModeDisabled, 0);
            /*end of feb 07 settings*/




        }
  #endif
    if (currentMode == 1) {
        GPIO_DbgSWDIOEnable(false);
//        GPIO_DbgSWDClkEnable(false);
//        GPIO_PinModeSet(gpioPortC,0, gpioModeInput, 0 );
//        GPIO_PinModeSet(gpioPortC,2, gpioModeInput, 0 );
//        GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeWiredOr, 0);
//        GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModePushPull, 0);
//        GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeWiredOr, 0);
//        GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeDisabled, 0);
    }
    else {
        GPIO_DbgSWDIOEnable(true);
//        GPIO_DbgSWDClkEnable(true);
    }
    if(currentMode != 3 && txon == 1)
    {
       sl_bt_test_dtm_end();
//       char_send("X",0);
        txon = 0;
    }
    switch(currentMode)
    {
        case 1:
          if(radio_is_sleeping)
            {
              SLEEP_SleepBlockEnd(sleepEM3);
              SLEEP_SleepBlockEnd(sleepEM2);  // resume normal power  management
              my_sys_is_ready_to_sleep = true;
//              GPIO_PinModeSet(gpioPortA, 5, gpioModeDisabled, 1);   //added for sleep current test - JP - 11242023
//              GPIO_PinModeSet(gpioPortA, 6, gpioModeDisabled, 1);
              GPIO_PinModeSet(gpioPortA, 5, gpioModeInput, 0);   //added for sleep current test - JP - 11242023
              SLEEP_Sleep();
//              UARTDRV_DeInit(sl_uartdrv_usart_debug_handle);
            }
//            sl_bt_test_dtm_tx_v4(test_pkt_10101010, 255, 1, 1, 6);
        break;
        case 2:
            SLEEP_SleepBlockEnd(sleepEM3);
//            if((MYdevice.lora.join_status ==0)&&(join_ctr<2))
//              {
//                MYdevice.lora.join_status = 2;
//                MYdevice.lora.join_test = 1;
//                join_ctr++;
//              }
        break;
        case 3:
            SLEEP_SleepBlockEnd(sleepEM3);
            if(txon == 0)
              {

                    MYdevice.tx_pwr = 60;
                    sl_bt_system_set_tx_power(MYdevice.tx_pwr,MYdevice.tx_pwr,&ret_min,&ret_max);
//                    sl_bt_test_dtm_tx_v4(test_pkt_carrier, 255, 1, 1, 0);  //high frequency, channel 39 (2480MHz), 6dBm output, modulated 0xAA continuously sent
                    sl_bt_test_dtm_tx_v4(test_pkt_pn9, 255, 1, 1, 6);
//                    sl_bt_test_dtm_tx_v4(test_pkt_10101010, 255, 1, 1, 6);
//                    sl_bt_test_dtm_tx_v4(test_pkt_11111111, 255, 1, 1, 0);

                    txon = 1;
                    UARTDRV_DeInit(sl_uartdrv_usart_testUART_handle);
                    GPIO_PinModeSet(gpioPortA, 5, gpioModePushPull, 0);   //added for sleep current test - JP - 11242023
//                    while(1);
//                    char_send('B', 0);
//                    UARTDRV_DeInit(sl_uartdrv_usart_debug_handle);

//                    GPIO_PinModeSet(gpioPortA, 6, gpioModePushPull , 0);   //added for sleep current test - JP - 11242023
                    GPIO_PinModeSet(LORA_RST_PORT, LORA_RST_PIN, gpioModeInput     , 0);
//                    while(1);
            }
        break;
        default:
          SLEEP_SleepBlockEnd(sleepEM3);
        break;
    }
}


void tpSetup(void) {

    /* Initialize peripherals */
//    CMU_ClockEnable(cmuClock_CORELE, true);
    // Initialize stack
//    sl_status_t sc;
//    sc = sl_bt_advertiser_create_set(&advertising_set_handle);
    GPIO_PinModeSet(gpioPortC, TP4, gpioModeInputPull, 0);  // TP4
    GPIO_PinModeSet(gpioPortC, TP5, gpioModeInputPull, 0);  // TP5

    //commented out temporarily - JP - 2021/04/06
//    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
//    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
//    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
//    NVIC_EnableIRQ(GPIO_ODD_IRQn);
//    while(1);
}

#ifdef LORA
bool app_is_ok_to_sleep(void)
{
    return my_sys_is_ready_to_sleep;
//    return false;
//      return true;
}
#endif
/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////

  uint8_t tx_flag = 0;
  uint8_t uart_init = 0;
  uint8_t lora_rst_flag = 0;
  BATTcheckinit();
  TempMeasure();
  MYdevice.tx_pwr = 0;
  MYdevice.fw_ver = 0x2A; //variable to send fw version in LoRa payload

  tpSetup();
  mode = TestPortScan();
  currentMode = mode;

//  currentMode = 3;
//  char_send(currentMode,1);
  if(currentMode != 0)
  {
//    UARTDRV_DeInit(sl_uartdrv_usart_testUART_handle);
    GPIO_PinModeSet(gpioPortA, 5, gpioModeDisabled, 1);
    GPIO_PinModeSet(gpioPortA, 6, gpioModeDisabled, 1);

	//uncommented - 01-05-2023
    SLEEP_Init_t sleepConfig = {0};
    SLEEP_InitEx(&sleepConfig);
    SLEEP_SleepBlockEnd(sleepEM3);
	//end of uncomment
    join_ctr=0;
    MYdevice.registration = 0x03;
    MYdevice.lora.install_join = 0;
    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIOINT_Init();
    GPIO_PinModeSet(gpioPortC, 4, gpioModeInput, 0);
    initLetimer(1);

    lowPowerDelay(100);
    sl_uartdrv_init_instances();
    if(currentMode != 3)
      {

    initUSART0();
    USART_IntEnable(USART0, USART_IEN_RXDATAV);
//      char_send(currentMode,1);
      }
//    initUSART1();
//    USART_IntEnable(USART1, USART_IEN_RXDATAV);
    MYdevice.lora.region = 1;
    MYdevice.lora.lora_rssi_last = 0xFF;
    MYdevice.lora.lora_snr_last=0xFF;
//    test_mode_active = 1;
    GPIO_PinModeSet(LORA_WAKE_PORT, LORA_WAKE_PIN, gpioModePushPull     , 1);
    GPIO_PinModeSet(LORA_RST_PORT, LORA_RST_PIN, gpioModePushPull     , 0);
    sl_udelay_wait(100000);
    sl_udelay_wait(100000);
    sl_udelay_wait(100000);
    sl_udelay_wait(100000);
    sl_udelay_wait(100000);
    GPIO_PinModeSet(LORA_RST_PORT, LORA_RST_PIN, gpioModeInput     , 0);
if(currentMode==2)
    test_mode_active = 1;
else
    test_mode_active = 0;

//    GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModePushPull, 1);
//    GPIO_PinModeSet(gpioPortC, BIT_ACTIVATION, gpioModeDisabled, 0);     // make input for all other traps
//    GPIO_PinModeSet(gpioPortB, BIT_MOVEMENT, gpioModeDisabled, 0);
GPIO_PinModeSet(gpioPortA, 5, gpioModePushPull, 0);   //added for sleep current test - JP - 11242023
//while(1);
    while(1)
    {
      TestState();
//      if(currentMode==2)
//          test_mode_active = 1;
//      else
//          test_mode_active = 0;
      if(test_mode_active==0)
        {
//          char_send('D',0);
          if(currentMode != 3)
            {
//              char_send('A',0);
              uart_init=0;
              lora_state_machine();
              check_radio_uart_read();
              API_EUSART_RADIO__TX_Data_Send();
            }
        }
      else
        {
          if(uart_init == 0)
            {
//              char_send('C',0);
              initUSART1();
              USART_IntEnable(USART1, USART_IEN_RXDATAV);
              uart_init=1;
            }
          check_radio_uart_read();
//          API_EUSART_RADIO__TX_Data_Send();
        }
//      if(MYdevice.lora.join_status==2)
//        {
//          if(tx_flag==0)
//            {
//              sendLoraJoinState();
//              tx_flag=1;
//            }
//        }
//      else if(lora_api_internal_join__is_network_joined())
//        {
//          MYdevice.lora.join_status = 1;
//          if((MYdevice.lora.lora_rssi_last != 0xFF)&&(MYdevice.lora.lora_snr_last != 0xFF))
//            {
//              if(tx_flag==1)
//                {
//                  sendLoraJoinState();
//                  tx_flag=0;
//                }
//            }
//        }
//      else
//        {
////          MYdevice.lora.join_status = 0;
//          if(tx_flag==1)
//            {
//              sendLoraJoinState();
//              tx_flag=0;
//            }
//
//        }
    }


  }
  else
  {
      test_mode_active = 0;

      CMU_ClockEnable(cmuClock_WDOG0, true);
      CMU_ClockSelectSet(cmuClock_WDOG0, cmuSelect_LFXO); /* ULFRCO as clock source */

      WDOGn_Init(DEFAULT_WDOG, &init);
      WDOG_Enable(true);

    #ifdef TREX
      #ifdef ENPULLIO2
        GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 1);      // make output for TREX
        GPIO_PinOutSet(gpioPortC, 1);
        GPIO_PinModeSet(gpioPortB,CAP_PWR, gpioModeInput, 0 );
        GPIO_PinModeSet(gpioPortC,0, gpioModeInput, 0 );
        GPIO_PinModeSet(gpioPortC,2, gpioModeInput, 0 );
      #else
        GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 1);      // make output for TREX
        GPIO_PinModeSet(gpioPortB, 2, gpioModeInput, 1);         // Movement
        GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeWiredOr, 1);
      #endif
              ps_load(DEVICE_SKU_PS,&MYdevice.sku,1);  //define a default type
        MYdevice.sku &= 0xF0;  // clear device type
        MYdevice.sku |= 0x04;  // define TREX here
      ps_save(DEVICE_SKU_PS,&MYdevice.sku,1);  //define a default type
      updateDevice();
    #else
//		MYdevice.detection_threshold = 0;
//      GPIO_PinModeSet(gpioPortB, BIT_ACTIVATION, gpioModeInputPull, 1);     // make input for all other traps
//      GPIO_PinModeSet(gpioPortB, BIT_MOVEMENT, gpioModeInputPull, 1);         // Movement
////    GPIO_PinModeSet(gpioPortC, BIT_ACTIVATION, gpioModeInput, 0);     // make input for all other traps
////    GPIO_PinModeSet(gpioPortB, BIT_MOVEMENT, gpioModeInput, 0);         // Movement
//      GPIO_PinModeSet(gpioPortC, CAP_PWR, gpioModeWiredOr, 1);
    #endif

    #ifndef TREX
//        GPIO_PinModeSet(gpioPortC, 2, gpioModeDisabled, 0);   // power for pull up resistors on I/O  only on express trapper ambush
//        GPIO_PinModeSet(gpioPortC, 0, gpioModeInput , 0);   // power for pull up resistors on I/O
    #endif

  #ifndef TREX
      CMU_ClockEnable(cmuClock_GPIO, true);
      GPIOINT_Init();


      GPIO_PinModeSet(MAGSENSE_PORT, MAGSENSE_PIN, gpioModeInput , 0);   // Magsense
      GPIO_PinModeSet(BIT_ACTIVATION_P, BIT_ACTIVATION, gpioModeInputPull, 1);     // make input for all other traps
      GPIO_PinModeSet(BIT_MOVEMENT_P, BIT_MOVEMENT, gpioModeInputPull, 1);         // Movement
//        GPIO_PinModeSet(BIT_ACTIVATION_P, BIT_ACTIVATION, gpioModeInput, 0);     // make input for all other traps
//        GPIO_PinModeSet(BIT_MOVEMENT_P, BIT_MOVEMENT, gpioModeInput, 0);
      GPIO_PinModeSet(CAP_PWR_P, CAP_PWR, gpioModeWiredOr, 1);
      GPIO_PinModeSet(gpioPortA, 0, gpioModeInput, 0);





      GPIOINT_CallbackRegister(0, Pin3IntHandler);
      GPIOINT_CallbackRegister( 1, Pin1IntHandler);
      GPIOINT_CallbackRegister( 2, Pin2IntHandler);

//        GPIOINT_CallbackRegister( 0, GPIOIntHandler);
//        GPIOINT_CallbackRegister( 1, GPIOIntHandler);
//        GPIOINT_CallbackRegister( 2, GPIOIntHandler);

//        GPIO_IntConfig( gpioPortB, BIT_MOVEMENT, 1, 1, 1);
//        GPIO_IntConfig( gpioPortC, BIT_ACTIVATION, 1, 1, 1);
//        GPIO_IntConfig( MAGSENSE_PORT, MAGSENSE_PIN, 1, 1, 1);

      GPIO_ExtIntConfig( BIT_MOVEMENT_P, BIT_MOVEMENT, BIT_MOVEMENT,1, 1, 1);
      GPIO_ExtIntConfig( BIT_ACTIVATION_P, BIT_ACTIVATION,BIT_ACTIVATION, 1, 1, 1);
      GPIO_ExtIntConfig(MAGSENSE_PORT, MAGSENSE_PIN, MAGSENSE_PIN, 1, 1, 1);



//        uint32_t polarityMask = (uint32_t)1 << (6 + _GPIO_EM4WUEN_EM4WUEN_SHIFT);
//        uint32_t pinmask =  1UL << (6 + _GPIO_EM4WUEN_EM4WUEN_SHIFT);
//        GPIO_EM4EnablePinWakeup(pinmask, polarityMask);
//        BUS_RegBitWrite(&(GPIO->IEN), 6 + _GPIO_IEN_EM4WUIEN0_SHIFT, true);
//        GPIO_EM4WUExtIntConfig(gpioPortC, 0, 3, true, true, true);

      NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
      NVIC_EnableIRQ(GPIO_EVEN_IRQn);

      NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
      NVIC_EnableIRQ(GPIO_ODD_IRQn);



  #endif

    collectionInit();
    coll.active = true;

    // This keeps UART on so we can test device through serial port
//    SLEEP_SleepBlockBegin(sleepEM1);   // don't go to sleep for production purposes

    //uncomment below 2 lines to erase registration information from locked device
#ifdef RESET_FACTORY
    MYdevice.registration = 0x03;  //update the shadow variable
    ps_save(DEVICE_REG_PS, &MYdevice.registration, 1);  // 1 = registration   0 = unregistered  2 = development  set to unregistered by default if never powered on

//    MYdevice.sku = 0x00;  //update the shadow variable
//    ps_save(DEVICE_SKU_PS, &MYdevice.sku, 1);  // 1 = registration   0 = unregistered  2 = development  set to unregistered by default if never powered on

    uint8_t data;
    data = 0;
    ps_save(DEVICE_INTIALIZED_PS, &data, 1);
#endif

#ifdef ECOLAB_DEV
    MYdevice.registration = 0x03;  //update the shadow variable
    ps_save(DEVICE_REG_PS, &MYdevice.registration, 1);  // 1 = registration   0 = unregistered  2 = development  set to unregistered by default if never powered on

//    MYdevice.sku = 0x00;  //update the shadow variable
//    ps_save(DEVICE_SKU_PS, &MYdevice.sku, 1);  // 1 = registration   0 = unregistered  2 = development  set to unregistered by default if never powered on

    uint8_t data;
    data = 0;
    ps_save(DEVICE_INTIALIZED_PS, &data, 1);
#endif


    MYdevice.production_mode_timer = 0;
    MYdevice.production_mode = 1;      // activate production wake mode
    MYdevice.operatingMode = factorywake;
    MYdevice.registration  = 0x03;
    MYdevice.advertising_period = MS_100;
    MTsystem.LocalTimeCounter = 0;
    MYdevice.tx_pwr = 0;


//    MYdevice.lora.lora_rssi_last = 200;
//    MYdevice.lora.region = 3;
//    MYdevice.lora.payload = 2;
//    MYdevice.lora.retry = 1;
//    MYdevice.lora.ack = 0;
//    MYdevice.lora.tx_interval = 0x1E;
//    MYdevice.lora.retry = 1;
//    MYdevice.lora.lora_en = 1;



    send_lora_pkt = false;
    send_lora_heartbeat=false;
    send_lora_bell_msg=false;
    lora_hb_cnt = 0;
    first_event_cnt = 0;
    MYdevice.lora.join_attempts=0;
    MYdevice.lora.rejoin_attempt=0;
    MYdevice.lora.lora_snr_max=0;
    MYdevice.lora.lora_snr_min=20;
    MYdevice.lora.lora_rssi_max=128;
    MYdevice.lora.lora_rssi_min=0xFF;
    MYdevice.lora.event_trigger = 0;
    MYdevice.lora.ack_fail = 0;
    MYdevice.lora.initial_join = 0;
    MYdevice.lora.unreg_flag = 1;
    MYdevice.lora.reg_payload_sent = 0;
//    MYdevice.lora.join_test=1;

//    switch(REGION)
//    {
//      case NA:
//        MYdevice.lora.region = 1; //region North America
//        break;
//      case ANZ:
//        MYdevice.lora.region = 3; //region Australia/New Zealand
//        break;
//      case EU:
//        MYdevice.lora.region = 2; //region Europe
//        break;
//      default:
//        MYdevice.lora.region = 0;
//    }
//        ps_save(CONFIG_ZONE_OFFSET, &MYdevice.lora.region, 1);  // check to see what type this device is
        ps_load(CONFIG_ZONE_OFFSET, &MYdevice.lora.region, 1);  // check to see what type this device is
        if((MYdevice.lora.region>3)||(MYdevice.lora.region==0))
          MYdevice.lora.region = 1;

//        if(MYdevice.lora.region==3)
//          MYdevice.lora.region = 1;
//        else
//          MYdevice.lora.region = MYdevice.lora.region+1;
//
//        MYdevice.lora.region = 2;
//        ps_save(CONFIG_ZONE_OFFSET, &MYdevice.lora.region, 1);  // check to see what type this device is
//    MYdevice.wakeFlag = 0;
//    MYdevice.sku = 0x51;
//    ps_save(DEVICE_SKU_PS,&MYdevice.sku,1);  // check to see what type this device is
    ps_load(DEVICE_SKU_PS,&MYdevice.sku,1);  // check to see what type this device is


//    MYdevice.lora.lora_en = 1;
//    ps_save(CONFIG_LORA_ENABLE_OFFSET, &MYdevice.lora.lora_en, 1);  // check to see what type this device is
    ps_load(CONFIG_LORA_HB_OFFSET, &MYdevice.lora.tx_interval, 1);  // check to see what type this device is
    if(MYdevice.lora.tx_interval==0)
      MYdevice.lora.tx_interval = 24;
    MYdevice.lora.heartbeat = MYdevice.lora.tx_interval*3600;
//      MYdevice.lora.heartbeat = 5;

    tmr3_samp_time = 0;
    load_settings();

    MYdevice.sampleTime = 0;


    value  = checkKEY();
    //debug manufacturing date to be deleted
  #ifdef DBGWRITE_USERPAGE_EN
     backdoor_access();
  #endif

     GPIO_PinModeSet(gpioPortC, TP4, gpioModeWiredOr, 0);  // TP4
     GPIO_PinModeSet(gpioPortC, TP5, gpioModeWiredOr, 1);  // TP5

     GPIO_PinModeSet(gpioPortC, 4, gpioModeInput, 0);
     sl_uartdrv_init_instances();
#ifdef LORA
     initLetimer(1);
     lowPowerDelay(100);
     initUSART0();
     USART_IntEnable(USART0, USART_IEN_RXDATAV);
#endif

#ifdef PASSTHROUGH
        initUSART1();
        USART_IntEnable(USART1, USART_IEN_RXDATAV);
#endif

        //Sample 1
//        uint8_t dev_id[8] = {0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x20};
//        ps_save(EEPROM_DEVICE_ID_OFFSET,dev_id, EEPROM_DEVICE_ID_LENGTH);
//        uint8_t app_key[16] = {0xfc, 0x95, 0xe0, 0x7f, 0xab, 0xba, 0x7f, 0x86, 0x36, 0x37, 0x3d, 0x24, 0x5a, 0xc3, 0x31, 0xa5};
//        ps_save(EEPROM_LORA_APP_KEY_OFFSET,app_key, EEPROM_LORA_APP_KEY_LENGTH);
   //
   //     //Sample 2
//        uint8_t dev_id[8] = {0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x21};
//        ps_save(EEPROM_DEVICE_ID_OFFSET,dev_id, EEPROM_DEVICE_ID_LENGTH);
//        uint8_t app_key[16] = {0xbf, 0x2f, 0x09, 0xda, 0x65, 0xba, 0x89, 0x41, 0xbf, 0x32, 0xbb, 0xb7, 0x7d, 0x5a, 0x86, 0x4a};
//        ps_save(EEPROM_LORA_APP_KEY_OFFSET,app_key, EEPROM_LORA_APP_KEY_LENGTH);

   //
   //     //Sample 3
//        uint8_t dev_id[8] = {0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x22};
//        ps_save(EEPROM_DEVICE_ID_OFFSET,dev_id, EEPROM_DEVICE_ID_LENGTH);
//        uint8_t app_key[16] = {0x40, 0x0c, 0xe3, 0xcd, 0xd8, 0x37, 0x16, 0x49, 0xcc, 0xa4, 0xee, 0x3c, 0x0d, 0xc3, 0x16, 0x3d};
//        ps_save(EEPROM_LORA_APP_KEY_OFFSET,app_key, EEPROM_LORA_APP_KEY_LENGTH);

   //
        //Sample 4
//        uint8_t dev_id[8] = {0x00, 0x80, 0x00, 0x00, 0x08, 0x00, 0x02, 0x66};
//        ps_save(EEPROM_DEVICE_ID_OFFSET,dev_id, EEPROM_DEVICE_ID_LENGTH);
//        uint8_t app_key[16] = {0x16, 0x0B, 0x01, 0xF5, 0x69, 0xC9, 0xE9, 0x34, 0xA0, 0x3F, 0xFF, 0x75, 0xFE, 0x63, 0xFF, 0x93};
//        ps_save(EEPROM_LORA_APP_KEY_OFFSET,app_key, EEPROM_LORA_APP_KEY_LENGTH);
//
//
//
//
//     uint8_t app_eui[8] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
        uint8_t app_eui[8] = {0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, };
     ps_save(EEPROM_LORA_APP_EUI_OFFSET,app_eui, EEPROM_LORA_APP_EUI_LENGTH);
//
//     ps_load(EEPROM_DEVICE_ID_OFFSET,MYdevice.lora.devEUI, EEPROM_DEVICE_ID_LENGTH);
//     ps_load(EEPROM_LORA_APP_KEY_OFFSET,MYdevice.lora.appKEY, EEPROM_LORA_APP_KEY_LENGTH);
     ps_load(EEPROM_LORA_APP_EUI_OFFSET,MYdevice.lora.appEUI, EEPROM_LORA_APP_EUI_LENGTH);
     load_appKEY();
     load_devEUI();
     load_serial();
  }
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
   if(MYdevice.sku != 0xFF)
     {
        if(MYdevice.lora.lora_en==1)
          {
            if(MYdevice.lora.join_status != 2)
              {
                if(lora_api_internal_join__is_network_joined())
                  MYdevice.lora.join_status = 1;
                else
                  {
                    MYdevice.lora.join_status = 0;
    //                char_send('E',0);
                  }
              }
          }
        else
          {
            if(MYdevice.lora.join_status != 2)
              {
                MYdevice.lora.join_status = 0;
//                char_send('F',0);
              }
//            else
//              char_send('X',0);
          }

     }

#ifndef PASSTHROUGH
  lora_state_machine();
  check_radio_uart_read();
  API_EUSART_RADIO__TX_Data_Send();
#endif
  if(MYdevice.registration==1)
    {
      if(send_lora_pkt&&MYdevice.lora.lora_en)
//      if(send_lora_pkt&&MYdevice.lora.lora_en&&connectionState==0)
      {
          send_lora_pkt = false;
          if(radio_is_sleeping)
            {
              //intilize 100ms state machine timer and UART
               initLetimer(1);
               lowPowerDelay(100);
               letmr_flag=0;
            }
//            radio_transmit_hex_data_12(0x07, 0xCD, myBeacon.devMACadd[0], myBeacon.devMACadd[1], myBeacon.devMACadd[2], myBeacon.devMACadd[3], myBeacon.devMACadd[4], myBeacon.devMACadd[5],0x01, 0, coll.EventsCounter,false, true);
//          if(MYdevice.lora.payload==0x01)
//            {
//              radio_transmit_hex_data_12(0x0A,                              /*communication schema*/
//                                         MYdevice.sku,                      /*device type*/
//                                         MYdevice.fw_ver,                              /*firmware version*/
//                                         ((MYdevice.tx_pwr&0x00FF)>>8)+128, /*transmit power*/
//                                         myBeacon.RRSSI,                    /*device mode*/
//                                         MTsystem.temperaturecurrent,       /*temperature*/
//                                         DeviceBatt.batterylvl,             /*battery level*/
//                                         coll.EventsCounter,                /*events*/
//                                         MYdevice.lora.pkt_fail,            /*failed lora transmissions*/
//                                         MYdevice.lora.join_attempts,       /*join attempts*/
//                                         0x00,                              /*group ID byte 1*/
//                                         false,
//                                         true);
//            }
//          else if(MYdevice.lora.payload == 0x02)
//            {
                radio_transmit_hex_data_12(0x07, 0xCD, myBeacon.devMACadd[0], myBeacon.devMACadd[1], myBeacon.devMACadd[2], myBeacon.devMACadd[3], myBeacon.devMACadd[4], myBeacon.devMACadd[5],0x04, coll.EventsCounter,DeviceBatt.batterylvl,false, true);
//            }
        }
    }
}




/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  sl_status_t ind;
  bd_addr address;
  uint8_t address_type, i;
  uint8_t system_id[8];
//  uint8_t ind_status[4] = {0};

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert_status(sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      MYdevice.operatingMode = unregwake;


//      sl_bt_test_dtm_tx_v4(2, 37, 0, 1, -3); //low frequency, channel 0 (2402MHz), 6dBm output, modulated 0xFF continuously sent
//      sl_bt_test_dtm_tx_v4(2, 37, 19, 1, -3);  //mid frequency, channel 19 (2440MHz), 6dBm output, modulated 0xFF continuously sent
//      sl_bt_test_dtm_tx_v4(2, 37, 39, 1, 3);  //high frequency, channel 39 (2480MHz), 6dBm output, modulated 0xFF continuously sent
    //  sl_bt_test_dtm_tx_v4(4, 255, 39, 1, 0);
//      sl_bt_test_dtm_tx_v4(4, 255, 39, 1, 0);
//while(1);

      sl_bt_system_set_lazy_soft_timer(49152,4915,TIMER3_HANDLE,0);
//      sl_bt_system_set_lazy_soft_timer(3277,328,TIMER3_HANDLE,0);
#ifdef TREX
      sl_bt_system_set_lazy_soft_timer(49152,4915,TIMER5_HANDLE,0); // timer for time repeating
#endif

//      swSetupAdvBeaconing();
//      WDOG_Enable(false);
      break;
    case sl_bt_evt_system_soft_timer_id:
//      char_send('A');
      if(evt->data.evt_system_soft_timer.handle == TIMER1_HANDLE){ // 300 sec
//          char_send('2',0);
          lora_payload();
          ind = sl_bt_gatt_server_send_notification(evt->data.evt_gatt_server_characteristic_status.connection, gattdb_lora_settings, 107, lora_dev_settings);
       }
      else if(evt->data.evt_system_soft_timer.handle == TIMER2_HANDLE) // collection time out
      {

#ifndef TREX
          if (GPIO_PinInGet(gpioPortB, BIT_MOVEMENT))     //&& (GPIO_PinInGet(gpioPortA, BIT_ACTIVATION)))
              SignalTOT();
          else // let's not terminate in the middle of movement pulse - very unlikely
          {
            prepare_to_stop(); // on the next rising movement pulse
            sl_bt_system_set_lazy_soft_timer(SHORT_COLLECTION_TIMEOUT, SHORT_COLLECTION_TIMEOUT/4,  TIMER2_HANDLE, 1);
          }
#endif
      }

      else if(evt->data.evt_system_soft_timer.handle == TIMER3_HANDLE)
      {
          if(MTsystem.realtimecounter < RTCC->CNT) MTsystem.realtimedifference += (RTCC->CNT - MTsystem.realtimecounter );   // this is the bucket holding the difference-
          else MTsystem.realtimedifference += (0x100000000 - MTsystem.realtimecounter ) + RTCC->CNT;                 //accumulated since the last wake up

          while( MTsystem.realtimedifference  >= 65536)
          {
              MTsystem.realtimedifference -= 65536;
              MTsystem.LocalTimeCounter++; //add 2 seconds
              if(myUNIXtime>0)
                myUNIXtime += 2;
//              if(MYdevice.lora.join_status = 1)
//                send_lora_pkt = true;
              if(MYdevice.registration==1)
                lora_hb_cnt += 2;

              if(MYdevice.lora.manual_join_delay_tm>0)
                MYdevice.lora.manual_join_delay_tm--;

              if(MYdevice.lora.join_delay_flag)
                {
//                  char_send('G', 0);
                  MYdevice.lora.join_delay_counter++;
//                  char_send(MYdevice.lora.join_delay_counter, 1);
                  if(MYdevice.lora.join_delay_counter>MYdevice.lora.join_delay)
                    {
//                      char_send('J', 0);
                      MYdevice.lora.join_delay_counter=0;
                      MYdevice.lora.join_delay_flag=0;
                      MYdevice.lora.rejoin = 1;
                    }
                }
              if(MYdevice.lora.ping_result==3)
                {
//                  char_send(ping_cnt,1);
                  ping_cnt++;
                }
              if(ping_cnt>20)
                {
                  ping_cnt = 0;
                  MYdevice.lora.ping_result = 1;
//                  char_send('A',0);
                }
              if(MYdevice.lora.join_status==3)
                join_cnt++;
              if(join_cnt>14)
                {
                  join_cnt = 0;
                  MYdevice.lora.join_status = 1;
                }
          }
          MTsystem.realtimecounter = RTCC->CNT;  // this is the saved time for the next wake up

          if(lora_hb_cnt>=MYdevice.lora.heartbeat) //send heartbeat every 24 hrs
//          if(lora_hb_cnt>=20) //send heartbeat every 24 hrs
            {

              lora_hb_cnt = 0;
              send_lora_pkt = true;
//              MYdevice.lora.join_test=1;
//              char_send('A',0);
            }
          if(first_event_flag)  //first event trigger pkt immediately and every 2 hrs for 12 hours
            {
              send_lora_pkt = true;
              lora_hb_cnt = 0;
              first_event_flag = 0;
            }
//          sendhb();
          if(newKey == 1)
          {
              if(keyValidTimer++ > 57600)
                NewKeySave();  // 57600 =  24 hours
              //newKey = 0;
          }
          //if(MYdevice.production_mode == 1) MYdevice.operatingMode = factorywake;
          if(MYdevice.production_mode == 1)
          {
#ifndef PASSTHROUGH
              sendVER();
              sendMAC();
              sendmfg();
              sendhwv();
              sendDevEui();
              sendAppk();
              sendAppEui();
              sendSerial();
              sendREGION();
#endif
            if((MYdevice.production_mode_timer++) >= PRODUCTION_MODE_TIME) // 20 x 1.5 seconds
            {
              MYdevice.production_mode = 0;   // clear production test_mode
#ifndef PASSTHROUGH
              UARTDRV_DeInit(sl_uartdrv_usart_debug_handle);
//              GPIO_PinModeSet(gpioPortA, 3, gpioModeDisabled, 1);   //added for sleep current test - JP - 11242023
//              GPIO_PinModeSet(gpioPortA, 4, gpioModeDisabled, 1);
//              SLEEP_SleepBlockEnd(sleepEM1);  // resume normal power  management
#endif
              defaultsLOAD();
              deviceLOADx();
              updateDevice();    //setup filter time and I/O for TREX
            }
          }

          if(MYdevice.operatingMode == factorywake || MYdevice.operatingMode == factorysleep)
          {
              if( (MYdevice.wakeTimer++ > FACTORY_SLEEP_TIME) && (connectionState==0)) // 60 * 1.5 = 90 seconds
              {
                  MYdevice.operatingMode = factorysleep;    // end factory wake mode
                  sl_bt_system_set_lazy_soft_timer(0,0,TIMER3_HANDLE,0); // turn off timer
                  deviceClear();   //clear device since we are going back to sleep
                  WDOG_Enable(false);
//                  sl_bt_advertiser_delete_set(advertising_set_handle);
              }
          }
          else if(MYdevice.operatingMode == unregwake || MYdevice.operatingMode == unregsleep)
          {
              if( (MYdevice.wakeTimer++ > FACTORY_SLEEP_TIME) && (connectionState==0)) // 60 * 1.5 = 90 seconds
              {
                  MYdevice.operatingMode = unregsleep;    // end unregistered wake mode
                  sl_bt_system_set_lazy_soft_timer(0,0,TIMER3_HANDLE,0); // turn off timer
                  deviceClear();   //clear device since we are going back to sleep
                  WDOG_Enable(false);
//                  sl_bt_advertiser_delete_set(advertising_set_handle);
              }
          }

          if(DeviceBatt.battTIMER++ > BATTSAMPLEPERIOD)
          {
              DeviceBatt.battTIMER = 0;
              BATTcheck();
          }
          if(MYdevice.sampleTime++ > TEMPSAMPLEPERIOD) // we measure temperature every 3600 seconds
          {
            MYdevice.sampleTime = 0;
            TempMeasure();
          }

#ifndef TREX
          //sensor timed operation implementation
          if(MYdevice.sensSAMP++>SENS_SAMP) //only enter this routing every 15 mins
            {
//              advertise_stop();
              MYdevice.sensSAMP = 0;

              //save local timer to NVM
              //changed by JP on Aug 6 2021, local time was saved in reverse order
//              data_tmp[0] = (uint8_t)(MTsystem.LocalTimeCounter >> 24);
//              data_tmp[1] = (uint8_t)((MTsystem.LocalTimeCounter & 0x00FF0000)>> 16);
//              data_tmp[2] = (uint8_t)((MTsystem.LocalTimeCounter & 0x0000FF00)>> 8);
//              data_tmp[3] = (uint8_t)MTsystem.LocalTimeCounter;
//              ps_save(UP_TIME, data_tmp , 4);

              if(myUNIXtime>0)  //only execute the following code if myUNIXtime has been initialised
              {
                //save UNIX time to NVM
                  //changed by JP on Aug 6 2021, unix time was saved in reverse order
//                data_tmp[0] = (uint8_t)(myUNIXtime >> 24);
//                data_tmp[1] = (uint8_t)((myUNIXtime & 0x00FF0000)>> 16);
//                data_tmp[2] = (uint8_t)((myUNIXtime & 0x0000FF00)>> 8);
//                data_tmp[3] = (uint8_t)myUNIXtime;
//                ps_save(UNIX_TIME, data_tmp, 4);

//#ifndef TREX
                if(MYdevice.operatingMode == registered) //only implement time-based sensing for registered units
                  {
                    if((MYdevice.sensON.data<96)&&(MYdevice.sensOFF.data<96))  //check if there is valid time settings.
                      {
                        calcDate(&time_i);
                        prev_senseFLAG = MYdevice.sensFLAG;
                        MYdevice.sensFLAG = 0;
                        if(MYdevice.sensON.hour>MYdevice.sensOFF.hour)
                          {

                              if((time_i.tm_hour>= MYdevice.sensON.hour)||(time_i.tm_hour<MYdevice.sensOFF.hour))
                                {
                                  if(time_i.tm_hour==MYdevice.sensON.hour)
                                    {
                                      if(time_i.tm_min>=MYdevice.sensON.min)
                                        MYdevice.sensFLAG = 1;
                                      else
                                        MYdevice.sensFLAG = 0;
                                    }
                                  else
                                    MYdevice.sensFLAG = 1;
                                }

                              if((time_i.tm_hour>= MYdevice.sensOFF.hour)&&(time_i.tm_hour<MYdevice.sensON.hour))
                                {
                                  if(time_i.tm_hour==MYdevice.sensOFF.hour)
                                    {
                                      if(time_i.tm_min>=MYdevice.sensOFF.min)
                                        MYdevice.sensFLAG = 0;
                                      else
                                        MYdevice.sensFLAG = 1;
                                    }
                                  else
                                    MYdevice.sensFLAG = 0;
                                }
                          }
                        else if(MYdevice.sensON.hour<MYdevice.sensOFF.hour)
                          {
                              if((time_i.tm_hour>= MYdevice.sensOFF.hour)||(time_i.tm_hour<MYdevice.sensON.hour))
                                {
                                  if(time_i.tm_hour==MYdevice.sensOFF.hour)
                                    {
                                      if(time_i.tm_min>=MYdevice.sensOFF.min)
                                        MYdevice.sensFLAG = 0;
                                      else
                                        MYdevice.sensFLAG = 1;
                                    }
                                  else
                                    MYdevice.sensFLAG = 0;
                                }

                              if((time_i.tm_hour>= MYdevice.sensON.hour)&&(time_i.tm_hour<MYdevice.sensOFF.hour))
                                {
                                  if(time_i.tm_hour==MYdevice.sensON.hour)
                                    {
                                      if(time_i.tm_min>=MYdevice.sensON.min)
                                        MYdevice.sensFLAG = 1;
                                      else
                                        MYdevice.sensFLAG = 0;
                                    }
                                  else
                                    MYdevice.sensFLAG = 1;
                                }
                          }
                        else if(MYdevice.sensON.hour==MYdevice.sensOFF.hour)
                          {
                            if(time_i.tm_hour==MYdevice.sensON.hour)
                              {
                                if(MYdevice.sensON.min>MYdevice.sensOFF.min)
                                  {
                                    if((time_i.tm_min<MYdevice.sensON.min)&&(time_i.tm_min>MYdevice.sensOFF.min))
                                      MYdevice.sensFLAG = 0;
                                    else if((time_i.tm_min<MYdevice.sensOFF.min)||(time_i.tm_min>=MYdevice.sensON.min))
                                      MYdevice.sensFLAG = 1;
                                  }
                                else
                                  {
                                    if((time_i.tm_min<MYdevice.sensON.min)||(time_i.tm_min>=MYdevice.sensOFF.min))
                                      MYdevice.sensFLAG = 0;
                                    else if((time_i.tm_min>=MYdevice.sensON.min)&&(time_i.tm_min<MYdevice.sensOFF.min))
                                      MYdevice.sensFLAG = 1;
                                  }
                              }
                            else if (time_i.tm_hour<MYdevice.sensON.hour)
                              {
                                if(MYdevice.sensON.min>MYdevice.sensOFF.min)
                                  MYdevice.sensFLAG = 1;
                                else
                                  MYdevice.sensFLAG = 0;
                              }
                            else
                              {
                                if(MYdevice.sensON.min>MYdevice.sensOFF.min)
                                  MYdevice.sensFLAG = 1;
                                else
                                  MYdevice.sensFLAG = 0;
                              }
                          }
                        if((MYdevice.sensFLAG)&&(MYdevice.sensFLAG != prev_senseFLAG)) //if flag is set, enable interrupts
                          {
                            GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeWiredOr, 1);
                            GPIO_PinModeSet(gpioPortB, BIT_ACTIVATION, gpioModeInputPull, 1);     // make input for all other traps
                            GPIO_PinModeSet(gpioPortB, BIT_MOVEMENT, gpioModeInput, 1);         // Movement
                            GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1);   // power for pull up resistors on I/O  only on express trapper ambush
                            GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 1);   // power for pull up resistors on I/O

                            GPIO_IntConfig( gpioPortB, BIT_MOVEMENT, 1, 1, 1);
                            GPIO_IntConfig( gpioPortB, BIT_ACTIVATION, 1, 1, 1);
                          }
                        else if ((MYdevice.sensFLAG==0)&&(MYdevice.sensFLAG != prev_senseFLAG))
                          {
                            NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
                            NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
                            GPIO_IntConfig( gpioPortB, BIT_MOVEMENT, 1, 1, 0);
                            GPIO_IntConfig( gpioPortB, BIT_ACTIVATION, 1, 1, 0);
                            GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeWiredAnd, 0);
//                            GPIO_PinModeSet(gpioPortB, CAP_PWR, gpioModeDisabled, 0);
                            GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredOr, 0);   // power for pull up resistors on I/O  only on express trapper ambush
                            GPIO_PinModeSet(gpioPortB, 2, gpioModeWiredOr, 0);   // power for pull up resistors on I/O
                            GPIO_PinModeSet(gpioPortC, 2, gpioModeWiredOr, 0);   // power for pull up resistors on I/O
                            GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredOr, 0);   // power for pull up resistors on I/O
                            sl_bt_system_set_lazy_soft_timer(0,0,TIMER4_HANDLE,0); // turn off timer
                            sl_bt_system_set_lazy_soft_timer(0,0,TIMER6_HANDLE,0); // turn off timer
                            sl_bt_system_set_lazy_soft_timer(0,0,TIMER7_HANDLE,0); // turn off timer
                            sl_bt_system_set_lazy_soft_timer(0,0,TIMER2_HANDLE,0); // turn off timer

                          }
                      }
                  }
//#endif
              }
            }

//#else
//      if(MYdevice.wakeFlag)
//        {
//          sl_bt_system_set_lazy_soft_timer(49152,4915,TIMER5_HANDLE,0); // timer for time repeating
//          MYdevice.wakeFlag = 0;
//        }
#endif

//          sendSKU();
//          sendLORAflag();
//          sendLORAstatus();

          if(connectionState==0)
            swSetupAdvBeaconing();
          WDOG_Feed();
//          char_send(MYdevice.lora.ping_result, 1);

      }
      else if(evt->data.evt_system_soft_timer.handle == TIMER4_HANDLE){
          coll.active = true; // now we can collect again
      }
      else if(evt->data.evt_system_soft_timer.handle == TIMER5_HANDLE){
#ifdef TREX
      #ifdef ENPULLIO2
          GPIO_PinModeSet(gpioPortB, 2, gpioModeInputPull, 1);         // set pull up on hall output open drain
      #else
          GPIO_PinOutSet(gpioPortC, 1);   // power up Hall Sensor every n Seconds check the state of hall next timer event
      #endif
          HallPower = 1;  // flag for Hall Power or Pull up
          HallCount = 0;
          sl_bt_system_set_lazy_soft_timer(3276, 204,  TIMER8_HANDLE, 1);  // was 2048
          WDOG_Feed();


#endif
//          lora_state_machine();
//          check_radio_uart_read();
      }
      else if(evt->data.evt_system_soft_timer.handle == TIMER6_HANDLE){
          coll.Filter = 0;  // filtering timed out
      }
      else if(evt->data.evt_system_soft_timer.handle == TIMER7_HANDLE){

          SuppressDetect = 0;       // clear suppress detections for filter time for PMP handling
      }
      else if(evt->data.evt_system_soft_timer.handle == TIMER8_HANDLE){
      #ifdef TREX
                    if(GPIO_PinInGet(gpioPortB, 2) && HallPower == 1)  // flag for Hall Power)
                    {
                        coll.EventsCounter = 0;  // trap is unsnapped leave power on
                        TREXarmed = 1;
                        if(Twake)
                        {
                            Twake = 0;  // TREX wake trigger clear
                            wakeDevice();
                        }
                        if(MYdevice.production_mode == 1 && SendTREXstatus == 1)
                            {
//                                sendHALL(1);
                                SendTREXstatus = 0; // new status
                            }
                    }
                    else
                    {
                      #ifdef DISPGHALL  //disable the power gating in the hall sensor, set to always "ON"
                          #ifdef ENPULLIO2
                            //disable the pull up
                            GPIO_PinModeSet(gpioPortB, 2, gpioModeDisabled, 0);  //disable the pullup
                            HallPower = 0;  // flag for Hall Power
                          #else
                            //disable power gating that charge the bypass cap that have a peak of 20mA
                            GPIO_PinOutSet(gpioPortC, 1);
                            HallPower = 1;  // flag for Hall Power
                          #endif
                      #else
                        //GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);      // make output for TREX
                        GPIO_PinOutClear(gpioPortC, 1);  // power off Hall Sensor trap has snapped
                        HallPower = 0;  // flag for Hall Power
                      #endif
                        coll.EventsCounter = 1;
                        if(TREXarmed)
                        {
                            SignalTOT();
                            TREXarmed = 0;
                        }
                        //HallPower = 0;
                        Twake = 1;  // TREX wake trigger set
                        if(MYdevice.production_mode == 1 && SendTREXstatus == 0)
                        {
//                            sendHALL(0);
                            SendTREXstatus = 1; // new status
                        }
                    }
                    WDOG_Feed();
      #endif
                }
      sl_bt_nvm_load( 0x4000, 1, (size_t*)1, &readval );
      break;

    case sl_bt_evt_gatt_server_user_read_request_id:
      if(evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_signature){ //gattdb_user_type
          if(FindSignature(EventRequested)== NULL)
          {
              uint8_t empty = 0x00;
              uint16_t sentlen = 1;
              sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection, gattdb_signature , 0, 1, &empty, &sentlen);
          }
          else{
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection, gattdb_signature , 0, (*FindSignature(EventRequested)-1), (FindSignature(EventRequested)+1),0);
          }

      }
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_min_temp )
      {
          uint16_t sentlen = 1;
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                    gattdb_min_temp, 0, 1, &minTemp, &sentlen);
      }
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_max_temp )
      {
          uint16_t sentlen = 1;
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                    gattdb_max_temp, 0, 1, &maxTemp, &sentlen);
      }
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_firmware_version )
      {
          uint16_t sentlen = 5;
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                    gattdb_firmware_version, 0, 5, firmware_version, &sentlen);
      }
      //need to fix this to send out advertising period
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_advertising_period )
      {
          uint8_t dummyad  = MYdevice.advertising_period/160;
          uint16_t sentlen = 1;
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                    gattdb_advertising_period, 0, 4, &dummyad, &sentlen);
      }
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_hardware_version )
      {
//          uint8_t dummyhw[]  = {"0.00"};
          uint16_t sentlen = 4;
          uint32_t hw_ver = USERDATA[HW_VERSION];
          if (hw_ver == 0xFFFFFFFF)
            {
              hw_ver = 0x312e3235; //mem address was not written
            }
      #ifdef USERPAGE_RD_EN
            uint8_t hardware_version[4] = {0};
            hardware_version[3] = (uint8_t) ((hw_ver &  0x000000FF));       //3
            hardware_version[2] = (uint8_t) (((hw_ver & 0x0000FF00) >> 8));   //2
            hardware_version[1] = (uint8_t) (((hw_ver & 0x00FF0000) >> 16));  //.
            hardware_version[0] = (uint8_t) (((hw_ver & 0xFF000000) >> 24));  //1
      #endif
            sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                      gattdb_hardware_version, 0, 4, hardware_version, &sentlen);
      }
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_device_type )
      {
          uint16_t sentlen = 1;
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                    gattdb_device_type, 0, 1, &MYdevice.sku, &sentlen);
      }
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_authentication_status )
      {
          uint16_t sentlen = 1;
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                    gattdb_authentication_status, 0, 1, &MYdevice.authSTATUS, &sentlen);
      }
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_detection_threshold )
      {
          uint16_t sentlen = 1;
          ps_load(DEVICE_CONFIGURATION_PS,&MYdevice.detection_threshold, 1);
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                    gattdb_detection_threshold, 0, 1, &MYdevice.detection_threshold, &sentlen);
      }
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_since_reset )
      {
//          uint8_t sincereset[4] = {0};
          uint16_t sentlen = 4;
          uint8_t events[4] = {0};
          events[0] = (uint8_t)EventsSinceReset & 0x000000FF;
          events[1] = (uint8_t)(EventsSinceReset & 0x0000FF00) >> 8;
          events[2] = (uint8_t)(EventsSinceReset & 0x00FF0000) >> 16;
          events[3] = (uint8_t)(EventsSinceReset & 0xFF000000) >> 24;
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                    gattdb_since_reset, 0, 4, events, &sentlen);
      }
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_transfer_next )
      {
          uint16_t sentlen = 4;
          uint8_t detects[4] = {0};
          detects[0] = (uint8_t)coll.DetectionsCounter & 0x000000FF;
          detects[1] = (uint8_t)(coll.DetectionsCounter & 0x0000FF00) >> 8;
          detects[2] = (uint8_t)(0x00000000 & 0x00FF0000) >> 16;
          detects[3] = (uint8_t)(0x00000000 & 0xFF000000) >> 24;
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                    gattdb_transfer_next, 0, 4, detects, &sentlen);
      }
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_random_key )
      {
          uint16_t sentlen = 32;
          uint8_t random[32] = {0};
          uint8_t registration = 1;
//          char_send('M');
          ps_load(DEVICE_REG_PS, &registration, 1);
          if ((registration != 0x01)||(MYdevice.authSTATUS==5))ps_load(0x4001, random, 32);  // if not registered send random key
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                    gattdb_random_key, 0, 32, random, &sentlen);
      }
      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_site_key )
      {
          uint16_t sentlen = 16;
          uint8_t site_read[16] = {0};
//          uint8_t registration = 1;
//          char_send('M',0);
//          ps_load(DEVICE_REG_PS, &registration, 1);
          if (MYdevice.authSTATUS==5)
            {
//              char_send('C',0);
              (void)ps_load(DEVICE_SITEKEY_PS, site_read, 16);  // if not registered send random key
            }
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
                                                    gattdb_site_key, 0, 16, site_read, &sentlen);
      }
//      else if( evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_lora_settings )
//      {
//          uint16_t sentlen = 88;
////          char_send('M');
//          lora_payload();
//          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection,
//                                                    gattdb_lora_settings, 0, 88, lora_dev_settings, &sentlen);
//      }
#ifdef USERPAGE_RD_EN
      else if(evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_manufacture_date) {
//          char_send('m');
        uint32_t manufact_date = USERDATA[MANUFAC_DATE];
        if ((manufact_date == 0xFFFFFFFF) | (manufact_date == 0x00000000))
          {
//            char_send('x');
            manufact_date = 0x00210101; //if mem address was not written
          }
          uint8_t manufacturing_date[6] = {0};
          uint16_t sentlen = 6;
          manufacturing_date[5] = (uint8_t) ((manufact_date & 0x0000000F)         + 48);  //D0
          manufacturing_date[4] = (uint8_t) (((manufact_date & 0x000000F0) >> 4)  + 48);  //D1
          manufacturing_date[3] = (uint8_t) (((manufact_date & 0x00000F00) >> 8)  + 48);  //M0
          manufacturing_date[2] = (uint8_t) (((manufact_date & 0x0000F000) >> 12) + 48);  //M1
          manufacturing_date[1] = (uint8_t) (((manufact_date & 0x000F0000) >> 16) + 48);  //Y0
          manufacturing_date[0] = (uint8_t) (((manufact_date & 0x00F00000) >> 20) + 48);  //Y1
          sl_bt_gatt_server_send_user_read_response(evt->data.evt_gatt_server_user_read_request.connection, gattdb_manufacture_date   , 0, 6, manufacturing_date, &sentlen);
        }
#endif

      break;
    case sl_bt_evt_gatt_server_user_write_request_id:
      if(gattdb_session_keys == evt->data.evt_gatt_server_user_write_request.characteristic)
        {
//          sl_bt_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_write_request.connection, gattdb_session_keys, 0);
          sl_bt_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_read_request.connection, gattdb_session_keys, 0);
                 uint8_t auth = 0xFF;
                 MYdevice.authSTATUS = 0;   // start at 0 to indicate no authentication
                 ps_load(DEVICE_REG_PS, &MYdevice.registration,1); // load registration state


//                 if(evt->data.evt_gatt_server_attribute_value.value.len==55){
                 if(evt->data.evt_gatt_server_user_write_request.value.len==55){
                 uint8_t i = 0, j=38;

                 uint8_t session[55] = {0};
                 session[54]=0xFF;

                 for(i=0; i<55; i++)
                   {
                     session[i] = evt->data.evt_gatt_server_user_write_request.value.data[i];
//                     char_send(session[i],1);
                   }
       //if(session[54]!=0xFF){
                 for(i=0; i<6; i++)
                   {
                     data_parse[i] = evt->data.evt_gatt_server_user_write_request.value.data[j];
                     j++;
                   }
                 j=44;
                 for(i=0; i<10; i++)
                   {
                     lora_parse[i] = evt->data.evt_gatt_server_user_write_request.value.data[j];
                     j++;
                   }
       //          settings_parse();
                 switch(MYdevice.registration)
                 {

                     case 0:
                         //auth = AuthenticateFactoryUser( &evt->data.evt_gatt_server_attribute_value.value);   // device was unregistered
                         auth = AuthenticateRegisterUser(&evt->data.evt_gatt_server_user_write_request.value, &MYdevice);  // maybe we want to register?
                     break;

                     case 1:
                       if(MYdevice.sku != 0xFF)
                         {
                           auth = AuthenticateSecureUser(&evt->data.evt_gatt_server_user_write_request.value, &MYdevice);  // now we are registered
                           if(auth != 0)
                             auth = AuthenticateLockedUser(&evt->data.evt_gatt_server_user_write_request.value, &MYdevice);  // maybe we want to register?
                         }
                       else
                           auth = AuthenticateInstallationUser(&evt->data.evt_gatt_server_user_write_request.value, &MYdevice);  // maybe we want to register?
                         break;

                     case 2:
                         //development mode
                         break;

                     case 3:
                       if(MYdevice.sku != 0xFF)
                         {
                         auth = AuthenticateFactoryUser( &evt->data.evt_gatt_server_user_write_request.value, &MYdevice);   // device is at factory
                         if(auth != 0)
                           auth = AuthenticateRegisterUser(&evt->data.evt_gatt_server_user_write_request.value, &MYdevice);  // maybe we want to register?
                         if(auth != 0)
                           auth = AuthenticateInstallationUser(&evt->data.evt_gatt_server_user_write_request.value, &MYdevice);  // maybe we want to register?
                         }
                       else
                         {
                           auth = AuthenticateFactoryUser( &evt->data.evt_gatt_server_user_write_request.value, &MYdevice);   // device is at factory
                           if(auth != 0)
                             auth = AuthenticateInstallationUser(&evt->data.evt_gatt_server_user_write_request.value, &MYdevice);  // maybe we want to register?
                         }

                         break;

                     default:
                         auth = AuthenticateFactoryUser( &evt->data.evt_gatt_server_user_write_request.value, &MYdevice);   // device is at factory
                         break;

                 }

       #ifdef GEN2
                 if((auth==0) && (MYdevice.authSTATUS != 5)){
                     //device passed the Authenticate function

                     settings_parse();
       //              if((lora_parse[6]==0)&&(MYdevice.registration==1))
       //              if(MYdevice.registration>0)
                     if(MYdevice.registration==1)
                       lora_settings_parse();
                     else if ((MYdevice.sku != 0xFF)&&(MYdevice.lora.lora_alive_flag == 0))
                       {
       //                  lora_api_internal_join__restart();
                         unreg_lora();
                       }
                     save_settings();
                     //added the following 2 functions to reset TMR3 to sync with advertising rate
                     advertise_stop();
       //              if(MYdevice.lora.rejoin)
       //                lora_api_internal_join__restart();
                     sl_bt_system_set_lazy_soft_timer(0,0,TIMER3_HANDLE,0); // turn off timer
       //              swSetupAdvBeaconing();  //added 2021-07-08 - JP
                     sl_bt_system_set_lazy_soft_timer(49152,4915,TIMER3_HANDLE,0); //added 2021-07-08 - JP

                 }

       #endif
       //          advertise_stop(); //removed 2021-07-08 - JP
       //          swSetupAdvBeaconing();
                 }
                 else
                   MYdevice.authSTATUS = 1;

             }
      else if(gattdb_site_key == evt->data.evt_gatt_server_user_write_request.characteristic)
      {
          uint8_t registration = 0xFF;
          uint8_t site_write[16]={0};
          uint8_t i;
//          sl_bt_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_read_request.connection, gattdb_site_key, 0);
          sl_bt_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_write_request.connection, gattdb_site_key, 0);
          ps_load(DEVICE_REG_PS, &registration, 1);    // if device is not registered save the site key
//          if(registration != 1) ps_save(DEVICE_SITEKEY_PS, evt->data.evt_gatt_server_attribute_value.value.data, 16);
//          char_send('D',0);
//          char_send(MYdevice.registration,1);
          if(MYdevice.registration != 0x01)
            {
//              char_send('E',0);
              for(i=0; i<16; i++)
                {
                  site_write[i] = evt->data.evt_gatt_server_user_write_request.value.data[i];
                }
//              ps_save(DEVICE_SITEKEY_PS, evt->data.evt_gatt_server_attribute_value.value.data, 16);
              ps_save(DEVICE_SITEKEY_PS, site_write, 16);
            }
      }


      break;

    case sl_bt_evt_gatt_server_attribute_value_id:
      if(gattdb_signature_index == evt->data.evt_gatt_server_attribute_value.attribute)
      {
          EventIndexRequest( &evt->data.evt_gatt_server_attribute_value.value);
      }
      else if(gattdb_session_keys == evt->data.evt_gatt_server_attribute_value.attribute)
      {

          uint8_t auth = 0xFF;
          MYdevice.authSTATUS = 0;   // start at 0 to indicate no authentication
          ps_load(DEVICE_REG_PS, &MYdevice.registration,1); // load registration state


          if(evt->data.evt_gatt_server_attribute_value.value.len==55){

          uint8_t i = 0, j=38;

          uint8_t session[55] = {0};
          session[54]=0xFF;

          for(i=0; i<55; i++)
            {
              session[i] = evt->data.evt_gatt_server_attribute_value.value.data[i];
            }
//if(session[54]!=0xFF){
          for(i=0; i<6; i++)
            {
              data_parse[i] = evt->data.evt_gatt_server_attribute_value.value.data[j];
              j++;
            }
          j=44;
          for(i=0; i<10; i++)
            {
              lora_parse[i] = evt->data.evt_gatt_server_attribute_value.value.data[j];
              j++;
            }
//          settings_parse();
          switch(MYdevice.registration)
          {

              case 0:
                  //auth = AuthenticateFactoryUser( &evt->data.evt_gatt_server_attribute_value.value);   // device was unregistered
                  auth = AuthenticateRegisterUser(&evt->data.evt_gatt_server_attribute_value.value, &MYdevice);  // maybe we want to register?
              break;

              case 1:
                if(MYdevice.sku != 0xFF)
                  {
                    auth = AuthenticateSecureUser(&evt->data.evt_gatt_server_attribute_value.value, &MYdevice);  // now we are registered
                    if(auth != 0)
                      auth = AuthenticateLockedUser(&evt->data.evt_gatt_server_attribute_value.value, &MYdevice);  // maybe we want to register?
                  }
                else
                    auth = AuthenticateInstallationUser(&evt->data.evt_gatt_server_attribute_value.value, &MYdevice);  // maybe we want to register?
                  break;

              case 2:
                  //development mode
                  break;

              case 3:
                if(MYdevice.sku != 0xFF)
                  {
                  auth = AuthenticateFactoryUser( &evt->data.evt_gatt_server_attribute_value.value, &MYdevice);   // device is at factory
                  if(auth != 0)
                    auth = AuthenticateRegisterUser(&evt->data.evt_gatt_server_attribute_value.value, &MYdevice);  // maybe we want to register?
                  if(auth != 0)
                    auth = AuthenticateInstallationUser(&evt->data.evt_gatt_server_attribute_value.value, &MYdevice);  // maybe we want to register?
                  }
                else
                  {
                    auth = AuthenticateFactoryUser( &evt->data.evt_gatt_server_attribute_value.value, &MYdevice);   // device is at factory
                    if(auth != 0)
                      auth = AuthenticateInstallationUser(&evt->data.evt_gatt_server_attribute_value.value, &MYdevice);  // maybe we want to register?
                  }

                  break;

              default:
                  auth = AuthenticateFactoryUser( &evt->data.evt_gatt_server_attribute_value.value, &MYdevice);   // device is at factory
                  break;

          }
#ifdef GEN2
          if((auth==0) && (MYdevice.authSTATUS != 5)){
              //device passed the Authenticate function

              settings_parse();
//              if((lora_parse[6]==0)&&(MYdevice.registration==1))
//              if(MYdevice.registration>0)
              if(MYdevice.registration==1)
                lora_settings_parse();
              else if ((MYdevice.sku != 0xFF)&&(MYdevice.lora.lora_alive_flag == 0))
                {
//                  lora_api_internal_join__restart();
                  unreg_lora();
                }
              save_settings();
              //added the following 2 functions to reset TMR3 to sync with advertising rate
              advertise_stop();
//              if(MYdevice.lora.rejoin)
//                lora_api_internal_join__restart();
              sl_bt_system_set_lazy_soft_timer(0,0,TIMER3_HANDLE,0); // turn off timer
//              swSetupAdvBeaconing();  //added 2021-07-08 - JP
              sl_bt_system_set_lazy_soft_timer(49152,4915,TIMER3_HANDLE,0); //added 2021-07-08 - JP

          }

#endif
//          advertise_stop(); //removed 2021-07-08 - JP
//          swSetupAdvBeaconing();
          }
          else
            MYdevice.authSTATUS = 1;

      }
      else if(gattdb_current_unix_time == evt->data.evt_gatt_server_attribute_value.attribute)
        {

//          char_send('C');
          uint8_t i = 0;
          uint32_t buff64[8] = {0};

//          length = evt->data.evt_gatt_server_attribute_value.value.len;

          i = 0;
          while (i < 8)
          {
            buff64[i] = evt->data.evt_gatt_server_attribute_value.value.data[i];
            last_set_time.time[i] = evt->data.evt_gatt_server_attribute_value.value.data[i];
            i++;
          }
          myUNIXtime = 0;
          myUNIXtime |= (uint32_t)(buff64[3] << 24);
          myUNIXtime |= (uint32_t)(buff64[2] << 16);
          myUNIXtime |= (uint32_t)(buff64[1] << 8);
          myUNIXtime |= buff64[0];
          calcDate(&time_i);
          data_tmp[0] = (uint8_t)(myUNIXtime >> 24);
          data_tmp[1] = (uint8_t)((myUNIXtime & 0x00FF0000)>> 16);
          data_tmp[2] = (uint8_t)((myUNIXtime & 0x0000FF00)>> 8);
          data_tmp[3] = (uint8_t)myUNIXtime;
          ps_save(UNIX_TIME, data_tmp, 4);
//          myUNIXtime = 0;

      }
      else if(gattdb_site_key == evt->data.evt_gatt_server_attribute_value.attribute)
      {
          uint8_t registration = 0xFF;
          uint8_t site_write[16]={0};
          uint8_t i;
          ps_load(DEVICE_REG_PS, &registration, 1);    // if device is not registered save the site key
//          if(registration != 1) ps_save(DEVICE_SITEKEY_PS, evt->data.evt_gatt_server_attribute_value.value.data, 16);
//          char_send('A',0);
//          char_send(MYdevice.registration,1);
          if(MYdevice.registration != 1)
            {
//              char_send('B',0);
              for(i=0; i<16; i++)
                {
                  site_write[i] = evt->data.evt_gatt_server_attribute_value.value.data[i];
                }
//              ps_save(DEVICE_SITEKEY_PS, evt->data.evt_gatt_server_attribute_value.value.data, 16);
              ps_save(DEVICE_SITEKEY_PS, site_write, 16);
            }
      }
      else if(gattdb_signature_clear == evt->data.evt_gatt_server_attribute_value.attribute)
      {

      }
      else if(gattdb_test_mode == evt->data.evt_gatt_server_attribute_value.attribute)
      {

      }
      else if(gattdb_advertising_period == evt->data.evt_gatt_server_attribute_value.attribute)
      {

          if(evt->data.evt_gatt_server_attribute_value.value.len == 1 && evt->data.evt_gatt_server_attribute_value.value.data[0] < 36 )
          {
              MYdevice.advertising_period_provisional = evt->data.evt_gatt_server_attribute_value.value.data[0] * 160;  // provisional advertising interval subject to security verification
              MTsystem.advertiserate = evt->data.evt_gatt_server_attribute_value.value.data[0] * 160;  // provisional advertising interval subject to security verification
              switch(evt->data.evt_gatt_server_attribute_value.value.data[0])
              {
                case 0:
                  MYdevice.advertising_period = MS_750;
                  break;
                case 1:
                  MYdevice.advertising_period = MS_1000;
                  break;
                case 2:
                  MYdevice.advertising_period = MS_1500;
                  break;
                case 3:
                  MYdevice.advertising_period = MS_100;
                  break;
                default:
                  break;
              }
          }
      }
      else if(gattdb_device_type_write == evt->data.evt_gatt_server_attribute_value.attribute)
      {

      }
//      else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
//          /* Set flag to enter to OTA mode */
//    //TBD to be discuss with alejandro, it was set in Bell_3
//          //boot_to_dfu = 1;
//          /* Send response to Write Request */
////          sl_bt_gatt_server_send_user_write_response(
////          evt->data.evt_gatt_server_user_write_request.connection,
////          gattdb_ota_control, bg_err_success);
//          sl_bt_gatt_server_send_user_write_response(
//          evt->data.evt_gatt_server_user_write_request.connection,
//          gattdb_ota_control, 0);
//              /* Close connection to enter to DFU OTA mode */
////          sl_bt_cmd_endpoint_close(evt->data.evt_gatt_server_user_write_request.connection);
//         sl_bt_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
//      }
      break;

    case sl_bt_evt_gatt_mtu_exchanged_id:
//      char_send('D');
       mtuSize = evt->data.evt_gatt_mtu_exchanged.mtu;
      break;

      case sl_bt_evt_connection_opened_id:
//        char_send('E');
//        sl_bt_connection_set_parameters(evt->data.evt_connection_opened.connection, 160, 160, 4, 450, 0, 0xFFFF);
        sl_bt_connection_set_parameters(evt->data.evt_connection_opened.connection, 250, 250, 10, 300, 0, 0xFFFF);
        connectionState = 1;
        MYdevice.lora.ping_result = 0;
        MYdevice.lora.lora_alive_status = 0;
        advertise_stop();

        break;

      case sl_bt_evt_connection_closed_id:
//        char_send('F');
        // Restart advertising after client has disconnected.

        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          sl_bt_system_reset(2);
        }
        connectionState = 0;
        MYdevice.authSTATUS = 0;  //authentication is idle
        MYdevice.lora.ping_result = 0; //ping status set to IDLE
        MYdevice.lora.lora_alive_status = 0;
        if(MYdevice.sku==0xFF)
          {
            lora_api_internal_join__restart();
            MYdevice.lora.join_status = 0;
//            char_send('G',0);
          }
        swSetupAdvBeaconing();
        sl_bt_system_set_lazy_soft_timer(0,0,TIMER1_HANDLE,0);

        break;
      case sl_bt_evt_gatt_server_characteristic_status_id:
//              char_send('G');
              /* Check that the characteristic in question is signature - its ID is defined
               * in gatt.xml as "gattdb_signature ". Also check that status_flags = 1, meaning that
               * the characteristic client configuration was changed (notifications or indications
               * enabled or disabled). */
              if ((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_sig_Indication )
                  && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01)) {
                  if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_indication) {
                  /* Indications have been turned ON - start */
                   indicationscnt = 0;
                   if(MYdevice.sigSTS==1)
//                     sl_bt_gatt_server_notify_all(gattdb_sig_Indication, 7, (FindSignature(indicationscnt)+1));
                     sl_bt_gatt_server_send_indication(evt->data.evt_gatt_server_characteristic_status.connection, gattdb_sig_Indication, 7, (FindSignature(indicationscnt)+1));
                   else
//                     sl_bt_gatt_server_notify_all(gattdb_sig_Indication, (*FindSignature(indicationscnt)-1), (FindSignature(indicationscnt)+1));
                     sl_bt_gatt_server_send_indication(evt->data.evt_gatt_server_characteristic_status.connection, gattdb_sig_Indication, (*FindSignature(indicationscnt)-1), (FindSignature(indicationscnt)+1));
                  }
                  else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_disable)
                    {
                       /* Indications have been turned OFF - stop  */
//                       swSetupAdvBeaconing();
                    }
              }
              else if ((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_sig_Indication )
                    && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x02))
                {
                   /* Confirmation */
                  indicationscnt++;
                  if(indicationscnt>=currentindex)
                      indicationscnt = 0;
                  else
                    {
                      if(MYdevice.sigSTS==1)
//                      sl_bt_gatt_server_notify_all(gattdb_sig_Indication, 7, (FindSignature(indicationscnt)+1));
                        sl_bt_gatt_server_send_indication(evt->data.evt_gatt_server_characteristic_status.connection, gattdb_sig_Indication, 7, (FindSignature(indicationscnt)+1));
                      else
//                      sl_bt_gatt_server_notify_all(gattdb_sig_Indication, (*FindSignature(indicationscnt)-1), (FindSignature(indicationscnt)+1));
                        sl_bt_gatt_server_send_indication(evt->data.evt_gatt_server_characteristic_status.connection, gattdb_sig_Indication, (*FindSignature(indicationscnt)-1), (FindSignature(indicationscnt)+1));
                    }
              }
              /* Check that the characteristic in question is signature - its ID is defined
               * in gatt.xml as "gattdb_signature ". Also check that status_flags = 1, meaning that
               * the characteristic client configuration was changed (notifications or indications
               * enabled or disabled). */
              else if ((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_lora_settings )
                  && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01)) {
//                  if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_indication) {
                  if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_notification) {
                  /* Indications have been turned ON - start */
//                      lora_payload();
                      ind_flag=0;
                      lora_payload();
                     sl_bt_gatt_server_send_notification(evt->data.evt_gatt_server_characteristic_status.connection, gattdb_lora_settings, 107, lora_dev_settings);
                     sl_bt_system_set_lazy_soft_timer(16384,1638,TIMER1_HANDLE,0);

                  }
                  else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_disable)
                    {
                       /* Indications have been turned OFF - stop  */
//                       swSetupAdvBeaconing();
//                      ind_flag=0;
                      sl_bt_system_set_lazy_soft_timer(0,0,TIMER1_HANDLE,0); // turn off timer
                    }
              }
              break;
//      case sl_bt_evt_gatt_server_user_write_request_id:
//        break;

    default:
      break;
  }
}
