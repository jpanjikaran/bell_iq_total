/*
 * swmodbeacon.h
 *
 *  Created on: Jan 26, 2018
 *      Author: joet
 */

#ifndef SWMODBEACON_H_
#define SWMODBEACON_H_

//#include "init_mcu.h"
//#include "init_board.h"
//#include "init_app.h"
//#include "ble-configuration.h"
//#include "board_features.h"
#include "em_adc.h"
#include "em_usart.h"

/* BG stack headers */
//#include "bg_types.h"
//#include "native_gecko.h"
//#include "gatt_db.h"
//#include "infrastructure.h"

/* libraries containing default gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"





/* Device initialization header */
//#include "hal-config.h"

typedef struct{

	uint8_t flagsLen;     /* Length of the Flags field. */
	uint8_t flagsType;    /* Type of the Flags field. */
	uint8_t flags;        /* Flags field. */
	uint8_t namelen;    //3
	uint8_t nametype;   //9
	uint8_t localname0;  // B
	uint8_t localname1; // L
	uint8_t mandataLen;   /* Length of the Manufacturer Data field. */
	uint8_t mandataType;  /* Type of the Manufacturer Data field. */
	uint8_t compId[2];    /* Company ID field. */
	//uint8_t beacType[2];  /* Beacon Type field. */
	uint8_t commscheme;   /* Communication Schema */
	uint8_t devtype;	  /* Type of device */
	uint8_t devMACadd[6]; /* MAC address */
	uint8_t txPwr;        /* Transmit power */
	uint8_t RRSSI;        /* Reference RSSI (calibrated) */
	uint8_t tempRead;     /* temperature */
	uint8_t battLvl;      /* battery level */
	uint8_t status;       /* status bits*/           // this is number of events
	uint8_t numEvents[2]; /* number of events */  // this is number of detections
	//uint8_t minsSinEvt[2]; /* 15 mins increments since last event*/
	uint8_t LocalTime[4];

}smartwavebcn;

typedef struct{

    uint8_t flagsLen;     /* Length of the Flags field. */
    uint8_t flagsType;    /* Type of the Flags field. */
    uint8_t flags;        /* Flags field. */
    uint8_t mandataLen;   /* Length of the Manufacturer Data field. */
    uint8_t mandataType;  /* Type of the Manufacturer Data field. */
    uint8_t compId[2];    /* Company ID field. */
    uint8_t beacType[2];  /* Beacon Type field. */
    uint8_t uuid[16];     /* 128-bit Universally Unique Identifier (UUID). The UUID is an identifier for the company using the beacon*/
    uint8_t majNum[2];    /* Beacon major number. Used to group related beacons. */
    uint8_t minNum[2];    /* Beacon minor number. Used to specify individual beacons within a group.*/
    uint8_t txPower;      /* The Beacon's measured RSSI at 1 meter distance in dBm. See the iBeacon specification for measurement guidelines. */
  }smartwaveibcn;

  typedef enum{

      disActive = 0,
      NoActivity,
      LowActivity,
      HighActivity
  }ActivityStates;


  typedef struct{

    uint32_t EventTime[20];
    uint8_t pntr;
    uint8_t cntr;
    ActivityStates statusflag;

  }ActivityType;

  typedef struct{

      uint32_t ActPeriod;
      uint32_t ActEvents;

  }ActivityRule;






/* prototypes */

void setupSensor(void);
float ConvertToCelsius(int32_t adcSample);
void swSetupAdvBeaconing(void);
uint32_t BattMeasure(void);
void TempMeasure(void);
void HallMeasure(void);
void AddActivity(ActivityType *Activity, uint32_t time);
void enterStandby(void);
void exitStandby(void);

#endif /* SWMODBEACON_H_ */
