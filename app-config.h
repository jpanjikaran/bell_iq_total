/*
 * app-config.h
 *
 *  Created on: Jun 6, 2018
 *      Author: joet
 */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#define LORA_XDOT
//#define TREX
//#define MONO
//#define CR		//define for Cost Reduction enable internal pull up feature

//#define DEVELOPMENT

//#define FACTORY         // this is for returning to a factory state

//#define OPEN

#define SAMPLEPERIOD   60    // 1.5 to 60  = 90 seconds    battery and temperature sampled every 90 seconds

#define namefirstchar 'B'
#define namesecondchar 'L'
#define DEVICE_SKU 0
// 0-Unspecified  1-Express   2- Ambush  3- 24/7  4- TREX
#define NUMBER_OF_SKU  4 //total number of possible sku
#define THRESHOLD_N    3  // default threshold
#define FILTER_TIME  1800 //14400   // express 30 minutes trapper 4 hours  ambush 30 minutes
#define SUPPRESS_TIME  (32768 * 1800)  // 30 minutes
#define TX_PWR  0
#define PRODUCTION_MODE_TIME 20  // 60
#define FACTORY_SLEEP_TIME 40   // 50
//For testing
//#define PRODUCTION_MODE_TIME 5  // 60
//#define FACTORY_SLEEP_TIME 10   // 50
#define BATTERYSAMPLES  8    //number of battery samples used
#define BATTSAMPLEPERIOD   240    //240x1.5=360 x 10 samples = 3600secods , 1 hour
//#define TEMPSAMPLEPERIOD   2400  // 1.5 to 60  = 3600 seconds  Temperature sampled every hour
#define TEMPSAMPLEPERIOD   2400  // 1.5 to 60  = 3600 seconds  Temperature sampled every hour

//FW 1.28 Feature
//#define DBGWRITE_USERPAGE_EN
#define USERPAGE_RD_EN

//FW1.29 feature
#ifndef TREX
#define SIGNATURE_EN    //defines signature encryption feature
#define FALSEEVENT      //advertise false event when buffer is full
#define XTTHRESHLD      //enables x threshold = MYdevice.detection_threshold filtering
#endif
#define BATTERYSAMPLES  8    //number of battery samples used
#endif /* APP_CONFIG_H_ */

//debug
#define DISPGHALL        //disable power gating in hall sensor
#define ENPULLIO2       //enables pullup on io2 when polling

