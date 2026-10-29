/*
 * collection.h
 *
 *  Created on: Jan. 8, 2021
 *      Author: joet
 */

#ifndef COLLECTION_H_
#define COLLECTION_H_

#include "em_common.h"
#include "sl_bt_api.h"
#include "thandles.h"
#include "app.h"
#include "utility.h"

#define NULL ((void *)0)
#define MAX_CONTIG_SIZE 8192

#define NEWALGO

// N * 2 => N - number of pulses to remove

#define DECIMATE_MOVEMENT_PULSES (2*2)
#define movscale 128 // scaling factors
#define actscale 128
//#define  XTTHRESHLD

#define EDGE_FALLING 0
#define EDGE_RISING  1

//7 byte offset of signature payload, app will read 1st byte to 7 byte when advertising
#define SIG_OFFSET   7
//4 byte key for signature encryption
#define KEY_SZE      4

//#define MAX_BUF_SIZE 246   // VB, this was replaced by two new defines, see below
// they must be even numbers
#define MAX_MBUF_SIZE (150*2)
#define MAX_ABUF_SIZE (25*2)

#define MAX_CONTIG_SIZE 8192
//#define MAX_CONTIG_SIZE 10

// 1- 1 second 2- 2 seconds
#define COLLECTION_TIMEOUT (32768*2)
#define SHORT_COLLECTION_TIMEOUT (32768 / 10)

#define IDLE_TIMEOUT (32768*5)

typedef enum {

  idle = 0,
  active,
  error

} actmoveStates;


typedef enum {

  move_leading = 0, // movement occurred before activation
  move_trailing     // movement occurred after activation

} StartCondition;


//#define MAX_BUF_SIZE 246   // VB, this was replaced by two new defines, see below
// they must be even numbers
#define MAX_MBUF_SIZE (150*2)
#define MAX_ABUF_SIZE (25*2)

#define MAX_CONTIG_SIZE 8192
//#define MAX_CONTIG_SIZE 512

// 1- 1 second 2- 2 seconds
#define COLLECTION_TIMEOUT (32768*2)
#define SHORT_COLLECTION_TIMEOUT (32768 / 10)

#define IDLE_TIMEOUT (32768*5)

// uncomment the next line if you want to stop collecting data for IDLE_TIMEOUT seconds after the current collection finished
// #define DO_IDLE

typedef struct {

    uint32_t actBuff[MAX_ABUF_SIZE];
    uint32_t movBuff[MAX_MBUF_SIZE];

    uint32_t actN, movN; // number of pulses

    uint32_t actInx;  // indexes
    uint32_t movInx;
    bool active; // interrupt active or not
    bool act_coll_stop;
    bool wating_for_last_mov;
    actmoveStates State;
    StartCondition Start;
    uint32_t savedMtick;
    uint32_t savedAtick;
    uint8_t  Filter;
    uint32_t  FilterTime;
    uint32_t timestamp;
    uint8_t  EventsCounter;
    uint16_t DetectionsCounter;
    uint32_t EventsSinceReset;

} collectionType;

// 246
#define MAX_EVENT_LEN (185-3)
#define MAX_ACT_FULL_PULSES 10



typedef struct {

  uint8_t length;
  uint8_t event[MAX_EVENT_LEN];
}eventtype;

#define MAX_EVENTS 1

typedef struct {

  eventtype Events[MAX_EVENTS];
  uint8_t EventIndex;
  uint8_t Load;
}EventsType;


void prepare_to_stop(void);
void SignalMovement(int edge);
void SignalActivation(int edge);
void SignalTOT(void);
void collectionInit(void);
void EventIndexRequest(uint8array *writeValue);
uint8_t AddSignature(EventsType *workingbuffer, uint8_t i);
uint8_t* FindSignature(uint16_t index);
void SessionKeyRequest(uint8array *writeValue);

#endif /* COLLECTION_H_ */
