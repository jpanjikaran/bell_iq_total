/*
 * collection.c
 *
 *  Created on: Jan. 8, 2021
 *      Author: joet
 */


#include "collection.h"
#include "app.h"
#include "utility.h"


/*******************************************************************************
 ***************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/


collectionType coll;
EventsType MyEvents;

//Variable declarations
uint8_t conbuffer[MAX_CONTIG_SIZE];
uint16_t currentindex;  // keep track of how many entries we have
uint8_t *conpoint = conbuffer;
uint16_t EventRequested;
uint8_t sessionkeyarray[36] = {0};
uint8_t     SuppressDetect = 0;
uint8_t      CollMemoryFULL;

int can_add_cnt = 0;
int cannot_save_cnt = 0;

//External declarations
extern MTsystem_t MTsystem;
extern deviceDATA MYdevice;
extern uint8_t      CollMemoryFULL;
//extern uint8_t            EventsCounter;
extern uint32_t    EventsSinceReset;
extern void adv_set_data(void);
extern bool send_lora_pkt;
extern uint8_t first_event_flag;
//uint16_t first_event_cnt;
//uint8_t first_event_tx;


static const uint8_t xorKEY[4] =
{
  0x41,0x32,0x23,0x14
};

void collectionInit(void)
{
    int i;

    // clear all the counters and flags
    coll.actInx = 0;
    coll.movInx = 0;
    coll.State = idle;
    coll.movN = 0;
    coll.actN = 0;

    for (i=0; i<MAX_ABUF_SIZE; i++)
    {
      coll.actBuff[i] = 0;
    }
    for (i=0; i<MAX_MBUF_SIZE; i++)
    {
       coll.movBuff[i] = 0;
    }

    coll.act_coll_stop = false;
    coll.wating_for_last_mov = false;

#ifdef DO_IDLE
     // wait before collecting a new signature
    gecko_cmd_hardware_set_soft_timer(IDLE_TIMEOUT,TIMER4_HANDLE,1); // reset idle time
#else
    coll.active = true;
#endif

}

//static void prepare_to_stop(void)
//{
//   coll.wating_for_last_mov = true;
//}

uint32_t getLocalTime(void)
{
    return MTsystem.LocalTimeCounter;
}

/*********************************************
 *  index - requested signature
 *  return - pointer to requested signature
 *********************************************/
uint8_t* FindSignature(uint16_t index)
{
    uint8_t *p;
    uint16_t i = 0;

    p = conbuffer;
    if(index >= currentindex) return NULL;
    while (i != index ){

        p += *p;  // add length to pointer to advance through buffer to next entry
        i++;     // count how many records we have gone through
    }
    if( p > (conbuffer + (MAX_CONTIG_SIZE- 1))) return NULL;
    return p;
}

uint8_t AddSignature(EventsType *workingbuffer, uint8_t i)
{
    uint8_t l;
    l = workingbuffer->Events[i].length; //  length
    uint8_t  j = 0;
    volatile uint16_t space;

    space = (conbuffer + (MAX_CONTIG_SIZE)) - conpoint;

    //if(space < l+1) return 1;
    if(space < 256) {
      CollMemoryFULL = 1;
      return 1;  // only save if we have maximum space available
    }

    *conpoint = workingbuffer->Events[i].length + 1;  //first byte is length
    conpoint++;

    while(l > 0){

        *conpoint = workingbuffer->Events[i].event[j];  // add byte
        l--;
        j++;
        conpoint++;

    }
    return 0;
}

void EventIndexRequest(uint8array *writeValue)
{
    uint16_t event = 0;

    event = (uint16_t)writeValue->data[0];   // Event requested
    event =  (uint16_t)(event |( writeValue->data[1] << 8)) ;
    EventRequested = event;   // todo    this should be returned from function instead of global variable
}

void SessionKeyRequest(uint8array *writeValue)
{
    uint8_t i = 0;

    while(i < 36)
    {
        sessionkeyarray[i] = writeValue->data[i];
        i++;
    }
}

uint32_t get_current_tick(void) //get the current time
{
     volatile uint32_t ct;

     ct = RTCC->CNT;
     return ct;
}

bool saveMovementTick(uint32_t tick, int edge) // returns true if tick is saved, false - no more room
 {
     bool can_save =  false;

     if (edge == EDGE_FALLING)   // makes sure we have room for 1 entry
     {
         if (coll.movInx <= (MAX_MBUF_SIZE - 1))  can_save = true;
     }
     else // rising, only need room for 2 entries
     {
         if (coll.movInx < (MAX_MBUF_SIZE - 1))  can_save = true;
     }

     if (can_save)
     {
         if(coll.savedMtick < tick)
         {
            coll.movBuff[coll.movInx] = (tick - coll.savedMtick) / movscale;
         }
         else if (coll.savedMtick ==  tick) // this should not happen but it is better to handle it
         {
             coll.movBuff[coll.movInx] = 1;
         }
         else
         {
             coll.movBuff[coll.movInx] = ((0xFFFFFFFF - coll.savedMtick) + tick)/ movscale;
         }
         if(coll.movBuff[coll.movInx] == 0) coll.movBuff[coll.movInx] = 1; // 0 has no meaning so make it 1
         coll.savedMtick = tick; // save for next
         coll.movInx++;
     }

     if (can_save == false)
         cannot_save_cnt++;

     return can_save;
 }


/*           SignalMovement()                                       */
/* An interrupt on Movement pin will be processed to extract timing */
void SignalMovement(int edge)
{
     uint32_t current_tick ;

     //if(SuppressDetect)return;   // suppress any detections after device is cleared

#ifdef DECIMATE_MOVEMENT_PULSES
     static int ignore_cnt;
     static bool ignore_flg = false;
#endif

     // ignore the interrupts if not active
     // coll.active is false during very short period of time, only during execution of SignalTOT() function
     if (coll.active)
     {
#ifdef DECIMATE_MOVEMENT_PULSES
        if (ignore_flg == true)  // decimation is active when ignore_flasg is true
        {
        if (coll.State == active)
        {
           ignore_cnt--;      // decimation counter, we ignore edges, so the counter must be initialized with even number
           if (ignore_cnt == 0)
              ignore_flg = false;
           return; // we ignore this edge as part of the decimation process
        }
        else
        {
              ignore_flg = false; // collection is not active, stop decimation process and use this edge
        }
        }

#endif
        {
              current_tick =  get_current_tick();
              if (edge == EDGE_FALLING) //  falling edge
              {
                   if(coll.State == idle) // if idle movement is leading
                   {
                        coll.State = active;   // going from idle to active on the falling edge
                        coll.Start = move_leading;
                        coll.savedMtick = current_tick; // save for first time
                        coll.savedAtick = current_tick; // save for first time   now we are all in sync
                        coll.timestamp = current_tick;  // we need first tick as a reference for the next
                        //   (total_ticks_saved + 1)/2 = number_of_movements
                   //gecko_cmd_hardware_set_soft_timer(COLLECTION_TIMEOUT,TIMER2_HANDLE,1); // rearm time out timer
                   sl_bt_system_set_soft_timer( COLLECTION_TIMEOUT, TIMER2_HANDLE, 1);  // rearm time out timer
                   }
                   else
                   {   // collection is active already, so this falling edge is not the first one, save the tick
                        if (saveMovementTick(current_tick, EDGE_FALLING) == true) // we do not increment movN here, we do it on the rising edge
                           //gecko_cmd_hardware_set_soft_timer(COLLECTION_TIMEOUT,TIMER2_HANDLE,1); // rearm time out timer
                            sl_bt_system_set_soft_timer( COLLECTION_TIMEOUT, TIMER2_HANDLE, 1); // rearm time out timer
                   }

              }
              else // rising edge
              {
                   // so, ignore the rising edge if we are idle
                   if (coll.State != idle)
                   {
#ifdef DECIMATE_MOVEMENT_PULSES
                        ignore_flg = true;
                        ignore_cnt = DECIMATE_MOVEMENT_PULSES;
#endif
                        if (saveMovementTick(current_tick, EDGE_RISING) == true)
                        {
                             coll.movN++; // vb we need to capture a pair of edges to claim new movement pulse
                             if (coll.wating_for_last_mov == false)
                                //gecko_cmd_hardware_set_soft_timer(COLLECTION_TIMEOUT,TIMER2_HANDLE,1); // rearm time out timer
                               sl_bt_system_set_soft_timer( COLLECTION_TIMEOUT, TIMER2_HANDLE, 1); // rearm time out timer
                        }
                   }
              }
          }
     }
}

/*           SignalMovement()                                       */
/* An interrupt on Movement pin will be processed to extract timing */
void SignalMovementold(int edge)
{
    uint32_t current_tick ;

    //if(SuppressDetect)return;   // suppress any detections after device is cleared

#ifdef DECIMATE_MOVEMENT_PULSES
    static int ignore_cnt;
    static bool ignore_flg = false;
#endif

    if (coll.active)
    {
#ifdef DECIMATE_MOVEMENT_PULSES
        if (ignore_flg == true)
        {
            ignore_cnt--;
            if (ignore_cnt == 0)
                ignore_flg = false;
        }
        else
#endif
        {
            current_tick =  get_current_tick();
            if (edge == EDGE_FALLING) //  falling edge
            {
                if(coll.State == idle) // if idle movement is leading
                {
                    coll.State = active;
                    coll.Start = move_leading;
                    coll.savedMtick = current_tick; // save for first time
                    coll.savedAtick = current_tick; // save for first time   now we are all in sync
                    coll.timestamp = current_tick;  // we need first tick as a reference for the next
                    //   (total_ticks_saved + 1)/2 = number_of_movements
                    //gecko_cmd_hardware_set_soft_timer(COLLECTION_TIMEOUT,TIMER2_HANDLE,1); // rearm time out timer
                    sl_bt_system_set_soft_timer( COLLECTION_TIMEOUT, TIMER2_HANDLE, 1); // rearm time out timer
                }
                else
                {
                    if (saveMovementTick(current_tick, EDGE_FALLING) == true) // we do not increment movN here, we do it on the rising edge
                        //gecko_cmd_hardware_set_soft_timer(COLLECTION_TIMEOUT,TIMER2_HANDLE,1); // rearm time out timer
                      sl_bt_system_set_soft_timer( COLLECTION_TIMEOUT, TIMER2_HANDLE, 1); // rearm time out timer
                }

            }
            else // rising edge
            {
                // so, ignore the rising edge if we are idle
                if (coll.State != idle)
                {
#ifdef DECIMATE_MOVEMENT_PULSES
                    ignore_flg = true;
                    ignore_cnt = DECIMATE_MOVEMENT_PULSES;
#endif
                    if (saveMovementTick(current_tick, EDGE_RISING) == true)
                    {
                         coll.movN++;   // vb we need to capture a pair of edges to claim new movement pulse
                         if (coll.wating_for_last_mov == false)
                            //gecko_cmd_hardware_set_soft_timer(COLLECTION_TIMEOUT,TIMER2_HANDLE,1); // rearm time out timer
                           sl_bt_system_set_soft_timer( COLLECTION_TIMEOUT, TIMER2_HANDLE, 1); // rearm time out timer
                    }
                }
            }
        }
    }
}

bool saveActivationTick(uint32_t tick, int edge) // return true if tick was saved, false - no room in the buffer
{
     bool can_save =  false;


     if (edge == EDGE_FALLING)   // makes sure we have room for 2 entries
     {
         if (coll.actInx < (MAX_ABUF_SIZE - 1))  can_save = true;
     }
     else // rising, only need room for one entry
     {
         if (coll.actInx <= (MAX_ABUF_SIZE - 1))  can_save = true;
     }

     if (can_save)
     {
        if(coll.savedAtick < tick)    // due to long integer arithmetic  ">" here was not a bug!!! Amazing!!! still changed to the correct comparison VB
        {
            coll.actBuff[coll.actInx] = (tick - coll.savedAtick) / actscale;
        }
        else if (coll.savedAtick == tick) // this should not happen but it is better to handle it
        {
            coll.actBuff[coll.actInx] = 1;
        }
        else
        {
            coll.actBuff[coll.actInx] = ((0xFFFFFFFF - coll.savedAtick) + tick) / actscale;
        }

        if(coll.actBuff[coll.actInx] == 0) coll.actBuff[coll.actInx] = 1; // 0 has no meaning so make it 1

        coll.savedAtick = tick; // save for start of the pause period
        coll.actInx++;
     }
     return can_save;
}

void SignalActivation(int edge){
    uint32_t current_tick;

    if(SuppressDetect)return;
    if (coll.act_coll_stop) // we collected maximum activation pulses
        return;

    if (coll.active) // ignore the interrupts if not active
    {
        current_tick =  get_current_tick();

        if (edge == EDGE_RISING)// rising edge
        {
            if (coll.State != idle) // ignore the rising edge interrupt if idle.
            {
               saveActivationTick(current_tick, EDGE_RISING);
               //gecko_cmd_hardware_set_soft_timer(COLLECTION_TIMEOUT,TIMER2_HANDLE,1); // reset time out timer
               sl_bt_system_set_soft_timer( COLLECTION_TIMEOUT, TIMER2_HANDLE, 1); // reset time out timer
               if (coll.actN == MAX_ACT_FULL_PULSES)
               {
                   coll.act_coll_stop = true;  // we have collected maximum ac pulses
                   return;
               }
            }
            // we are not resetting the timer if edge was ignored
        }
        else // falling edge
        {
            if(coll.State == idle) //if idle movement is trailing, because activation interrupt came first
            {
                coll.actN = 1;               // as long as we have an edge of activation, we can claim one activation
                coll.Start = move_trailing;
                coll.State = active;
                coll.savedAtick = current_tick; // save for first time
                coll.savedMtick = current_tick; // save for first time now we are all in sync
                coll.timestamp = current_tick;
            }
            else
            {   // we already collected either some movement or activation pulses
                if (coll.actN < MAX_ACT_FULL_PULSES)
                {
                  if (saveActivationTick(current_tick, EDGE_FALLING))
                      coll.actN++; // increment if it was saved
                }
            }
            //gecko_cmd_hardware_set_soft_timer(COLLECTION_TIMEOUT,TIMER2_HANDLE,1); // reset time out timer
            sl_bt_system_set_soft_timer( COLLECTION_TIMEOUT, TIMER2_HANDLE, 1); // reset time out timer
        }
    }
}

void addTempTimestamp(int inx)
{
     int l = 0;

    #ifdef NEWALGO
            MyEvents.Events[inx].event[l++] = MTsystem.temperaturecurrent;  // todo  change from int to uint when sampled
            MyEvents.Events[inx].event[l++] = (uint8_t)(coll.timestamp & 0x000000FF);
            MyEvents.Events[inx].event[l++] = (uint8_t)((coll.timestamp & 0x0000FF00) >> 8);
            MyEvents.Events[inx].event[l++] = (uint8_t)((coll.timestamp & 0x00FF0000) >> 16);
            MyEvents.Events[inx].event[l++] = (uint8_t)((coll.timestamp & 0xFF000000) >> 24);
            //AddActivity(&DeviceActivity, getLocalTime());  // Activity...show in advertising package in status byte
    #endif
    MyEvents.Events[inx].length = l;
}

/*
 * 0 ... 254 - range for activation
 * 0 ... 126 - range for movements
 * 255 - flag that the next 2 bytes represent the long value of activation
 * 127 - flag that the next 2 bytes represent the long value of movement
 *
 */
bool addMovToEvent(int inx, uint32_t v) // true if data v saved, false - no room
{
    int l;
    uint8_t d, r;
    bool can_add = false;

    l = MyEvents.Events[inx].length;

    if (v > 126)
    {
        if(l < (MAX_EVENT_LEN - 1 - 2)) // leave room for 2 extra bytes and the terminator
        {
            can_add = true;
            d = v / 256; // note: we do not handle if d> 255, shame
            r = v % 256;
            MyEvents.Events[inx].event[l++] = 127; //  the next 2 bytes - the long movement period
            MyEvents.Events[inx].event[l++] = d;
            MyEvents.Events[inx].event[l++] = r;
        }
    }
    else
    {
        if(l < (MAX_EVENT_LEN - 1)) // leave room for the terminator
        {
            can_add = true;
            MyEvents.Events[inx].event[l++] = v;
        }
    }

    if (can_add ==  false)
        can_add_cnt++;

    MyEvents.Events[inx].length = l;
    return can_add;
}

// adding only one byte to the buffer, either number of activations or movements
void addAMNToEvent(int inx, uint32_t v)
{
    int l;
    l = MyEvents.Events[inx].length;
    MyEvents.Events[inx].event[l++] = (uint8_t)v;
    MyEvents.Events[inx].length = l;
}

bool addActToEvent(int inx, uint32_t v) // true if data v saved, false - no room
{
    int l;
    uint8_t d, r;
    bool can_add = false;


    l = MyEvents.Events[inx].length;
    if (v > 254)
    {
        if(l < (MAX_EVENT_LEN - 1 - 2))
        {
            can_add = true;
            d = v / 256; // note: we do not handle if d> 255, shame
            r = v % 256;
            MyEvents.Events[inx].event[l++] = 255; // the next 2 bytes - the long negative activation pulse
            MyEvents.Events[inx].event[l++] = d;
            MyEvents.Events[inx].event[l++] = r;
        }
    }
    else
    {
        if(l < (MAX_EVENT_LEN - 1))
        {
            can_add = true;
            MyEvents.Events[inx].event[l++] = v;
        }
    }
    MyEvents.Events[inx].length = l;

    if (can_add ==  false)
        can_add_cnt++;

    return can_add;
}

/*
 * long "negative movement pulses"
 * 0... 126 -> 128... 254
 * or
 * 255 d r
 *
 *
 */
bool addLongMovToEvent(int inx, uint32_t v, uint32_t vp) // neg + period
{
   int l, room;
   uint8_t d, r;
   bool can_add = false;

   l = MyEvents.Events[inx].length;

   // lets figure out how much room we need
   room = 0;
   if (v < 127) room++; else room = room + 3;
   if (vp > 126) room = room + 3; else room++;

   if (l < (MAX_EVENT_LEN - room))
   {
       can_add = true;
       // negative part first
       if (v < 127 )
          MyEvents.Events[inx].event[l++] = (uint8_t)v + 128; // the reader will treat number > 127 as long negative movement pulse
       else
       {
           d = v / 256;  // note we are not handling if d > 255, shame
           r = v % 256;
           MyEvents.Events[inx].event[l++] = 255; // the next 2 bytes - the long negative movement pulse, note 255 and 127 from the addMovActToEvent function
           MyEvents.Events[inx].event[l++] = d;
           MyEvents.Events[inx].event[l++] = r;
       }
       // period now
       if (vp > 126)
       {
            d = vp / 256; // note: we do not handle if d> 255, shame
            r = vp % 256;
            MyEvents.Events[inx].event[l++] = 127; //  the next 2 bytes - the long movement period
            MyEvents.Events[inx].event[l++] = d;
            MyEvents.Events[inx].event[l++] = r;
        }
        else
            MyEvents.Events[inx].event[l++] = (uint8_t)vp;
   }
   else
   {
       MyEvents.Events[inx].length = l;
   }
   MyEvents.Events[inx].length = l;

    if (can_add ==  false) // debugging
        can_add_cnt++;

   return can_add;
}

// what happens if timer expires when the activation pulse is still 0?

void SignalTOT(void) // Time out function
{

    uint32_t mt;
    //uint16_t fillbuffer = 0;

    static int iA, aInd, mInd;
    static bool room_avail;
    static int iM;

    ps_load(DEVICE_CONFIGURATION_PS,&MYdevice.detection_threshold, 1);
    if(MYdevice.detection_threshold == 0)
      {
        MYdevice.detection_threshold = 3;
        ps_save(DEVICE_CONFIGURATION_PS,&MYdevice.detection_threshold, 1);
      }

    if((MYdevice.sku & 0x0F) != 4)  // if not a T-REX
    //if(MYdevice.sku != 4)  // if not a T-REX
    {
        if(coll.State == idle) return;  // avoid doint it second time
        coll.active = false;
    }

    MyEvents.EventIndex = 0;
    MyEvents.Events[MyEvents.EventIndex].length = 0;
    coll.timestamp  =  getLocalTime();

    addTempTimestamp(MyEvents.EventIndex);
    //addAMNToEvent(MyEvents.EventIndex, coll.actN); // number of trigger pulses   for ECOLAB
    //addAMNToEvent(MyEvents.EventIndex, coll.movN); // number of movement pulses  for ECOLAB
#ifndef NOTECOLAB
    if((MYdevice.sku & 0x0F) != 4)  // if not a T-REX
    {
        // let's handle the activation pulses, simple bytes, do not use coded saving
        addAMNToEvent(MyEvents.EventIndex, coll.actN); // number of trigger pulses
        addAMNToEvent(MyEvents.EventIndex, coll.movN); // number of movement pulses
    }
#endif


    if((MYdevice.sku & 0x0F) != 4)  // if not a T-REX
    {
        // let's handle the activation pulses, simple bytes, do not use coded saving
        addAMNToEvent(MyEvents.EventIndex, coll.actN); // number of trigger pulses
        addAMNToEvent(MyEvents.EventIndex, coll.movN); // number of movement pulses


        if (coll.actN > 0) // only if we have trigger pulses
        {
            if (coll.Start == move_trailing)
            {
                addActToEvent(MyEvents.EventIndex, 0); // delay from first movement pulse is 0 if activation pulse starts first
                // if buffer is 0 it means that the rising edge of the pulse did not arrive
                addActToEvent(MyEvents.EventIndex, coll.actBuff[0]); // width of trigger pulse
                iA = 0;
            }
            else
            {
                addActToEvent(MyEvents.EventIndex, coll.actBuff[0]);  // delay from first movement pulse
                // if buffer is 0 it means that the rising edge of the pulse did not arrive
                addActToEvent(MyEvents.EventIndex, coll.actBuff[1]); // width of trigger pulse
                iA = 1;
            }
            aInd = 1;

            while (aInd < coll.actN) // note we are not handling buffer overflow as a pair, shame
            {
                addActToEvent(MyEvents.EventIndex, coll.actBuff[iA + 1]); // delay between act pulses
                iA++;
                // if buffer is 0 it means that the rising edge of the pulse did not arrive
                addActToEvent(MyEvents.EventIndex, coll.actBuff[iA + 1]); // width of activation pulse
                iA++;
                aInd++;
            }
        }

        // let's handle the movement pulses

        if (coll.movN > 0 ) // any movement?
        {
            if(coll.Start == move_leading)  // delay from first movement pulse  0 if movement pulse first
            {
                addMovToEvent(MyEvents.EventIndex, 0);   // movement leading, 0 delay
                iM = 0;
            }
            else
            {
                addMovToEvent(MyEvents.EventIndex, coll.movBuff[0]);  // delay from first activation pulse
                iM = 1;
            }
            mInd = 1; room_avail = true;
            // we do not transmit the last collected timing but in the viewer we pretend that we got it
            // the viewer assumes that the length of the last movement pulse is short
            // it could be a problem here if number of movement pulses is "1" - do not care to fix it
            while ((mInd < coll.movN) && room_avail)  // we only transmit "movN-1" pulses
            {
                mt = coll.movBuff[iM] + coll.movBuff[iM+1]; // neg + pos parts =  period
                if (coll.movBuff[iM] < 3) // short pulse, code it without coding "neg" long pulses
                {
                    if (!addMovToEvent(MyEvents.EventIndex, mt)) // short pulses coded with their period only
                        room_avail = false;
                }
                else // long negative movement pulses coded with the pulse duration + the period
                {
                    if (!addLongMovToEvent(MyEvents.EventIndex, coll.movBuff[iM], mt)) // Overflow is handled as a pair
                        room_avail = false;
                }
                // we do not check for the buffer here because that was done before, if we have too many points we take care of this by
                // reducing movN counter
                iM = iM + 2;  // make sure we don't exceed buffer length
                mInd++;
            }
        }
    }

    MyEvents.Events[MyEvents.EventIndex].event[MyEvents.Events[MyEvents.EventIndex].length] = 0;  // terminate with 0 -  because we transmit movN points
    // this terminator is really not needed if the viewer will only expect movN-1 points
    MyEvents.Events[MyEvents.EventIndex].length += 1; // add 1 to the length for the terminating 0

    if((MYdevice.sku & 0x0F) == 4) coll.actN = 1;

    /***************************************************************************//**
     * Signature Encryption
     *
     * Start  encrypting payload at MyEvents.Events[MyEvents.EventIndex].event[7] to MyEvents.Events[MyEvents.EventIndex].length+1
     * Encrypting process takes each byte uint8_t and xored with respective key in byte also
     * Encrypting process is a 4 byte input with a 4 byte key.
     *
     ******************************************************************************/
  #ifdef SIGNATURE_EN
      for(uint8_t i = 0; i < MyEvents.Events[MyEvents.EventIndex].length-SIG_OFFSET ; i++) {
        MyEvents.Events[MyEvents.EventIndex].event[i+SIG_OFFSET] = MyEvents.Events[MyEvents.EventIndex].event[i+SIG_OFFSET] ^ xorKEY[i % KEY_SZE];
      }

  //for test purposes, decrypt
    //for(uint8_t i = 0; i < MyEvents.Events[MyEvents.EventIndex].length-SIG_OFFSET ; i++) {
    //  MyEvents.Events[MyEvents.EventIndex].event[i+SIG_OFFSET] = MyEvents.Events[MyEvents.EventIndex].event[i+SIG_OFFSET] ^ xorKEY[i % KEY_SZE];
  //}
  #endif

  //set event if the Coll Memory is Full to notify the user to clear and download sig from the device.
  //In this case, this scenario will not happen since we only save signatures that generates events
    //and those events are verified if coll.movN >= MYdevice.detection_threshold
    //therefore there is unlikely chance that CollMemory is Full with EventCounter ==0
  #ifdef FALSEEVENT
      if((CollMemoryFULL) && (coll.EventsCounter == 0)) {
        coll.EventsCounter++;



      }
  #endif

  //Logic that enables collection of signatures into fw buffers/mem
  //movement must be >= configured threshold
  #ifdef XTTHRESHLD
          if(coll.actN > 0 && coll.movN >= MYdevice.detection_threshold && CollMemoryFULL == 0 && SuppressDetect == 0)
  #else
      if(coll.actN > 0 && CollMemoryFULL == 0 && SuppressDetect == 0)
  #endif
    {

   // while(fillbuffer++ < 360)
   // {
      if(AddSignature(&MyEvents, MyEvents.EventIndex) == 0)
        {
            uint8_t movements = 0; // test for sensor sensitivity
            if((MYdevice.sku & 0x0F) == 4)  // if a T-REX
            {

                movements = 1;
                currentindex++;
                coll.DetectionsCounter++;  // number of events
            }
            else  // all other traps
            {
                currentindex++;
                coll.DetectionsCounter++;  // number of events
                if(coll.movN > 255) movements = 255;
                else movements = coll.movN;
//                send_lora_pkt = true;
            }
//            if(MYdevice.production_mode == 1) sendSENS(movements);

            if(coll.Filter == 0 || coll.FilterTime == 0)
            {
                if (coll.movN >= MYdevice.detection_threshold )
                {
                    coll.EventsCounter++;  // only count Event if Filtering time is over
                    if(MYdevice.lora.event_trigger>0)
                      {
//                        if(coll.EventsCounter==MYdevice.lora.event_trigger && MYdevice.registration==1)
                        if(coll.EventsCounter==MYdevice.lora.event_trigger && MYdevice.registration==1)
                          first_event_flag = 1;
                      }
                    else
                      {
                        if(coll.EventsCounter==1 && MYdevice.registration==1)
                          first_event_flag = 1;
                      }
                    if(coll.FilterTime != 0)
                    {
                        //gecko_cmd_hardware_set_lazy_soft_timer(FilterTime, FilterTime/10,  TIMER6_HANDLE, 1);
                        sl_bt_system_set_lazy_soft_timer( coll.FilterTime, coll.FilterTime/10, TIMER6_HANDLE, 1 );
                        coll.Filter = 1;  // start filtering
                    }
                }
            }
        }
   // }
        coll.EventsSinceReset++;  // Events since Power Up  Change to detections since reset
        //swSetupAdvBeaconing();
//        adv_set_data();   //commented out for potential reset issue - 10272023 - JP
    }
    //else CollMemoryFULL = 1;
    collectionInit(); // clear all the counters and flags
}

void prepare_to_stop(void)
{
   coll.wating_for_last_mov = true;
}

