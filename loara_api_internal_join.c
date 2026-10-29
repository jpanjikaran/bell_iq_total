/*
 * loara_api_internal_join.c
 *
 *  Created on: Apr. 5, 2022
 *      Author: jasonp
 */


//#include "main.h"

//#ifdef LORA

#include "lora_api_internal_join.h"
#include "lora_xdot_radio.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "sl_bt_api.h"
#include "utility.h"
//#include "timers.h"
//#include "utils.h"
//#include "eeprom.h"

static bool lora_api_internal_join__state = LORA_API_INTERNAL_JOIN__STATE__NOT_JOINED;
extern deviceDATA MYdevice;

//-------------------------------------------------------------
bool lora_api_internal_join__is_network_joined(void)
{
    return lora_api_internal_join__state;
}
//-------------------------------------------------------------
void lora_api_internal_join__set_state(bool join_state)
{
//    if (join_state == LORA_API_INTERNAL_JOIN__STATE__JOINED)
//        TRC_RADIO_IMPORTANT("JOIN - TRUE");
//
//    else
//        TRC_RADIO_IMPORTANT("JOIN - FALSE");

    lora_api_internal_join__state = join_state;
//    MYdevice.lora.join_status = join_state;
}

//-------------------------------------------------------------
void lora_api_internal_join__restart(void)
{
    stop_radio_response_timeout();

    lora_api_internal_join__set_state(LORA_API_INTERNAL_JOIN__STATE__NOT_JOINED);
//    MYdevice.lora.join_status = 0;
}

//-------------------------------------------------------------
void lora_api_internal_join__successful(void)
{
    stop_radio_response_timeout();

    lora_api_internal_join__state = LORA_API_INTERNAL_JOIN__STATE__JOINED;
//    MYdevice.lora.join_status = 1;

//    TRC_RADIO_IMPORTANT("JOIN - SUCCESS SUCCESS");
}
//-------------------------------------------------------------
void lora_api_internal_join__unsuccessful(void)
{
    stop_radio_response_timeout();

    lora_api_internal_join__state = LORA_API_INTERNAL_JOIN__STATE__NOT_JOINED;
//    MYdevice.lora.join_status = 0;

//    TRC_RADIO_IMPORTANT("JOIN - FAILURE FAILURE");
}
