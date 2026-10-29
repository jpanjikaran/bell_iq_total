/*
 * lora_api_internal_join.h
 *
 *  Created on: Mar. 22, 2022
 *      Author: jasonp
 */

#ifndef LORA_API_INTERNAL_JOIN_H_
#define LORA_API_INTERNAL_JOIN_H_

//#include "sl_app_assert.h"
//#include "sl_bluetooth.h"
#include "sl_bt_api.h"


//#if ! defined(LORA_XDOT) && ! defined(LORA_MURATA)
//    // This file contains only lora_common functions that are used internal
//    // to the Murata or xDot C files
//    #error Must be included from only lora_xdot_radio.c or lora_radio.c
//#endif // ! defined(LORA_XDOT) && ! defined(LORA_MURATA)

#define LORA_API_INTERNAL_JOIN__STATE__JOINED        true
#define LORA_API_INTERNAL_JOIN__STATE__NOT_JOINED    false

bool lora_api_internal_join__is_network_joined(void);

void lora_api_internal_join__set_state(bool join_state);
void lora_api_internal_join__restart(void);
void lora_api_internal_join__successful();
void lora_api_internal_join__unsuccessful(void);

#endif /* LORA_API_INTERNAL_JOIN_H_ */
