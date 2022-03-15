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
#include "em_cmu.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "sl_sleeptimer.h"

void my_events_callback(sl_power_manager_em_t from,
                       sl_power_manager_em_t to);

#define EM_EVENT_MASK_ALL  (SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM0   \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM0  \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM1 \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM1  \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM2 \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM2  \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM3 \
                            | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM3)

sl_power_manager_em_transition_event_handle_t event_handle;
sl_power_manager_em_transition_event_info_t event_info = {
  .event_mask = EM_EVENT_MASK_ALL,
  .on_event = my_events_callback,
};

#define TOOGLE_DELAY_MS 10000

bool toggle_timeout = false;
bool sleep = false;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

bool running = false;
uint32_t remaining_time = 0;
sl_sleeptimer_timer_handle_t my_timer;
bool toggle_timeout;

sl_power_manager_em_t stato = SL_POWER_MANAGER_EM0;

static void on_timeout(sl_sleeptimer_timer_handle_t *handle, void *data);




/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  app_log_debug("*************************************\n");
  sl_sleeptimer_start_timer_ms(&my_timer,
                                        TOOGLE_DELAY_MS,
                                        on_timeout, NULL,
                                        0,
                                        SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  sl_sleeptimer_get_timer_time_remaining(&my_timer, &remaining_time);
  remaining_time = sl_sleeptimer_tick_to_ms(remaining_time);
  app_log_debug("%u", remaining_time);
  if (toggle_timeout == true) {
    toggle_timeout = false;
    sl_sleeptimer_start_timer_ms(&my_timer,
                                          TOOGLE_DELAY_MS,
                                          on_timeout, NULL,
                                          0,
                                          SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
  }
  if(!sleep)
    {
      sl_power_manager_sleep();
      sleep = true;
    }




  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

static void on_timeout(sl_sleeptimer_timer_handle_t *handle,
                       void *data)
{
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
  app_log_debug("timeout");
  (void)&handle;
  (void)&data;
  toggle_timeout = true;
  sleep = false;
}

void my_events_callback(sl_power_manager_em_t from,
                        sl_power_manager_em_t to)
{
  app_log_debug("Event:%s-%s\r\n", from, to);
}
/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/

/*
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

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

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }

}

*/

