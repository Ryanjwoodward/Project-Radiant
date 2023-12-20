/**
 * * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *
 *
  __ _ _ __  _ __     _ __  _ __ ___   ___ ___  ___ ___   ___
 / _` | '_ \| '_ \   | '_ \| '__/ _ \ / __/ _ \/ __/ __| / __|
| (_| | |_) | |_) |  | |_) | | | (_) | (_|  __/\__ \__ \| (__
 \__,_| .__/| .__/   | .__/|_|  \___/ \___\___||___/___(_)___|
      | |   | |______| |
      |_|   |_|______|_|

 *  * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *
 * @file app_process.c
 *
 * @brief Application Process Source
 *
 * This source file contains the implementation of the application's main process.
 * It includes necessary headers, defines global variables, and implements the
 * application state machine and related functions. The application state machine
 * handles different states such as packet received, packet sent, RX packet error,
 * TX packet error, calibration error, and idle.
 *
 * @date September 15, 2023
 * @author Ryan Woodward
 * @organization RedPawLabs
 *
 *  * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

/***************************************************************************//**
 * @file
 * @brief app_tick.c
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
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

//*-----------------------------------------------------------------------------
//*                                   Includes
//*-----------------------------------------------------------------------------
#include "usb_cdc_acm_app.h"
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "sl_component_catalog.h"
#if defined(SL_CATALOG_APP_ASSERT_PRESENT)
#include "app_assert.h"
#endif
#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif
#include "rail.h"
#include "app_process.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_led_instances.h"
#include "sl_flex_rail_package_assistant.h"
#include "sl_flex_rail_config.h"
#include "sl_flex_rail_channel_selector.h"

#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "app_task_init.h"
#endif

#include "rail_types.h"
#include "cmsis_compiler.h"

#if defined(SL_CATALOG_RAIL_SIMPLE_CPC_PRESENT)
#include "sl_rail_simple_cpc.h"
#endif

//*-----------------------------------------------------------------------------
//*                              Macros and Typedefs
//*-----------------------------------------------------------------------------
/// Transmit data length
#define TX_PAYLOAD_LENGTH 16U//4096//(16U)

/// State machine of simple_trx
typedef enum {
  S_PACKET_RECEIVED,
  S_PACKET_SENT,
  S_RX_PACKET_ERROR,
  S_TX_PACKET_ERROR,
  S_CALIBRATION_ERROR,
  S_IDLE,
} state_t;


//? Define a Timestamp Struct for tracking the start and end time of data transmission
struct Timestamps {
    clock_t start;
    clock_t end;
};


//*-----------------------------------------------------------------------------
//*                          Static Function Declarations
//*-----------------------------------------------------------------------------

//? Set the current application state to the provided state.
static void set_app_state(state_t updated_state);

//? Get and return the current application state.
static state_t get_app_state();

//? Perform the operation associated with the provided state.
//? Returns an error code (always -1 in this case).
static int perform_current_state_operation(state_t state);

//? Perform operations related to handling a received packet.
static void operation_packet_received();

//? Perform operations related to handling a successfully sent packet.
static void operation_packet_sent();

//? Perform operations related to handling a radio RX packet error.
static void operation_rx_packet_error();

//? Perform operations related to handling a radio TX packet error.
static void operation_tx_packet_error();

//? Perform operations related to handling a radio calibration error.
static void operation_calibration_error();

//? Perform operations related to the application being in the idle state.
static void operation_idle();

//? Function that starts the timer for calculating the Bit Rate
void startTimer(struct Timestamps* timestamps);

//? Function that Stops the timer for calculating the bit rate
void stopTimer(struct Timestamps* timestamps);

double calculateAndPrintBitRate(uint32_t dataLength, struct Timestamps timestamps);


//*-----------------------------------------------------------------------------
//*                               Global Variables
//*-----------------------------------------------------------------------------
/// Flag, indicating transmit request (button was pressed / CLI transmit request has occurred)
volatile bool tx_requested = false;
/// Flag, indicating received packet is forwarded on CLI or not
volatile bool rx_requested = true;

//? RAIL Rx Packet Handles
RAIL_RxPacketHandle_t rx_packet_handle;
RAIL_RxPacketInfo_t packet_info;

//? Status indicator of the RAIL API calls
RAIL_Status_t rail_status = RAIL_STATUS_NO_ERROR;
RAIL_Status_t calibration_status_buff = RAIL_STATUS_NO_ERROR;

//? Define the maximum packet size
#define PACKET_SIZE 64  // Adjust the size as needed

//? Initialize variables to keep track of the current position in your data
static uint16_t data_index = 0;

//? Define an array to store the current packet data
uint8_t packet_data[PACKET_SIZE];

static double bitRate;

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------
/// The variable shows the actual state of the state machine
static volatile state_t state = S_IDLE;

/// Contains the last RAIL Rx/Tx error events
static volatile uint64_t error_code = 0;

/// Contains the status of RAIL Calibration
static volatile RAIL_Status_t calibration_status = 0;

/// Receive and Send FIFO
static __ALIGNED(RAIL_FIFO_ALIGNMENT) uint8_t rx_fifo[SL_FLEX_RAIL_RX_FIFO_SIZE];

static __ALIGNED(RAIL_FIFO_ALIGNMENT) uint8_t tx_fifo[SL_FLEX_RAIL_TX_FIFO_SIZE];

/// Transmit packet
static uint8_t out_packet[TX_PAYLOAD_LENGTH] = {
  0x0F, 0x16, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
  0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE,
};

/// Flags to update state machine from interrupt
static volatile bool packet_recieved = false;
static volatile bool packet_sent = false;
static volatile bool rx_error = false;
static volatile bool tx_error = false;
static volatile bool cal_error = false;

//? Static instance of the RAIL handle, assigned in the app_process_action function
//? this is how our application makes RAIL API calls
static RAIL_Handle_t* rpl_rail_handle;

//? Create an instance of the Timestamps for tracking the BitRate
static struct Timestamps timestamps;

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * Application state machine, called infinitely
 *****************************************************************************/
/**
 * //? I heavily edited this function that this files' readability is greatly improved.
 *
 * This function first assigns the passed Rail Handle (used for RAIL API calls) to this
 * files static rail_handle instance so that it can be accessed by static functions.
 * the function process the packet flags to determine which state the application should
 * be in finally the function passes that state to another function that will perform
 * the operation associated with the current state.
 *
*/
void app_process_action(RAIL_Handle_t rail_handle)
{

  rpl_rail_handle = rail_handle;

  // Status indicator of the RAIL API calls
  rail_status = RAIL_STATUS_NO_ERROR;
  calibration_status_buff = RAIL_STATUS_NO_ERROR;

  #if defined(SL_CATALOG_RAIL_SIMPLE_CPC_PRESENT)
  uint8_t success_sent = 0x01;
  #endif

  //? determine which flag is true and return the associated State_t
  state = get_app_state();

  //? perform the operation associated with the state, just, previously determined
  perform_current_state_operation(state);
}//? app_process_action()

/******************************************************************************
 * RAIL callback, called if a RAIL event occurs.
 *****************************************************************************/
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  error_code = events;
  // Handle Rx events
  if ( events & RAIL_EVENTS_RX_COMPLETION ) {
    if (events & RAIL_EVENT_RX_PACKET_RECEIVED) {
      // Keep the packet in the radio buffer, download it later at the state machine
      RAIL_HoldRxPacket(rail_handle);
      packet_recieved = true;
    } else {
      // Handle Rx error
      rx_error = true;
    }
  }
  // Handle Tx events
  if ( events & RAIL_EVENTS_TX_COMPLETION) {
    if (events & RAIL_EVENT_TX_PACKET_SENT) {
      packet_sent = true;
    } else {
      // Handle Tx error
      tx_error = true;
    }
  }

  // Perform all calibrations when needed
  if ( events & RAIL_EVENT_CAL_NEEDED ) {
    calibration_status = RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
    if (calibration_status != RAIL_STATUS_NO_ERROR) {
      cal_error = true;
    }
  }
#if defined(SL_CATALOG_KERNEL_PRESENT)
  app_task_notify();
#endif
}

/******************************************************************************
 * Button callback, called if any button is pressed or released.
 *****************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    tx_requested = true;
  }
#if defined(SL_CATALOG_KERNEL_PRESENT)
  app_task_notify();
#endif
}

#if defined(SL_CATALOG_RAIL_SIMPLE_CPC_PRESENT)
void sl_rail_simple_cpc_receive_cb(sl_status_t status, uint32_t len, uint8_t *data)
{
  //SL_STATUS_OK
  if (status == SL_STATUS_OK) {
    if (len == 1) {
      if (data[0] == 0x01 || data[0] == '1') {
        tx_requested = true;
      }
      if (data[0] == 0x00 || data[0] == '0') {
        rx_requested = !rx_requested;
      }
    }
  }
}
#endif

/******************************************************************************
 * Set up the rail TX fifo for later usage
 * @param rail_handle Which rail handler should be updated
 *****************************************************************************/
void set_up_tx_fifo(RAIL_Handle_t rail_handle)
{
  uint16_t allocated_tx_fifo_size = 0;
  allocated_tx_fifo_size = RAIL_SetTxFifo(rail_handle, tx_fifo, 0, SL_FLEX_RAIL_TX_FIFO_SIZE);
#if defined(SL_CATALOG_APP_ASSERT_PRESENT)
  app_assert(allocated_tx_fifo_size == SL_FLEX_RAIL_TX_FIFO_SIZE,
             "RAIL_SetTxFifo() failed to allocate a large enough fifo (%d bytes instead of %d bytes)\n",
             allocated_tx_fifo_size,
             SL_FLEX_RAIL_TX_FIFO_SIZE);
#endif
}


#if defined(SL_CATALOG_RAIL_SIMPLE_CPC_PRESENT)
void sl_rail_simple_cpc_receive_cb(sl_status_t status, uint32_t len, uint8_t *data)
{
  //SL_STATUS_OK
  if (status == SL_STATUS_OK) {
    if (len == 1) {
      if (data[0] == 0x01 || data[0] == '1') {
        tx_requested = true;
      }
      if (data[0] == 0x00 || data[0] == '0') {
        rx_requested = !rx_requested;
      }
    }
  }
}
#endif


/*
 *
 */
double calculateAndPrintBitRate(uint32_t dataLength, struct Timestamps timestamps) {

  double elapsedMilliseconds = (double)(timestamps.end - timestamps.start);

    if(elapsedMilliseconds == 0){
        elapsedMilliseconds = 0.000075;
    }

    if (elapsedMilliseconds > 0) {
        double bitRate = (double)dataLength * 8 / elapsedMilliseconds;
        double kbpsBitRate = bitRate / 1000.0;
        return kbpsBitRate;
    } else {
        return 128.8; // Invalid bit rate
    }
}



// Function to start the timer
void startTimer(struct Timestamps* timestamps) {
    timestamps->start = clock();
}

// Function to stop the timer
void stopTimer(struct Timestamps* timestamps) {
    timestamps->end = clock();
}



// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------

/**
 * @brief Sets the current application state to the specified state.
 *
 * This function updates the value of the global variable `current_app_state` with the provided
 * `updated_state` value, which represents the new application state.
 *
 * @param updated_state The new application state to be set.
 */
static void set_app_state(state_t updated_state)
{
  state = updated_state;
}

/**
 * @brief Checks the current state of the application.
 *
 * This function checks the values of global variables representing various
 * application states to determine the current state of the application. The
 * different states include packet received, packet sent, RX packet error,
 * TX packet error, calibration error, and idle.
 *
 * @return The current state of the application, represented by the app_state enum.
 *
 * @par Examples:
 * @code
 * // Call the function to check the current application state
 * app_state current_state = check_app_state();
 *
 * // Process the current state
 * switch (current_state) {
 *   case S_PACKET_RECEIVED:
 *     // Handle packet received state
 *     break;
 *   case S_PACKET_SENT:
 *     // Handle packet sent state
 *     break;
 *   // ... other state cases ...
 *   case S_IDLE:
 *     // Handle idle state
 *     break;
 *   default:
 *     // Handle unknown state
 *     break;
 * }
 * @endcode
 */
static state_t
get_app_state ()
{
  if (packet_recieved)
    {
      packet_recieved = false;
      return S_PACKET_RECEIVED;

    }
  else if (packet_sent)
    {
      packet_sent = false;
      return S_PACKET_SENT;

    }
  else if (rx_error)
    {
      rx_error = false;
      return S_RX_PACKET_ERROR;

    }
  else if (tx_error)
    {
      tx_error = false;
      return S_TX_PACKET_ERROR;

    }
  else if (cal_error)
    {
      cal_error = false;
      return S_CALIBRATION_ERROR;

    }
  else
    {
      return S_IDLE;
    }
} //? process_current_app_state()


/**
 * @brief Performs an operation based on the current application state.
 *
 * This function takes the current application state as input and performs a
 * corresponding operation based on that state. It uses a switch statement to
 * determine the appropriate operation to execute for each possible state,
 * including packet received, packet sent, RX packet error, TX packet error,
 * calibration error, and idle.
 *
 * @param current_state The current state of the application, represented by
 *                      the app_state enum.
 *
 * @return An error code indicating the result of the operation. This function
 *         always returns -1 to indicate an error code.
 *
 * @par Examples:
 * @code
 * // Call the function to perform an operation based on the current state
 * int result = perform_current_state_operation(current_state);
 *
 * // Check the result and handle accordingly
 * if (result == -1) {
 *   // Handle error case
 * } else {
 *   // Handle successful operation case
 * }
 * @endcode
 */
static int perform_current_state_operation(state_t current_state)
{

  //? Status Indicators from RAIL API Calls
  rail_status = RAIL_STATUS_NO_ERROR;
  calibration_status_buff = RAIL_STATUS_NO_ERROR;

  #if defined(SL_CATALOG_RAIL_SIMPLE_CPC_PRESENT)
  uint8_t success_sent = 0x01;
  #endif

  switch(current_state)
  {
    case S_PACKET_RECEIVED:
          operation_packet_received();
          set_app_state(S_IDLE);
      break;

    case S_PACKET_SENT:
          operation_packet_sent();
          set_app_state(S_IDLE);
      break;

    case S_RX_PACKET_ERROR:
         operation_rx_packet_error();
         set_app_state(S_IDLE);
      break;

    case S_TX_PACKET_ERROR:
        operation_tx_packet_error();
        set_app_state(S_IDLE);
      break;

    case S_CALIBRATION_ERROR:
        operation_calibration_error();
        set_app_state(S_IDLE);
        break;

    case S_IDLE:
       operation_idle();
      break;

    default:
        printf("Unexpected State Occurred. State: %d", current_state);
      break;
  }//? switch

  return -1;  //? Return an Error Code
}//? perform_current_state_operation


/**
 * @brief Handles operations for received packets.
 *
 * This function processes received packets by extracting packet information, handling the data,
 * and performing appropriate actions based on the received data.
 */
static void operation_packet_received()
{
        rx_packet_handle = RAIL_GetRxPacketInfo(rpl_rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
        while (rx_packet_handle != RAIL_RX_PACKET_HANDLE_INVALID)
          {
              uint8_t *start_of_packet = 0;
              uint16_t packet_size = unpack_packet(rx_fifo, &packet_info, &start_of_packet);

              rail_status = RAIL_ReleaseRxPacket(rpl_rail_handle, rx_packet_handle);

              if (rail_status != RAIL_STATUS_NO_ERROR)
                {

                  #if defined(SL_CATALOG_APP_LOG_PRESENT)
                    app_log_warning("RAIL_ReleaseRxPacket() result:%d", rail_status);
                  #endif
                }

              if (rx_requested)
                {
                  // Print or process the received data
                  if (packet_size <= RECEIVED_DATA_MAX_SIZE) {
                    memcpy(received_data_buffer, start_of_packet, packet_size);
                    received_data_length = packet_size;

                    // Call the print_acknowledgment_received function with the received data buffer and length
                    print_acknowledgment_received(received_data_buffer, received_data_length);

                  } else {
                    #if defined(SL_CATALOG_APP_LOG_PRESENT)
                    app_log_error("Received data exceeds the buffer size, dropping the data!\n");
                    #endif
                  }
                }

              sl_led_toggle(&sl_led_led0);
              rx_packet_handle = RAIL_GetRxPacketInfo(rpl_rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);

          }
}//?


/**
 * @brief Handles operations after a packet is successfully sent.
 *
 * This function handles tasks that need to be performed after a packet has been successfully sent.
 */
static void operation_packet_sent()
{
  printf("A Packet has been sent...\n");

  #if defined(SL_CATALOG_LED1_PRESENT)
      sl_led_toggle(&sl_led_led1);
  #else
      sl_led_toggle(&sl_led_led0);
  #endif

  stopTimer(&timestamps);

  // Calculate and print the bit rate using app_log_info
  bitRate = calculateAndPrintBitRate(TX_PAYLOAD_LENGTH, timestamps);

  call_print_bitrate(bitRate);

}//? operation_packet_sent()


/**
 * @brief Handles operations when a radio RX packet error occurs.
 *
 * This function processes actions to be taken when an RX packet error is encountered.
 *
 * @param rpl_error_code The error code associated with the error.
 */
static void operation_rx_packet_error()
{
  printf("Radio Rx Error Occurred. Events: %llX\n", error_code);
}//? operation_rx_packet_error()


/**
 * @brief Handles operations when a radio TX packet error occurs.
 *
 * This function processes actions to be taken when a TX packet error is encountered.
 *
 * @param rpl_error_code The error code associated with the error.
 */
static void operation_tx_packet_error()
{
  printf("Radio Tx Error Occurred. Events: %llX\n", error_code);

}//? operation_tx_packet_error()


/**
 * @brief Handles operations when a radio calibration error occurs.
 *
 * This function processes actions to be taken when a calibration error is encountered.
 *
 * @param rpl_calibration_status The calibration status associated with the error.
 */
static void operation_calibration_error()
{
  calibration_status_buff = calibration_status;

  printf("Radio Calibration Error Occurred. Evetns: %llX\n", error_code);
  printf("Contents of the Calibration Status Buffer: %d", calibration_status_buff);

}//? operation_calibration_error()


/**
 * @brief Handles operations when the application is in idle state.
 *
 * This function processes actions to be taken when the application is in the idle state.
 */
static void operation_idle()
{
        if (tx_requested)
          {

              startTimer(&timestamps);

              //? Update the out_packet with the data from the user_input_buffer
              memcpy(out_packet, user_input_buffer, USER_INPUT_BUFFER_SIZE);

              call_print_user_input_buffer();

              prepare_package(rpl_rail_handle, out_packet, sizeof(out_packet));
              rail_status = RAIL_StartTx(rpl_rail_handle, get_selected_channel(), RAIL_TX_OPTIONS_DEFAULT, NULL);

              if (rail_status != RAIL_STATUS_NO_ERROR)
                {

                  #if defined(SL_CATALOG_APP_LOG_PRESENT)
                    app_log_warning("RAIL_StartTx() result:%d ", rail_status);
                  #endif
                }

          tx_requested = false;
        }
}//? operation_idle()




