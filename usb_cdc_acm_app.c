/**
 * * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *
 *
           _             _
          | |           | |
 _   _ ___| |__   ___ __| | ___   __ _  ___ _ __ ___    __ _ _ __  _ __   ___
| | | / __| '_ \ / __/ _` |/ __| / _` |/ __| '_ ` _ \  / _` | '_ \| '_ \ / __|
| |_| \__ \ |_) | (_| (_| | (__ | (_| | (__| | | | | || (_| | |_) | |_) | (__
 \__,_|___/_.__/ \___\__,_|\___| \__,_|\___|_| |_| |_| \__,_| .__/| .__(_)___|
             ______          ______                ______   | |   | |
            |______|        |______|              |______|  |_|   |_|

 *  * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *
 * @file usb_cdc_acm_app.c
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "sl_usbd_core.h"
#include "sl_usbd_class_cdc.h"
#include "sl_usbd_class_cdc_acm.h"

#include "sl_usbd_class_cdc_acm_instances.h"

//*-----------------------------------------------------------------------------
//*                          TYPEDEFS & MACROS
//*-----------------------------------------------------------------------------
// Task configuration
#define TASK_STACK_SIZE         512u
#define TASK_PRIO               24u
#define TASK_DELAY_MS           250u

// Menu Message and Length
#define  ACM_TERMINAL_BUF_LEN                   512u
#define  ACM_TERMINAL_SCREEN_SIZE               80u

#define  ACM_TERMINAL_CURSOR_START              "\033[H"
#define  ACM_TERMINAL_CURSOR_START_SIZE         3u

#define  ACM_TERMINAL_SCREEN_CLR                "\033[2J\033[H"
#define  ACM_TERMINAL_SCREEN_CLR_SIZE           7u

#define  ACM_TERMINAL_MSG                       "===== USB CDC ACM Serial Emulation Demo ======" \
                                                "\r\n"                                           \
                                                "\r\n"                                           \
                                                "1. Echo 1 demo.\r\n"                            \
                                                "2. Echo N demo.\r\n"                            \
                                                "Option: "

#define  ACM_TERMINAL_MSG_SIZE                  92u

#define  ACM_TERMINAL_MSG1                      "Echo 1 demo... \r\n\r\n>> "
#define  ACM_TERMINAL_MSG1_SIZE                 22u

#define  ACM_TERMINAL_MSG2                      "Echo N demo. You can send up to 512 characters at once... \r\n\r\n>> "
#define  ACM_TERMINAL_MSG2_SIZE                 65u

#define  ACM_TERMINAL_NEW_LINE                  "\n\r>> "
#define  ACM_TERMINAL_NEW_LINE_SIZE             5u

//?-------------------
//? Defined by RPL
//?-------------------
//? Define the size of the buffer that will store the user input characters.
//? You can adjust this value to change the maximum number of characters that
//? can be stored in the buffer.
#define USER_INPUT_BUFFER_SIZE 100

#define RECEIVED_DATA_MAX_SIZE 200


// Terminal Menu States
SL_ENUM(terminal_state_t) {
  ACM_TERMINAL_STATE_MENU = 0u,
  ACM_TERMINAL_STATE_ECHO_1,
  ACM_TERMINAL_STATE_ECHO_N
};

//*-----------------------------------------------------------------------------
//*                     GLOBAL VARIABLES
//*-----------------------------------------------------------------------------

// FreeRTOS Task handle
static TaskHandle_t task_handle;

// Universal buffer used to transmit and recevie data.
__ALIGNED(4) static uint8_t acm_terminal_buffer[ACM_TERMINAL_BUF_LEN];


//? Create a static character array (string) with the size specified by the
//? USER_INPUT_BUFFER_SIZE macro. This array will be used to store the characters
//? entered by the user via the terminal.
char user_input_buffer[USER_INPUT_BUFFER_SIZE];

//? Create a static variable of type uint32_t (unsigned 32-bit integer) to keep
//? track of the number of characters currently stored in the user_input_buffer.
//? The initial value is set to 0, as there are no characters in the buffer yet.
static uint32_t user_input_length = 0;

uint8_t received_data_buffer[RECEIVED_DATA_MAX_SIZE];
uint16_t received_data_length = 0;

//? I made this variable global so I can easily access it for a variety of functions
uint8_t cdc_acm_nbr;

bool conn;
uint8_t line_state;
uint8_t ch;
terminal_state_t state;
uint32_t xfer_len;
uint32_t xfer_len_dummy;
sl_status_t status;

const TickType_t xDelay = pdMS_TO_TICKS(TASK_DELAY_MS);

//? This variable is used to hold the state of the enter button so when the user presses it
//? The buffer containing the inputed data is confirmed and can then be submitted
bool buffer_writeable = false;

//*-----------------------------------------------------------------------------
//*                     STATIC FUNCTION DECLARATIONS
//*-----------------------------------------------------------------------------

static void terminal_task(void *p_arg);

static void print_user_input_buffer(uint8_t cdc_acm_nbr);

void print_acknowledgment_received(uint8_t* buffer, uint32_t length);

static void print_received_data_buffer(uint8_t cdc_acm_nbr, uint8_t* buffer, uint32_t length);

static void app_state_branch(terminal_state_t acm_state);

static void operation_acm_terminal_state_menu();

static void operation_state_echo_1();

static void operation_state_echo_n();

static void operation_ch_branch(uint8_t ch);

extern void call_print_user_input_buffer();

static void print_bitrate(double bitRate);

extern void call_print_bitrate(double bitRate);

//*-----------------------------------------------------------------------------
//*                         HOOK FUNCTIONS
//*-----------------------------------------------------------------------------

/***************************************************************************//**
 *                          sl_usbd_on_bus_event()
 *
 * @brief  USB bus events.
 ******************************************************************************/
void sl_usbd_on_bus_event(sl_usbd_bus_event_t event)
{
  switch (event) {
    case SL_USBD_EVENT_BUS_CONNECT:
      // called when usb cable is inserted in a host controller
      break;

    case SL_USBD_EVENT_BUS_DISCONNECT:
      // called when usb cable is removed from a host controller
      break;

    case SL_USBD_EVENT_BUS_RESET:
      // called when the host sends reset command
      break;

    case SL_USBD_EVENT_BUS_SUSPEND:
      // called when the host sends suspend command
      break;

    case SL_USBD_EVENT_BUS_RESUME:
      // called when the host sends wake up command
      break;

    default:
      break;
  }
}

/***************************************************************************//**
 *                         sl_usbd_on_config_event()
 *
 * @brief  USB configuration events.
 ******************************************************************************/
void sl_usbd_on_config_event(sl_usbd_config_event_t event, uint8_t config_nbr)
{
  (void)config_nbr;

  switch (event) {
    case SL_USBD_EVENT_CONFIG_SET:
      // called when the host sets a configuration after reset
      break;

    case SL_USBD_EVENT_CONFIG_UNSET:
      // called when a configuration is unset due to reset command
      break;

    default:
      break;
  }
}

//*-----------------------------------------------------------------------------
//*                       GLOBAL FUNCTIONS
//*-----------------------------------------------------------------------------

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void usb_device_cdc_acm_app_init(void)
{
  BaseType_t xReturned = pdFAIL;

  // Create application task
  xReturned = xTaskCreate(terminal_task,
                          "USB CDC ACM Termninal task",
                          TASK_STACK_SIZE,
                          (void *)(uint32_t)sl_usbd_cdc_acm_acm0_number,
                          TASK_PRIO,
                          &task_handle);
  EFM_ASSERT(xReturned == pdPASS);
}

//*-----------------------------------------------------------------------------
//*                   STATIC FUNCTION DEFINITIONS
//*-----------------------------------------------------------------------------


/***************************************************************************//**
 *                          usb_terminal_task()
 *
 * @brief  USB CDC ACM terminal emulation demo task.
 *
 * @param  p_arg  Task argument pointer.
 *
 * @note   (1) This task manages the display of the terminal in according to
 *             the user's inputs.
 ******************************************************************************/
static void terminal_task(void *p_arg)
{
  conn = false;
  line_state = 0;
  ch = 0u;
  state = ACM_TERMINAL_STATE_MENU;
  cdc_acm_nbr = (uint8_t)(uint32_t)p_arg;
  xfer_len = 0u;
  xfer_len_dummy = 0u;
  status = SL_STATUS_OK;

  while (true) {
    // Wait until device is in configured state.
    status = sl_usbd_cdc_acm_is_enabled(cdc_acm_nbr, &conn);                    //? Check if the USB CDC ACM interface is enabled and if DTR (Data Terminal Ready) is set.
    EFM_ASSERT(status == SL_STATUS_OK);

    status = sl_usbd_cdc_acm_get_line_control_state(cdc_acm_nbr, &line_state);
    EFM_ASSERT(status == SL_STATUS_OK);

    while ((conn != true)
           || ((line_state & SL_USBD_CDC_ACM_CTRL_DTR) == 0)) {
      // Delay Task
      vTaskDelay(xDelay);                                                       //? If the device is not configured or DTR is not set, delay for a short period and check again.

      status = sl_usbd_cdc_acm_is_enabled(cdc_acm_nbr, &conn);
      EFM_ASSERT(status == SL_STATUS_OK);

      status = sl_usbd_cdc_acm_get_line_control_state(cdc_acm_nbr, &line_state);
      EFM_ASSERT(status == SL_STATUS_OK);
    }

    /// Call app_branch here:
    ///
    app_state_branch(state);

  }
}


//*-----------------------------------------------------------------------------
//*                   RPL STATIC FUNCTION DEFINITIONS
//*-----------------------------------------------------------------------------

/**
 * This Function is used, simply, to display the contents of the user input buffer to the terminal
 * following every entry to user makes. This permits us to visualize the input and it's storage.
 *
 * @author  Ryan Woodward
 *
 * @param cdc_acm_nbr
 */
static void print_user_input_buffer(uint8_t cdc_acm_nbr)
{
  char message[128];                                                         //? Construct a buffer to hold the message with the user entered characters

  snprintf(message, sizeof(message), "\r\nInput Buffer: %s", user_input_buffer);      //? Create a message combining the "Buffer: " prefix with the user_input_buffer contents
                                                                            //? using the snprintf function. The formatted message will be stored in the 'message' buffer.
                                                                            //? 'sizeof(message)' ensures no buffer overflow. The result will be "Buffer: <user entered characters>".

  uint32_t xfer_len_dummy;                                                  //? Dummy variable to hold the number of bytes transferred.

  /*
   Write the message to the USB CDC ACM interface.
   The sl_usbd_cdc_acm_write function sends the 'message' buffer to the USB CDC ACM interface specified by 'cdc_acm_nbr'.
   The 'strlen(message)' argument specifies the length of the message to be sent (excluding the null terminator).
   The 0u argument indicates that we want to start writing from the beginning of the buffer.
   The 'xfer_len_dummy' variable will hold the actual number of bytes transferred, but it is not used in this context.
  */
  sl_status_t status = sl_usbd_cdc_acm_write(cdc_acm_nbr,
                                               (uint8_t*)message,
                                               strlen(message),
                                               0u,
                                               &xfer_len_dummy);
    if (status != SL_STATUS_OK) {
      // Handle error if needed
    }
}


/**
 * This function is used, simply, to display the contents of the acknowledgment buffer to the terminal.
 *
 * @author Ryan Woodward
 *
 * @param cdc_acm_nbr
 * @param buffer
 * @param length
 */
void print_acknowledgment_received(uint8_t* buffer, uint32_t length)
{

  char message[128];                                                         //? Construct a buffer to hold the message with the acknowledgment data


  //? Create a message combining the "Acknowledgment: " prefix with the acknowledgment data
  //? using the snprintf function. The formatted message will be stored in the 'message' buffer.
  //? 'sizeof(message)' ensures no buffer overflow. The result will be "Acknowledgment: <acknowledgment data>".

  //? To make it easier to see the Tx/Rx I've added A.) B.) to the snprintf before the ACK
  snprintf(message, sizeof(message), "\r\nAcknowledgment: %.*s", length, buffer);



  uint32_t xfer_len_dummy;                                                  //? Dummy variable to hold the number of bytes transferred.

  /*
   Write the message to the USB CDC ACM interface.
   The sl_usbd_cdc_acm_write function sends the 'message' buffer to the USB CDC ACM interface specified by 'cdc_acm_nbr'.
   The 'strlen(message)' argument specifies the length of the message to be sent (excluding the null terminator).
   The 0u argument indicates that we want to start writing from the beginning of the buffer.
   The 'xfer_len_dummy' variable will hold the actual number of bytes transferred, but it is not used in this context.
  */
  sl_status_t status = sl_usbd_cdc_acm_write(cdc_acm_nbr,
                                             (uint8_t*)message,
                                             strlen(message),
                                             0u,
                                             &xfer_len_dummy);
  if (status != SL_STATUS_OK) {
    // Handle error if needed
  }






}


static void print_received_data_buffer(uint8_t cdc_acm_nbr, uint8_t* buffer, uint32_t length)
{
    char message2[128];
    snprintf(message2, sizeof(message2), "Received: %.*s", length, buffer);

    uint32_t xfer_len_dummy;
    sl_status_t status = sl_usbd_cdc_acm_write(cdc_acm_nbr,
                                               (uint8_t*)message2,
                                               strlen(message2),
                                               0u,
                                               &xfer_len_dummy);
    if (status != SL_STATUS_OK) {
        // Handle error if needed
    }
}


static void print_bitrate(double bit_Rate)
{
  char message[128];                                                         //? Construct a buffer to hold the message with the user entered characters

  snprintf(message, sizeof(message), "\r\nBit_Rate: %.2f kbps", bit_Rate);      //? Create a message combining the "Buffer: " prefix with the user_input_buffer contents
                                                                            //? using the snprintf function. The formatted message will be stored in the 'message' buffer.
                                                                            //? 'sizeof(message)' ensures no buffer overflow. The result will be "Buffer: <user entered characters>".

  uint32_t xfer_len_dummy;                                                  //? Dummy variable to hold the number of bytes transferred.

  /*
   Write the message to the USB CDC ACM interface.
   The sl_usbd_cdc_acm_write function sends the 'message' buffer to the USB CDC ACM interface specified by 'cdc_acm_nbr'.
   The 'strlen(message)' argument specifies the length of the message to be sent (excluding the null terminator).
   The 0u argument indicates that we want to start writing from the beginning of the buffer.
   The 'xfer_len_dummy' variable will hold the actual number of bytes transferred, but it is not used in this context.
  */
  sl_status_t status = sl_usbd_cdc_acm_write(cdc_acm_nbr,
                                               (uint8_t*)message,
                                               strlen(message),
                                               0u,
                                               &xfer_len_dummy);
    if (status != SL_STATUS_OK) {
      // Handle error if needed
    }
}



static void app_state_branch(terminal_state_t acm_state)
{
  switch (state) {
    //operation_acm_terminal_state_menu()
    case ACM_TERMINAL_STATE_MENU:
      operation_acm_terminal_state_menu();
      break;

    // 'Echo 1' state.
    case ACM_TERMINAL_STATE_ECHO_1:
      operation_state_echo_1();
      break;

    // 'Echo N' state.
    case ACM_TERMINAL_STATE_ECHO_N:
       operation_state_echo_n();
       break;
    default:
      break;
  }//end switch statement
}// app_state_branch()


static void operation_acm_terminal_state_menu()
{
  // Display start cursor.
  //? Moves the cursor to the start of the terminal
  memcpy(acm_terminal_buffer, ACM_TERMINAL_CURSOR_START, ACM_TERMINAL_CURSOR_START_SIZE);

  status = sl_usbd_cdc_acm_write (cdc_acm_nbr, acm_terminal_buffer, ACM_TERMINAL_CURSOR_START_SIZE, 0u, &xfer_len_dummy);

  if (status != SL_STATUS_OK) {  return;  }

  // Display main menu.
  memcpy(acm_terminal_buffer, ACM_TERMINAL_MSG, ACM_TERMINAL_MSG_SIZE);

  status = sl_usbd_cdc_acm_write(cdc_acm_nbr, acm_terminal_buffer, ACM_TERMINAL_MSG_SIZE,  0u, &xfer_len_dummy);

  if (status != SL_STATUS_OK) {  return; }

  // Wait for character.
  status = sl_usbd_cdc_acm_read(cdc_acm_nbr, acm_terminal_buffer, 1u, 0u, &xfer_len_dummy);

  if (status != SL_STATUS_OK) {  return;  }

  ch = acm_terminal_buffer[0u];

  // Echo back character.
  status = sl_usbd_cdc_acm_write(cdc_acm_nbr, acm_terminal_buffer, 1u, 0u, &xfer_len_dummy);

  if (status != SL_STATUS_OK) {  return; }

  operation_ch_branch(ch);

  return;
}

static void operation_state_echo_1()
{
  // Wait for character.
       status = sl_usbd_cdc_acm_read(cdc_acm_nbr,acm_terminal_buffer,1u,0u,&xfer_len_dummy);

       if (status != SL_STATUS_OK) {  return; }

       //? Save the received character from user input into the 'ch' variable
       ch = acm_terminal_buffer[0u];

       // If 'Ctrl-c' character is received, return to 'menu' state.
       if (ch == 0x03) {
         state = ACM_TERMINAL_STATE_MENU;

         // Clear screen.
         memcpy(acm_terminal_buffer,ACM_TERMINAL_SCREEN_CLR,ACM_TERMINAL_SCREEN_CLR_SIZE);

         status = sl_usbd_cdc_acm_write(cdc_acm_nbr,acm_terminal_buffer,ACM_TERMINAL_SCREEN_CLR_SIZE, 0u,&xfer_len_dummy);

         if (status != SL_STATUS_OK) { return; }

       } else {
         // Echo back character.
         status = sl_usbd_cdc_acm_write(cdc_acm_nbr, acm_terminal_buffer,1u,0u,&xfer_len_dummy);

         if (status != SL_STATUS_OK) { return; }

         // Move to next line.
         memcpy(acm_terminal_buffer,ACM_TERMINAL_NEW_LINE,ACM_TERMINAL_NEW_LINE_SIZE);

         status = sl_usbd_cdc_acm_write(cdc_acm_nbr, acm_terminal_buffer, ACM_TERMINAL_NEW_LINE_SIZE, 0u,&xfer_len_dummy);

         if (status != SL_STATUS_OK) { return; }
       }//nested if-else
}

static void operation_state_echo_n()
{

  // Wait for N characters.
  //? Read multiple characters from the USB CDC ACM interface (user input).
  status = sl_usbd_cdc_acm_read(cdc_acm_nbr,acm_terminal_buffer,ACM_TERMINAL_BUF_LEN, 0u,
                                &xfer_len);
  if (status == SL_STATUS_OK) {
    // Add the received characters to the user input buffer.
    // Ensure we don't exceed the buffer size.
    uint32_t remaining_space = USER_INPUT_BUFFER_SIZE - user_input_length;
    uint32_t bytes_to_copy = (xfer_len < remaining_space) ? xfer_len : remaining_space;
    memcpy(&user_input_buffer[user_input_length], acm_terminal_buffer, bytes_to_copy);
    user_input_length += bytes_to_copy;

    // If 'Ctrl-c' character is received, return to 'menu' state.
    if (user_input_buffer[user_input_length - 1] == 0x03) {
      state = ACM_TERMINAL_STATE_MENU;

      // Clear screen.
      memcpy(acm_terminal_buffer,ACM_TERMINAL_SCREEN_CLR,ACM_TERMINAL_SCREEN_CLR_SIZE);

      status = sl_usbd_cdc_acm_write(cdc_acm_nbr, acm_terminal_buffer,ACM_TERMINAL_SCREEN_CLR_SIZE,0u,&xfer_len_dummy);

      if (status != SL_STATUS_OK) { return; }

    } else {
      // Echo back characters.
      //? Write back the received characters (user input) to the USB CDC ACM interface (echo).
      status = sl_usbd_cdc_acm_write(cdc_acm_nbr,&acm_terminal_buffer[0],xfer_len,0u, &xfer_len_dummy);

      if (status != SL_STATUS_OK) { return; }

      // Move to next line.
      memcpy(acm_terminal_buffer,ACM_TERMINAL_NEW_LINE,ACM_TERMINAL_NEW_LINE_SIZE);

      status = sl_usbd_cdc_acm_write(cdc_acm_nbr,acm_terminal_buffer,ACM_TERMINAL_NEW_LINE_SIZE,0u, &xfer_len_dummy);

      if (status != SL_STATUS_OK) { return; }
    }

    //? Call the function to print the user input buffer
    //print_user_input_buffer(cdc_acm_nbr);
    // Display the received data buffer
    //print_received_data_buffer(cdc_acm_nbr, received_data_buffer, received_data_length);

  }
  return;
}

static void operation_ch_branch(uint8_t ch)
{
  switch (ch) {
//--------------------------------
// Case '2' : Send 1 Character
//--------------------------------
    case '1':

      //  Clear screen.
      memcpy(acm_terminal_buffer,ACM_TERMINAL_SCREEN_CLR,ACM_TERMINAL_SCREEN_CLR_SIZE);

      status = sl_usbd_cdc_acm_write(cdc_acm_nbr, acm_terminal_buffer,ACM_TERMINAL_SCREEN_CLR_SIZE,0u,&xfer_len_dummy);

      if (status != SL_STATUS_OK) { break; }

      // Display option 1 instructions.
      memcpy(acm_terminal_buffer,ACM_TERMINAL_MSG1,ACM_TERMINAL_MSG1_SIZE);

      status = sl_usbd_cdc_acm_write(cdc_acm_nbr,acm_terminal_buffer,ACM_TERMINAL_MSG1_SIZE,0u,&xfer_len_dummy);

      if (status != SL_STATUS_OK) { break; }

      state = ACM_TERMINAL_STATE_ECHO_1;

      break;

//--------------------------------
// Case '2' : Send N Characters
//--------------------------------
    case '2':

      // Clear screen.
      memcpy(acm_terminal_buffer,ACM_TERMINAL_SCREEN_CLR,ACM_TERMINAL_SCREEN_CLR_SIZE);

      status = sl_usbd_cdc_acm_write(cdc_acm_nbr,acm_terminal_buffer,ACM_TERMINAL_SCREEN_CLR_SIZE,0u,&xfer_len_dummy);

      if (status != SL_STATUS_OK) { break; }

      // Display option 2 instructions.
      memcpy(acm_terminal_buffer, ACM_TERMINAL_MSG2,ACM_TERMINAL_MSG2_SIZE);

      status = sl_usbd_cdc_acm_write(cdc_acm_nbr,acm_terminal_buffer,ACM_TERMINAL_MSG2_SIZE,0u,&xfer_len_dummy);

      if (status != SL_STATUS_OK) { break; }

      state = ACM_TERMINAL_STATE_ECHO_N;

      break;

    default:

      break;
  }
}



extern void call_print_user_input_buffer()
{
  print_user_input_buffer(cdc_acm_nbr);
}


extern void call_print_bitrate(double bitRate)
{
  print_bitrate(bitRate);
}













