/******************************************************************************
 *
 *  Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 *  Copyright (C) 2009-2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  Filename:      upio.h
 *
 *  Description:   Contains definitions used for I/O controls
 *
 ******************************************************************************/

#ifndef UPIO_H
#define UPIO_H

/******************************************************************************
**  Constants & Macros
******************************************************************************/

#define UPIO_BT_POWER_OFF 0
#define UPIO_BT_POWER_ON  1

/* UPIO signals */
enum {
    UPIO_BT_WAKE = 0,
    UPIO_HOST_WAKE,
    UPIO_LPM_MODE,
    UPIO_MAX_COUNT
};

/* UPIO assertion/deassertion */
enum {
    UPIO_UNKNOWN = 0,
    UPIO_DEASSERT,
    UPIO_ASSERT
};

/******************************************************************************
**  Extern variables and functions
******************************************************************************/

/******************************************************************************
**  Functions
******************************************************************************/

/*******************************************************************************
**
** Function        upio_init
**
** Description     Initialization
**
** Returns         None
**
*******************************************************************************/
void upio_init(void);

/*******************************************************************************
**
** Function        upio_cleanup
**
** Description     Clean up
**
** Returns         None
**
*******************************************************************************/
void upio_cleanup(void);

#if (INTEL_WP2_UART == TRUE || INTEL_LNP_UART == TRUE || INTEL_WP2_USB == TRUE)
/*******************************************************************************
**
** Function        upio_set_bluetooth_power
**
** Description     Interact with low layer driver to set Bluetooth power
**                 on/off.
**
** Returns         0  : SUCCESS or Not-Applicable
**                 <0 : ERROR
**
*******************************************************************************/
int upio_set_bluetooth_power(int on);

/*******************************************************************************
**
** Function        upio_set
**
** Description     Set i/o based on polarity
**
** Returns         None
**
*******************************************************************************/
void upio_set(uint8_t pio, uint8_t action, uint8_t polarity);
#endif

#if (INTEL_AG6XX_UART == TRUE)
/*******************************************************************************
**
** Function        upio_set_d_state
**
** Description     Set d states: d0,  d0i2, d0i3, d3
**
** Returns         None
**
*******************************************************************************/
int upio_set_d_state(uint8_t state);

/*******************************************************************************
**
** Function        upio_set_bt_wake_state
**
** Description     Set bt_wake high or low
**
** Returns         CTS state
**
*******************************************************************************/
uint8_t upio_set_bt_wake_state(uint8_t bt_wake_state);

/*******************************************************************************
**
** Function        upio_get_cts_state
**
** Description     Set gets current CTS state
**
** Returns         None
**
*******************************************************************************/
uint8_t upio_get_cts_state(void);

/*******************************************************************************
**
** Function        upio_set_rts_state
**
** Description     Set sets RTS state to high/low
**
** Returns         None
**
*******************************************************************************/
void upio_set_rts_state(uint8_t rts_state);

/*******************************************************************************
**
** Function       upio_create_netlink_socket
**
** Description    Create a NETLINK Socket
**
** Returns         netlinksocket fd
**
*******************************************************************************/
int upio_create_netlink_socket(void);

/*******************************************************************************
**
** Function       upio_netlink_send_msg
**
** Description    send a msg to kernel
**
** Returns        None
**
*******************************************************************************/
void upio_netlink_send_msg();

/*******************************************************************************
**
** Function       upio_netlink_receive_message
**
** Description    thread to listen netlink packets from kernel
**
** Returns        None
**
*******************************************************************************/
void * upio_netlink_receive_message(void *ptr);

/*******************************************************************************
**
** Function       upio_close_netlink_socket
**
** Description    thread to listen netlink packets from kernel
**
** Returns        None
**
*******************************************************************************/
void upio_close_netlink_socket();

/*******************************************************************************
**
** Function       upio_netlink_listen_thread
**
** Description    start a thread to listen netlink packets from kernel
**
** Returns        None
**
*******************************************************************************/
int upio_netlink_listen_thread(void);
#endif
#endif /* UPIO_H */

