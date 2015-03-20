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
 *  Filename:      bt_vendor.h
 *
 *  Description:   A wrapper header file of bt_vendor_lib.h
 *
 *                 Contains definitions specific for interfacing with Intel
 *                 Bluetooth chipsets
 *
 ******************************************************************************/

#ifndef BT_VENDOR_H
#define BT_VENDOR_H

#include "bt_vendor_lib.h"
#include "vnd_buildcfg.h"
#include "bt_hci_bdroid.h"
#ifdef HAS_BDROID_BUILDCFG
#include "bdroid_buildcfg.h"
#endif


/******************************************************************************
**  Constants & Macros
******************************************************************************/

#ifndef FALSE
#define FALSE  0
#endif

#ifndef TRUE
#define TRUE   (!FALSE)
#endif

#ifndef INTEL_WP2_UART
#if (INTEL_LNP_UART == TRUE)
#define INTEL_WP2_UART TRUE
#endif
#endif

#ifndef LNP_LPM_ENABLED
#if (INTEL_LNP_UART == TRUE)
#define LNP_LPM_ENABLED FALSE
#endif
#endif

#ifndef BT_EN_VIA_USERIAL_IOCTL
#if (INTEL_LNP_UART == TRUE)
#if (INTEL_HSB_PLATFORM == TRUE)
#define BT_EN_VIA_USERIAL_IOCTL FALSE
#else
#define BT_EN_VIA_USERIAL_IOCTL TRUE
#endif
#endif
#else
#define BT_EN_VIA_USERIAL_IOCTL FALSE
#endif

#ifndef VENDOR_LIB_RUNTIME_TUNING_ENABLED
#define VENDOR_LIB_RUNTIME_TUNING_ENABLED   FALSE
#endif

/* Run-time configuration file */
#ifndef VENDOR_LIB_CONF_FILE
#if (INTEL_AG6XX_UART == TRUE)
#define VENDOR_LIB_CONF_FILE             "/system/etc/bluetooth/bt_vendor.conf"
#else
#define VENDOR_LIB_CONF_FILE             "/etc/bluetooth/bt_vendor.conf"
#endif
#endif

/* Device port name where Bluetooth controller attached */
#ifndef BLUETOOTH_UART_DEVICE_PORT
#if (INTEL_AG6XX_UART == TRUE)
#define BLUETOOTH_UART_DEVICE_PORT       "/dev/ttyBT0"    /* XMM6321, XMM6321A1 */
#else
#define BLUETOOTH_UART_DEVICE_PORT       "/dev/ttyO1"    /* maguro */
#endif
#endif

/* Location of firmware patch files */
#ifndef FW_PATCHFILE_LOCATION
#if (INTEL_AG6XX_UART == TRUE)
#define FW_PATCHFILE_LOCATION            "/system/etc/bluetooth/"  /* XMM6321 */
#else
#define FW_PATCHFILE_LOCATION            "/vendor/firmware/"  /* maguro */
#endif
#endif

/* sleep mode

    0: disable
    1: UART with Host wake/BT wake out of band signals
*/
#ifndef LPM_SLEEP_MODE
#define LPM_SLEEP_MODE                 1
#endif

/* Host Stack Idle Threshold in 300ms or 25ms

  In sleep mode 1, this is the number of firmware loops executed with no
    activity before the Host wake line is deasserted. Activity includes HCI
    traffic excluding certain sleep mode commands and the presence of SCO
    connections if the "Allow Host Sleep During SCO" flag is not set to 1.
    Each count of this parameter is roughly equivalent to 300ms or 25ms.
*/
#ifndef LPM_IDLE_THRESHOLD
#define LPM_IDLE_THRESHOLD              1
#endif

/* Host Controller Idle Threshold in 300ms or 25ms

    This is the number of firmware loops executed with no activity before the
    HC is considered idle. Depending on the mode, HC may then attempt to sleep.
    Activity includes HCI traffic excluding certain sleep mode commands and
    the presence of ACL/SCO connections.
*/
#ifndef LPM_HC_IDLE_THRESHOLD
#define LPM_HC_IDLE_THRESHOLD           1
#endif

/* BT_WAKE Polarity - 0=Active Low, 1= Active High */
#ifndef LPM_BT_WAKE_POLARITY
#define LPM_BT_WAKE_POLARITY            1    /* maguro */
#endif

/* HOST_WAKE Polarity - 0=Active Low, 1= Active High */
#ifndef LPM_HOST_WAKE_POLARITY
#define LPM_HOST_WAKE_POLARITY          1    /* maguro */
#endif

/* LPM_ALLOW_HOST_SLEEP_DURING_SCO

    When this flag is set to 0, the host is not allowed to sleep while
    an SCO is active. In sleep mode 1, the device will keep the host
    wake line asserted while an SCO is active.
    When this flag is set to 1, the host can sleep while an SCO is active.
    This flag should only be set to 1 if SCO traffic is directed to the PCM
    interface.
*/
#ifndef LPM_ALLOW_HOST_SLEEP_DURING_SCO
#define LPM_ALLOW_HOST_SLEEP_DURING_SCO  1
#endif

/* LPM_COMBINE_SLEEP_MODE_AND_LPM

    In Mode 0, always set byte 7 to 0. In sleep mode 1, device always
    requires permission to sleep between scans / periodic inquiries regardless
    of the setting of this byte. In sleep mode 1, if byte is set, device must
    have "permission" to sleep during the low power modes of sniff, hold, and
    park. If byte is not set, device can sleep without permission during these
    modes. Permission to sleep in Mode 1 is obtained if the BT_WAKE signal is
    not asserted.
*/
#ifndef LPM_COMBINE_SLEEP_MODE_AND_LPM
#define LPM_COMBINE_SLEEP_MODE_AND_LPM   1
#endif

/* LPM_ENABLE_UART_TXD_TRI_STATE

    When set to 0, the device will not tristate its UART TX line before going
    to sleep.
    When set to 1, the device will tristate its UART TX line before going to
    sleep.
*/
#ifndef LPM_ENABLE_UART_TXD_TRI_STATE
#define LPM_ENABLE_UART_TXD_TRI_STATE    0
#endif

/* LPM_PULSED_HOST_WAKE
*/
#ifndef LPM_PULSED_HOST_WAKE
#define LPM_PULSED_HOST_WAKE             0
#endif

/* LPM_IDLE_TIMEOUT_MULTIPLE

    The multiple factor of host stack idle threshold in 300ms/25ms
*/
#ifndef LPM_IDLE_TIMEOUT_MULTIPLE
#define LPM_IDLE_TIMEOUT_MULTIPLE        10
#endif

/* BT_WAKE_VIA_USERIAL_IOCTL

    Use userial ioctl function to control BT_WAKE signal
*/
#ifndef BT_WAKE_VIA_USERIAL_IOCTL
#define BT_WAKE_VIA_USERIAL_IOCTL       FALSE
#endif

/* BT_WAKE_VIA_PROC

    LPM & BT_WAKE control through PROC nodes
*/
#ifndef BT_WAKE_VIA_PROC
#define BT_WAKE_VIA_PROC                FALSE
#endif

/* SCO_CFG_INCLUDED

    SCO Configuration is not needed for iBT.
*/
#ifndef SCO_CFG_INCLUDED
#define SCO_CFG_INCLUDED                 FALSE
#endif

/* HW_END_WITH_HCI_RESET

    Sample code implementation of sending a HCI_RESET command during the epilog
    process. It calls back to the callers after command complete of HCI_RESET
    is received.
*/
#ifndef HW_END_WITH_HCI_RESET
#define HW_END_WITH_HCI_RESET            TRUE
#endif

/* Includes A2DP Offload  TRUE */
#ifndef A2DP_OFFLOAD_INCLUDED
#define A2DP_OFFLOAD_INCLUDED            FALSE       /* TRUE includes A2DP Offload code */
#endif

/* 64bit kernel support */
#ifndef SUPPORT_64BIT_KERNEL
#define SUPPORT_64BIT_KERNEL             TRUE
#endif

/******************************************************************************
**  Extern variables and functions
******************************************************************************/

extern bt_vendor_callbacks_t *bt_vendor_cbacks;

#define N_INTEL_LDISC 25

#endif /* BT_VENDOR_H */
