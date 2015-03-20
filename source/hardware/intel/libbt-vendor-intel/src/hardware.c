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
 *  Filename:      hardware.c
 *
 *  Description:   Contains controller-specific functions, like
 *                      firmware patch download
 *                      low power mode operations
 *
 ******************************************************************************/

#define LOG_TAG "bt_hwcfg"

#include <ctype.h>
#include <cutils/properties.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <utils/Log.h>
#include "bt_hci_bdroid.h"
#include "bt_vendor.h"
#include "userial.h"
#include "userial_vendor.h"
#include "upio.h"

/******************************************************************************
**  Constants & Macros
******************************************************************************/

/*
#ifndef BTHW_DBG
#define BTHW_DBG TRUE
#endif
*/
#define BTHW_DBG TRUE

#if (BTHW_DBG == TRUE)
#define BTHWDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTHWDBG(param, ...) {}
#endif
#define BTHWERR(param, ...) {ALOGE(param, ## __VA_ARGS__);}

#if (INTEL_AG6XX_UART == TRUE)
#define FW_PATCHFILE_EXTENSION      ".pbn"
#ifdef BT_USE_NVM
#define BDDATA_FILE                 "/nvm_fs_partition/bluetooth/bddata"
#else
#define BDDATA_FILE                 "/system/etc/bluetooth/bddata"
#endif
#elif (INTEL_LNP_UART == TRUE)
#define FW_PATCHFILE_EXTENSION      ".sfi"
#define BDDATA_FILE                 "/system/etc/bluetooth/bddata"
#define BDDATA_FILE_A0              "/system/etc/bluetooth/bddata_A0"
#define BDDATA_FILE_K0              "/system/etc/bluetooth/bddata_K0"
#define BDDATA_FILE_B0              "/system/etc/bluetooth/bddata_B0"
#define BTNVM_FILE                  "/config/bluetooth/.btnvm"
#else
#define FW_PATCHFILE_EXTENSION      ".seq"
#define BDDATA_FILE                 "/system/etc/bluetooth/bddata"
#endif

#if (INTEL_LNP_UART == TRUE)
#define USE_AUX_FILE    TRUE
#endif

#define FW_PATCHFILE_EXTENSION_LEN  4
#define FW_PATCHFILE_PATH_MAXLEN    128 /* FW patch path Max length */
#define HCI_CMD_MAX_PARAM_SIZE      255
#define HCI_CMD_MAX_LEN             256
#define HCI_EVT_MAX_LEN             HCI_CMD_MAX_LEN
#define SEQ_FILE_MAX_LINE_SIZE      1024
#define FIRMWARE_BUILD_BYTE_1       12
#define FIRMWARE_BUILD_BYTE_2       13

#define DDC_CMD_PARAM_MAX_LEN         16
#define DDC_PKT_HDR_SIZE              3
#define DDC_NUMBER_OF_FIELDS_PER_CMD  3

/* HCI command opcode */
#define HCI_RESET                               0x0C03
#define HCI_INTEL_READ_SW_VERSION               0xFC05
#define HCI_INTEL_SET_UART_BAUD                 0xFC06
#define HCI_INTEL_MANUFACTURE_MODE              0xFC11
#define HCI_INTEL_INF_BDDATA                    0xFC2F
#define HCI_INTEL_INF_MEM_WRITE                 0xFC8E
#define HCI_INTEL_SWITCHED_TO_FLASH_LDR         0xFC01
#define HCI_INTEL_SSEND                         0xFC09
#define HCI_INTEL_READ_BOOT_PARAMS              0xFC0D
#define HCI_INTEL_WRITE_BD_ADDRESS              0xFC31
#define HCI_INTEL_DDC_CONFIG_WRITE              0xFC8B

/* HCI command param sizes */
#define HCI_INTEL_MEM_WRITE_MODE_BYTE               0
#define HCI_CMD_PREAMBLE_SIZE                       3
#define HCI_INTEL_MANUFACTURE_MODE_PARAM_SIZE       2
#define HCI_INTEL_SET_UART_BAUD_PARAM_SIZE          1
#define HCI_INTEL_READ_SW_VERSION_PARAM_SIZE        0
#define HCI_INTEL_INF_BDDATA_PARAM_SIZE             80
#define HCI_INTEL_BD_ADDRESS_LEN                    6

/* HCI read sw version number byte location */
#define HCI_EVT_READ_HW_VARIANT                     7
#define HCI_EVT_READ_HW_REVISION                    8
#define HCI_CMD_CMPL_EVT_STATUS_BYTE                5
#define HCI_CMD_STATUS_EVT_STATUS_BYTE              2
#define HCI_INTEL_EVT_STATUS_RET_BYTE               3
#define HCI_CMD_OPCODE_FIRST_BYTE                   2
#define HCI_CMD_CMPL_EVT_OPCODE_FIRST_BYTE          3
#define HCI_CMD_CMPL_EVT_OPCODE_SECOND_BYTE         4
#define HCI_CMD_STATUS_EVT_OPCODE_FIRST_BYTE        4
#define HCI_CMD_STATUS_EVT_OPCODE_SECOND_BYTE       5
#define HCI_CMD_LEN_OFFSET                          2

/* HCI event code & event id */
#define HCI_COMMAND_CMPL_EVT_CODE                   0X0E
#define HCI_COMMAND_STATUS_EVT_CODE                 0x0F
#define HCI_INTEL_DEBUG_EVT_CODE                    0xFF
#define HCI_INTEL_STARTUP_EVT_CODE                  0x00
#define HCI_INTEL_STARTUP                           0x00
#define HCI_INTEL_DEFAULT_BD_DATA                   0x05
#define HCI_INTEL_WRITE_BD_DATA_CMPL                0x19
#define HCI_INTEL_SET_UART_BAUD_CMPL                0x12
#define HCI_INTEL_AFH_INFO                          0x20

/*HCI secure send commands result */
#define HCI_INTEL_SSEND_RST_EVT_STATUS              0x00
#define HCI_INTEL_SSEND_RST_EVT_ID                  0x06
#define HCI_INTEL_SSEND_RST_EVT_ID_BYTE             2
#define HCI_INTEL_SSEND_RST_EVT_STATUS_BYTE         3

#define PATCH_MAX_LENGTH                            244

#define HCI_INTEL_INIT_TYPE                         0
#define HCI_INTEL_DATA_TYPE                         1
#define HCI_INTEL_SIGN_TYPE                         2
#define HCI_INTEL_PKEY_TYPE                         3
#define INT_CMD_PKT_MAX_COUNT                       1        //Should be <= INT_CMD_PKT_MAX_COUNT defined for H4 layer

#define HCI_MSG_TYPE_CMD                            0x01
#define HCI_MSG_TYPE_EVT                            0x02

#if (A2DP_OFFLOAD_INCLUDED == TRUE)
#define HCI_INTEL_A2DP_STREAM_CONFIG                0xFC78
#define HCI_INTEL_A2DP_SET_STREAM_STATE             0xFC79
#define HCI_INTEL_A2DP_STREAM_CONFIG_PARAM_SIZE     13
#define HCI_INTEL_A2DP_SET_STREAM_STATE_PARAM_SIZE  1
#endif

#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}
#define BDADDR_TO_STREAM(p, a)   {register int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (uint8_t) a[BD_ADDR_LEN - 1 - ijk];}
#define STREAM_TO_UINT16(u16, p) {u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); (p) += 2;}
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT32_TO_STREAM(p, u32) {*(p)++ = (uint8_t)(u32); *(p)++ = (uint8_t)((u32) >> 8); *(p)++ = (uint8_t)((u32) >> 16); *(p)++ = (uint8_t)((u32) >> 24);}

/******************************************************************************
**  Local type definitions
******************************************************************************/

/* Return Types */
enum
{
    FAILURE = -1,
    SUCCESS = 0
};

/* enum to identify use of var_param of fw_config_seq_t structure */
enum
{
    NOT_USED = 1,            /* var_param is not used for particular sequence */
    IGNORE_EVT,              /* Do not verify the event received from controller
                                for particular sequence */
    VERIFY_EVT,              /* Verify the event received from controller
                                for particular sequence */
    NEXT_STATE_FW_DOWNLOADED /* Indicate the next state to jump
                                if firmware is already downloaded */
};

enum
{
    INT_EVT_CALLBACK_REG,
    INT_EVT_CALLBACK_DEREG /* not using currently */
};

enum
{
    RTS_STATE_ONE = 0,
    RTS_STATE_ZERO
};

enum
{
    OP_INVALID,
    OP_HCI_CMD,
    OP_HCI_EVT,
    OP_FIND_PATCH,
    OP_PATCH_DOWNLOAD,
    OP_RTS_STATE_CHANGE,
    OP_HOST_BAUD_CHANGE,
    OP_WRITE_BDDATA,
    OP_WRITE_BTNVM,
    OP_REG_ASYNC_EVENT,
    OP_REPORT_SUCCESS,
    OP_REPORT_FAILURE,
    OP_MAX
}; /* HW configuration operations */

enum
{
    BT_TURNING_OFF,
    BT_TURNING_ON
};

enum
{
    HW_CFG_SEND_CSS,
    HW_CFG_SEND_PKEY1,
    HW_CFG_SEND_PKEY2,
    HW_CFG_SEND_SIGN1,
    HW_CFG_SEND_SIGN2,
    HW_CFG_SEND_DATA
};

typedef struct
{
    uint8_t operation;
    uint8_t var_param;        /* use of var_param for different HW
                                 configuration operations

                              OP_HCI_CMD  ->  Not used
                              OP_HCI_EVT  ->  Tells whether received event
                                              should be verified or not
                              OP_FIND_PATCH  ->  Not used
                              OP_PATCH_DOWNLOAD  ->  Not used
                              OP_RTS_STATE_CHANGE  ->  Denotes post operatoin
                                                       RTS state
                              OP_HOST_BAUD_CHANGE  ->  Denotes post operation
                                                       host baud rate
                              OP_WRITE_BDDATA  ->  Not used
                              OP_WRITE_BTNVM  ->  Not used
                              OP_REG_ASYNC_EVENT  ->  Denotes registration for
                                                      asynchronous event
                              OP_REPORT_SUCCESS  ->  Not used
                              OP_REPORT_FAILURE  ->  Not used */
    uint8_t buffer[HCI_CMD_MAX_LEN];
} fw_config_seq_t;

/* Structure of action handlers */
typedef void (*tHW_CONFIG_OP_INVALID_HANDLER)(void);
typedef uint8_t (*tHW_CONFIG_OP_HCI_CMD_HANDLER)(fw_config_seq_t* config_seq);
typedef uint8_t (*tHW_CONFIG_OP_HCI_EVT_HANDLER)(fw_config_seq_t* config_seq, void* buf);
typedef uint8_t (*tHW_CONFIG_OP_FIND_PATCH_HANDLER)(char *p_chip_id_str);
typedef uint8_t (*tHW_CONFIG_OP_PATCH_DOWNLOAD_HANDLER)(void* buf);
typedef void (*tHW_CONFIG_OP_RTS_STATE_CHANGE_HANDLER)(int state);
typedef void (*tHW_CONFIG_OP_HOST_BAUD_CHANGE_HANDLER)(uint8_t* host_baud_rate);
typedef uint8_t (*tHW_CONFIG_OP_WRITE_BDDATA_HANDLER)(void);
typedef uint8_t (*tHW_CONFIG_OP_WRITE_BTNVM_HANDLER)(void);
typedef void (*tHW_CONFIG_OP_REG_ASYNC_EVENT_HANDLER)(uint8_t state);
typedef void (*tHW_CONFIG_OP_REPORT_SUCCESS_HANDLER)(void);
typedef void (*tHW_CONFIG_OP_REPORT_FAILURE_HANDLER)(void);

typedef struct
{
    tHW_CONFIG_OP_INVALID_HANDLER invalid_handler;
    tHW_CONFIG_OP_HCI_CMD_HANDLER hci_cmd_handler;
    tHW_CONFIG_OP_HCI_EVT_HANDLER hci_evt_handler;
    tHW_CONFIG_OP_FIND_PATCH_HANDLER find_patch_handler;
    tHW_CONFIG_OP_PATCH_DOWNLOAD_HANDLER patch_download_handler;
    tHW_CONFIG_OP_RTS_STATE_CHANGE_HANDLER rts_state_change_handler;
    tHW_CONFIG_OP_HOST_BAUD_CHANGE_HANDLER host_baud_change_handler;
    tHW_CONFIG_OP_WRITE_BDDATA_HANDLER write_bddata_handler;
    tHW_CONFIG_OP_WRITE_BTNVM_HANDLER write_btnvm_handler;
    tHW_CONFIG_OP_REG_ASYNC_EVENT_HANDLER reg_async_event_handler;
    tHW_CONFIG_OP_REPORT_SUCCESS_HANDLER report_success_handler;
    tHW_CONFIG_OP_REPORT_FAILURE_HANDLER report_failure_handler;
} tHW_CONFIG_OP_HANDLER;

#if (INTEL_WP2_USB == TRUE)
fw_config_seq_t fw_config_bt_on_seq[] =
{
    {OP_REG_ASYNC_EVENT, INT_EVT_CALLBACK_REG, {0x00}},
                                            /* register callback for async event */
                                            /* it is not required for WP but still
                                               registering for async events */
    {OP_HCI_CMD, NOT_USED, {0x03, 0x0c, 0x00}}, /* HCI Reset */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00}},
    {OP_HCI_CMD, NOT_USED, {0x05, 0xfc, 0x00}}, /* Read SW Version */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x0d, 0x01, 0x05, 0xfc, 0x00}},
    {OP_FIND_PATCH, NOT_USED, {0x00}},
    {OP_HCI_CMD, NOT_USED, {0x11, 0xfc, 0x02, 0x01, 0x00}}, /* Manufacture mode on */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x11,0xfc,0x00}},
    {OP_PATCH_DOWNLOAD, NOT_USED, {0x00}},
    {OP_HCI_CMD, NOT_USED, {0x11, 0xfc, 0x02, 0x00, 0x02}}, /* Manufacture mode off */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x11,0xfc,0x00}},
    {OP_HCI_CMD, NOT_USED, {0x05, 0xfc, 0x00}}, /* Read SW Version */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x0d, 0x01, 0x05, 0xfc, 0x00}},
    {OP_REPORT_SUCCESS, NEXT_STATE_FW_DOWNLOADED, {0}}  /* Report FW init success to stack */
};
#endif

#if (INTEL_LNP_UART == TRUE)
/* BT on sequence for A0 and A3 boot loader */
fw_config_seq_t fw_config_bt_on_a0_a3_seq[] =
{
    {OP_REG_ASYNC_EVENT, INT_EVT_CALLBACK_REG, {0x00}}, /* register callback for async event */
    {OP_FIND_PATCH, NOT_USED, {0x00}},
    {OP_HCI_CMD, NOT_USED, {0x11, 0xfc, 0x02, 0x01, 0x00}}, /* Manufacture mode on */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x11,0xfc,0x00}},
    /* Change baud rate to 3M */
    {OP_RTS_STATE_CHANGE, RTS_STATE_ONE, {0x00}},      /* Set RTS state to zero */
    {OP_HCI_CMD, NOT_USED, {0x06, 0xfc, 0x01, 0x0b}},   /* Change the baud rate to 3M */
    {OP_HOST_BAUD_CHANGE, USERIAL_BAUD_3M, {0x00}},     /* Set host baud rate to 3M */
    {OP_HCI_EVT, IGNORE_EVT, {0x00}},
    {OP_PATCH_DOWNLOAD, NOT_USED, {0x00}},
    /* Change baud rate to 115200 */
    {OP_RTS_STATE_CHANGE, RTS_STATE_ONE, {0x00}},      /* Set RTS state to zero */
    {OP_HCI_CMD, NOT_USED, {0x06, 0xfc, 0x01, 0x04}},  /* Change the baud rate to 115200 */
    {OP_HOST_BAUD_CHANGE, USERIAL_BAUD_115200, {0x00}},  /* Set host baud rate to 115200 */
    {OP_HCI_EVT, IGNORE_EVT, {0x0e, 0x04, 0x01, 0x06, 0xfc, 0x00}},
    {OP_HCI_CMD, NOT_USED, {0x01, 0xfc, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x80, 0x00, 0x00}},  /* Intel Reset */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x04, 0x01, 0x01, 0xfc, 0x00}},
                                              /* Command complete for Intel Reset */
                                              /* next event will be HCI Reset CMP
                                                 which will go to stack and
                                                 stack will ignore it */
    {OP_HCI_EVT, VERIFY_EVT, {0xff, 0x01,0x00}},  /* Intel start up event */
    {OP_HCI_CMD, NEXT_STATE_FW_DOWNLOADED, {0x11, 0xfc, 0x02, 0x01, 0x00}},  /* Manufacture mode on */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x11,0xfc,0x00}},
    {OP_WRITE_BDDATA, NOT_USED, {0x00}},     /* writing BD data from AUX file */
    {OP_WRITE_BTNVM, NOT_USED, {0x00}},      /* writing DDC commands from btnvm file */
    {OP_HCI_CMD, NOT_USED, {0x11, 0xfc, 0x02, 0x01, 0x00}}, /* Manufacture mode on */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x11,0xfc,0x00}},
    /* Change baud rate to 3M */
    {OP_RTS_STATE_CHANGE, RTS_STATE_ONE, {0x00}},      /* Set RTS state to zero */
    {OP_HCI_CMD, NOT_USED, {0x06, 0xfc, 0x01, 0x0b}}, /* Change the baud rate to 3M */
    {OP_HOST_BAUD_CHANGE, USERIAL_BAUD_3M, {0x00}},    /* Set host baud rate to 3M */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x04, 0x01, 0x06, 0xfc, 0x00}},
    {OP_HCI_CMD, NOT_USED, {0x11, 0xfc, 0x02, 0x00, 0x00}}, /* Manufacture mode off */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x11,0xfc,0x00}},
    {OP_HCI_CMD, NOT_USED, {0x03, 0x0c, 0x00}}, /* HCI Reset */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00}},
    {OP_HCI_EVT, VERIFY_EVT, {0xff, 0x01,0x00}},  /* Intel start up event */
    /* PCM SCO Configuration */
    {OP_HCI_CMD, NOT_USED, {0x08, 0xfc, 0x14, 0x02, 0x00,
                            0x00, 0x07, 0x20, 0x00, 0x00, 0x00, 0x20, 0x02, 0x20,
                            0x00, 0x00, 0x02, 0x87, 0x01, 0x00, 0x00, 0x00, 0x00}},
    /*
    LENGTH                  = 0x14
    USECASE                 = 0x02 - FMR
    PCM BLOCK               = 0x00 - PCM BLOCK 1
    PCM CHANNEL             = 0x00 - CHANNEL 1
    SAMPLING_FREQ           = 0x07 - 48KHZ
    FRAME_LENGTH            = 0x20 - 32 BITS
    FRAME_LENGTH            = 0x00
    CHANNEL_ONE_SLOT_START1 = 0x00 - 0x00
    CHANNEL_ONE_SLOT_START2 = 0x00
    CHANNEL_ONE_SLOT_LEN    = 0x20 - 32
    CHANNEL_ONE_CONFIG_BITS = 0x02 - WORD LENGTH is 32, FRINV is not used
    CHANNEL_TWO_SLOT_START1 = 0x20 - 32
    CHANNEL_TWO_SLOT_START2 = 0x00
    CHANNEL_TWO_SLOT_LEN    = 0x00 - 00 , not used
    CHANNEL_TWO_CONFIG_BITS = 0x02
    BLOCK_CONFIG_BITS       = 0x87 - CLOCK&FRAME SLAVE, IDLELEVEL is low,DOUBLE CLK not used,PCM CLK not inverted, Frame signal MSB justified, Frame signal mode is early.
    FRAME_SIGNAL_LENGTH     = 0x01 - Normal FRAME MODE
    PORT_SELECTION          = 0x00 - PRIMARY PORT
    DISABLED_PCM_PIN_STATE  = 0x00 - ALL PINS ARE LOW
    RESERVED1               = 0x00
    RESERVED2               = 0x00
    */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x08,0xfc,0x00}},
    {OP_HCI_CMD, NOT_USED, {0x08, 0xfc, 0x14, 0x00, 0x00,
                            0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10,
                            0x00, 0x00, 0x00, 0x87, 0x01, 0x00, 0x00, 0x00, 0x00}},
    /*
    LENGTH                  = 0x14
    USECASE                 = 0x00 - SCO NBS
    PCM BLOCK               = 0x00 - PCM BLOCK 1
    PCM CHANNEL             = 0x00 - CHANNEL 1
    SAMPLING_FREQ           = 0x00 - 8KHZ
    FRAME_LENGTH            = 0x10 - 16 BITS
    FRAME_LENGTH            = 0x00
    CHANNEL_ONE_SLOT_START1 = 0x00 - 0x00
    CHANNEL_ONE_SLOT_START2 = 0x00
    CHANNEL_ONE_SLOT_LEN    = 0x10 - 16
    CHANNEL_ONE_CONFIG_BITS = 0x00 - WORD LENGTH is 16, FRINV is not used
    CHANNEL_TWO_SLOT_START1 = 0x10 - 16
    CHANNEL_TWO_SLOT_START2 = 0x00
    CHANNEL_TWO_SLOT_LEN    = 0x00 - 00 , not used
    CHANNEL_TWO_CONFIG_BITS = 0x00
    BLOCK_CONFIG_BITS       = 0x87 - CLOCK&FRAME SLAVE, IDLELEVEL is low,DOUBLE CLK not used,PCM CLK not inverted, Frame signal MSB justified, Frame signal mode is early.
    FRAME_SIGNAL_LENGTH     = 0x01 - Normal FRAME MODE
    PORT_SELECTION          = 0x00 - PRIMARY PORT
    DISABLED_PCM_PIN_STATE  = 0x00 - ALL PINS ARE LOW
    RESERVED1               = 0x00
    RESERVED2               = 0x00
    */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x08,0xfc,0x00}},
#if (A2DP_OFFLOAD_INCLUDED == TRUE)
    {OP_HCI_CMD, NOT_USED, {0x08, 0xfc, 0x14, 0x03, 0x00,
                             0x00, 0x07, 0x28, 0x00, 0x00, 0x00, 0x20, 0x02, 0x29,
                             0x00, 0x20, 0x02, 0x87, 0x01, 0x00, 0x00, 0x00, 0x00}},
    /*
    LENGTH                  = 0x14
    USECASE                 = 0x03 - A2DP
    PCM BLOCK               = 0x00 - PCM BLOCK 1
    PCM CHANNEL             = 0x00 - CHANNEL 1
    SAMPLING_FREQ           = 0x07 - 48KHZ
    FRAME_LENGTH            = 0x28 - 32 BITS
    FRAME_LENGTH            = 0x00
    CHANNEL_ONE_SLOT_START1 = 0x00 - 0x00
    CHANNEL_ONE_SLOT_START2 = 0x00
    CHANNEL_ONE_SLOT_LEN    = 0x20 - 32
    CHANNEL_ONE_CONFIG_BITS = 0x02 - WORD LENGTH is 32, FRINV is not used
    CHANNEL_TWO_SLOT_START1 = 0x29 - 32
    CHANNEL_TWO_SLOT_START2 = 0x00
    CHANNEL_TWO_SLOT_LEN    = 0x20
    CHANNEL_TWO_CONFIG_BITS = 0x02
    BLOCK_CONFIG_BITS       = 0x87 - CLOCK&FRAME SLAVE, IDLELEVEL is low,DOUBLE CLK not used,PCM CLK not inverted, Frame signal MSB justified, Frame signal mode is early.
    FRAME_SIGNAL_LENGTH     = 0x01 - Normal FRAME MODE
    PORT_SELECTION          = 0x00 - PRIMARY PORT
    DISABLED_PCM_PIN_STATE  = 0x00 - ALL PINS ARE LOW
    RESERVED1               = 0x00
    RESERVED2               = 0x00
    */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x08,0xfc,0x00}},
#endif
    {OP_REPORT_SUCCESS, NOT_USED, {0}}
};

/* BT on sequence for K0 boot loader */
fw_config_seq_t fw_config_bt_on_k0_seq[] =
{
    {OP_REG_ASYNC_EVENT, INT_EVT_CALLBACK_REG, {0x00}}, /* register callback for async event */
    {OP_FIND_PATCH, NOT_USED, {0x00}},
    /* Change baud rate to 3M */
    {OP_RTS_STATE_CHANGE, RTS_STATE_ONE, {0x00}},    /* Set RTS state to zero */
    {OP_HCI_CMD, NOT_USED, {0x06, 0xfc, 0x01, 0x0b}},  /* Change the baud rate to 3M */
    {OP_HOST_BAUD_CHANGE, USERIAL_BAUD_3M, {0x00}},     /* Set host baud rate to 3M */
    {OP_HCI_EVT, IGNORE_EVT, {0x00}},
    {OP_HCI_CMD, NOT_USED, {0x0D, 0xfc, 0x00}}, /* Read Boot Parameters */
    {OP_HCI_EVT, IGNORE_EVT, {0x00}},
    {OP_PATCH_DOWNLOAD, NOT_USED, {0x00}},
    {OP_HCI_EVT, IGNORE_EVT, {0}},
    /* Change baud rate to 115200 */
    {OP_RTS_STATE_CHANGE, RTS_STATE_ONE, {0x00}},      /* Set RTS state to zero */
    {OP_HCI_CMD, NOT_USED, {0x06, 0xfc, 0x01, 0x04}}, /* Change the baud rate to 115200 */
    {OP_HOST_BAUD_CHANGE, USERIAL_BAUD_115200, {0x00}},     /* Set host baud rate to 115200 */
    {OP_HCI_EVT, IGNORE_EVT, {0x0e, 0x04, 0x01, 0x06, 0xfc, 0x00}},
    {OP_HCI_CMD, NOT_USED, {0x01, 0xfc, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x80, 0x00, 0x00}},  /* Intel Reset */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x04, 0x01, 0x01, 0xfc, 0x00}},
                                              /* Command complete for Intel Reset */
                                              /* next event will be HCI Reset CMP
                                                 which will go to stack and
                                                 stack will ignore it */
    {OP_HCI_EVT, VERIFY_EVT, {0xff, 0x01, 0x00}},       /* Intel start up event */
    {OP_HCI_CMD, NEXT_STATE_FW_DOWNLOADED, {0x11, 0xfc, 0x02, 0x01, 0x00}},  /* Manufacture mode on */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x11,0xfc,0x00}},
    {OP_WRITE_BDDATA, NOT_USED, {0x00}},    /* writing BD data from AUX file */
    {OP_WRITE_BTNVM, NOT_USED, {0x00}},      /* writing DDC commands from btnvm file */
    {OP_HCI_CMD, NOT_USED, {0x11, 0xfc, 0x02, 0x01, 0x00}}, /* Manufacture mode on */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x11,0xfc,0x00}},
    /* Change baud rate to 3M */
    {OP_RTS_STATE_CHANGE, RTS_STATE_ONE, {0x00}},      /* Set RTS state to zero */
    {OP_HCI_CMD, NOT_USED, {0x06, 0xfc, 0x01, 0x0b}}, /* Change the baud rate to 3M */
    {OP_HOST_BAUD_CHANGE, USERIAL_BAUD_3M, {0x00}},    /* Set host baud rate to 3M */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x04, 0x01, 0x06, 0xfc, 0x00}},
    {OP_HCI_CMD, NOT_USED, {0x11, 0xfc, 0x02, 0x00, 0x00}}, /* Manufacture mode off */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x11,0xfc,0x00}},
    {OP_HCI_CMD, NOT_USED, {0x03, 0x0c, 0x00}}, /* HCI Reset */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00}},
    {OP_HCI_EVT, VERIFY_EVT, {0xff, 0x01,0x00}},  /* Intel start up event */
    /* PCM SCO Configuration */
    {OP_HCI_CMD, NOT_USED, {0x08, 0xfc, 0x14, 0x02, 0x00,
                            0x00, 0x07, 0x20, 0x00, 0x00, 0x00, 0x20, 0x02, 0x20,
                            0x00, 0x00, 0x02, 0x87, 0x01, 0x00, 0x00, 0x00, 0x00}},
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x08,0xfc,0x00}},
    {OP_HCI_CMD, NOT_USED, {0x08, 0xfc, 0x14, 0x00, 0x00,
                            0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10,
                            0x00, 0x00, 0x00, 0x87, 0x01, 0x00, 0x00, 0x00, 0x00}},
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x08,0xfc,0x00}},
#if (A2DP_OFFLOAD_INCLUDED == TRUE)
    {OP_HCI_CMD, NOT_USED, {0x08, 0xfc, 0x14, 0x03, 0x00,
                            0x00, 0x07, 0x28, 0x00, 0x00, 0x00, 0x20, 0x02, 0x29,
                            0x00, 0x20, 0x02, 0x87, 0x01, 0x00, 0x00, 0x00, 0x00}},
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x08,0xfc,0x00}},
#endif
    {OP_REPORT_SUCCESS, NOT_USED, {0}}
};

/* BT on sequence for B0 boot loader */
fw_config_seq_t fw_config_bt_on_b0_seq[] =
{
    {OP_FIND_PATCH, NOT_USED, {0x00}},
    /* Change baud rate to 3M */
    {OP_RTS_STATE_CHANGE, RTS_STATE_ONE, {0x00}},    /* Set RTS state to zero */
    {OP_HCI_CMD, NOT_USED, {0x06, 0xfc, 0x01, 0x0b}},  /* Change the baud rate to 3M */
    {OP_HOST_BAUD_CHANGE, USERIAL_BAUD_3M, {0x00}},     /* Set host baud rate to 3M */
    {OP_HCI_EVT, IGNORE_EVT, {0x00}},

    {OP_HCI_CMD, NOT_USED, {0x0D, 0xfc, 0x00}}, /* Read Boot Parameters */
    {OP_HCI_EVT, IGNORE_EVT, {0x00}},
    {OP_PATCH_DOWNLOAD, NOT_USED, {0x00}},
    {OP_REG_ASYNC_EVENT, INT_EVT_CALLBACK_REG, {0x00}}, /* register callback for async event */
    {OP_HCI_EVT, VERIFY_EVT, {0xff, 0x05, 0x06, 0x00, 0x00, 0x00, 0x00}},
    /* Change host baud rate to 115200 */
    {OP_RTS_STATE_CHANGE, RTS_STATE_ONE, {0x00}},      /* Set RTS state to zero */
    {OP_HOST_BAUD_CHANGE, USERIAL_BAUD_115200, {0x00}},     /* Set host baud rate to 115200 */
    {OP_HCI_CMD, NOT_USED, {0x01, 0xfc, 0x08, 0x00, 0x00, 0x00,
                            0x01, 0x00, 0x08, 0x04, 0x00}},
                                                 /* Intel Reset */
                                                 /* next event will be HCI Reset CMP
                                                 which will go to stack and
                                                 stack will ignore it */
    {OP_REG_ASYNC_EVENT, INT_EVT_CALLBACK_REG, {0x00}}, /* unregister callback for async event */
    {OP_HCI_EVT, VERIFY_EVT, {0xff, 0x01, 0x00}},       /* Intel start up event */
    {OP_WRITE_BDDATA, NOT_USED, {0x00}},    /* writing BD data from AUX file */
    {OP_WRITE_BTNVM, NOT_USED, {0x00}},      /* writing DDC commands from btnvm file */
#if (LNP_LPM_ENABLED == TRUE)
    {OP_HCI_CMD, NOT_USED, {0x8b, 0xfc, 0x04, 0x03, 0x07, 0x01, 0x0b}},  /* DDC Command for LPM */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x06,0x01,0x8b,0xfc,0x00,0x00,0x00}},
#endif
    /* Change baud rate to 3M */
    {OP_RTS_STATE_CHANGE, RTS_STATE_ONE, {0x00}},      /* Set RTS state to zero */
    {OP_HCI_CMD, NOT_USED, {0x06, 0xfc, 0x01, 0x0b}}, /* Change the baud rate to 3M */
    {OP_HOST_BAUD_CHANGE, USERIAL_BAUD_3M, {0x00}},    /* Set host baud rate to 3M */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x04, 0x01, 0x06, 0xfc, 0x00}},
    {OP_HCI_CMD, NOT_USED, {0x03, 0x0c, 0x00}}, /* HCI Reset */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00}},
    {OP_HCI_EVT, VERIFY_EVT, {0xff, 0x01,0x00}},  /* Intel start up event */
    /* PCM SCO Configuration */
    {OP_HCI_CMD, NOT_USED, {0x08, 0xfc, 0x14, 0x02, 0x00,
                            0x00, 0x07, 0x20, 0x00, 0x00, 0x00, 0x20, 0x02, 0x20,
                            0x00, 0x00, 0x02, 0x87, 0x01, 0x00, 0x00, 0x00, 0x00}},
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x08,0xfc,0x00}},
    {OP_HCI_CMD, NOT_USED, {0x08, 0xfc, 0x14, 0x00, 0x00,
                            0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10,
                            0x00, 0x00, 0x00, 0x87, 0x01, 0x00, 0x00, 0x00, 0x00}},
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x08,0xfc,0x00}},
#if (A2DP_OFFLOAD_INCLUDED == TRUE)
    {OP_HCI_CMD, NOT_USED, {0x08, 0xfc, 0x14, 0x03, 0x00,
                            0x00, 0x07, 0x28, 0x00, 0x00, 0x00, 0x20, 0x02, 0x29,
                            0x00, 0x20, 0x02, 0x87, 0x01, 0x00, 0x00, 0x00, 0x00}},
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x08,0xfc,0x00}},
#endif
    {OP_REPORT_SUCCESS, NOT_USED, {0}}
};

fw_config_seq_t fw_config_bt_on_seq[] =
{
    {OP_HCI_CMD, NOT_USED, {0x05, 0xfc, 0x00}}, /* Read SW Version */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x0d, 0x01, 0x05, 0xfc, 0x00}}
};
#endif

#if (INTEL_AG6XX_UART == TRUE)
fw_config_seq_t fw_config_bt_on_seq[] =
{
    {OP_HCI_EVT, VERIFY_EVT, {0xff, 0x02, 0x05, 0x02}},
    {OP_HCI_CMD, NOT_USED, {0x11, 0xfc, 0x02, 0x01, 0x00}}, /* Manufacture mode on */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x11,0xfc,0x00}},
    {OP_WRITE_BDDATA, NOT_USED, {0x00}},  /* Open bd data file ->
                                             send write bd data->
                                             recv cmd status ->
                                             recv wr bd data cmpl evt */
    {OP_HCI_EVT, IGNORE_EVT, {0x00}},
    {OP_HCI_EVT, IGNORE_EVT, {0x00}},
    {OP_HCI_CMD, NOT_USED, {0x05, 0xfc, 0x00}}, /* Read SW Version */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x0d, 0x01, 0x05, 0xfc, 0x00}},
    {OP_FIND_PATCH, NOT_USED, {0x00}},
    {OP_PATCH_DOWNLOAD, NOT_USED, {0x00}},
    {OP_HCI_CMD, NEXT_STATE_FW_DOWNLOADED, {0x11, 0xfc, 0x02, 0x00, 0x02}}, /* Manufacture mode off + reset */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e,0x04,0x01,0x11,0xfc,0x00}},
    {OP_HCI_EVT, VERIFY_EVT, {0xff, 0x01, 0x00}},     /* Intel Startup event */
    {OP_REPORT_SUCCESS, NOT_USED, {0}}
};
#endif

#if (INTEL_WP2_USB == TRUE)
fw_config_seq_t fw_config_bt_off_seq[] =
{
    {OP_HCI_CMD, NOT_USED, {0x03, 0x0c, 0x00}}, /* HCI Reset */
    {OP_HCI_EVT, VERIFY_EVT, {0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00}},
    /* Report FW init success to stack */
    {OP_REPORT_SUCCESS, NOT_USED, {0}}
};
#endif

#if (INTEL_LNP_UART == TRUE)
fw_config_seq_t fw_config_bt_off_seq[] =
{
    {OP_REPORT_SUCCESS, NOT_USED, {0}}
};
#endif

#if (INTEL_AG6XX_UART == TRUE)
fw_config_seq_t fw_config_bt_off_seq[] =
{
    {OP_REPORT_SUCCESS, NOT_USED, {0}}
};
#endif

#if (INTEL_AG6XX_UART == TRUE)
/* LPM params */
static uint32_t idle_timeout                        = 300;
static uint32_t pkt_rate_monitor_period             = 100;
static uint32_t pkt_rate_monitor_threshold          = 10;
static uint8_t  pkt_rate_monitor_correction_factor  = 5;
static uint32_t wakeup_time                         = 300;

uint8_t signaling_is_enabled = TRUE;
uint8_t lpm_is_enabled = TRUE;
uint8_t fw_cfg_reg_value = 0xff;
#endif
#if (LNP_LPM_ENABLED == TRUE)
uint8_t lpm_is_enabled = FALSE;
#endif
uint32_t hw_operation_baud = 2000000;

/* h/w config control block */
typedef struct
{
    FILE*   fw_fd;                    /* FW patch file pointer */
    FILE*   aux_fd;                   /* AUX file pointer */
    FILE*   btnvm_fd;                 /* BTNVM file pointer */
    uint8_t patch_file_opened;
    uint8_t patch_file_present;
    uint8_t patch_download_done;
    uint8_t aux_file_opened;
    uint8_t btnvm_file_opened;
    uint8_t bddata_download_done;
    uint8_t btnvm_download_done;
    char patchfile[NAME_MAX];
    int config_state;
    fw_config_seq_t* fw_config_seq;
    uint8_t max_seq_number;
    uint8_t bt_state;
    uint32_t address;       /* Address in which data has to be written for pbn file*/
    uint32_t no_of_bytes;   /* No of bytes to be written field in pbn file */
    uint8_t sfi_state;
} bt_hw_cfg_cb_t;

/* low power mode parameters */
typedef struct
{
    uint8_t sleep_mode;                     /* 0(disable),1(UART),9(H5) */
    uint8_t host_stack_idle_threshold;      /* Unit scale 300ms/25ms */
    uint8_t host_controller_idle_threshold; /* Unit scale 300ms/25ms */
    uint8_t bt_wake_polarity;               /* 0=Active Low, 1= Active High */
    uint8_t host_wake_polarity;             /* 0=Active Low, 1= Active High */
    uint8_t allow_host_sleep_during_sco;
    uint8_t combine_sleep_mode_and_lpm;
    uint8_t enable_uart_txd_tri_state;      /* UART_TXD Tri-State */
    uint8_t sleep_guard_time;               /* sleep guard time in 12.5ms */
    uint8_t wakeup_guard_time;              /* wakeup guard time in 12.5ms */
    uint8_t txd_config;                     /* TXD is high in sleep state */
    uint8_t pulsed_host_wake;               /* pulsed host wake if mode = 1 */
} bt_lpm_param_t;

/* SFI format download structure */
struct sfi_css {
    /*** following fields must be contiguous ***/
    uint16_t op;           /* FC09 */
    uint8_t  len;          /* 128 + 1. */
    uint8_t  type;         /* 0 is css data */
    uint8_t  data[128];    /* the css data */
};
struct sfi_pka {
    /*** following fields must be contiguous ***/
    uint16_t op;           /* FC09 */
    uint8_t  len;          /* 252+1 */
    uint8_t  type;         /* 3 is pkey */
    uint8_t  data[252];    /* the pkey apart 1 */
};
struct sfi_pkb {
    /*** following fields must be contiguous ***/
    uint16_t op;           /* FC09 */
    uint8_t  len;          /* 4+1 */
    uint8_t  type;         /* 3 is pkey */
    uint8_t  data[4];      /* the pkey part 2 and modulus */
};
struct sfi_signa {
    /*** following fields must be contiguous ***/
    uint16_t op;           /* FC09 */
    uint8_t  len;          /* 252 + 1 */
    uint8_t  type;         /* 2 is sign */
    uint8_t  data[252];    /* the sign part 1 */
};
struct sfi_signb {
    /*** following fields must be contiguous ***/
    uint16_t op;           /* FC09 */
    uint8_t  len;          /* 4+1 */
    uint8_t  type;         /* 2 is sign */
    uint8_t  data[4];      /* the sign part 2 */
};
struct sfi_cmd {
    uint16_t hci_op;       /* hci command */
    uint8_t  hci_len;      /* length of command */
    uint8_t  data[249];    /* hci command data */
};
struct sfi_data {
    /*** following fields must be contiguous ***/
    uint16_t op;           /* FC09 */
    uint8_t  len;          /* varies + 1 */
    uint8_t  type;         /* 1 is data */
    uint8_t  data[252];    /* hci commands */
};

/******************************************************************************
**  Externs
******************************************************************************/
extern uint8_t vnd_local_bd_addr[6];

/******************************************************************************
**  Static variables and functions
******************************************************************************/
static char fw_patchfile_path[256] = FW_PATCHFILE_LOCATION;
static char fw_patchfile_name[128] = { 0 };
static char fw_patchfile_extension[5] = { 0 };
static char hw_bddata_location[128] = BDDATA_FILE;
static char hw_btnvm_location[128] = BTNVM_FILE;
static char hw_operation_baud_str[64] = "";
static bt_hw_cfg_cb_t hw_cfg_cb;

static bt_lpm_param_t lpm_param =
{
    LPM_SLEEP_MODE,
    LPM_IDLE_THRESHOLD,
    LPM_HC_IDLE_THRESHOLD,
    LPM_BT_WAKE_POLARITY,
    LPM_HOST_WAKE_POLARITY,
    LPM_ALLOW_HOST_SLEEP_DURING_SCO,
    LPM_COMBINE_SLEEP_MODE_AND_LPM,
    LPM_ENABLE_UART_TXD_TRI_STATE,
    0,  /* not applicable */
    0,  /* not applicable */
    0,  /* not applicable */
    LPM_PULSED_HOST_WAKE
};

tHW_CONFIG_OP_HANDLER *p_hw_config_op_handler = NULL;
uint8_t hw_config_seq_patch_download(void* buf);
uint8_t hw_config_pbn_patch_download(void* buf);
uint8_t hw_config_sfi_patch_download(void* buf);
uint8_t hw_config_aux_file_download(void* buf);
void hw_config_get_patch_file_name(uint8_t *p);
void command_handler(void* buf);
unsigned char char_to_hex(char c);

/******************************************************************************
**  Global functions
******************************************************************************/
/*******************************************************************************
**
** Function          register_int_evt_callbaks
**
** Description       Registers callback function to get internal async event.
**
** Returns           success/failure to register.
**
*******************************************************************************/
uint8_t register_int_evt_callback()
{
    /* Initialize the state */
    BTHWDBG("%s", __func__);
    return bt_vendor_cbacks->int_evt_callback_reg_cb(command_handler);
}

#if (INTEL_AG6XX_UART == TRUE)
/*******************************************************************************
**
** Function        hw_read_conf_value
**
** Description     Reads conf value to long int. If error then puts default value
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
static long hw_read_conf_value(char *str, long default_value)
{
    char *endptr;
    long ret;
    ret = strtol(str, &endptr , 10);
    if (str == endptr)
    {
        /* no value provided. Error? provide a default value */
        ret = default_value;
    }
    return ret;
}
#endif

/*******************************************************************************
**
** Function         open_bddata
**
** Description      Function to open the bddata file stored in
**                  /system/etc/bluetooth/bddata
**
** Returns          None
**
*******************************************************************************/
int open_bddata(uint8_t *ptr)
{
    FILE            *fp;
    unsigned int    i;
    int             ret;
    unsigned char   c = 0x00;
    uint8_t         x;
    uint8_t         *line = NULL;
    int             read;
    size_t          cmd_size;
    uint8_t         *p = ptr;

    BTHWDBG("%s hw_bddata_location:%s",__func__, hw_bddata_location);
    fp = fopen(hw_bddata_location, "rb");
    if (fp == NULL)
    {
        BTHWERR("cannot open:%s",hw_bddata_location);
        return FAILURE;
    }
    line = (uint8_t*) malloc (1024 * sizeof (uint8_t));
    if (line == NULL)
    {
        BTHWERR("Malloc failure");
        fclose(fp);
        return FAILURE;
    }
    read = fread(line, sizeof(uint8_t), 1024, fp);
    if (read < 0)
    {
        BTHWERR("line is not read properly. read:%d errno:%d strerror:%s"
                                                , read, errno, strerror(errno));
        free(line);
        fclose(fp);
        return FAILURE;
    }
    line[read] = '\0';
    cmd_size = read;
    for(i=0; i<cmd_size; i++)
    {
        if (i%2 == 1)
        {
            x = char_to_hex(line[i]);
            c |= x ;
            *p++ = c;
        }
        else
        {
            x = char_to_hex(line[i]);
            c = x << 4;
        }
    }
    if (line)
        free(line);
    fclose(fp);
#if (INTEL_LNP_UART == TRUE)
    /* Bluedroid stack provided the address */
    for (i=0;i<6;i++)
        ptr[i] = vnd_local_bd_addr[5-i];
#endif
    return SUCCESS;
}

/*******************************************************************************
**
** Function        line_speed_to_userial_baud
**
** Description     helper function converts line speed number into USERIAL baud
**                 rate symbol
**
** Returns         unit8_t (USERIAL baud symbol)
**
*******************************************************************************/
uint8_t line_speed_to_userial_baud(uint32_t line_speed)
{
    uint8_t baud;

    if (line_speed == 4000000)
        baud = USERIAL_BAUD_4M;
    else if (line_speed == 3000000)
        baud = USERIAL_BAUD_3M;
    else if (line_speed == 2000000)
        baud = USERIAL_BAUD_2M;
    else if (line_speed == 1000000)
        baud = USERIAL_BAUD_1M;
    else if (line_speed == 921600)
        baud = USERIAL_BAUD_921600;
    else if (line_speed == 460800)
        baud = USERIAL_BAUD_460800;
    else if (line_speed == 230400)
        baud = USERIAL_BAUD_230400;
    else if (line_speed == 115200)
        baud = USERIAL_BAUD_115200;
    else if (line_speed == 57600)
        baud = USERIAL_BAUD_57600;
    else if (line_speed == 19200)
        baud = USERIAL_BAUD_19200;
    else if (line_speed == 9600)
        baud = USERIAL_BAUD_9600;
    else if (line_speed == 1200)
        baud = USERIAL_BAUD_1200;
    else if (line_speed == 600)
        baud = USERIAL_BAUD_600;
    else
    {
        BTHWERR( "userial vendor: unsupported baud speed %d", line_speed);
        baud = USERIAL_BAUD_115200;
    }

    return baud;
}

/*******************************************************************************
**
** Function        ms_delay
**
** Description     sleep unconditionally for timeout milliseconds
**
** Returns         None
**
*******************************************************************************/
void ms_delay (uint32_t timeout)
{
    struct timespec delay;
    int err;

    if (timeout == 0)
        return;

    delay.tv_sec = timeout / 1000;
    delay.tv_nsec = 1000 * 1000 * (timeout%1000);

    /* [u]sleep can't be used because it uses SIGALRM */
    do {
        err = nanosleep(&delay, &delay);
    } while (err < 0 && errno ==EINTR);
}

/*******************************************************************************
**
** Function         hw_strncmp
**
** Description      Used to compare two strings in caseless
**
** Returns          0: match, otherwise: not match
**
*******************************************************************************/
static int hw_strncmp (const char *p_str1, const char *p_str2, const int len)
{
    int i;

    if (!p_str1 || !p_str2)
        return (1);

    for (i = 0; i < len; i++)
    {
        if (toupper(p_str1[i]) != toupper(p_str2[i]))
            return (i+1);
    }

    return 0;
}

/*******************************************************************************
**
** Function         char_to_hex
**
** Description      Convert char to hex
**
** Returns          hex value of the character
**
*******************************************************************************/
unsigned char char_to_hex(char c)
{
    volatile uint8_t x;
    char str[2];
    str[0] = c;
    str[1] = '\0';
    x = strtol(str, NULL, 16);
    return x;
}

/*******************************************************************************
**
** Function         form_byte
**
** Description      Convert input to a byte
**
** Returns          formed byte
**
*******************************************************************************/
unsigned char form_byte(char msb, char lsb)
{
    unsigned char byte;
    byte = (char_to_hex(msb)) << 4;
    byte |= char_to_hex(lsb);
    return byte;
}

/*******************************************************************************
**
** Function         form_word
**
** Description      Convert input to a word
**
** Returns          formed word
**
*******************************************************************************/
uint16_t form_word(uint8_t msb, uint8_t lsb)
{
    uint16_t byte;
    byte = msb << 8;
    byte |= lsb;
    return byte;
}

/*******************************************************************************
**
** Function         hex_print
**
** Description      Print buffer 'a' in hex
**
** Returns          None
**
*******************************************************************************/
static void hex_print(char* msg, unsigned char *a, unsigned int size)
{
    unsigned int i;
    if (a==NULL)
    {
        BTHWERR("%s nothing to print.", __func__);
        return;
    }
    char *p = (char*) malloc (sizeof (char) * ((size*3) + strlen(msg)+1));
    if (!p)
        return;
    bzero(p, size + strlen(msg)+1+(size));
    sprintf (p,"%s%s:", p, msg);
    for (i=0;i<size;i++)
        sprintf (p, "%s %X", p, a[i]);
    p[(size*3) + strlen(msg)] = '\0';
    BTHWDBG ("%s\n",p);
    if (p)
    {
        free(p);
        p = NULL;
    }
}

/*******************************************************************************
**
** Function        hw_host_baud_change
**
** Description     set host baud rate to 2M fixed (FIXME)
**
** Returns         1: if cmd sending is success
**
*******************************************************************************/
void* hw_host_baud_change(void* ptr)
{
    uint32_t host_baud_rate = *((uint32_t*) ptr);

    ms_delay(10);
    BTHWDBG("%s HW_SET_HOST_BAUD host_baud_rate:%d", __func__, host_baud_rate);

    /* change host baud rate. */
    userial_vendor_set_baud(host_baud_rate);

    /* Sets RTS line to receive event */
    BTHWDBG("SETTING RTS TO 0");
    userial_vendor_set_rts(RTS_STATE_ZERO);

    return NULL;
}

/*******************************************************************************
**
** Function        open_aux_file
**
** Description     Open the Auxiliary file.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t open_aux_file()
{
    BTHWDBG("%s hw_bddata_location:%s",__func__, hw_bddata_location);
    if(hw_cfg_cb.aux_fd == NULL)
    {
        hw_cfg_cb.aux_fd = fopen(hw_bddata_location, "rb");
        if(hw_cfg_cb.aux_fd != NULL) {
            return TRUE;
        }
    }
    return FALSE;
}

/*******************************************************************************
 **
** Function        open_btnvm_file
**
** Description     Open the btnvm file.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t open_btnvm_file()
{
    BTHWDBG("%s hw_btnvm_location:%s",__func__, hw_btnvm_location);
    if(hw_cfg_cb.btnvm_fd == NULL) {
        hw_cfg_cb.btnvm_fd = fopen(hw_btnvm_location, "r");
        if(hw_cfg_cb.btnvm_fd != NULL) {
            return TRUE;
        }
    }
    return FALSE;
}

/*******************************************************************************
**
** Function        hw_config_write_bd_address
**
** Description     Writes the BD Address.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t hw_config_write_bd_address()
{
    HC_BT_HDR   *p_buf = NULL;
    uint8_t     *p;
    uint8_t     i = 0;
    uint8_t     status = FALSE;
    uint16_t    cmd_u16 = HCI_CMD_PREAMBLE_SIZE + HCI_INTEL_BD_ADDRESS_LEN;

    BTHWDBG("%s",__func__);
    p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + cmd_u16);
    if(p_buf != NULL)
    {
        memset(p_buf, 0, BT_HC_HDR_SIZE + HCI_INTEL_BD_ADDRESS_LEN);
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + HCI_INTEL_BD_ADDRESS_LEN;
        p_buf->layer_specific = 0;

        p = (uint8_t*)(p_buf + 1);
        UINT16_TO_STREAM(p, HCI_INTEL_WRITE_BD_ADDRESS);
        *p++ = HCI_INTEL_BD_ADDRESS_LEN;
        for (i = 0; i < HCI_INTEL_BD_ADDRESS_LEN; i++) {
            *p++ = vnd_local_bd_addr[5-i];
        }
        status = bt_vendor_cbacks->xmit_cb(HCI_INTEL_WRITE_BD_ADDRESS,
                                                   p_buf, command_handler);
    }
    return status;
}

/*******************************************************************************
**
** Function        open_patch_file
**
** Description     Open firmware patch file.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t open_patch_file()
{
    BTHWDBG("%s",__func__);
    if(hw_cfg_cb.patch_file_present)
    {
        if(hw_cfg_cb.fw_fd == NULL)
        {
            hw_cfg_cb.fw_fd = fopen(hw_cfg_cb.patchfile, "r");
            if(hw_cfg_cb.fw_fd != NULL) {
                return TRUE;
            }
        }
    }
    return FALSE;
}

/*******************************************************************************
**
** Function        hw_config_seq_patch_download
**
** Description     Perform patch download from sequence file. Read the command
**                 from SFI file and send it to lower layer and verify the event.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t hw_config_seq_patch_download(void* buf)
{
    BTHWDBG("%s",__func__);
    if(!hw_cfg_cb.patch_file_opened)
    {
        if(!open_patch_file())
            return TRUE;
        hw_cfg_cb.patch_file_opened = TRUE;
    }

    uint8_t status = FALSE;
    char line[SEQ_FILE_MAX_LINE_SIZE];
    int pos;
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    HC_BT_HDR    *p_evt_buf = NULL;
    uint8_t     *p_evt;
    uint8_t *p_seq_evt = NULL;
    uint8_t second_consecutive_event = FALSE;
    FILE previous_fd;

    p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);
    memset(p_buf, 0, BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);

    p_seq_evt = (uint8_t *) bt_vendor_cbacks->alloc(HCI_EVT_MAX_LEN);
    memset(p_seq_evt, 0, HCI_EVT_MAX_LEN);

    if(p_buf != NULL)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = 0;
        p_buf->layer_specific = 0;
    }
    p = (uint8_t*)(p_buf + 1);

    p_evt_buf = (HC_BT_HDR*) buf;
    p_evt = (uint8_t*)(p_evt_buf+1);

    memset(line, 0, SEQ_FILE_MAX_LINE_SIZE);

    if(hw_cfg_cb.fw_fd != NULL)
    {
    if (!feof(hw_cfg_cb.fw_fd))
    {
        BTHWERR("file not empty");
        fgets(line, sizeof(line), hw_cfg_cb.fw_fd);

        while((line[0] == '*') || (line[0] == 0xd ) ||
                                    (line[0] == 'F') || (line[1] == '2'))
        {
            if (feof(hw_cfg_cb.fw_fd)) {
                BTHWDBG("End of file");
                if (hw_cfg_cb.fw_fd != NULL)
                {
                    fclose(hw_cfg_cb.fw_fd);
                    hw_cfg_cb.fw_fd = NULL;
                    hw_cfg_cb.patch_file_opened = FALSE;
                }
                status = TRUE;
                hw_cfg_cb.patch_download_done = TRUE;
                break;
            }
            if ((line[0] == '0') && (line[1] == '2'))
            {
            BTHWDBG("Event Length = %d\n",strlen(line));
            if (!second_consecutive_event)
                {
                previous_fd = *hw_cfg_cb.fw_fd;
                second_consecutive_event = TRUE;
                }
            else
                {
                *hw_cfg_cb.fw_fd = previous_fd;
                status = TRUE;
                break;
                }
            int length = 0;
            int cmp_length = 0;
            uint8_t *p_seq_evt1 = p_seq_evt;
            *p_seq_evt1 = form_byte(line[3], line[4]);
                p_seq_evt1++;
                length = strlen(line);
                cmp_length = (length - 8)/2;
            for(pos=0; pos < (length - 8); pos++){
                *p_seq_evt1 = form_byte(line[8+pos], line[9+pos]);
                p_seq_evt1++;
                pos++;
            }
                if (p_evt != NULL)
                {
                    if (!memcmp(p_seq_evt, p_evt, cmp_length))
                    {
                        status = TRUE;
                    }
                    else
                    {
                        status = FALSE;
                        break;
                    }
                }
            }

            fgets(line, sizeof(line), hw_cfg_cb.fw_fd);
        }
        if ((line[0] == '0') && (line[1] == '1')){
            int length = 0;
            int parameter_length = 0;
            uint8_t opcodeByte[2];
            uint16_t opcode = 0;
            opcodeByte[0] = form_byte(line[3], line[4]);
            opcodeByte[1] = form_byte(line[5], line[6]);
            opcode = form_word(opcodeByte[1], opcodeByte[0]);
            length = strlen(line);

            UINT16_TO_STREAM(p, opcode);
            parameter_length = form_byte(line[8], line[9]);
            *p = parameter_length;
            p++;
            p_buf->len = HCI_CMD_PREAMBLE_SIZE + parameter_length;
            BTHWDBG("Length =%X, %d\n", parameter_length, length);

            for(pos=0; pos < (length - 10); pos++){
                *p = form_byte(line[10+pos], line[11+pos]);
                p++;
                pos++;
            }

            if(opcode == HCI_INTEL_INF_BDDATA)
            {
                status = bt_vendor_cbacks->xmit_cb(opcode,
                                    p_buf, command_handler);
            }
            else
            {
                status = bt_vendor_cbacks->xmit_cb(opcode,
                                    p_buf, command_handler);
            }
        }
    }
    }
    if (bt_vendor_cbacks)
    {
        if (p_seq_evt != NULL)
        {
            bt_vendor_cbacks->dealloc(p_seq_evt);
            p_seq_evt = NULL;
        }
    }

    return status;
}

/*******************************************************************************
**
** Function        hw_config_pbn_patch_download
**
** Description     Perform patch download from PBN file. Read the command from
**                 PBN file and send it to lower layer.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t hw_config_pbn_patch_download(void * buf)
{
    BTHWDBG("%s",__func__);
    if(!hw_cfg_cb.patch_file_opened)
    {
        if(!open_patch_file())
            return TRUE;
        hw_cfg_cb.patch_file_opened = TRUE;
    }

    int         read;
    uint8_t*    pData = NULL;
    uint8_t status = FALSE;
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;

    p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);
    memset(p_buf, 0, BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);

    if(p_buf != NULL)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = 0;
        p_buf->layer_specific = 0;
    }
    p = (uint8_t*)(p_buf + 1);
    if(hw_cfg_cb.no_of_bytes <= 0)
    {
        //Read the address to patch
        read = fread(&hw_cfg_cb.address, 1,sizeof(uint32_t), hw_cfg_cb.fw_fd);
        if (!read)
        {
            //This should never happen
            BTHWERR("Read Fw patch file failed.");
            hw_cfg_cb.no_of_bytes = 0;
            status = FALSE;
        }
        else if (hw_cfg_cb.address == 0xFFFFFFFF)
        {
            //FW patch download DONE.
            BTHWDBG("FW patch download DONE.");
            hw_cfg_cb.patch_download_done = TRUE;
            hw_cfg_cb.no_of_bytes = 0;
            status = TRUE;
        }
        else
        {
            BTHWDBG("Address read:0x%x",hw_cfg_cb.address);
            //Read the length of the patch
            read = fread(&hw_cfg_cb.no_of_bytes, 1 , sizeof(uint32_t), hw_cfg_cb.fw_fd);
            if (!read)
            {
                BTHWERR("Read Fw patch file failed.");
                hw_cfg_cb.no_of_bytes = 0;
                status = FALSE;
            }
        }
    }
    if (hw_cfg_cb.no_of_bytes > 0)
    {
        uint8_t data_length = (hw_cfg_cb.no_of_bytes > PATCH_MAX_LENGTH) ?PATCH_MAX_LENGTH:hw_cfg_cb.no_of_bytes;
        pData = (unsigned char *) malloc(data_length);
        read = fread(pData, data_length, sizeof(unsigned char), hw_cfg_cb.fw_fd);
        if (!read)
        {
            BTHWERR("Read Fw patch file failed.");
            status = FALSE;
        }
        else
        {
            UINT16_TO_STREAM(p, HCI_INTEL_INF_MEM_WRITE);
            /* parameter length (address size + mode size + length + data size)*/
            uint8_t param_length =      4 + 1 + 1 + data_length;
            *p++ = param_length;
            *p++ = (hw_cfg_cb.address & 0x000000FF) >> 0;
            *p++ = (hw_cfg_cb.address & 0x0000FF00) >> 8;
            *p++ = (hw_cfg_cb.address & 0x00FF0000) >> 16;
            *p++ = (hw_cfg_cb.address & 0xFF000000) >> 24;
            BTHWDBG("Address of p:%p",p);
            *p++ = HCI_INTEL_MEM_WRITE_MODE_BYTE;
            *p++ = data_length; //Length of the data buffer
            memcpy (p, pData, data_length);
            if (pData!=NULL)
            {
                free(pData);
            }
            //BTHWDBG("param_length:%u", param_length);
            p_buf->len = HCI_CMD_PREAMBLE_SIZE +
                         param_length;
            hw_cfg_cb.no_of_bytes -= data_length;
            hw_cfg_cb.address += data_length;
            /****************************************************************/
#if (BTHW_DBG == TRUE)
            BTHWDBG("p_buf->len:%u", p_buf->len);
            hex_print("Before sending. CMD SENT", (uint8_t*)(p_buf+1), p_buf->len);
#endif
            /****************************************************************/
            status = bt_vendor_cbacks->xmit_cb(HCI_INTEL_INF_MEM_WRITE,
                                                p_buf, command_handler);
        }
    }
    return status;
}

/*******************************************************************************
**
** Function        hw_config_aux_file_download
**
** Description     Read commands from AUX file and send to lower layer.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t hw_config_aux_file_download(void * buf)
{
    BTHWDBG("%s",__func__);
    if(!hw_cfg_cb.aux_file_opened)
    {
        if(!open_aux_file())
            return TRUE;
        hw_cfg_cb.aux_file_opened = TRUE;
    }

    int         read;
    uint8_t     type;
    uint8_t     opcodebyte[2];
    uint16_t    opcode;
    uint8_t     length;
    uint8_t     event;
    uint8_t     *pData = NULL;
    uint8_t     status = FALSE;
    HC_BT_HDR   *p_buf = NULL;
    uint8_t     *p;
    uint8_t      i = 0;
    uint8_t second_consecutive_event = FALSE;
    FILE previous_fd;

    p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);
    memset(p_buf, 0, BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);

    if(p_buf != NULL)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = 0;
        p_buf->layer_specific = 0;
    }
    p = (uint8_t*)(p_buf + 1);

    read = fread(&type, 1, sizeof(type), hw_cfg_cb.aux_fd);
    if (read == 0)         //if read is 0 then its either end of file or read failure
    {
        BTHWDBG("End of file.");
        fclose(hw_cfg_cb.aux_fd);
        status = TRUE;
        hw_cfg_cb.aux_fd = NULL;
        hw_cfg_cb.bddata_download_done = TRUE;
        hw_cfg_cb.aux_file_opened = FALSE;
    }

    while(type == HCI_MSG_TYPE_EVT)    //check for event
    {
        if (second_consecutive_event)
        {
            *hw_cfg_cb.aux_fd = previous_fd;
            status = TRUE;
            break;
        }

        BTHWDBG("Ignoring as of now");
        read = fread(&event, 1, sizeof(event), hw_cfg_cb.aux_fd);
        read = fread(&length, 1, sizeof(length), hw_cfg_cb.aux_fd);
        pData = (uint8_t*) bt_vendor_cbacks->alloc(length);
        memset(pData, 0, length);
        read = fread(pData, 1, length, hw_cfg_cb.aux_fd);
        if(pData != NULL)
        {
            bt_vendor_cbacks->dealloc(pData);
            pData = NULL;
        }

        if (!second_consecutive_event)
        {
            previous_fd = *hw_cfg_cb.aux_fd;
            second_consecutive_event = TRUE;
        }
        type = 0;
        read = fread(&type, 1, sizeof(type), hw_cfg_cb.aux_fd);
        if (read == 0)
        {
            BTHWDBG("End of file.");

            /* Write the BD Address with Hci_Intel_Write_BD_Address cmd*/
            status = hw_config_write_bd_address();
            break;
        }
    }

    if(type == HCI_MSG_TYPE_CMD)    //check for command
    {
        read = fread(opcodebyte, 1,sizeof(opcodebyte), hw_cfg_cb.aux_fd);    //Read the opcode
        *p++ = opcodebyte[0];
        *p++ = opcodebyte[1];
        opcode = form_word(opcodebyte[1], opcodebyte[0]);
        read = fread(&length, 1,sizeof(type), hw_cfg_cb.aux_fd);    //Read the param length
        *p++ = length;
        if(length > 0)
        {
            pData = (uint8_t*) bt_vendor_cbacks->alloc(length);
            memset(pData, 0, length);
            read = fread(pData, 1, length, hw_cfg_cb.aux_fd);    //Read the data
            memcpy(p, pData, length);
            if(pData != NULL)
            {
                bt_vendor_cbacks->dealloc(pData);
                pData = NULL;
            }
        }

        p_buf->len = HCI_CMD_PREAMBLE_SIZE + length;
        hex_print("BDDATA Buffer:", (uint8_t*)(p_buf+1), p_buf->len);

        if(opcode == HCI_INTEL_INF_BDDATA)
        {
            status = bt_vendor_cbacks->xmit_cb(opcode,
                                p_buf, command_handler);
        }
        else
        {
            status = bt_vendor_cbacks->xmit_cb(opcode,
                                p_buf, command_handler);
        }
    }

    return status;
}

/*******************************************************************************
 **
** Function        hw_config_op_write_btnvm_handler
**
** Description     Read the ddc commands from btnvm file and send it to lower layer.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t hw_config_op_write_btnvm_handler()
{
    BTHWDBG("%s",__func__);
    if (!hw_cfg_cb.btnvm_file_opened) {
        if(!open_btnvm_file()) {
            hw_cfg_cb.btnvm_download_done = TRUE;
            return TRUE;
    }
        hw_cfg_cb.btnvm_file_opened = TRUE;
    }

    uint8_t    *pData = NULL;
    uint8_t    status = FALSE;
    HC_BT_HDR  *p_buf = NULL;
    uint8_t    *p;
    uint8_t    i = 0;
    FILE       previous_fd;

    uint16_t   id;
    uint8_t    size;
    uint8_t    value[DDC_CMD_PARAM_MAX_LEN];
    uint8_t    ddc_pkt_size = 0;
    uint8_t    hci_param_len;
    uint8_t    read;
    uint8_t    c = 0x00;
    uint8_t    x;

    p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);
    memset(p_buf, 0, BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);

    if (p_buf != NULL) {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = 0;
        p_buf->layer_specific = 0;
    }
    p = (uint8_t*) (p_buf + 1);
    UINT16_TO_STREAM(p, HCI_INTEL_DDC_CONFIG_WRITE);

    hci_param_len = 0;
    pData = p;
    pData++;

    do {
    read = fscanf(hw_cfg_cb.btnvm_fd, "%hx,%hhx,%s", &id, &size, value);
    if (read != DDC_NUMBER_OF_FIELDS_PER_CMD) { /* -1:end of file; other than 3 read elements:read failure */
        BTHWDBG("btnvm End of file.");
        if (hci_param_len)
        break;
        else {
        fclose(hw_cfg_cb.btnvm_fd);
        status = TRUE;
        hw_cfg_cb.btnvm_fd = NULL;
        hw_cfg_cb.btnvm_download_done = TRUE;
        hw_cfg_cb.btnvm_file_opened = FALSE;
        return status;
     }
    }
    ddc_pkt_size = size + DDC_PKT_HDR_SIZE;

    if (HCI_CMD_MAX_PARAM_SIZE < (hci_param_len + ddc_pkt_size)) {
        *hw_cfg_cb.btnvm_fd = previous_fd;
        status = TRUE;
        break;
    } else {
        hci_param_len += ddc_pkt_size;
        previous_fd = *hw_cfg_cb.btnvm_fd;

        *pData++ = (size + 2); /* adding size of 'id' */
        UINT16_TO_STREAM(pData, id);

        for (i = 0; i < (size * 2); i++) {
        if (i%2 == 1) {
            x = char_to_hex(value[i]);
            c |= x ;
            *pData++ = c;
        } else {
            x = char_to_hex(value[i]);
            c = x << 4;
        }
        }
    }
    }while (1);

    *p++ = hci_param_len;   /* parameter length */
    pData = NULL;

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + hci_param_len;
    hex_print("BTNVM Buffer:", (uint8_t*)(p_buf+1), p_buf->len);

    status = bt_vendor_cbacks->xmit_cb(HCI_INTEL_DDC_CONFIG_WRITE,
                           p_buf, command_handler);
    return status;
}

/*******************************************************************************
**
** Function        hw_config_sfi_patch_download
**
** Description     Perform secure patch download. Read the command from SFI
**                 file and send it to lower layer.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t hw_config_sfi_patch_download(void * buf)
{
    BTHWDBG("%s",__func__);
    if(!hw_cfg_cb.patch_file_opened)
    {
        if(!open_patch_file())
            return TRUE;
        hw_cfg_cb.patch_file_opened = TRUE;
    }

    HC_BT_HDR   *p_buf = NULL;
    uint16_t opcode = 0;
    uint8_t* p;
    uint8_t status = FALSE;
    static int in_flight;


    if (bt_vendor_cbacks != NULL)
    {
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE +
                                                       HCI_CMD_MAX_LEN);
    }

    if (p_buf != NULL)
    {
        memset(p_buf, 0, BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = 0;
        p_buf->layer_specific = 0;

        p = (uint8_t *) (p_buf + 1);

        switch(hw_cfg_cb.sfi_state)
        {
            case HW_CFG_SEND_CSS: {
                struct sfi_css *sfi_css = (struct sfi_css *)p;
                int read;

                /* prepare the css block with content */
                sfi_css->op = HCI_INTEL_SSEND;
                sfi_css->len = 1 + sizeof(sfi_css->data);
                sfi_css->type = HCI_INTEL_INIT_TYPE;
                read = fread(&sfi_css->data, 1, sizeof(sfi_css->data),
                        hw_cfg_cb.fw_fd);
                if (read != sizeof(sfi_css->data)) {
                    BTHWDBG("Failed to read the css, read %d bytes", read);
                    status = FALSE;
                    break;
                }
                p_buf->len = HCI_CMD_PREAMBLE_SIZE + sfi_css->len;
                hw_cfg_cb.sfi_state= HW_CFG_SEND_PKEY1;
                BTHWDBG("HW_CFG_SEND_CSS");
                status = bt_vendor_cbacks->xmit_cb(
                                   HCI_INTEL_SSEND,
                                   p_buf, command_handler);
                break;
            }
            case HW_CFG_SEND_PKEY1: {
                struct sfi_pka *sfi_pka = (struct sfi_pka *)p;
                int read;

                /* prepare the pka block with content */
                sfi_pka->op = HCI_INTEL_SSEND;
                sfi_pka->len = 1 + sizeof(sfi_pka->data);
                sfi_pka->type = HCI_INTEL_PKEY_TYPE;
                read = fread(&sfi_pka->data, 1, sizeof(sfi_pka->data),
                               hw_cfg_cb.fw_fd);
                if (read != sizeof(sfi_pka->data)) {
                    BTHWDBG("Failed to read the pka, read %d bytes", read);
                    status = FALSE;
                    break;
                }
                p_buf->len = HCI_CMD_PREAMBLE_SIZE + sfi_pka->len;
                hw_cfg_cb.sfi_state = HW_CFG_SEND_PKEY2;
                BTHWDBG("HW_CFG_SEND_PKA");
                status = bt_vendor_cbacks->xmit_cb(
                                   HCI_INTEL_SSEND,
                                   p_buf, command_handler);
                break;
            }
            case HW_CFG_SEND_PKEY2: {
                struct sfi_pkb *sfi_pkb = (struct sfi_pkb *)p;
                int read;

                /* prepare the pkb block with content */
                sfi_pkb->op = HCI_INTEL_SSEND;
                sfi_pkb->len = 1 + sizeof(sfi_pkb->data);
                sfi_pkb->type = HCI_INTEL_PKEY_TYPE;
                read = fread(&sfi_pkb->data, 1, sizeof(sfi_pkb->data),
                         hw_cfg_cb.fw_fd);
                if (read != sizeof(sfi_pkb->data)) {
                    BTHWDBG("Failed to read the pkb, read %d bytes", read);
                    status = FALSE;
                    break;
                }
                p_buf->len = HCI_CMD_PREAMBLE_SIZE + sfi_pkb->len;
                hw_cfg_cb.sfi_state= HW_CFG_SEND_SIGN1;
                BTHWDBG("HW_CFG_SEND_PKB");
                status = bt_vendor_cbacks->xmit_cb(
                                    HCI_INTEL_SSEND,
                                    p_buf, command_handler);
                break;
            }
            case HW_CFG_SEND_SIGN1: {
                struct sfi_signa *sfi_signa = (struct sfi_signa *)p;
                int read;
                uint8_t modulus[4];

                /* Ignoring 4 extra modulus bytes in SFI file
                   which we should send to controller */
                read = fread(modulus, 1, sizeof(modulus), hw_cfg_cb.fw_fd);

                /* prepare the signa block with content */
                sfi_signa->op = HCI_INTEL_SSEND;
                sfi_signa->len = 1 + sizeof(sfi_signa->data);
                sfi_signa->type = HCI_INTEL_SIGN_TYPE;

                read = fread(&sfi_signa->data, 1, sizeof(sfi_signa->data),
                         hw_cfg_cb.fw_fd);
                if (read != sizeof(sfi_signa->data)) {
                    BTHWDBG("Failed to read the signa, read %d bytes", read);
                    status = FALSE;
                    break;
                }
                p_buf->len = HCI_CMD_PREAMBLE_SIZE + sfi_signa->len;
                hw_cfg_cb.sfi_state = HW_CFG_SEND_SIGN2;
                BTHWDBG("HW_CFG_SEND_SIGN1");
                status = bt_vendor_cbacks->xmit_cb(
                                    HCI_INTEL_SSEND,
                                    p_buf, command_handler);
                break;
            }
            case HW_CFG_SEND_SIGN2: {
                struct sfi_signb *sfi_signb = (struct sfi_signb *)p;
                int read;

                /* prepare the signb block with content */
                sfi_signb->op = HCI_INTEL_SSEND;
                sfi_signb->len = 1 + sizeof(sfi_signb->data);
                sfi_signb->type = HCI_INTEL_SIGN_TYPE;
                read = fread(&sfi_signb->data, 1, sizeof(sfi_signb->data),
                         hw_cfg_cb.fw_fd);
                if (read != sizeof(sfi_signb->data)) {
                    BTHWDBG("Failed to read the signb, read %d bytes", read);
                    status = FALSE;
                    break;
                }
                p_buf->len = HCI_CMD_PREAMBLE_SIZE + sfi_signb->len;
                hw_cfg_cb.sfi_state = HW_CFG_SEND_DATA;
                BTHWDBG("HW_CFG_SEND_SIGN2");
                in_flight = 1;
                status = bt_vendor_cbacks->xmit_cb(
                                    HCI_INTEL_SSEND,
                                    p_buf, command_handler);
                break;
            }
            case HW_CFG_SEND_DATA: {
                struct sfi_data *sfi_data;
                int read;
                int offset = 0;

                in_flight--;

                do {
                    if (p_buf == NULL) {
                        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(
                                            BT_HC_HDR_SIZE +
                                            HCI_CMD_MAX_LEN);
                        if (p_buf == NULL) {
                            BTHWDBG("Memory allocation returned NULL.");
                            break;
                        }
                        memset(p_buf, 0, BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);
                        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
                        p_buf->offset = 0;
                        p_buf->len = 0;
                        p_buf->layer_specific = 0;

                        p = (uint8_t *) (p_buf + 1);
                    }

                    sfi_data = (struct sfi_data *)p;
                    /* prepare the data block with content */
                    sfi_data->op = HCI_INTEL_SSEND;
                    sfi_data->type = HCI_INTEL_DATA_TYPE;
                    offset = 0;

                    sfi_data->len = 1; //type of secure send write command
                    p_buf->len = HCI_CMD_PREAMBLE_SIZE;
                    do {
                        struct sfi_cmd cmd;
                        read = fread(&cmd.hci_op, 1, HCI_CMD_PREAMBLE_SIZE,
                        hw_cfg_cb.fw_fd);
                        if (read == 0 && feof(hw_cfg_cb.fw_fd)) {
                            BTHWDBG("End of File");
                            break;
                        } else if (read != HCI_CMD_PREAMBLE_SIZE) {
                            BTHWDBG("Failed to read the data, read %d bytes", read);
                            status = FALSE;
                            break;
                        }
                        if (cmd.hci_len > sizeof(cmd.data)) {
                            BTHWDBG("Read a command with too much data (%d).", cmd.hci_len);
                            status = FALSE;
                            break;
                            /*
                            * using the length read above, read in the rest of the data
                            * associated with this opcode (may be zero)
                            */
                        } else if (cmd.hci_len) {
                            read = fread(&cmd.data, 1, cmd.hci_len,
                            hw_cfg_cb.fw_fd);
                            if (read != cmd.hci_len) {
                                BTHWDBG("Failed to read the data, read %d bytes", read);
                                status = FALSE;
                                break;
                            }
                        }

                        memcpy(&sfi_data->data[offset], &cmd, cmd.hci_len + HCI_CMD_PREAMBLE_SIZE);
                        offset += cmd.hci_len + HCI_CMD_PREAMBLE_SIZE;
                        sfi_data->len += cmd.hci_len + HCI_CMD_PREAMBLE_SIZE;
                        p_buf->len = sfi_data->len + HCI_CMD_PREAMBLE_SIZE;

                    } while((p_buf->len % 4) != 0); /* Total length of Secure send write command
                            should be multiple of 4 */

                    if(p_buf->len == HCI_CMD_PREAMBLE_SIZE)
                    {
                        // handling end of file here
                        status = TRUE;
                        if(in_flight == 0) {
                            hw_cfg_cb.patch_download_done = TRUE;
                            fclose(hw_cfg_cb.fw_fd);
                        }
                        bt_vendor_cbacks->dealloc(p_buf);
                        p_buf = NULL;
                        break;
                    }
                    BTHWDBG("HW_CFG_SEND_DATA");
                    BTHWDBG("p_buf->len:%u", p_buf->len);
                    hex_print("Before sending. CMD SENT", (uint8_t*)(p_buf+1), p_buf->len);
                    in_flight ++;
                    BTHWDBG("DONF  in_flight is now %d", in_flight);

                    status = bt_vendor_cbacks->xmit_cb(
                                        HCI_INTEL_SSEND,
                                        p_buf, command_handler);
                    p_buf  = NULL;
                }while(in_flight < INT_CMD_PKT_MAX_COUNT);
                break;
            }
            default:
                break;
        }
    }
    return status;
}

/*******************************************************************************
**
** Function        hw_config_op_patch_download_handler
**
** Description     Perform firmware patch download operation.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t hw_config_op_patch_download_handler(void * buf)
{
    uint8_t status = FALSE;

#if (INTEL_WP2_USB == TRUE)
    status = hw_config_seq_patch_download(buf);
#endif

#if (INTEL_AG6XX_UART == TRUE)
    status = hw_config_pbn_patch_download(buf);
#endif

#if (INTEL_LNP_UART == TRUE)
    if(hw_strncmp(".seq", fw_patchfile_extension,
    FW_PATCHFILE_EXTENSION_LEN) == 0)
    {
        status = hw_config_seq_patch_download(buf);
    }
    else if(hw_strncmp(".sfi", fw_patchfile_extension,
    FW_PATCHFILE_EXTENSION_LEN) == 0)
    {
        status = hw_config_sfi_patch_download(buf);
    }
#endif

    return status;
}

/*******************************************************************************
**
** Function        hw_config_op_find_patch_handler
**
** Description     Find firmware patch file.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t hw_config_op_find_patch_handler(char *p_chip_id_str)
{
    DIR *dirp;
    struct dirent *dp;
    int filenamelen;
    uint8_t retval = FALSE;

    BTHWDBG("Target name = [%s]", p_chip_id_str);

    if (strlen(fw_patchfile_name)> 0)
    {
        /* If specific filepath and filename have been given in run-time
         * configuration /etc/bluetooth/bt_vendor.conf file, we will use them
         * to concatenate the filename to open rather than searching a file
         * matching to chipset name in the fw_patchfile_path folder.
         */
        snprintf(p_chip_id_str, strlen(fw_patchfile_path), "%s",
                                                            fw_patchfile_path);
        if (fw_patchfile_path[strlen(fw_patchfile_path)- 1] != '/')
        {
            strncat(p_chip_id_str, "/", 1);
        }
        strncat(p_chip_id_str, fw_patchfile_name, strlen(fw_patchfile_name));

        BTHWDBG("FW patchfile: %s", p_chip_id_str);
        return TRUE;
    }

    if ((dirp = opendir(fw_patchfile_path)) != NULL)
    {
        /* Fetch next filename in patchfile directory */
        while ((dp = readdir(dirp)) != NULL)
        {
            /* Check if filename starts with chip-id name */
            if ((hw_strncmp(dp->d_name, p_chip_id_str, strlen(p_chip_id_str))
                ) == 0)
            {
                /* Check if it has .seq extenstion */
                filenamelen = strlen(dp->d_name);
                if ((filenamelen >= FW_PATCHFILE_EXTENSION_LEN) &&
                    ((hw_strncmp(
                          &dp->d_name[filenamelen-FW_PATCHFILE_EXTENSION_LEN],
                          fw_patchfile_extension,
                          FW_PATCHFILE_EXTENSION_LEN)
                     ) == 0))
                {
                    BTHWDBG("Found patchfile: %s/%s",
                        fw_patchfile_path, dp->d_name);

                    /* Make sure length does not exceed maximum */
                    if ((filenamelen + strlen(fw_patchfile_path)) >
                        (PATH_MAX - 2))
                    {
                        BTHWERR("Invalid patchfile name (too long)");
                    }
                    else
                    {
                        memset(p_chip_id_str, 0, NAME_MAX);
                        /* Found patchfile. Store location and name */
                        strncpy(p_chip_id_str, fw_patchfile_path,
                                                    strlen(fw_patchfile_path));
                        if (fw_patchfile_path[
                            strlen(fw_patchfile_path)- 1
                            ] != '/')
                        {
                            strncat(p_chip_id_str, "/", 1);
                        }
                        strncat(p_chip_id_str, dp->d_name, strlen(dp->d_name));
                        retval = TRUE;
                    }
                    break;
                }
            }
        }

        closedir(dirp);

        if (retval == FALSE)
        {
            BTHWERR("Could not find patchfile %s at %s",
                                              p_chip_id_str, fw_patchfile_path);
        }
    }
    else
    {
        BTHWERR("Could not open %s", fw_patchfile_path);
    }

    return (retval);
}

/*******************************************************************************
**
** Function        hw_config_op_write_bddata_handler
**
** Description     Read the bddata from file and send it to lower layer.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t hw_config_op_write_bddata_handler()
{
    BTHWDBG("%s", __func__);
    uint8_t status = FALSE;

#if (USE_AUX_FILE == TRUE)
    status = hw_config_aux_file_download(NULL);
#else
    HC_BT_HDR   *p_buf = NULL;
    uint8_t* p;

    /* Construct the command buffer */
    p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);
    memset(p_buf, 0, BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN);
    if(p_buf != NULL)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = 0;
        p_buf->layer_specific = 0;
    }

    p = (uint8_t *) (p_buf + 1);
    UINT16_TO_STREAM(p, HCI_INTEL_INF_BDDATA);
    *p++ = HCI_INTEL_INF_BDDATA_PARAM_SIZE; /* parameter length */

    int ret = open_bddata(p);
    if (ret == SUCCESS)
    {
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + HCI_INTEL_INF_BDDATA_PARAM_SIZE;
        hex_print("BDDATA Buffer:", (uint8_t*)(p_buf+1), p_buf->len);
        status = bt_vendor_cbacks->xmit_cb(HCI_INTEL_INF_BDDATA,
                            p_buf, command_handler);
    }
    hw_cfg_cb.bddata_download_done = TRUE;
#endif

    return status;
}

/*******************************************************************************
**
** Function        hw_config_op_rts_state_change_handler
**
** Description     Handle the RTS state change in host.
**
** Returns         None
**
*******************************************************************************/
void hw_config_op_rts_state_change_handler(int state)
{
    BTHWDBG("%s - set RTS to state %d", __func__, state);
    userial_vendor_set_rts(state);
    return;
}

/*******************************************************************************
**
** Function        hw_config_op_host_baud_change_handler
**
** Description     Change the host baud rate.
**
** Returns         None
**
*******************************************************************************/
void hw_config_op_host_baud_change_handler(uint8_t* host_baud_rate)
{
    BTHWDBG("%s", __func__);
    pthread_t thread1;
    pthread_create( &thread1, NULL, hw_host_baud_change, host_baud_rate);
}

/*******************************************************************************
**
** Function        hw_config_op_reg_async_event_handler
**
** Description     Register to H4 layer to receive asynchronous event coming
**                 from controller
**
** Returns         None
**
*******************************************************************************/
void hw_config_op_reg_async_event_handler(uint8_t state)
{
    BTHWDBG("%s", __func__);
    if(bt_vendor_cbacks != NULL)
    {
        if(state == INT_EVT_CALLBACK_REG)
            bt_vendor_cbacks->int_evt_callback_reg_cb(command_handler);
    }
}

/*******************************************************************************
**
** Function        hw_config_op_report_failure_handler
**
** Description     Report the failure of HW configuration to caller.
**
** Returns         None
**
*******************************************************************************/
void hw_config_op_report_failure_handler()
{
    BTHWDBG("%s", __func__);
    if(hw_cfg_cb.bt_state == BT_TURNING_ON)
    {
#if (INTEL_LNP_UART == TRUE)
#if (BT_EN_VIA_USERIAL_IOCTL == TRUE)
        unsigned long fw_cfg_result;
        fw_cfg_result = FW_FAILED;
        userial_vendor_ioctl(USERIAL_OP_FW_CFG_CMPL, (void*)&fw_cfg_result);
#endif
#endif
        //Report the fw download failure
        bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
    }
#if (HW_END_WITH_HCI_RESET == TRUE)
    else
        bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_FAIL);
#endif
}

/*******************************************************************************
**
** Function        hw_config_op_report_success_handler
**
** Description     Report the success of HW configuration to caller.
**
** Returns         None
**
*******************************************************************************/
void hw_config_op_report_success_handler()
{
    BTHWDBG("%s", __func__);
    if(hw_cfg_cb.bt_state == BT_TURNING_ON)
    {
#if (INTEL_LNP_UART == TRUE)
#if (BT_EN_VIA_USERIAL_IOCTL == TRUE)
        unsigned long fw_cfg_result;
        fw_cfg_result = FW_SUCCESS;
        userial_vendor_ioctl(USERIAL_OP_FW_CFG_CMPL, (void*)&fw_cfg_result);
#endif
#endif
        //Report the fw download success
        bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
    }
#if (HW_END_WITH_HCI_RESET == TRUE)
    else
        bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
#endif
}

/*******************************************************************************
**
** Function        hw_config_op_hci_cmd_handler
**
** Description     Handle the HCI command of HW configuration sequence.
**                 It sends the command to lower layer.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t hw_config_op_hci_cmd_handler(fw_config_seq_t* config_seq)
{
    HC_BT_HDR   *p_buf = NULL;
    uint16_t opcode = 0;
    uint8_t* p = (uint8_t*)config_seq->buffer;
    uint8_t status = FALSE;

    BTHWDBG("%s", __func__);
    /* Construct the command buffer */
    p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + p[HCI_CMD_LEN_OFFSET] + HCI_CMD_PREAMBLE_SIZE);
    if(p_buf != NULL)
    {
        memset(p_buf, 0, BT_HC_HDR_SIZE + p[HCI_CMD_LEN_OFFSET] + HCI_CMD_PREAMBLE_SIZE);
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = p[HCI_CMD_LEN_OFFSET] + HCI_CMD_PREAMBLE_SIZE;
        p_buf->layer_specific = 0;

        STREAM_TO_UINT16(opcode, p)
        memcpy((uint8_t*)(p_buf+1), config_seq->buffer, p_buf->len);

        status = bt_vendor_cbacks->xmit_cb(opcode, p_buf, command_handler);
    }
    return status;
}

/*******************************************************************************
**
** Function        hw_config_get_patch_file_name
**
** Description     Build the FW patch file name.
**
** Returns         None
**
*******************************************************************************/
void hw_config_get_patch_file_name(uint8_t *p)
{
    BTHWDBG("%s", __func__);

#if (INTEL_WP2_USB == TRUE)
    strncpy(fw_patchfile_extension, ".seq", FW_PATCHFILE_EXTENSION_LEN);
#endif

#if (INTEL_AG6XX_UART == TRUE)
    strncpy(fw_patchfile_extension, ".pbn", FW_PATCHFILE_EXTENSION_LEN);
#endif

#if (INTEL_LNP_UART == TRUE)
    if((p[FIRMWARE_BUILD_BYTE_1] == 0x28) && (p[FIRMWARE_BUILD_BYTE_2] == 0x0d))
                                     /* check for firmware build bytes,
                                        for A0, A3 = 28, 0d and for K0 = 0d, 0e*/
    {
        BTHWDBG("LnP A0 Controller");
        strncpy(fw_patchfile_extension, ".seq", FW_PATCHFILE_EXTENSION_LEN);
        hw_cfg_cb.config_state = -1;       /* -1 because it will get increased
                                              by 1 before getting used */
        hw_cfg_cb.fw_config_seq = fw_config_bt_on_a0_a3_seq;
        hw_cfg_cb.max_seq_number = sizeof(fw_config_bt_on_a0_a3_seq) /
                                                    sizeof(fw_config_seq_t);

        //copy bddata file name
        strncpy(hw_bddata_location, BDDATA_FILE_A0, strlen(BDDATA_FILE_A0));
    }
    else if((p[FIRMWARE_BUILD_BYTE_1] == 0x0d) && (p[FIRMWARE_BUILD_BYTE_2] == 0x0e))
    {
        BTHWDBG("Lnp K0 Controller");
        strncpy(fw_patchfile_extension, ".sfi", FW_PATCHFILE_EXTENSION_LEN);
        hw_cfg_cb.config_state = -1;       /* -1 because it will get increased
                                              by 1 before getting used */
        hw_cfg_cb.fw_config_seq = fw_config_bt_on_k0_seq;
        hw_cfg_cb.max_seq_number = sizeof(fw_config_bt_on_k0_seq) /
                                                    sizeof(fw_config_seq_t);
        //copy bddata file name
        strncpy(hw_bddata_location, BDDATA_FILE_K0, strlen(BDDATA_FILE_K0));
    }
    else if((p[FIRMWARE_BUILD_BYTE_1] == 0x22) && (p[FIRMWARE_BUILD_BYTE_2] == 0x0e))
    {
        BTHWDBG("Lnp B0 Controller");
        strncpy(fw_patchfile_extension, ".sfi", FW_PATCHFILE_EXTENSION_LEN);
        hw_cfg_cb.config_state = -1;       /* -1 because it will get increased
                                              by 1 before getting used */
        hw_cfg_cb.fw_config_seq = fw_config_bt_on_b0_seq;
        hw_cfg_cb.max_seq_number = sizeof(fw_config_bt_on_b0_seq) /
                                                    sizeof(fw_config_seq_t);

        //copy bddata file name
        strncpy(hw_bddata_location, BDDATA_FILE_B0, strlen(BDDATA_FILE_B0));
    }
    else
    {
        BTHWERR("Unknown LnP Controller");
        strncpy(fw_patchfile_extension, ".sfi", FW_PATCHFILE_EXTENSION_LEN);
    }
#endif

    if(hw_strncmp(
      ".seq",
      fw_patchfile_extension,
      FW_PATCHFILE_EXTENSION_LEN) == 0)
    {
        memset(hw_cfg_cb.patchfile, 0, sizeof(hw_cfg_cb.patchfile));
        BTHWDBG("seq patch file name: %02x%02x%02x%02x%02x%02x%02x%02x%02x.seq"
        , p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14]);
        snprintf(hw_cfg_cb.patchfile, NAME_MAX,
         "%02x%02x%02x%02x%02x%02x%02x%02x%02x.seq", p[6], p[7],
        p[8], p[9], p[10], p[11],
        p[12], p[13], p[14]);
    }
    else if(hw_strncmp(
          ".sfi",
          fw_patchfile_extension,
          FW_PATCHFILE_EXTENSION_LEN) == 0)
    {
        memset(hw_cfg_cb.patchfile, 0, sizeof(hw_cfg_cb.patchfile));
        BTHWDBG("sfi patch file name: %02x%02x%02x%02x%02x%02x%02x%02x%02x.sfi"
        , p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14]);
        snprintf(hw_cfg_cb.patchfile, NAME_MAX,
        "%02x%02x%02x%02x%02x%02x%02x%02x%02x.sfi", p[6], p[7],
        p[8], p[9], p[10], p[11],
        p[12], p[13], p[14]);
    }
    else if(hw_strncmp(
      ".pbn",
      fw_patchfile_extension,
      FW_PATCHFILE_EXTENSION_LEN) == 0)
    {
        uint8_t hw_variant = p[HCI_EVT_READ_HW_VARIANT];
        uint8_t hw_revision = p[HCI_EVT_READ_HW_REVISION];
        uint16_t dev_id = (hw_variant << 8) | (hw_revision << 0);
        BTHWDBG("hw_varient:0x%x hw_revision:0x%x Device id:0x%x"
                     , hw_variant, hw_revision,dev_id);
        /* AG620 file name should be a00.pbn unless specified in conf file */
        sprintf(hw_cfg_cb.patchfile, "%x", dev_id);
        BTHWDBG("pbn file name name: %s",hw_cfg_cb.patchfile);
    }
}

/*******************************************************************************
**
** Function        hw_config_op_hci_evt_handler
**
** Description     Handle the HCI event of HW configuration sequence. It
**                 verifies the event coming from lower layer.
**
** Returns         TRUE or FALSE
**
*******************************************************************************/
uint8_t hw_config_op_hci_evt_handler(fw_config_seq_t* config_seq, void* buf)
{
    HC_BT_HDR   *p_evt_buf = NULL;
    uint8_t     *p;
    uint8_t event_code, sub_event_code;
    uint8_t status = FALSE;
    p_evt_buf = (HC_BT_HDR*) buf;
    p = (uint8_t*)(p_evt_buf+1);

    BTHWDBG("%s", __func__);
    if (config_seq->var_param != IGNORE_EVT)
    {
        event_code = p[0];
        BTHWDBG("%s event_code:0x%x", __func__, event_code);
        switch(event_code)
        {
            case HCI_INTEL_DEBUG_EVT_CODE:
                if((p[HCI_INTEL_SSEND_RST_EVT_ID_BYTE] == HCI_INTEL_SSEND_RST_EVT_ID) &&
                    (p[HCI_INTEL_SSEND_RST_EVT_STATUS_BYTE] != HCI_INTEL_SSEND_RST_EVT_STATUS))
                {
                    BTHWERR("firmware download failure: %0x", p[HCI_INTEL_SSEND_RST_EVT_STATUS_BYTE]);
                    status = FALSE;
                    break;
                }
                status = TRUE;
                break;
            case HCI_COMMAND_CMPL_EVT_CODE:
                if(!((p[HCI_CMD_CMPL_EVT_STATUS_BYTE] == config_seq->buffer[HCI_CMD_CMPL_EVT_STATUS_BYTE]) &&
                    (p[HCI_CMD_CMPL_EVT_OPCODE_FIRST_BYTE] == config_seq->buffer[HCI_CMD_CMPL_EVT_OPCODE_FIRST_BYTE]) &&
                    (p[HCI_CMD_CMPL_EVT_OPCODE_SECOND_BYTE] ==config_seq->buffer[HCI_CMD_CMPL_EVT_OPCODE_SECOND_BYTE])))
                {
                    BTHWERR("Not a currect event");
                    break;
                }
                status = TRUE;
                if(p[HCI_CMD_CMPL_EVT_OPCODE_FIRST_BYTE] == 0x05 &&
                   p[HCI_CMD_CMPL_EVT_OPCODE_SECOND_BYTE] == 0xFC)  // check for Read SW Version Event
                {
                    hw_config_get_patch_file_name(p);
                }
                break;
            case HCI_COMMAND_STATUS_EVT_CODE:
                if(!((p[HCI_CMD_STATUS_EVT_STATUS_BYTE] == config_seq->buffer[HCI_CMD_STATUS_EVT_STATUS_BYTE]) &&
                    (p[HCI_CMD_STATUS_EVT_OPCODE_FIRST_BYTE] == config_seq->buffer[HCI_CMD_STATUS_EVT_OPCODE_FIRST_BYTE]) &&
                    (p[HCI_CMD_STATUS_EVT_OPCODE_SECOND_BYTE] ==config_seq->buffer[HCI_CMD_STATUS_EVT_OPCODE_SECOND_BYTE])))
                {
                    BTHWERR("Not a currect event");
                    break;
                }
                status = TRUE;
                break;
            default:
                break;
        }
    }
    else
        status = TRUE;

    return status;
}

tHW_CONFIG_OP_HANDLER hw_config_op_handler_table =
{
    NULL,
    hw_config_op_hci_cmd_handler,
    hw_config_op_hci_evt_handler,
    hw_config_op_find_patch_handler,
    hw_config_op_patch_download_handler,
    hw_config_op_rts_state_change_handler,
    hw_config_op_host_baud_change_handler,
    hw_config_op_write_bddata_handler,
    hw_config_op_write_btnvm_handler,
    hw_config_op_reg_async_event_handler,
    hw_config_op_report_success_handler,
    hw_config_op_report_failure_handler
};

/*******************************************************************************
**
** Function        command_handler
**
** Description     This is the main function of HW configuration. It execute the
**                 HW configuration sequence for a particular controller and
**                 maintain the state of it.
**
** Returns         None
**
*******************************************************************************/
void command_handler(void* buf)
{
    BTHWDBG("%s", __func__);
    uint8_t status ;

    if (hw_cfg_cb.fw_config_seq[hw_cfg_cb.config_state].operation > OP_INVALID && hw_cfg_cb.fw_config_seq[hw_cfg_cb.config_state].operation < OP_MAX)
    {
        do
        {
            status = FALSE;
            switch(hw_cfg_cb.fw_config_seq[hw_cfg_cb.config_state].operation)
            {
                case OP_HCI_CMD:
                    status = p_hw_config_op_handler->hci_cmd_handler
                            (&hw_cfg_cb.fw_config_seq[hw_cfg_cb.config_state]);
                    break;
                case OP_HCI_EVT:
                    status = p_hw_config_op_handler->hci_evt_handler
                        (&hw_cfg_cb.fw_config_seq[hw_cfg_cb.config_state], buf);
                    break;
                case OP_FIND_PATCH:
                    if(hw_cfg_cb.patchfile != NULL)
                        hw_cfg_cb.patch_file_present = p_hw_config_op_handler->
                                        find_patch_handler(hw_cfg_cb.patchfile);
                    if(hw_cfg_cb.patch_file_present)
                        status = TRUE;
                    break;
                case OP_PATCH_DOWNLOAD:
                    status = p_hw_config_op_handler->patch_download_handler(buf);
                    break;
                case OP_WRITE_BDDATA:
                    status = p_hw_config_op_handler->write_bddata_handler();
                    break;
                     case OP_WRITE_BTNVM:
                    status = p_hw_config_op_handler->write_btnvm_handler();
                    break;
                case OP_RTS_STATE_CHANGE:
                    p_hw_config_op_handler->rts_state_change_handler(hw_cfg_cb.
                               fw_config_seq[hw_cfg_cb.config_state].var_param);
                    status = TRUE;
                    break;
                case OP_HOST_BAUD_CHANGE:
                    p_hw_config_op_handler->host_baud_change_handler(&hw_cfg_cb.
                               fw_config_seq[hw_cfg_cb.config_state].var_param);
                    status = TRUE;
                    break;
                case OP_REG_ASYNC_EVENT:
                    p_hw_config_op_handler->reg_async_event_handler(hw_cfg_cb.
                               fw_config_seq[hw_cfg_cb.config_state].var_param);
                    status = TRUE;
                    break;
                case OP_REPORT_SUCCESS:
                    p_hw_config_op_handler->report_success_handler();
                    status = TRUE;
                    break;
                default:
                    break;
            }

            if((hw_cfg_cb.fw_config_seq[hw_cfg_cb.config_state].operation ==
                OP_PATCH_DOWNLOAD) && !hw_cfg_cb.patch_download_done && status)
                    break;

            if((hw_cfg_cb.fw_config_seq[hw_cfg_cb.config_state].operation ==
                OP_WRITE_BDDATA) && !hw_cfg_cb.bddata_download_done && status)
                    break;

            if((hw_cfg_cb.fw_config_seq[hw_cfg_cb.config_state].operation ==
                OP_WRITE_BTNVM) && !hw_cfg_cb.btnvm_download_done && status)
                    break;

            if(status)
               hw_cfg_cb.config_state++;
            else
            {
                if(hw_cfg_cb.fw_config_seq[hw_cfg_cb.config_state].operation ==
                                                                  OP_FIND_PATCH)
                {
                    do
                    {
                        hw_cfg_cb.config_state++;
                    }while(hw_cfg_cb.fw_config_seq[hw_cfg_cb.config_state].
                                         var_param != NEXT_STATE_FW_DOWNLOADED);
                    status = TRUE;
                }
                else
                {
                    p_hw_config_op_handler->report_failure_handler();
                    break;
                }
            }
        }while((hw_cfg_cb.fw_config_seq[hw_cfg_cb.config_state].operation
            != OP_HCI_EVT) && (hw_cfg_cb.config_state <
            hw_cfg_cb.max_seq_number) && status);
    }

    if (bt_vendor_cbacks)
    {
        if (buf != NULL)
        {
            bt_vendor_cbacks->dealloc(buf);
            buf = NULL;
        }
    }
}

/*****************************************************************************
**   Hardware Configuration Interface Functions
*****************************************************************************/

/*******************************************************************************
**
** Function        hw_config_start
**
** Description     Kick off controller initialization process
**
** Returns         None
**
*******************************************************************************/
void hw_config_start(void)
{
    BTHWDBG("%s - INTEL bludroid stack", __func__);
    hw_cfg_cb.fw_fd = NULL;
    hw_cfg_cb.aux_fd = NULL;
    hw_cfg_cb.btnvm_fd = NULL;
    hw_cfg_cb.patch_file_opened = FALSE;
    hw_cfg_cb.patch_file_present = FALSE;
    hw_cfg_cb.patch_download_done = FALSE;
    hw_cfg_cb.aux_file_opened = FALSE;
    hw_cfg_cb.btnvm_file_opened = FALSE;
    hw_cfg_cb.bddata_download_done = FALSE;
    hw_cfg_cb.btnvm_download_done = FALSE;
    hw_cfg_cb.config_state = 0;
    hw_cfg_cb.fw_config_seq = fw_config_bt_on_seq;
    hw_cfg_cb.max_seq_number = sizeof(fw_config_bt_on_seq) /
                                                sizeof(fw_config_seq_t);
    hw_cfg_cb.bt_state = BT_TURNING_ON;
    hw_cfg_cb.address = 0;
    hw_cfg_cb.no_of_bytes = 0;
    hw_cfg_cb.sfi_state = HW_CFG_SEND_CSS;
    p_hw_config_op_handler = &hw_config_op_handler_table;
#if (INTEL_AG6XX_UART != TRUE)
    command_handler(NULL);
#endif
    return;
}

/*******************************************************************************
**
** Function        hw_lpm_enable
**
** Description     Enalbe/Disable LPM
**
** Returns         TRUE/FALSE
**
*******************************************************************************/
uint8_t hw_lpm_enable(uint8_t turn_on)
{
#if (LNP_LPM_ENABLED == TRUE)
    uint8_t status = BT_VND_OP_RESULT_SUCCESS;
    int retval = BT_VND_OP_RESULT_SUCCESS;
    static uint32_t lpm_status;

    BTHWDBG("%s", __func__);

    if(lpm_status == turn_on)
    {
       BTHWDBG("lpm already turned on, status:%d", lpm_status);
       return TRUE;
    }
    else
    {
       lpm_status = (uint32_t)turn_on;
    }

    BTHWDBG("lpm status:%d", lpm_status);

    if (lpm_is_enabled == TRUE)
    {
        status = userial_vendor_ioctl(USERIAL_OP_LPM_ENABLE, \
          (void*)&lpm_status);

        if (status == BT_VND_OP_RESULT_SUCCESS)
        {
           BTHWDBG("USERIAL_OP_LPM_ENABLE success");
           retval = BT_VND_OP_RESULT_SUCCESS;
        }
        else
        {
           BTHWDBG("USERIAL_OP_LPM_ENABLE failed");
           retval = BT_VND_OP_RESULT_FAIL;
        }
        bt_vendor_cbacks->lpm_cb(retval);
    }
    return retval;
#else
    BTHWDBG("%s", __func__);
    return FALSE;
#endif
}

#if (LNP_LPM_ENABLED == TRUE)
/*******************************************************************************
**
** Function        hw_is_lpm_enabled
**
** Description     Sets if lpm is enabled
**
** Returns         None
**
*******************************************************************************/
void hw_is_lpm_enabled(char *p_conf_name, char *p_conf_value, int param)
{
    BTHWDBG("%s conf_value:%s", __func__, p_conf_value);
    if (strcmp(p_conf_value, "true") == 0)
    {
        lpm_is_enabled = TRUE;
    }
    else if (strcmp(p_conf_value, "false") == 0)
    {
        lpm_is_enabled = FALSE;
    }
    return;
}
#endif

/*******************************************************************************
**
** Function        hw_lpm_get_idle_timeout
**
** Description     Calculate idle time based on host stack idle threshold
**
** Returns         idle timeout value
**
*******************************************************************************/
uint32_t hw_lpm_get_idle_timeout(void)
{
    uint32_t timeout_ms;

    /* set idle time to be LPM_IDLE_TIMEOUT_MULTIPLE times of
     * host stack idle threshold (in 300ms/25ms)
     */
    timeout_ms = (uint32_t)lpm_param.host_stack_idle_threshold
                            * LPM_IDLE_TIMEOUT_MULTIPLE;

    return timeout_ms;
}

/*******************************************************************************
**
** Function        hw_lpm_set_wake_state
**
** Description     Assert/Deassert BT_WAKE
**
** Returns         None
**
*******************************************************************************/
void hw_lpm_set_wake_state(uint8_t wake_assert)
{
    uint8_t state = (wake_assert) ? UPIO_ASSERT : UPIO_DEASSERT;
#if (INTEL_AG6XX_UART != TRUE)
    upio_set(UPIO_BT_WAKE, state, lpm_param.bt_wake_polarity);
#endif
}

#if (SCO_CFG_INCLUDED == TRUE)
/*******************************************************************************
**
** Function         hw_sco_config
**
** Description      Configure SCO related hardware settings
**
** Returns          None
**
*******************************************************************************/
void hw_sco_config(void)
{
    /* Implement if any vendor specific SCO init is needed. */
}
#endif  // SCO_CFG_INCLUDED

/*******************************************************************************
**
** Function        hw_set_patch_file_path
**
** Description     Set the location of firmware patch file
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_file_path(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(fw_patchfile_path, p_conf_value);
    return 0;
}

/*******************************************************************************
**
** Function        hw_set_patch_file_name
**
** Description     Give the specific firmware patch filename
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_file_name(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(fw_patchfile_name, p_conf_value);
    return 0;
}

#if (INTEL_LNP_UART == TRUE)
/*******************************************************************************
**
** Function        hw_set_patch_file_name
**
** Description     Give the specific firmware patch filename
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_bddata_file_location(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(hw_bddata_location, p_conf_value);
    return 0;
}

/*******************************************************************************
**
** Function        hw_operational_baud_value
**
** Description     Specify operational FW baud rate
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_operational_baud_value(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(hw_operation_baud_str, p_conf_value);
    if (strcmp(hw_operation_baud_str,"")!=0)
        hw_operation_baud = atoi(hw_operation_baud_str);
    BTHWDBG("%s hw_operation_baud:%d", __func__, hw_operation_baud);
    return 0;
}
#endif

#if (INTEL_AG6XX_UART == TRUE)
/*******************************************************************************
**
** Function        hw_lpm_get_lpm_param
**
** Description     Calculate idle time based on host stack idle threshold
**
** Returns         idle timeout value
**
*******************************************************************************/
void hw_lpm_get_lpm_param(void* param)
{
    bt_vendor_lpm_param_t* p_param = (bt_vendor_lpm_param_t*) param;
    p_param->idle_timeout = idle_timeout;
    p_param->pkt_rate_monitor_period = pkt_rate_monitor_period;
    p_param->pkt_rate_monitor_threshold = pkt_rate_monitor_threshold;
    p_param->pkt_rate_monitor_correction_factor =
                                             pkt_rate_monitor_correction_factor;
    p_param->wakeup_time = wakeup_time;
}

/*******************************************************************************
**
** Function        hw_is_signaling_enabled
**
** Description     Sets if signaling is enabled
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_is_signaling_enabled(char *p_conf_name, char *p_conf_value, int param)
{
    BTHWDBG("%s conf_value:%s", __func__, p_conf_value);
    if (strcmp(p_conf_value, "true") == 0)
    {
        signaling_is_enabled = TRUE;
    }
    else if (strcmp(p_conf_value, "false") == 0)
    {
        signaling_is_enabled = FALSE;
    }
    return 0;
}

/*******************************************************************************
**
** Function        hw_is_lpm_enabled
**
** Description     Sets if lpm is enabled
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_is_lpm_enabled(char *p_conf_name, char *p_conf_value, int param)
{
    BTHWDBG("%s conf_value:%s", __func__, p_conf_value);
    if (strcmp(p_conf_value, "true") == 0)
    {
        lpm_is_enabled = TRUE;
    }
    else if (strcmp(p_conf_value, "false") == 0)
    {
        lpm_is_enabled = FALSE;
    }
    return 0;
}

/*******************************************************************************
**
** Function        hw_read_idle_timeout
**
** Description     Reads idle timeout
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_read_idle_timeout(char *p_conf_name, char *p_conf_value, int param)
{
    idle_timeout = hw_read_conf_value(p_conf_value, 300);
    return 0;
}

/*******************************************************************************
**
** Function        hw_read_pkt_rate_monitor_period
**
** Description     Reads pkt rate monitor period
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_read_pkt_rate_monitor_period(char *p_conf_name, char *p_conf_value, int param)
{
    pkt_rate_monitor_period = hw_read_conf_value(p_conf_value, 100);
    return 0;
}

/*******************************************************************************
**
** Function        hw_read_pkt_rate_monitor_threshold
**
** Description     Reads idle timeout
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_read_pkt_rate_monitor_threshold(char *p_conf_name, char *p_conf_value, int param)
{
    pkt_rate_monitor_threshold = hw_read_conf_value(p_conf_value, 10);
    return 0;
}

/*******************************************************************************
**
** Function        hw_read_pkt_rate_monitor_correction_factor
**
** Description     Reads idle timeout
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_read_pkt_rate_monitor_correction_factor
                            (char *p_conf_name, char *p_conf_value, int param)
{
    pkt_rate_monitor_correction_factor = hw_read_conf_value(p_conf_value, 5);
    return 0;
}

/*******************************************************************************
**
** Function        hw_read_wakeup_time
**
** Description     Reads system wakeup time from D0I3 to D0
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_read_wakeup_time(char *p_conf_name, char *p_conf_value, int param)
{
    wakeup_time = hw_read_conf_value(p_conf_value, 300);
    return 0;
}

/*******************************************************************************
**
** Function        hw_set_bddata_file_name
**
** Description     Give the specific BD Data filename
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_bddata_file_name(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(hw_bddata_location, p_conf_value);
    return 0;
}

/*******************************************************************************
**
** Function        hw_set_fw_ctl_reg
**
** Description     Configures the value of the fw register
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_fw_ctl_reg(char *p_conf_name, char *p_conf_value, int param)
{
    char* end_ptr;
    unsigned long l = -1;
    if (p_conf_value != NULL)
    {
        /* Get the hex from string */
        l = strtol(p_conf_value, &end_ptr, 16);
        if (*end_ptr)
            l = 0xff;
        fw_cfg_reg_value = l;
    }
    return 0;
}
#endif

/*******************************************************************************
**
** Function        hw_set_nvm_file_name
**
** Description     Give the specific nvm filename
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_nvm_file_name(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(hw_btnvm_location, p_conf_value);
    return 0;
}


#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
/*******************************************************************************
**
** Function        hw_set_patch_settlement_delay
**
** Description     Give the specific firmware patch settlement time in milliseconds
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_settlement_delay(char *p_conf_name, char *p_conf_value, int param)
{
    fw_patch_settlement_delay = atoi(p_conf_value);

    return 0;
}
#endif  //VENDOR_LIB_RUNTIME_TUNING_ENABLED

#if (HW_END_WITH_HCI_RESET == TRUE)

/*******************************************************************************
**
** Function         hw_epilog_process
**
** Description      Sample implementation of epilog process
**
** Returns          None
**
*******************************************************************************/
void hw_epilog_process(void)
{
    BTHWDBG("%s", __func__);
    hw_cfg_cb.fw_fd = NULL;
    hw_cfg_cb.aux_fd = NULL;
    hw_cfg_cb.btnvm_fd = NULL;
    hw_cfg_cb.patch_file_opened = FALSE;
    hw_cfg_cb.patch_file_present = FALSE;
    hw_cfg_cb.patch_download_done = FALSE;
    hw_cfg_cb.aux_file_opened = FALSE;
    hw_cfg_cb.btnvm_file_opened = FALSE;
    hw_cfg_cb.bddata_download_done = FALSE;
    hw_cfg_cb.btnvm_download_done = FALSE;
    hw_cfg_cb.config_state = 0;
    hw_cfg_cb.fw_config_seq = fw_config_bt_off_seq;
    hw_cfg_cb.max_seq_number = sizeof(fw_config_bt_off_seq) /
                                                        sizeof(fw_config_seq_t);
    hw_cfg_cb.bt_state = BT_TURNING_OFF;
    hw_cfg_cb.address = 0;
    hw_cfg_cb.no_of_bytes = 0;
    p_hw_config_op_handler = &hw_config_op_handler_table;
    command_handler(NULL);
    return;
}
#endif // (HW_END_WITH_HCI_RESET == TRUE)

#if (A2DP_OFFLOAD_INCLUDED == TRUE)
/*******************************************************************************
**
** Function         a2dp_offload_set_stream_state_cback
**
** Description      Callback function for A2DP stream state
**
** Returns          None
**
*******************************************************************************/
void a2dp_offload_set_stream_state_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);
}

/*******************************************************************************
**
** Function         a2dp_offload_set_stream_state
**
** Description     Sets the A2DP state in the BT controller via Vendor Specific HCI command
**
** Returns          None
**
*******************************************************************************/
void a2dp_offload_set_stream_state(uint8_t param)
{
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p, ret;

    BTHWDBG("%s send HCI_INTEL_A2DP_SET_STREAM_STATE", __func__);
    uint16_t cmd_u16 = HCI_CMD_PREAMBLE_SIZE + HCI_INTEL_A2DP_SET_STREAM_STATE_PARAM_SIZE;
    if (bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE+cmd_u16);

    if (p_buf){
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p = (uint8_t *) (p_buf + 1);
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + HCI_INTEL_A2DP_SET_STREAM_STATE_PARAM_SIZE;
        UINT16_TO_STREAM(p, HCI_INTEL_A2DP_SET_STREAM_STATE);
        *p++ = HCI_INTEL_A2DP_SET_STREAM_STATE_PARAM_SIZE;
        *p++ = param;
        bt_vendor_cbacks->xmit_cb(HCI_INTEL_A2DP_SET_STREAM_STATE,
             p_buf, a2dp_offload_set_stream_state_cback);
    }
}

/*******************************************************************************
**
** Function         a2dp_offload_send_stream_config_cback
**
** Description      Callback function for A2DP stream state
**
** Returns          None
**
*******************************************************************************/
void a2dp_offload_send_stream_config_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;

    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);
}

/*******************************************************************************
**
** Function         a2dp_offload_send_stream_config
**
** Description     Sets the A2DP state in the BT controller via Vendor Specific HCI command
**
** Returns          None
**
*******************************************************************************/
void a2dp_offload_send_stream_config(A2DP_OFFLOAD_STREAM_CFG_PARAMS *params)
{
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p, ret;
    BTHWDBG("%s send HCI_INTEL_A2DP_STREAM_CONFIG", __func__);
    uint16_t cmd_u16 = HCI_CMD_PREAMBLE_SIZE + HCI_INTEL_A2DP_STREAM_CONFIG_PARAM_SIZE;

    if (bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE+cmd_u16);

    if (p_buf){
    p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
    p_buf->offset = 0;
    p_buf->layer_specific = 0;
    p = (uint8_t *) (p_buf + 1);
    p_buf->len = HCI_CMD_PREAMBLE_SIZE + HCI_INTEL_A2DP_STREAM_CONFIG_PARAM_SIZE;
    UINT16_TO_STREAM(p, HCI_INTEL_A2DP_STREAM_CONFIG);
    *p++ = HCI_INTEL_A2DP_STREAM_CONFIG_PARAM_SIZE;

       if (NULL != params->bd_addr)
        BDADDR_TO_STREAM(p, params->bd_addr);

        UINT16_TO_STREAM (p,params->handle);
        UINT8_TO_STREAM  (p, params->codec_type);
        UINT16_TO_STREAM (p,params->lcid);
        UINT16_TO_STREAM (p,params->peer_mtu);

     bt_vendor_cbacks->xmit_cb(HCI_INTEL_A2DP_STREAM_CONFIG,
                                    p_buf, a2dp_offload_send_stream_config_cback);

    }
}
#endif
