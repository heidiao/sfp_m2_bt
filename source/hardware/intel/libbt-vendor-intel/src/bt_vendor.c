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
 *  Filename:      bt_vendor.c
 *
 *  Description:   Intel vendor specific library implementation
 *
 ******************************************************************************/

#define LOG_TAG "bt_vendor"

#include <utils/Log.h>
#include "bt_vendor.h"
#include "upio.h"
#include "userial_vendor.h"

//#ifndef BTVND_DBG
#define BTVND_DBG TRUE
//#endif

#if (BTVND_DBG == TRUE)
#define BTVNDDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTVNDDBG(param, ...) {}
#endif

#define BTVNDERR(param, ...) {ALOGE(param, ## __VA_ARGS__);}

/******************************************************************************
**  Externs
******************************************************************************/
void hw_config_start(void);
void ms_delay(uint32_t timeout);
uint8_t register_int_evt_callback();
uint8_t hw_lpm_enable(uint8_t turn_on);
uint32_t hw_lpm_get_idle_timeout(void);
void hw_lpm_set_wake_state(uint8_t wake_assert);
void vnd_load_prop();
void vnd_load_conf(const char *p_path);
#if (INTEL_AG6XX_UART == TRUE)
extern void hw_lpm_get_lpm_param(void* param);
extern uint8_t signaling_is_enabled;
extern uint8_t lpm_is_enabled;
extern uint8_t fw_cfg_reg_value;
#endif
#if (HW_END_WITH_HCI_RESET == TRUE)
void hw_epilog_process(void);
#endif
#if (LNP_LPM_ENABLED == TRUE)
extern uint8_t lpm_is_enabled;
#endif
extern uint32_t hw_operation_baud;
/******************************************************************************
**  Variables
******************************************************************************/

bt_vendor_callbacks_t *bt_vendor_cbacks = NULL;
uint8_t vnd_local_bd_addr[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#if (INTEL_AG6XX_UART == TRUE)
uint8_t lpm_set_status;
pthread_mutex_t netlink_listen_start_mutex     = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  netlink_listen_start_cond_var   = PTHREAD_COND_INITIALIZER;
#endif
/******************************************************************************
**  Local type definitions
******************************************************************************/

/******************************************************************************
**  Static Variables
******************************************************************************/

static const tUSERIAL_CFG userial_init_cfg =
{
    (USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1),
    USERIAL_BAUD_115200
};

/******************************************************************************
**  Functions
******************************************************************************/

#if (INTEL_AG6XX_UART == TRUE)
/*******************************************************************************
**
** Function          set_fw_config_reg
**
** Description       Sets Firmware config register value from bt_vendor.conf configuration
**
** Returns           None
**
*******************************************************************************/
void set_fw_config_reg(uint8_t val)
{
    if (val == 0xff)
    {
        BTVNDDBG("%s Nothing specified.", __func__);
    }
    else if (val == 0xf0 || val == 0xf4)
    {
        userial_vendor_ioctl(USERIAL_OP_SET_FWCFG_REG, &val);
    }
    else if (!(val & 0x20))
    {
        BTVNDERR("%s For turning off signaling, please use the SignalingEnabled flag in bt_vendor.conf"
        , __func__);
    }
    else
    {
        BTVNDERR("%s Not allowed to change the specified bit(s), please refer to FW spec for details"
        , __func__);
    }
}
#endif

/*****************************************************************************
**
**   BLUETOOTH VENDOR INTERFACE LIBRARY FUNCTIONS
**
*****************************************************************************/

static int init(const bt_vendor_callbacks_t* p_cb, unsigned char *local_bdaddr)
{
    BTVNDDBG("init");

    if (p_cb == NULL)
    {
        BTVNDERR("init failed with no user callbacks!");
        return -1;
    }

#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
    BTVNDDBG("*****************************************************************");
    BTVNDDBG("*****************************************************************");
    BTVNDDBG("** Warning - BT Vendor Lib is loaded in debug tuning mode!");
    BTVNDDBG("**");
    BTVNDDBG("** If this is not intentional, rebuild libbt-vendor.so ");
    BTVNDDBG("** with VENDOR_LIB_RUNTIME_TUNING_ENABLED=FALSE and ");
    BTVNDDBG("** check if any run-time tuning parameters needed to be");
    BTVNDDBG("** carried to the build-time configuration accordingly.");
    BTVNDDBG("*****************************************************************");
    BTVNDDBG("*****************************************************************");
#endif

    userial_vendor_init();
    upio_init();

    vnd_load_conf(VENDOR_LIB_CONF_FILE);

    /* Properties values override previous configuration */
    vnd_load_prop();

    /* store reference to user callbacks */
    bt_vendor_cbacks = (bt_vendor_callbacks_t *) p_cb;

    /* This is handed over from the stack */
    memcpy(vnd_local_bd_addr, local_bdaddr, 6);

    return 0;
}

#if (INTEL_AG6XX_UART == TRUE)
#define CASE_RETURN_STR(const) case const: return #const;

/*******************************************************************************
**
** Function          dump_vendor_op
**
** Description       converts the enum types bt_vendor_opcode into strings
**
** Returns           String
**
*******************************************************************************/
static const char* dump_vendor_op(bt_vendor_opcode_t opcode)
{
    switch(opcode)
    {
        CASE_RETURN_STR(BT_VND_OP_POWER_CTRL);
        CASE_RETURN_STR(BT_VND_OP_FW_CFG);
        CASE_RETURN_STR(BT_VND_OP_SCO_CFG);
        CASE_RETURN_STR(BT_VND_OP_USERIAL_OPEN);
        CASE_RETURN_STR(BT_VND_OP_LPM_SET_MODE);
        CASE_RETURN_STR(BT_VND_OP_USERIAL_CLOSE);
        CASE_RETURN_STR(BT_VND_OP_GET_LPM_IDLE_TIMEOUT);
        CASE_RETURN_STR(BT_VND_OP_LPM_WAKE_SET_STATE);
#if (INTEL_AG6XX_UART == TRUE)
        CASE_RETURN_STR(BT_VND_OP_LPM_SET_DEVICE_STATE);
        CASE_RETURN_STR(BT_VND_OP_LPM_SET_BT_WAKE_STATE);
        CASE_RETURN_STR(BT_VND_OP_LPM_GET_CTS_STATE);
        CASE_RETURN_STR(BT_VND_OP_LPM_SET_RTS_STATE);
#endif
        default:
            return "unknown opcode";
    }
}
#endif

/*******************************************************************************
**
** Function          op
**
** Description       This is interface for bluedroid stack to libbt. It exposes
**                   several functionality to bluedroid stack.
**
** Returns           0 or -1
**
*******************************************************************************/
static int op(bt_vendor_opcode_t opcode, void *param)
{
    int retval = 0;

    switch(opcode)
    {
        case BT_VND_OP_POWER_CTRL:
            {
                BTVNDDBG("op: BT_VND_OP_POWER_CTRL");
                int *state = (int *) param;
                if (*state == BT_VND_PWR_OFF)
#if (INTEL_WP2_UART == TRUE || INTEL_LNP_UART == TRUE || INTEL_WP2_USB == TRUE)
                    retval = upio_set_bluetooth_power(UPIO_BT_POWER_OFF);
#else
                    ;
#endif
                else if (*state == BT_VND_PWR_ON)
                {
#if (INTEL_WP2_UART == TRUE || INTEL_LNP_UART == TRUE || INTEL_WP2_USB == TRUE)
                    retval = upio_set_bluetooth_power(UPIO_BT_POWER_ON);
#else
                   ;
#endif
                }
            }
            break;
        case BT_VND_OP_FW_CFG:
            {
                BTVNDDBG("BT_VND_OP_FW_CFG");
                hw_config_start();
            }
            break;

        case BT_VND_OP_SCO_CFG:
            {
                BTVNDDBG("op: BT_VND_OP_SCO_CFG");
#if (SCO_CFG_INCLUDED == TRUE)
                hw_sco_config();
#else
                bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_SUCCESS);
                retval = -1;
#endif
            }
            break;

        case BT_VND_OP_USERIAL_OPEN:
            {
                BTVNDDBG("op: BT_VND_OP_USERIAL_OPEN");
                int (*fd_array)[] = (int (*)[]) param;
                int fd, idx;
#if (INTEL_LNP_UART == TRUE)
                fd = userial_vendor_open();
#if (LNP_LPM_ENABLED == TRUE)
                hw_lpm_enable((uint8_t)TRUE);
#endif
#if (INTEL_HSB_PLATFORM == TRUE)
                userial_vendor_configure_serial((tUSERIAL_CFG *)&userial_init_cfg, userial_init_cfg.baud);
#endif
#else
                fd = userial_vendor_open();
                userial_vendor_configure_serial((tUSERIAL_CFG *)&userial_init_cfg, userial_init_cfg.baud);
#endif
                if (fd != -1)
                {
                    for (idx=0; idx < CH_MAX; idx++)
                        (*fd_array)[idx] = fd;

                    retval = 1;
                }
                /* retval contains numbers of open fd of HCI channels */
            }
            break;

#if (BT_EN_VIA_USERIAL_IOCTL == TRUE)
        case BT_VND_OP_USERIAL_CONFIG:
            {
                int baud_rate;
                bt_vendor_baud_config_t baud_config = 115200;
                if (param)
                    baud_config = *((int*) param);
                if (baud_config == STANDARD_BAUD)
                    baud_rate = 115200;
                else if (baud_config == HIGHER_BAUD)
                    baud_rate = hw_operation_baud;
                BTVNDDBG("op: BT_VND_OP_USERIAL_CONFIG:%d", \
                                        line_speed_to_userial_baud(baud_rate));
                userial_vendor_configure_serial\
                               ((tUSERIAL_CFG *)&userial_init_cfg, \
                                        line_speed_to_userial_baud(baud_rate));
            }
            break;
        case BT_VND_OP_FW_DL_COMPLETE:
            {
                unsigned long fw_cfg_result = FW_SUCCESS;
                userial_vendor_ioctl(USERIAL_OP_FW_CFG_CMPL, \
                                                    (void*)&fw_cfg_result);
            }
            break;
        case BT_VND_OP_FW_DL_STATUS:
            {
                int on = UPIO_BT_POWER_ON;
                retval = userial_vendor_ioctl(USERIAL_OP_BT_EN, &on);
                ms_delay(100);
            }
            break;
#endif

        case BT_VND_OP_USERIAL_CLOSE:
            {
                BTVNDDBG("op: BT_VND_OP_USERIAL_CLOSE");
#if (LNP_LPM_ENABLED == TRUE)
                hw_lpm_enable((uint8_t)FALSE);
#endif
                userial_vendor_close();
            }
            break;

        case BT_VND_OP_GET_LPM_IDLE_TIMEOUT:
            {
                BTVNDDBG("op: BT_VND_OP_GET_LPM_IDLE_TIMEOUT");
#if (INTEL_AG6XX_UART == TRUE)
                hw_lpm_get_lpm_param(param);
#else
                uint32_t *timeout_ms = (uint32_t *) param;
                *timeout_ms = hw_lpm_get_idle_timeout();
#endif
            }
            break;

        case BT_VND_OP_LPM_SET_MODE:
            {
                BTVNDDBG("op: BT_VND_OP_LPM_SET_MODE");
                uint8_t mode = *(uint8_t *) param;
#if (INTEL_AG6XX_UART == TRUE)
                uint8_t device_state;
                BTVNDDBG("%s mode:%d", __func__, mode);
                if( mode == BT_VND_LPM_ENABLE)
                {
                    /*
                     * Before power on the chip. Enable the callback
                     * to receive the first default bd data event
                     */
                    register_int_evt_callback();
                    /* Call setFwCfg reg */
                    set_fw_config_reg(fw_cfg_reg_value);
                    if (signaling_is_enabled == FALSE)
                    {
                        device_state = 0; /* State value for D0 */
                        /* first event is yet to come */
                        BTVNDDBG("%s signaling is disabled from conf file.", __func__);
                        retval = -1;
                        userial_vendor_ioctl(USERIAL_OP_DISABLE_SIGNALING, NULL);
                        upio_set_d_state(device_state);
                        break;
                    }
                    int netlinkfd = upio_create_netlink_socket();
                    if(netlinkfd > 0)
                    {
                        upio_netlink_send_msg();
                        pthread_mutex_lock(&netlink_listen_start_mutex);
                        if (upio_netlink_listen_thread() < 0)
                        {
                            pthread_mutex_unlock(&netlink_listen_start_mutex);
                            lpm_set_status = FALSE;
                            retval = -1;
                        }
                        else
                        {
                            pthread_mutex_lock(&netlink_listen_start_mutex);
                            pthread_mutex_unlock(&netlink_listen_start_mutex);
                            lpm_set_status = TRUE;
                            retval = 0;
                        }
                        /*if(bt_vendor_cbacks && (status == 0))
                            bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_SUCCESS);
                    }else
                    {
                        if(bt_vendor_cbacks)
                            bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_FAIL);
                        */
                    }
                }
                else
                {
                    if (signaling_is_enabled == FALSE)
                    {
                        device_state = 5;
                        upio_set_d_state(device_state);
                        break;
                    }
                    upio_close_netlink_socket();
                }

#elif (LNP_LPM_ENABLED == TRUE)
                BTVNDDBG("op: BT_VND_OP_LPM_SET_MODE mode:%d", mode);
                hw_lpm_enable(mode);
#else
                //retval = hw_lpm_enable(*mode);
                bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_SUCCESS);
#endif
            }
            break;

        case BT_VND_OP_LPM_WAKE_SET_STATE:
            {
                uint8_t *state = (uint8_t *) param;
                uint8_t wake_assert = (*state == BT_VND_LPM_WAKE_ASSERT) ? \
                                        TRUE : FALSE;

                //hw_lpm_set_wake_state(wake_assert);
            }
            break;
#if (LNP_LPM_ENABLED == TRUE)
        case BT_VND_OP_LPM_SET_IDLE_STATE:
            {
                BTVNDDBG("op: BT_VND_OP_LPM_SET_IDLE_STATE");
                userial_vendor_ioctl(USERIAL_OP_LPM_SET_IDLE_STATE, \
                                                    (void*)param);
            }
            break;
#endif
#if (HW_END_WITH_HCI_RESET == TRUE)
        case BT_VND_OP_EPILOG:
            hw_epilog_process();
            break;
#endif
#if (A2DP_OFFLOAD_INCLUDED == TRUE)
        case BT_VND_OP_A2DP_STREAM_CONFIG:
            BTVNDDBG("op: BT_VND_BT_VND_OP_A2DP_STREAM_CONFIG");
            a2dp_offload_send_stream_config((A2DP_OFFLOAD_STREAM_CFG_PARAMS *)param);
            break;
        case BT_VND_OP_A2DP_SET_STREAM_STATE:
            BTVNDDBG("op: BT_VND_OP_A2DP_SET_STREAM_STATE");
            uint8_t *state = (uint8_t *) param;
            a2dp_offload_set_stream_state(*state);
            break;
#endif
#if (INTEL_AG6XX_UART == TRUE)
        case BT_VND_OP_LPM_SET_DEVICE_STATE:
            {
                uint8_t state = *(uint8_t*) param;
                /* If LPM is disabled, only allow D0 and D3 states */
                if (!((!lpm_is_enabled) && (state != 0) && (state != 5)))
                    retval = upio_set_d_state(state);
            }
            break;
        case BT_VND_OP_LPM_SET_BT_WAKE_STATE:
            {
                uint8_t state = *(uint8_t*) param;
                retval = upio_set_bt_wake_state(state);
                BTVNDDBG("%s cts_state:%d", __func__, retval);
            }
            break;
        case BT_VND_OP_LPM_GET_CTS_STATE:
            return upio_get_cts_state();
        case BT_VND_OP_LPM_SET_RTS_STATE:
            {
                uint8_t state = *(uint8_t*) param;
                upio_set_rts_state(state);
            }
            break;
#endif
    }

    return retval;
}

/*******************************************************************************
**
** Function          cleanup
**
** Description       Closes the interface
**
** Returns           None
**
*******************************************************************************/
static void cleanup( void )
{
    BTVNDDBG("cleanup");

    upio_cleanup();

    bt_vendor_cbacks = NULL;
}

// Entry point of DLib
const bt_vendor_interface_t BLUETOOTH_VENDOR_LIB_INTERFACE = {
    sizeof(bt_vendor_interface_t),
    init,
    op,
    cleanup
};
