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
 *  Filename:      userial_vendor.c
 *
 *  Description:   Contains vendor-specific userial functions
 *
 ******************************************************************************/

#define LOG_TAG "bt_userial_vendor"

#include <utils/Log.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include "bt_vendor.h"
#include "userial.h"
#include "userial_vendor.h"
#if (INTEL_AG6XX_UART == TRUE)
#include "imc_idi_bt_ioctl.h"
#endif
#include <errno.h>
#include <string.h>
#include <linux/tty.h>

/******************************************************************************
**  Constants & Macros
******************************************************************************/

//#ifndef VNDUSERIAL_DBG
#define VNDUSERIAL_DBG TRUE
//#endif

#if (VNDUSERIAL_DBG == TRUE)
#define VNDUSERIALDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define VNDUSERIALDBG(param, ...) {}
#endif

#define VNDUSERIALERR(param, ...) {ALOGE(param, ## __VA_ARGS__);}

#define VND_PORT_NAME_MAXLEN    256

/******************************************************************************
**  Local type definitions
******************************************************************************/

/* vendor serial control block */
typedef struct
{
    int fd;                     /* fd to Bluetooth device */
    struct termios termios;     /* serial terminal of BT port */
    char port_name[VND_PORT_NAME_MAXLEN];
} vnd_userial_cb_t;

/******************************************************************************
**  Static variables
******************************************************************************/

static vnd_userial_cb_t vnd_userial;

/*****************************************************************************
**   Helper Functions
*****************************************************************************/

/*******************************************************************************
**
** Function        userial_to_tcio_baud
**
** Description     helper function converts USERIAL baud rates into TCIO
**                  conforming baud rates
**
** Returns         TRUE/FALSE
**
*******************************************************************************/
uint8_t userial_to_tcio_baud(uint8_t cfg_baud, uint32_t *baud)
{
    if (cfg_baud == USERIAL_BAUD_115200)
        *baud = B115200;
    else if (cfg_baud == USERIAL_BAUD_4M)
        *baud = B4000000;
    else if (cfg_baud == USERIAL_BAUD_3M)
        *baud = B3000000;
    else if (cfg_baud == USERIAL_BAUD_2M)
        *baud = B2000000;
    else if (cfg_baud == USERIAL_BAUD_1M)
        *baud = B1000000;
    else if (cfg_baud == USERIAL_BAUD_921600)
        *baud = B921600;
    else if (cfg_baud == USERIAL_BAUD_460800)
        *baud = B460800;
    else if (cfg_baud == USERIAL_BAUD_230400)
        *baud = B230400;
    else if (cfg_baud == USERIAL_BAUD_57600)
        *baud = B57600;
    else if (cfg_baud == USERIAL_BAUD_19200)
        *baud = B19200;
    else if (cfg_baud == USERIAL_BAUD_9600)
        *baud = B9600;
    else if (cfg_baud == USERIAL_BAUD_1200)
        *baud = B1200;
    else if (cfg_baud == USERIAL_BAUD_600)
        *baud = B600;
    else
    {
        VNDUSERIALERR( "userial vendor open: unsupported baud idx %i", cfg_baud);
        *baud = B115200;
        return FALSE;
    }

    return TRUE;
}

#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
/*******************************************************************************
**
** Function        userial_ioctl_init_bt_wake
**
** Description     helper function to set the open state of the bt_wake if ioctl
**                  is used. it should not hurt in the rfkill case but it might
**                  be better to compile it out.
**
** Returns         none
**
*******************************************************************************/
void userial_ioctl_init_bt_wake(int fd)
{
    uint32_t bt_wake_state;

    /* assert BT_WAKE through ioctl */
    ioctl(fd, USERIAL_IOCTL_BT_WAKE_ASSERT, NULL);
    ioctl(fd, USERIAL_IOCTL_BT_WAKE_GET_ST, &bt_wake_state);
    VNDUSERIALDBG("userial_ioctl_init_bt_wake read back BT_WAKE state=%i", \
               bt_wake_state);
}
#endif // (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)


/*****************************************************************************
**   Userial Vendor API Functions
*****************************************************************************/

/*******************************************************************************
**
** Function        userial_vendor_init
**
** Description     Initialize userial vendor-specific control block
**
** Returns         None
**
*******************************************************************************/
void userial_vendor_init(void)
{
    vnd_userial.fd = -1;
    snprintf(vnd_userial.port_name, VND_PORT_NAME_MAXLEN, "%s", \
            BLUETOOTH_UART_DEVICE_PORT);
}

/*******************************************************************************
**
** Function        userial_vendor_st_drv_attach
**
** Description     Attach st driver here
**
** Returns         None
**
*******************************************************************************/
static void userial_vendor_st_drv_attach(void)
{
    int ret;
    int ldisc = N_INTEL_LDISC;
    ret = ioctl(vnd_userial.fd, TIOCSETD ,&ldisc);
    VNDUSERIALDBG("ioctl:%d",ret);
    ioctl(vnd_userial.fd, TIOCGETD, &ldisc);
    VNDUSERIALDBG("ldisc:%d", ldisc);
}

/*******************************************************************************
**
** Function        userial_vendor_open
**
** Description     Open the serial port with the given configuration
**
** Returns         device fd
**
*******************************************************************************/
int userial_vendor_open(void)
{
    vnd_userial.fd = -1;

    VNDUSERIALDBG("userial vendor open: opening %s", vnd_userial.port_name);

    if ((vnd_userial.fd = open(vnd_userial.port_name, O_RDWR | O_NOCTTY)) == -1)
    {
        VNDUSERIALERR("userial vendor open: unable to open %s, %s", vnd_userial.port_name, strerror(errno));
        return -1;
    }

    userial_vendor_st_drv_attach();
    return vnd_userial.fd;
}

/*******************************************************************************
**
** Function        userial_vendor_configure_serial
**
** Description     configures userial port.
**
** Returns         None
**
*******************************************************************************/
int userial_vendor_configure_serial(tUSERIAL_CFG *p_cfg, uint32_t baud_number)
{
    uint32_t baud;
    uint8_t data_bits;
    uint16_t parity;
    uint8_t stop_bits;
    if (!userial_to_tcio_baud(baud_number, &baud))
    {
        return -1;
    }

    if(p_cfg->fmt & USERIAL_DATABITS_8)
        data_bits = CS8;
    else if(p_cfg->fmt & USERIAL_DATABITS_7)
        data_bits = CS7;
    else if(p_cfg->fmt & USERIAL_DATABITS_6)
        data_bits = CS6;
    else if(p_cfg->fmt & USERIAL_DATABITS_5)
        data_bits = CS5;
    else
    {
        VNDUSERIALERR("userial vendor open: unsupported data bits");
        return -1;
    }

    if(p_cfg->fmt & USERIAL_PARITY_NONE)
        parity = 0;
    else if(p_cfg->fmt & USERIAL_PARITY_EVEN)
        parity = PARENB;
    else if(p_cfg->fmt & USERIAL_PARITY_ODD)
        parity = (PARENB | PARODD);
    else
    {
        VNDUSERIALERR("userial vendor open: unsupported parity bit mode");
        return -1;
    }

    if(p_cfg->fmt & USERIAL_STOPBITS_1)
        stop_bits = 0;
    else if(p_cfg->fmt & USERIAL_STOPBITS_2)
        stop_bits = CSTOPB;
    else
    {
        VNDUSERIALERR("userial vendor open: unsupported stop bits");
        return -1;
    }

    /*
     * serial port configuration :
     * CS8 8 bits data
     * CREAD to enable the receiver (should be enabled by default)
     * CRTSCTS to enable flow control
     */
    VNDUSERIALDBG("%s configuring serial with baud:%d", __func__, baud);
    tcgetattr(vnd_userial.fd, &vnd_userial.termios);
    vnd_userial.termios.c_cflag |=  CS8 | CREAD | CLOCAL | CRTSCTS;
    cfmakeraw(&vnd_userial.termios);

    cfsetospeed(&vnd_userial.termios,baud);
    cfsetispeed(&vnd_userial.termios, baud);
    tcsetattr(vnd_userial.fd, TCSANOW, &vnd_userial.termios);

    tcflush(vnd_userial.fd, TCIOFLUSH);
    return 0;
}

/*******************************************************************************
**
** Function        userial_vendor_close
**
** Description     Conduct vendor-specific close work
**
** Returns         None
**
*******************************************************************************/
void userial_vendor_close(void)
{
    int result;

    if (vnd_userial.fd == -1)
        return;

#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
    /* de-assert bt_wake BEFORE closing port */
    ioctl(vnd_userial.fd, USERIAL_IOCTL_BT_WAKE_DEASSERT, NULL);
#endif

    VNDUSERIALDBG("device fd = %d close", vnd_userial.fd);

    if ((result = close(vnd_userial.fd)) < 0)
        VNDUSERIALERR( "close(fd:%d) FAILED result:%d", vnd_userial.fd, result);

    vnd_userial.fd = -1;
}

/*******************************************************************************
**
** Function        userial_vendor_set_baud
**
** Description     Set new baud rate
**
** Returns         None
**
*******************************************************************************/
void userial_vendor_set_baud(uint8_t userial_baud)
{
    uint32_t tcio_baud;
    int ret, ldisc = 0;
    struct termios termAttr;
    speed_t baud_rate;
    userial_to_tcio_baud(userial_baud, &tcio_baud);
    ret = tcgetattr(vnd_userial.fd,&termAttr);
    baud_rate = cfgetospeed(&termAttr);
    VNDUSERIALDBG(" o : baud_rate : %d " , baud_rate);
    baud_rate = cfgetispeed(&termAttr);
    VNDUSERIALDBG(" i : baud_rate : %d " , baud_rate);
    cfmakeraw(&vnd_userial.termios);
    cfsetospeed(&vnd_userial.termios, tcio_baud);
    cfsetispeed(&vnd_userial.termios, tcio_baud);
    ret = tcsetattr(vnd_userial.fd, TCSANOW, &vnd_userial.termios);
    VNDUSERIALDBG("tcsetattr ret:%d errno:%d strerror:%s", ret, errno, strerror(errno));
    tcflush(vnd_userial.fd,TCIOFLUSH);
    ret = tcgetattr(vnd_userial.fd,&termAttr);
    baud_rate = cfgetospeed(&termAttr);
    VNDUSERIALDBG(" o : baud_rate : %d " , baud_rate);
    baud_rate = cfgetispeed(&termAttr);
    VNDUSERIALDBG(" i : baud_rate : %d " , baud_rate);
    ioctl(vnd_userial.fd, TIOCGETD, &ldisc);
    VNDUSERIALDBG(" ldisc: %d : ", ldisc);
    VNDUSERIALDBG("tcsetattr ret:%d errno:%d strerror:%s", ret, errno, strerror(errno));
}

/*******************************************************************************
**
** Function        userial_vendor_ioctl
**
** Description     ioctl inteface
**
** Returns         None
**
*******************************************************************************/
int userial_vendor_ioctl(userial_vendor_ioctl_op_t op, void *p_data)
{
    unsigned long data;
#if (LNP_LPM_ENABLED == TRUE)
    data = *((unsigned long *)p_data);
#else
    data = *((uint8_t*)p_data);
#endif
    VNDUSERIALDBG("userial_vendor_ioctl op:%d DATA: %lu\n",op, data);
    switch(op)
    {
#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
        case USERIAL_OP_ASSERT_BT_WAKE:
            VNDUSERIALDBG("## userial_vendor_ioctl: Asserting BT_Wake ##");
            ioctl(vnd_userial.fd, USERIAL_IOCTL_BT_WAKE_ASSERT, NULL);
            break;

        case USERIAL_OP_DEASSERT_BT_WAKE:
            VNDUSERIALDBG("## userial_vendor_ioctl: De-asserting BT_Wake ##");
            ioctl(vnd_userial.fd, USERIAL_IOCTL_BT_WAKE_DEASSERT, NULL);
            break;

        case USERIAL_OP_GET_BT_WAKE_STATE:
            ioctl(vnd_userial.fd, USERIAL_IOCTL_BT_WAKE_GET_ST, p_data);
            break;
#endif  //  (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
#if (BT_EN_VIA_USERIAL_IOCTL == TRUE)
        case USERIAL_OP_BT_EN:
            if (p_data)
            {
                data = *((unsigned long *)p_data);
                VNDUSERIALDBG("## userial_vendor_ioctl: USERIAL_OP_BT_EN:%lu", data);
                return ioctl(vnd_userial.fd, BT_FW_DOWNLOAD_INIT, data);
            }
            break;
        case USERIAL_OP_FW_CFG_CMPL:
            if (p_data)
            {
                data = *(unsigned long *)p_data;
                VNDUSERIALDBG("## userial_vendor_ioctl: USERIAL_OP_FW_CFG_CMPL:%lu", data);
                return ioctl(vnd_userial.fd, BT_FW_DOWNLOAD_COMPLETE, data);
            }
            break;
#endif  // (BT_EN_VIA_USERIAL_IOCTL == TRUE)
#if (LNP_LPM_ENABLED == TRUE)
        case USERIAL_OP_LPM_SET_IDLE_STATE:
            if (p_data)
            {
                data = *((unsigned long *)p_data);
                VNDUSERIALDBG("## userial_vendor_ioctl: USERIAL_OP_LPM_SET_IDLE_STATE:%lu", data);
                return ioctl(vnd_userial.fd, BT_FMR_IDLE, data);
            }
            break;
         case USERIAL_OP_LPM_ENABLE:
            if (p_data)
            {
                data = *((unsigned long *)p_data);
                VNDUSERIALDBG("## userial_vendor_ioctl: USERIAL_OP_LPM_ENABLE:%lu", data);
                return ioctl(vnd_userial.fd, BT_FMR_LPM_ENABLE, data);
            }
            break;
#endif
#if (INTEL_AG6XX_UART == TRUE)
        case USERIAL_OP_SET_DEVICE_STATE:
                VNDUSERIALDBG("%s USERIAL_OP_SET_DEVICE_STATE: %d", __func__, \
                                                                           data);
                return ioctl(vnd_userial.fd, IMC_IDI_BT_SET_POWER_STATE, data);
            break;
        case USERIAL_OP_SET_BT_WAKE_UP:
            VNDUSERIALDBG("%s USERIAL_OP_SET_BT_WAKE_UP:%d", __func__, data);
            return ioctl(vnd_userial.fd, IMC_IDI_BT_SET_BT_WUP, data);
            break;
        case USERIAL_OP_GET_CTS:
            VNDUSERIALDBG("%s USERIAL_OP_GET_CTS", __func__);
            return ioctl(vnd_userial.fd, IMC_IDI_BT_GET_CTS, NULL);
        case USERIAL_OP_SET_RTS:
            VNDUSERIALDBG("%s USERIAL_OP_SET_RTS:%d", __func__, data);
            return ioctl(vnd_userial.fd, IMC_IDI_BT_SET_RTS, data);
            break;
        case USERIAL_OP_DISABLE_SIGNALING:
            VNDUSERIALDBG("%s USERIAL_OP_DISABLE_SIGNALING", __func__);
            return ioctl(vnd_userial.fd, IMC_IDI_BT_DISABLE_SIGNALING, data);
        case USERIAL_OP_SET_FWCFG_REG:
            VNDUSERIALDBG("%s USERIAL_OP_SET_FWCFG_REG", __func__);
            return ioctl(vnd_userial.fd, IMC_IDI_BT_SET_SCU_FWCTL, data);
#endif
        default:
            break;
    }
    return 0;
}

/*******************************************************************************
**
** Function        userial_set_port
**
** Description     Configure UART port name
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int userial_set_port(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(vnd_userial.port_name, p_conf_value);

    return 0;
}

/*******************************************************************************
**
** Function        userial_vendor_set_rts
**
** Description     Set/clear the RTS bit
**                 'state': (0)= CLEAR, (1)= SET RTS
**
** Returns         return the value given to us
**
*******************************************************************************/
int userial_vendor_set_rts(int state)
{
    int flag, ret ,ldisc;
    ioctl(vnd_userial.fd, TIOCMGET, &flag);
    VNDUSERIALDBG("+%s RTS:%d flag:%d state:%d", __func__, state, flag, state);
    // Do we SET or CLEAR this bit (flag)?
    if(state == 1)        // OR 'flag' with TCIOCM_RTS, to set the bit
        flag|= TIOCM_RTS;
    else
        // AND 'flag' with the inverse of TCIOCM_RTS, to clear the bit
        flag&= ~TIOCM_RTS;

    // Set the new flag value
    ret = ioctl(vnd_userial.fd, TIOCMSET, &flag);
    VNDUSERIALDBG("%s SET RTS:%d flag:%d ret:%d", __func__, state, flag, ret);
    ret = ioctl(vnd_userial.fd, TIOCMGET, &flag);
    VNDUSERIALDBG("%s GET RTS:%d flag:%d ret:%d", __func__, state, flag, ret);
    ioctl(vnd_userial.fd, TIOCGETD, &ldisc);
    VNDUSERIALDBG("%s GET ldisc:%d", __func__, ldisc);
    return(state);
}
