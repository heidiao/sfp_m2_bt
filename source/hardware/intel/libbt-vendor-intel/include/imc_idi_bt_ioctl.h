/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
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
 */

#ifndef IMC_IDI_BT_IOCTL_H
#define IMC_IDI_BT_IOCTL_H
#include <linux/ioctl.h>

#define IMC_IDI_MAGIC 'i'

/* Changes Device states */
#define IMC_IDI_BT_SET_POWER_STATE _IOWR(IMC_IDI_MAGIC, 1, unsigned long)
/* Changes BT WAKEUP high/low */
#define IMC_IDI_BT_SET_BT_WUP _IOW(IMC_IDI_MAGIC, 2, unsigned long)
/* Changes HOST_WAKEUP high/low: Not used */
#define IMC_IDI_BT_GET_HOST_WUP _IOR(IMC_IDI_MAGIC, 3, unsigned long)
/* Sets RTS */
#define IMC_IDI_BT_SET_RTS _IOW(IMC_IDI_MAGIC, 4, unsigned long)
/* Gets RTS: Not used */
#define IMC_IDI_BT_GET_RTS _IOR(IMC_IDI_MAGIC, 5, unsigned long)
/* Gets CTS status */
#define IMC_IDI_BT_GET_CTS _IOR(IMC_IDI_MAGIC, 6, unsigned long)
#define IMC_IDI_BT_SET_TEMP_MEAS _IOW(IMC_IDI_MAGIC, 7, unsigned long)
#define IMC_IDI_BT_GET_TEMP_MEAS _IOR(IMC_IDI_MAGIC, 8, unsigned long)
#define IMC_IDI_BT_GET_TEMP_MEAS _IOR(IMC_IDI_MAGIC, 8, unsigned long)
#define IMC_IDI_BT_DISABLE_SIGNALING _IO(IMC_IDI_MAGIC, 9)
#define IMC_IDI_BT_SET_SCU_FWCTL _IOW(IMC_IDI_MAGIC, 10, unsigned long)
#define IMC_IDI_BT_GET_SCU_FWCTL _IOR(IMC_IDI_MAGIC, 11, unsigned long)

#endif
