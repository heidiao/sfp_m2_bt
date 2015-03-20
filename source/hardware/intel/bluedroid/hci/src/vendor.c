/******************************************************************************
 *
 *  Copyright (C) 2014 Google, Inc.
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

#define LOG_TAG "bt_vendor"

#include <assert.h>
#include <dlfcn.h>
#include <utils/Log.h>

#include "bt_hci_bdroid.h"
#include "bt_vendor_lib.h"
#include "userial.h"
#include "hci.h"
#include "osi.h"


tINT_CMD_CBACK p_int_evt_cb=NULL;
// TODO: eliminate these three.
extern tHCI_IF *p_hci_if;
extern bool fwcfg_acked;
extern unsigned long module_name;

void lpm_vnd_cback(uint8_t vnd_result);

static const char *VENDOR_LIBRARY_NAME = "libbt-vendor.so";
static const char *VENDOR_LIBRARY_SYMBOL_NAME = "BLUETOOTH_VENDOR_LIB_INTERFACE";

static void *lib_handle;
static bt_vendor_interface_t *vendor_interface;

static void firmware_config_cb(bt_vendor_op_result_t result);
static void sco_config_cb(bt_vendor_op_result_t result);
static void low_power_mode_cb(bt_vendor_op_result_t result);
static void sco_audiostate_cb(bt_vendor_op_result_t result);
static void *buffer_alloc(int size);
static void buffer_free(void *buffer);
static uint8_t transmit_cb(uint16_t opcode, void *buffer, tINT_CMD_CBACK callback);
static void epilog_cb(bt_vendor_op_result_t result);
static uint8_t int_evt_callback_reg_cb(tINT_CMD_CBACK p_cb);

static const bt_vendor_callbacks_t vendor_callbacks = {
  sizeof(vendor_callbacks),
  firmware_config_cb,
  sco_config_cb,
  low_power_mode_cb,
  sco_audiostate_cb,
  buffer_alloc,
  buffer_free,
  transmit_cb,
  epilog_cb
  , int_evt_callback_reg_cb
};

bool vendor_open(const uint8_t *local_bdaddr) {
  assert(lib_handle == NULL);

  lib_handle = dlopen(VENDOR_LIBRARY_NAME, RTLD_NOW);
  if (!lib_handle) {
    ALOGE("%s unable to open %s: %s", __func__, VENDOR_LIBRARY_NAME, dlerror());
    goto error;
  }

  vendor_interface = (bt_vendor_interface_t *)dlsym(lib_handle, VENDOR_LIBRARY_SYMBOL_NAME);
  if (!vendor_interface) {
    ALOGE("%s unable to find symbol %s in %s: %s", __func__, VENDOR_LIBRARY_SYMBOL_NAME, VENDOR_LIBRARY_NAME, dlerror());
    goto error;
  }

  int status = vendor_interface->init(&vendor_callbacks, (unsigned char *)local_bdaddr);
  if (status) {
    ALOGE("%s unable to initialize vendor library: %d", __func__, status);
    goto error;
  }

  return true;

error:;
  vendor_interface = NULL;
  if (lib_handle)
    dlclose(lib_handle);
  lib_handle = NULL;
  return false;
}

void vendor_close(void) {
  if (vendor_interface)
    vendor_interface->cleanup();

  if (lib_handle)
    dlclose(lib_handle);

  vendor_interface = NULL;
  lib_handle = NULL;
}

int vendor_send_command(bt_vendor_opcode_t opcode, void *param) {
  assert(vendor_interface != NULL);

  return vendor_interface->op(opcode, param);
}

// Called back from vendor library when the firmware configuration
// completes.
static void firmware_config_cb(bt_vendor_op_result_t result) {
  assert(bt_hc_cbacks != NULL);

  fwcfg_acked = true;
  p_int_evt_cb = NULL;

  if (module_name == MODULE_FM)
      userial_stop_read_thread();

  bt_hc_postload_result_t status = (result == BT_VND_OP_RESULT_SUCCESS)
      ? BT_HC_PRELOAD_SUCCESS
      : BT_HC_PRELOAD_FAIL;
  bt_hc_cbacks->preload_cb(NULL, status);
}

// Called back from vendor library to indicate status of previous
// SCO configuration request. This should only happen during the
// postload process.
static void sco_config_cb(UNUSED_ATTR bt_vendor_op_result_t result) {
  assert(p_hci_if != NULL);

  // Continue the rest of the postload process.
  p_hci_if->get_acl_max_len();
}

// Called back from vendor library to indicate status of previous
// LPM enable/disable request.
static void low_power_mode_cb(bt_vendor_op_result_t result) {
  lpm_vnd_cback(result != BT_VND_OP_RESULT_SUCCESS);
}

/******************************************************************************
**
** Function         sco_audiostate_cb
**
** Description      HOST/CONTROLLER VENDOR LIB CALLBACK API - This function is
**                  called when the libbt-vendor completed vendor specific codec
**                  setup request
**
** Returns          None
**
******************************************************************************/
static void sco_audiostate_cb(bt_vendor_op_result_t result)
{
    uint8_t status = (result == BT_VND_OP_RESULT_SUCCESS) ? 0 : 1;

    ALOGI("sco_audiostate_cb(status: %d)",status);
}

// Called by vendor library when it needs an HCI buffer.
static void *buffer_alloc(int size) {
  assert(bt_hc_cbacks != NULL);
  return bt_hc_cbacks->alloc(size);
}

// Called by vendor library when it needs to free a buffer allocated with
// |buffer_alloc|.
static void buffer_free(void *buffer) {
  assert(bt_hc_cbacks != NULL);
  bt_hc_cbacks->dealloc(buffer);
}

// Called back from vendor library when it wants to send an HCI command.
static uint8_t transmit_cb(uint16_t opcode, void *buffer, tINT_CMD_CBACK callback) {
  assert(p_hci_if != NULL);
  return p_hci_if->send_int_cmd(opcode, (HC_BT_HDR *)buffer, callback);
}

// Called back from vendor library when the epilog procedure has
// completed. It is safe to call vendor_interface->cleanup() after
// this callback has been received.
static void epilog_cb(UNUSED_ATTR bt_vendor_op_result_t result) {
}

/******************************************************************************
**
** Function         int_evt_callback_reg_cb
**
** Description      HOST/CONTROLLER VEDNOR LIB CALLBACK API - This function is
**                  called from the libbt-vendor to configure callback function
**                  to send out anyc events. This is used by libbt to get first
**                  default bd data event after turning on BT IP.
**
** Returns          TRUE/FALSE
**
******************************************************************************/
static uint8_t int_evt_callback_reg_cb(tINT_CMD_CBACK p_cb)
{
    if (p_cb)
    {
        p_int_evt_cb = p_cb;
        ALOGI("%s register DONE", __func__);
        return BT_HC_STATUS_SUCCESS;
    }
    return BT_HC_STATUS_FAIL;
}
