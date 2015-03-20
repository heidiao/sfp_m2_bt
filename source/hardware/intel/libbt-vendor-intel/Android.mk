 ######################################################################
 #
 # Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #      http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 #####################################################################

LOCAL_PATH := $(call my-dir)

ifeq ($(BOARD_HAVE_BLUETOOTH_INTEL), true)

#bt_vendor_test

include $(CLEAR_VARS)

BDROID_DIR := $(TOP_DIR)hardware/intel/bluedroid

LOCAL_SRC_FILES := src/test_hs.c

LOCAL_SHARED_LIBRARIES := \
        libcutils

LOCAL_C_INCLUDES += \
        $(LOCAL_PATH)/include \
        $(bdroid_C_INCLUDES)

LOCAL_MODULE := bt_vendor_test
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := intel

include $(BUILD_EXECUTABLE)

# libbt-vendor.so

include $(CLEAR_VARS)

BDROID_DIR := $(TOP_DIR)hardware/intel/bluedroid

ifneq ($(BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR),)
LOCAL_CFLAGS += -DHAS_BDROID_BUILDCFG
endif

LOCAL_SRC_FILES := \
        src/bt_vendor.c \
        src/hardware.c \
        src/userial_vendor.c \
        src/upio.c \
        src/conf.c

LOCAL_C_INCLUDES += \
        $(LOCAL_PATH)/include \
        $(BDROID_DIR)/hci/include

ifneq ($(BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR),)
LOCAL_C_INCLUDES += \
        $(BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR)
endif

LOCAL_SHARED_LIBRARIES := \
        libcutils

ifneq ($(BOARD_SUPPORTS_NVM), false)
LOCAL_CFLAGS += -DBT_USE_NVM
endif

ifeq ($(TARGET_PRODUCT), hsb)
LOCAL_CFLAGS += -DINTEL_HSB_PLATFORM
endif

ifeq ($(BOARD_USE_64BIT_KERNEL), true)
LOCAL_CFLAGS += -DSUPPORT_64BIT_KERNEL
endif

LOCAL_MODULE := libbt-vendor
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_OWNER := intel
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR_SHARED_LIBRARIES)

include $(LOCAL_PATH)/vnd_buildcfg.mk

include $(BUILD_SHARED_LIBRARY)

include $(LOCAL_PATH)/conf/intel/Android.mk

endif # BOARD_HAVE_BLUETOOTH_INTEL
