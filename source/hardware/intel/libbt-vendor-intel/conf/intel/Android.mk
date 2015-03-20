 ######################################################################
 # Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 #
 # This software is licensed under the terms of the GNU General Public
 # License version 2, as published by the Free Software Foundation, and
 # may be copied, distributed, and modified under those terms.
 #
 # This program is distributed in the hope that it will be useful,
 # but WITHOUT ANY WARRANTY; without even the implied warranty of
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 # GNU General Public License for more details.
 #####################################################################

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := bt_vendor.conf
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT)/etc/bluetooth
ifneq ($(TARGET_BOARD_PLATFORM), )
LOCAL_SRC_FILES := $(TARGET_BOARD_PLATFORM)/$(LOCAL_MODULE)
else
LOCAL_SRC_FILES := $(TARGET_PRODUCT)/$(LOCAL_MODULE)
endif

include $(BUILD_PREBUILT)
