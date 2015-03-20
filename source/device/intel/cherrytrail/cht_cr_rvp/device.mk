# FIXME surfaceflinger explodes if this is not "gmin"
TARGET_BOARD_PLATFORM := gmin

PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/init.rc:root/init.cht_cr_rvp.rc \
    $(LOCAL_PATH)/init.recovery.rc:root/init.recovery.cht_cr_rvp.rc \
    $(LOCAL_PATH)/ueventd.rc:root/ueventd.cht_cr_rvp.rc \
    $(LOCAL_PATH)/fstab:root/fstab

# These files are extremely board-specific and doesn't go in the mix-in
PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/camera_profiles.xml:system/etc/camera_profiles.xml \
    $(LOCAL_PATH)/media_profiles.xml:system/etc/media_profiles.xml \
    $(LOCAL_PATH)/../media_codecs.xml:system/etc/media_codecs.xml \
    $(LOCAL_PATH)/../video_isv_profile.xml:system/etc/video_isv_profile.xml

# Camera ISP Firmware
PRODUCT_PACKAGES += \
    shisp_2401a0_v21.bin \

# FIXME: this should go into a mix-in, but how to organize it?
PRODUCT_AAPT_CONFIG := large xlarge mdpi tvdpi

# Audio package
PRODUCT_PACKAGES += meta.package.audio

# Vibrator package
PRODUCT_PACKAGES += \
    parameter-framework.vibrator.cherrytrail \
    libfs-subsystem

# HAL prebuilts - Vibrator
PRODUCT_PACKAGES += \
    vibrator.$(TARGET_BOARD_PLATFORM)

# reboot in charger mode after a gracefull shutdown
PRODUCT_PROPERTY_OVERRIDES += ro.sys.powerctl.no.shutdown = 1

BUILD_KERNEL_FROM_SOURCES := 1

# ----------------- BEGIN MIX-IN DEFINITIONS -----------------
# Mix-In definitions are auto-generated by mixin-update
##############################################################
# Source: device/intel/mixins/groups/boot-arch/efi/product.mk
##############################################################
TARGET_UEFI_ARCH := x86_64
BIOS_VARIANT := release

$(call inherit-product,build/target/product/verity.mk)

PRODUCT_SYSTEM_VERITY_PARTITION := /dev/block/by-name/android_system

PRODUCT_PACKAGES += \
	setup_fs \
	pstore-clean

PRODUCT_COPY_FILES += \
	frameworks/native/data/etc/android.software.verified_boot.xml:system/etc/permissions/android.software.verified_boot.xml

BOARD_SFU_UPDATE := hardware/intel/efi_capsules/$(BIOS_VARIANT)/$(TARGET_PRODUCT).fv
EFI_IFWI_BIN := hardware/intel/efi_capsules/$(BIOS_VARIANT)/$(TARGET_PRODUCT)_ifwi.bin
EFI_EMMC_BIN := hardware/intel/efi_capsules/$(BIOS_VARIANT)/$(TARGET_PRODUCT)_emmc.bin

ifneq ($(CALLED_FROM_SETUP),true)
ifeq ($(wildcard $(BOARD_SFU_UPDATE)),)
$(warning $(BOARD_SFU_UPDATE) not found, OTA updates will not provide a firmware capsule)
BOARD_SFU_UPDATE :=
endif
ifeq ($(wildcard $(EFI_EMMC_BIN)),)
$(warning $(EFI_EMMC_BIN) not found, flashfiles will not include 2nd stage EMMC firmware)
EFI_EMMC_BIN :=
endif
ifeq ($(wildcard $(EFI_IFWI_BIN)),)
$(warning $(EFI_IFWI_BIN) not found, IFWI binary will not be provided in out/dist/)
EFI_IFWI_BIN :=
endif
endif

ifneq ($(BOARD_SFU_UPDATE),)
PRODUCT_COPY_FILES += $(BOARD_SFU_UPDATE):system/etc/firmware/BIOSUPDATE.fv
endif

##############################################################
# Source: device/intel/mixins/groups/kernel/gmin64/product.mk.1
##############################################################
TARGET_KERNEL_ARCH := x86_64

##############################################################
# Source: device/intel/mixins/groups/kernel/gmin64/product.mk
##############################################################

ifneq ($(filter kernel,$(MAKECMDGOALS)),)
  BUILD_KERNEL_FROM_SOURCES := 1
endif

MAKECMDGOALS := $(strip $(filter-out kernel,$(MAKECMDGOALS)))

# if using prebuilts
ifeq ($(BUILD_KERNEL_FROM_SOURCES),)

LOCAL_KERNEL_MODULE_FILES :=
ifeq ($(TARGET_PREBUILT_KERNEL),)
  # use default kernel
  LOCAL_KERNEL_PATH := device/intel/gmin-kernel/$(TARGET_KERNEL_ARCH)
  LOCAL_KERNEL := $(LOCAL_KERNEL_PATH)/bzImage
  LOCAL_KERNEL_MODULE_FILES := $(wildcard $(LOCAL_KERNEL_PATH)/modules/*)
  LOCAL_KERNEL_MODULE_TREE_PATH := $(LOCAL_KERNEL_PATH)/lib/modules
else
  # use custom kernel
  LOCAL_KERNEL := $(TARGET_PREBUILT_KERNEL)
  ifneq ($(TARGET_PREBUILT_KERNEL_MODULE_PATH),)
    LOCAL_KERNEL_MODULE_FILES := $(wildcard $(TARGET_PREBUILT_KERNEL_MODULE_PATH)/*)
  endif
  ifneq ($(TARGET_PREBUILT_KERNEL_MODULE_TREE_PATH),)
    LOCAL_KERNEL_MODULE_TREE_PATH := $(TARGET_PREBUILT_KERNEL_MODULE_TREE_PATH)
  endif
endif

ifneq ($(LOCAL_KERNEL_MODULE_TREE_PATH),)
  LOCAL_KERNEL_VERSION := $(shell file -k $(LOCAL_KERNEL) | sed -nr 's|.*version ([^ ]+) .*|\1|p')
  ifeq ($(LOCAL_KERNEL_VERSION),)
    $(error Cannot get version for kernel '$(LOCAL_KERNEL)')
  endif

  FULL_TREE_PATH := $(LOCAL_KERNEL_MODULE_TREE_PATH)/$(LOCAL_KERNEL_VERSION)
  # Verify FULL_TREE_PATH is an existing folder
  ifneq ($(shell test -d $(FULL_TREE_PATH) && echo 1), 1)
    $(error '$(FULL_TREE_PATH)' does not exist or is not a directory)
  endif

  LOCAL_KERNEL_MODULE_TREE_FILES := $(shell cd $(LOCAL_KERNEL_MODULE_TREE_PATH) && \
                                            find $(LOCAL_KERNEL_VERSION) -type f)
endif

# Copy kernel into place
PRODUCT_COPY_FILES += \
	$(LOCAL_KERNEL):kernel \
	$(foreach f, $(LOCAL_KERNEL_MODULE_FILES), $(f):system/lib/modules/$(notdir $(f))) \
  $(foreach f, $(LOCAL_KERNEL_MODULE_TREE_FILES), $(LOCAL_KERNEL_PATH)/lib/modules/$(f):system/lib/modules/$(f))

endif
##############################################################
# Source: device/intel/mixins/groups/dalvik-heap/tablet-7in-hdpi-1024/product.mk
##############################################################
#include frameworks/native/build/tablet-7in-hdpi-1024-dalvik-heap.mk
PRODUCT_PROPERTY_OVERRIDES += \
    dalvik.vm.heapstartsize=8m \
    dalvik.vm.heapgrowthlimit=100m \
    dalvik.vm.heapsize=174m \
    dalvik.vm.heaptargetutilization=0.75 \
    dalvik.vm.heapminfree=512k \
    dalvik.vm.heapmaxfree=8m
##############################################################
# Source: device/intel/mixins/groups/houdini/true/product.mk
##############################################################
$(call inherit-product-if-exists, vendor/intel/houdini/houdini.mk)

##############################################################
# Source: device/intel/mixins/groups/graphics/ufo_gen8/product.mk
##############################################################
#
# Hardware Accelerated Graphics
#
PRODUCT_PACKAGES += \
    libdrm \
    libdrm_intel \
    ufo.prop

#
# Color conversion library
#
PRODUCT_PACKAGES += \
    libI420colorconvert

ifneq ($(BOARD_HAVE_GEN_GFX_SRC),true)
    # UFO prebuilts
    PRODUCT_PACKAGES += ufo_prebuilts
    ifneq ($(TARGET_2ND_ARCH),)
        PRODUCT_PACKAGES += ufo_prebuilts_32
    endif

else # ufo packages when building from source
    PRODUCT_PACKAGES += ufo
    PRODUCT_PACKAGES += ufo_test
endif

PRODUCT_PROPERTY_OVERRIDES += ro.opengles.version = 196609
PRODUCT_COPY_FILES += \
    device/intel/common/ufo/init.ufo.sh:system/etc/init.ufo.sh

##############################################################
# Source: device/intel/mixins/groups/camera/isp3/product.mk
##############################################################
ifeq ($(TARGET_BOARD_PLATFORM),)
    $(error Please define TARGET_BOARD_PLATFORM in product-level Makefile)
endif

# Android framework boilerplate.
PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.hardware.camera.xml:system/etc/permissions/android.hardware.camera.xml \
    frameworks/native/data/etc/android.hardware.camera.front.xml:system/etc/permissions/android.hardware.camera.front.xml \

# Camera HAL (camera3hal)
PRODUCT_PACKAGES += camera.$(TARGET_BOARD_PLATFORM)

# Kernel module initialization helper script
PRODUCT_COPY_FILES += $(LOCAL_PATH)/init.atomisp.sh:system/etc/init.atomisp.sh

# 3A/AIQ
PRODUCT_PACKAGES += \
    libmfldadvci \
    dummy.aiqb \

# ISP Firmware
PRODUCT_PACKAGES += \
    shisp_2400b0_v21.bin \

#Refcam
PRODUCT_PACKAGES_DEBUG += \
    RefCam1 \
    RefCam2
##############################################################
# Source: device/intel/mixins/groups/rfkill/true/product.mk
##############################################################
PRODUCT_COPY_FILES += device/intel/common/rfkill/rfkill-init.sh:system/bin/rfkill-init.sh
##############################################################
# Source: device/intel/mixins/groups/bluetooth/lnp/product.mk
##############################################################
PRODUCT_PACKAGES += \
    bd_prov \
    libbt-vendor \
    audio.a2dp.default \
    rfkill_bt.sh \
    bt_vendor.default.conf \
    rtl8723b_config \
    rtl8723b_fw \
    bdt \
    lnp_default_conf \
    bt_lightning_peak \
    bddata_A0\
    bddata_B0\
    bddata_K0\
    370b000600020d0e00.sfi\
    370b00060002280d00.seq\
    370b10060002220e00.sfi\

PRODUCT_COPY_FILES += frameworks/native/data/etc/android.hardware.bluetooth.xml:system/etc/permissions/android.hardware.bluetooth.xml \
		frameworks/native/data/etc/android.hardware.bluetooth_le.xml:system/etc/permissions/android.hardware.bluetooth_le.xml\
    device/intel/common/bluetooth/lnp/load_btfw.sh:system/bin/load_btfw.sh
##############################################################
# Source: device/intel/mixins/groups/nfc/pn547/product.mk.1
##############################################################
# Default (fallback) location of generic PN547 conf files
# (if needed, can be overridden in board's device.mk
# with board-customized conf file location)
NFC_CONF_PATH ?= device/intel/common/nfc/pn547

# PN547-specific configuration files
PRODUCT_COPY_FILES += \
    $(NFC_CONF_PATH)/libnfc-brcm.conf:system/etc/libnfc-brcm.conf \
    $(NFC_CONF_PATH)/libnfc-nxp.conf:system/etc/libnfc-nxp.conf

# PN547-specific NFC packages
PRODUCT_PACKAGES += \
    libnfc-nci \
    libnfc_nci_jni \
    nfc_nci.pn54x.default \
    NfcNci \
    Tag \
    com.android.nfc_extras \
    libpn547_fw
    #nfc_nci_pn547.grouper \

##############################################################
# Source: device/intel/mixins/groups/nfc/pn547/product.mk
##############################################################
# Common NFC feature files
PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.hardware.nfc.xml:system/etc/permissions/android.hardware.nfc.xml \
    frameworks/native/data/etc/android.hardware.nfc.hce.xml:system/etc/permissions/android.hardware.nfc.hce.xml

# Common NFC packages
PRODUCT_PACKAGES += \
    NfcNci
# Tag application may be also included with GMS package
ifneq ($(FLAG_GMS_AVAILABLE),yes)
PRODUCT_PACKAGES += \
    Tag
endif
##############################################################
# Source: device/intel/mixins/groups/wlan/iwlwifi/product.mk
##############################################################
PRODUCT_PACKAGES += \
    hostapd \
    hostap_cli \
    wpa_supplicant \
    wpa_cli

#copy iwlwifi wpa p2p config files
 PRODUCT_COPY_FILES += \
        device/intel/common/wlan/iwlwifi/p2p_supplicant.conf:system/etc/wifi/p2p_supplicant.conf \
        device/intel/common/wlan/iwlwifi/wpa_supplicant.conf:system/etc/wifi/wpa_supplicant.conf \
        frameworks/native/data/etc/android.hardware.wifi.xml:system/etc/permissions/android.hardware.wifi.xml \
        frameworks/native/data/etc/android.hardware.wifi.direct.xml:system/etc/permissions/android.hardware.wifi.direct.xml

PRODUCT_COPY_FILES += \
        device/intel/common/wlan/iwlwifi/load_iwlwifi.sh:system/bin/load_iwlwifi.sh

$(call inherit-product-if-exists, vendor/intel/fw/iwl/iwl-fw-8000-cht-gmin.mk)
$(call inherit-product-if-exists, kernel/gmin/uefi/modules/wlan/iwlwifi/Androidiwlwifi.mk)

##############################################################
# Source: device/intel/mixins/groups/widi/gen/product.mk
##############################################################
#widi
PRODUCT_PACKAGES += widi
##############################################################
# Source: device/intel/mixins/groups/audio/cht-tlv320aic31xx/product.mk
##############################################################
# Tinyalsa
PRODUCT_PACKAGES_DEBUG += \
         tinymix \
         tinyplay \
         tinycap

# SST Firmware
PRODUCT_PACKAGES += \
    fw_sst_22a8.bin

# DFW Firmware
PRODUCT_PACKAGES += \
    dfw_sst.bin

# SST probing tools
PRODUCT_PACKAGES_DEBUG += \
    tinyprobe

# Support background music playback for Widi Multitasking
ENABLE_BACKGROUND_MUSIC := true
##############################################################
# Source: device/intel/mixins/groups/media/ufo/product.mk
##############################################################
# libstagefrighthw
BUILD_WITH_FULL_STAGEFRIGHT := true
PRODUCT_PACKAGES += \
    libstagefrighthw

# Media SDK and OMX IL components
PRODUCT_PACKAGES += \
    libmfxhw32 \
    libmfxsw32 \
    libmfx_omx_core \
    libmfx_omx_components_hw \
    libmfx_omx_components_sw \
    libgabi++-mfx \
    libstlport-mfx

# Enable ALAC
PRODUCT_PACKAGES += \
    libstagefright_soft_alacdec

# Build OMX wrapper codecs
PRODUCT_PACKAGES += \
    libmdp_omx_core \
    libstagefright_soft_mp3dec_mdp \
    libstagefright_soft_aacdec_mdp \
    libstagefright_soft_aacenc_mdp \
    libstagefright_soft_vorbisdec_mdp \
    libstagefright_soft_wmadec_mdp \
    libstagefright_soft_amrenc_mdp \
    libstagefright_soft_amrdec_mdp

# Decoding ASF/MPEG4-ASP/H263
PRODUCT_PACKAGES += \
	libasfparser \
	libmixvbp \
	libmixvbp_mpeg4 \
	libva_videodecoder \
	libOMXVideoDecoderMPEG4 \
	libOMXVideoDecoderH263 \
	libOMXVideoDecoderVP9HWR \
	libva_videoencoder \
	libOMXVideoEncoderMPEG4 \
	libOMXVideoEncoderH263 \
	libintelmetadatabuffer \
	libwrs_omxil_core_pvwrapped \
	libwrs_omxil_base

# Copy config files to system
PRODUCT_COPY_FILES += device/intel/common/media/wrs_omxil_components.list:system/etc/wrs_omxil_components.list

# skia accelerations
PRODUCT_PACKAGES += \
    libskjpegturbo \
    libskiaimagehw \
    libvaimagedecoder \
    libvaimagedecoder_genx
##############################################################
# Source: device/intel/mixins/groups/usb/host+acc/product.mk
##############################################################
PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.hardware.usb.accessory.xml:system/etc/permissions/android.hardware.usb.accessory.xml \
    frameworks/native/data/etc/android.hardware.usb.host.xml:system/etc/permissions/android.hardware.usb.host.xml

# usb accessory
PRODUCT_PACKAGES += \
    com.android.future.usb.accessory

##############################################################
# Source: device/intel/mixins/groups/usb-gadget/g_android/product.mk
##############################################################
# Set default USB interface
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += persist.sys.usb.config=mtp

# Enable Secure Debugging
ifneq ($(TARGET_BUILD_VARIANT),eng)
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.adb.secure=1
endif

# Add Intel adb keys for userdebug/eng builds
ifneq ($(TARGET_BUILD_VARIANT),user)
PRODUCT_COPY_FILES += device/intel/common/usb-gadget/adb_keys:root/adb_keys
endif
##############################################################
# Source: device/intel/mixins/groups/touch/atmel1000/product.mk
##############################################################
PRODUCT_COPY_FILES += \
        device/intel/common/touch/ATML1000_00_4a_03EB_8C0E.idc:system/usr/idc/ATML1000_00_4a_03EB_8C0E.idc \
        device/intel/common/touch/atmel_mxt_ts.idc:system/usr/idc/atmel_mxt_ts.idc \
        frameworks/native/data/etc/android.hardware.touchscreen.multitouch.jazzhand.xml:system/etc/permissions/android.hardware.touchscreen.multitouch.jazzhand.xml

##############################################################
# Source: device/intel/mixins/groups/device-type/tablet/product.mk
##############################################################
PRODUCT_CHARACTERISTICS := tablet

PRODUCT_COPY_FILES += \
        frameworks/native/data/etc/tablet_core_hardware.xml:system/etc/permissions/tablet_core_hardware.xml

##############################################################
# Source: device/intel/mixins/groups/gms/true/product.mk
##############################################################
$(call inherit-product-if-exists, vendor/google/gms/products/intel_gms.mk)
##############################################################
# Source: device/intel/mixins/groups/debug-tools/true/product.mk
##############################################################
# If this a debugging build include the public debug modules
ifneq ($(filter eng userdebug,$(TARGET_BUILD_VARIANT)),)

PRODUCT_PACKAGES += \
    AndroidTerm \
    libjackpal-androidterm4 \
    busybox \
    peeknpoke

endif
##############################################################
# Source: device/intel/mixins/groups/charger/true/product.mk
##############################################################
PRODUCT_PACKAGES += charger charger_res_images

##############################################################
# Source: device/intel/mixins/groups/thermal/ituxd/product.mk.1
##############################################################
ENABLE_ITUXD := true
ifeq (false, true)
ITUX_MODEM_ZONE := true
endif
PRODUCT_PACKAGES += ituxd
PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/config/thermal_sensor_config.xml:/system/etc/thermal_sensor_config.xml \
    $(LOCAL_PATH)/config/thermal_throttle_config.xml:/system/etc/thermal_throttle_config.xml
##############################################################
# Source: device/intel/mixins/groups/thermal/ituxd/product.mk
##############################################################
# Thermal daemon
PRODUCT_PACKAGES += thermal-daemon thermal_lite

PRODUCT_COPY_FILES += \
    external/thermal_daemon/data/thermal-conf.xml:system/etc/thermal-daemon/thermal-conf.xml \
    external/thermal_daemon/data/thermal-cpu-cdev-order.xml:system/etc/thermal-daemon/thermal-cpu-cdev-order.xml
##############################################################
# Source: device/intel/mixins/groups/gps/bcm4752/product.mk
##############################################################
PRODUCT_PACKAGES += \
    gps.conf \
    gps.xml \
    gps.$(TARGET_BOARD_PLATFORM) \
    gpsd \
    gpscerd \

PRODUCT_PROPERTY_OVERRIDES += \
        ro.spid.gps.FrqPlan=FRQ_PLAN_26MHZ_2PPM\
        ro.spid.gps.RfType=GL_RF_4752_BRCM_EXT_LNA\
        ro.gnss.sv.status=true

PRODUCT_COPY_FILES += \
        frameworks/native/data/etc/android.hardware.location.gps.xml:system/etc/permissions/android.hardware.location.gps.xml \

##############################################################
# Source: device/intel/mixins/groups/debug-logs/true/product.mk
##############################################################
# Enable logs on file system for eng and userdebug builds
ifneq ($(TARGET_BUILD_VARIANT),user)
PRODUCT_COPY_FILES += \
    device/intel/common/debug/init.logs.rc:root/init.logs.rc \
    vendor/intel/tools/log_capture/log_service/start_log_srv.sh:system/bin/start_log_srv.sh
PRODUCT_PACKAGES += logcatext
endif
##############################################################
# Source: device/intel/mixins/groups/debug-crashlogd/true/product.mk
##############################################################
# Enable crashlogd for eng and userdebug builds
ifneq ($(TARGET_BUILD_VARIANT),user)
PRODUCT_COPY_FILES += \
	device/intel/common/debug/init.crashlogd.rc:root/init.crashlogd.rc \
	$(call add-to-product-copy-files-if-exists,$(LOCAL_PATH)/../ingredients.conf:system/etc/ingredients.conf)
PRODUCT_PACKAGES += crashlogd
endif
##############################################################
# Source: device/intel/mixins/groups/debug-coredump/true/product.mk
##############################################################
# Enable core dump on /data/logs/core for eng and userdebug builds
ifneq ($(TARGET_BUILD_VARIANT),user)
PRODUCT_COPY_FILES += device/intel/common/debug/init.coredump.rc:root/init.coredump.rc
endif
##############################################################
# Source: device/intel/mixins/groups/debug-phonedoctor/true/product.mk
##############################################################
# Enable Phone Doctor for eng and userdebug builds
ifneq ($(TARGET_BUILD_VARIANT),user)
PRODUCT_COPY_FILES += $(call add-to-product-copy-files-if-exists,\
    vendor/intel/tools/log_infra/crashinfo/crashinfo:system/bin/crashinfo:intel)
PRODUCT_PACKAGES += crash_package
endif
##############################################################
# Source: device/intel/mixins/groups/debug-charging/true/product.mk
##############################################################
# make console and adb available in charging mode for eng and userdebug builds
ifneq ($(TARGET_BUILD_VARIANT),user)
PRODUCT_COPY_FILES += device/intel/common/debug/init.debug-charging.rc:root/init.debug-charging.rc
endif
##############################################################
# Source: device/intel/mixins/groups/hdmi_settings/true/product.mk
##############################################################
PRODUCT_PACKAGES += HdmiSettings
DEVICE_PACKAGE_OVERLAYS += device/intel/common/overlays_hdmisettings
##############################################################
# Source: device/intel/mixins/groups/lights/true/product.mk
##############################################################
# Lights HAL
PRODUCT_PACKAGES += lights.$(TARGET_BOARD_PLATFORM)

##############################################################
# Source: device/intel/mixins/groups/security/txei/product.mk
##############################################################
#Identity Protection Technology (IPT)
PRODUCT_PACKAGES += \
    libiha \
    epid_certificates \
    SepService

PRODUCT_PACKAGES_DEBUG += \
    TXEI_TEST TXEI_SEC_TOOLS
##############################################################
# Source: device/intel/mixins/groups/hw-keystore/txei/product.mk
##############################################################
PRODUCT_PACKAGES += \
	keystore.${TARGET_BOARD_PLATFORM} \
	keymaster_meid

##############################################################
# Source: device/intel/mixins/groups/hdcpd/true/product.mk
##############################################################
# Enable media content protection services

# HDCP Daemon
PRODUCT_PACKAGES += hdcpd
##############################################################
# Source: device/intel/mixins/groups/bcu/true/product.mk
##############################################################
# App for releasing the capped cpu freq
PRODUCT_PACKAGES += \
    bcu_cpufreqrel
##############################################################
# Source: device/intel/mixins/groups/silentlake/true/product.mk
##############################################################
SILENTLAKE_CONFIG_DEFINED := true

# sl_vmm.efi is the Silent Lake VMM, but it does not need to
# be "built" which is why it is not in the list.
# It is picked up directly bythe  part of the build that constructs
# the ESP partition in the case that silent lake is enabled
PRODUCT_PACKAGES += silentlake_OEM_libs sl_si_service vidt_sign.bin
PRODUCT_PACKAGES += libsl_installer.so
##############################################################
# Source: device/intel/mixins/groups/libmds/mds/product.mk
##############################################################
# MDS - Intel multidisplay service

# copy permission files
PRODUCT_COPY_FILES += \
    vendor/intel/hardware/libmedia_utils/libmultidisplay/com.intel.multidisplay.xml:system/etc/permissions/com.intel.multidisplay.xml

PRODUCT_PACKAGES += \
    libmultidisplay \
    libmultidisplayjni \
    com.intel.multidisplay
##############################################################
# Source: device/intel/mixins/groups/vpp/isv/product.mk
##############################################################
# libisv_omx_core
PRODUCT_PACKAGES += \
    libisv_omx_core

# isv settings
PRODUCT_PACKAGES += \
    VppSettings

##############################################################
# Source: device/intel/mixins/groups/ccf/enable/product.mk
##############################################################

#ccf
PRODUCT_PACKAGES += \
    SimpleChat \
    SimpleDiscovery \
    MultiConnect \
    FileTransfer

##############################################################
# Source: device/intel/mixins/groups/sensor-hubs/ish/product.mk
##############################################################
# ISH sensorhubd modules
PRODUCT_PACKAGES += \
    sensorhubd      \
    libsensorhub

# ISH Sensor HAL modules
PRODUCT_PACKAGES += \
    sensors.$(TARGET_BOARD_PLATFORM)

PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.hardware.sensor.accelerometer.xml:system/etc/permissions/android.hardware.sensor.accelerometer.xml \
    frameworks/native/data/etc/android.hardware.sensor.barometer.xml:system/etc/permissions/android.hardware.sensor.barometer.xml \
    frameworks/native/data/etc/android.hardware.sensor.compass.xml:system/etc/permissions/android.hardware.sensor.compass.xml \
    frameworks/native/data/etc/android.hardware.sensor.gyroscope.xml:system/etc/permissions/android.hardware.sensor.gyroscope.xml \
    frameworks/native/data/etc/android.hardware.sensor.light.xml:system/etc/permissions/android.hardware.sensor.light.xml \
    frameworks/native/data/etc/android.hardware.sensor.proximity.xml:system/etc/permissions/android.hardware.sensor.proximity.xml \
    frameworks/native/data/etc/android.hardware.sensor.stepcounter.xml:system/etc/permissions/android.hardware.sensor.stepcounter.xml \
    frameworks/native/data/etc/android.hardware.sensor.stepdetector.xml:system/etc/permissions/android.hardware.sensor.stepdetector.xml \
##############################################################
# Source: device/intel/mixins/groups/initrc-aosp/override/product.mk
##############################################################
PRODUCT_COPY_FILES += \
        device/intel/common/init.rc:root/init.rc \
        system/core/rootdir/init.rc:root/init.aosp.rc

##############################################################
# Source: device/intel/mixins/groups/wov/lpal/product.mk
##############################################################
# LPAL(Wov)
PRODUCT_PACKAGES += \
       sound_trigger_bundle

#Sound Trigger HAL
PRODUCT_PACKAGES += \
       sound_trigger.primary.$(TARGET_PRODUCT)

##############################################################
# Source: device/intel/mixins/groups/asf/true/product.mk
##############################################################
# Android Security Framework (IDPT-SE)
PRODUCT_PACKAGES += \
        com.intel.config \
        com.intel.config.xml \
        com.intel.asf \
        com.intel.asf.xml \
        securityfileservice \
        SecurityManagerService
##############################################################
# Source: device/intel/mixins/groups/widevine/true/product.mk
##############################################################
#enable Widevine drm
PRODUCT_PROPERTY_OVERRIDES += drm.service.enabled=true

PRODUCT_COPY_FILES += device/intel/common/media/mfx_omxil_core_widevine.conf:system/etc/mfx_omxil_core.conf

# There is an additional dependency on hdcpd that should be controlled
# through the content-protection mixin

PRODUCT_PACKAGES += com.google.widevine.software.drm.xml \
    com.google.widevine.software.drm \
    libdrmwvmplugin \
    libwvm \
    libdrmdecrypt \
    libWVStreamControlAPI_L1 \
    libwvdrm_L1

PRODUCT_PACKAGES_ENG += WidevineSamplePlayer

# WV Modular
PRODUCT_PACKAGES += libwvdrmengine

PRODUCT_PACKAGES_ENG += ExoPlayerDemo

PRODUCT_PACKAGES += liboemcrypto

PRODUCT_PACKAGES += libmeimm libsecmem

BOARD_WIDEVINE_OEMCRYPTO_LEVEL := 1
##############################################################
# Source: device/intel/mixins/groups/pmic/dollar_cove_ti/product.mk
##############################################################
PRODUCT_PACKAGES += fg_algo_iface

##############################################################
# Source: device/intel/mixins/groups/art-config/default/product.mk
##############################################################
# This is needed to enable silver art optimizer.
# This will build the plugins/libart-extension.so library,  which is dynamically loaded by
# AOSP and contains Intel optimizations to the compiler.
PRODUCT_PACKAGES += libart-extension
##############################################################
# Source: device/intel/mixins/groups/cellcoex/default/product.mk
##############################################################
PRODUCT_PACKAGES += CwsCellularCoexMgrService

##############################################################
# Source: device/intel/mixins/groups/security_assist/default/product.mk
##############################################################
# Intel Security Assist
PRODUCT_PACKAGES += \
        IntelSecurityAssist
PRODUCT_PACKAGES_ENG += \
        IntelSecurityAssistTest


##############################################################
# Source: device/intel/mixins/groups/telephony/default/product.mk
##############################################################
# product.mk common to Telephony disabled platforms

# Inherit from common Open Source Telephony product configuration
$(call inherit-product, $(SRC_TARGET_DIR)/product/aosp_base.mk)
# ------------------ END MIX-IN DEFINITIONS ------------------
