[main]
mixinsdir: device/intel/mixins/groups

[mapping]
product.mk: device.mk

[groups]
boot-arch: efi(uefi_arch=x86_64,fastboot=efi,acpi_permissive=false)
config-partition: enabled
kernel: gmin64(path=gmin,loglevel=5)
display-density: medium
dalvik-heap: tablet-7in-hdpi-1024
cpu-arch: slm
houdini: true
bugreport: default
graphics: ufo_gen8
storage: sdcard-mmcblk1-4xUSB-sda-emulated
ethernet: dhcp
camera: isp3
rfkill: true(force_disable=gps bluetooth)
bluetooth: lnp 
nfc: pn547
wlan: iwlwifi
widi: gen
audio: cht-tlv320aic31xx
media: ufo
usb: host+acc
usb-gadget: g_android
touch: atmel1000
navigationbar: true
device-type: tablet
gms: true
debug-tools: true
factory-scripts: true
sepolicy: intel
charger: true
disk-bus: mmc-cht
thermal: ituxd(modem_zone=false)
gps: bcm4752(tty=ttyS2,RfType=GL_RF_4752_BRCM_EXT_LNA,FrqPlan=FRQ_PLAN_26MHZ_2PPM)
serialport: ttyS0
flashfiles: true
debug-logs: true
debug-crashlogd: true
debug-coredump: true
debug-phonedoctor: true
debug-charging: true
hdmi_settings: true
lights: true
security: txei
hw-keystore: txei
hdcpd: true
bcu: true
silentlake: true
libmds: mds
vpp: isv
libhealthd: intel
ccf: enable
sensor-hubs: ish
initrc-aosp: override
wov: lpal
asf: true
widevine: true
debug-lmdump:true
pmic: dollar_cove_ti
