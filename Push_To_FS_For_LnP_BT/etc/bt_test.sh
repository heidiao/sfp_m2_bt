#!/system/bin/sh

# This script is started from /init.rc.

DATE=`date +"%Y%m%d %H:%M:%S"`
echo "[$DATE]" >> /data/tmp
echo "start test..." >> /data/tmp
echo 'file lnp_ldisc.c +p' > /d/dynamic_debug/control
logcat -v time -f /dev/kmsg | cat /proc/kmsg >/data/logs/log.txt &
#sleep 30
#input keyevent MENU
#am start -a android.settings.BLUETOOTH_SETTINGS
#sleep 1
#input keyevent DPAD_UP
#sleep 1
#input keyevent DPAD_CENTER
echo " "
exit 0
