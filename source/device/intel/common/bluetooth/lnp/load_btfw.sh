#!/system/bin/sh
cp /system/etc/firmware/bt/370b10060002220e00.sfi /config/bt/
cp /system/etc/firmware/bt/370b00060002280d00.seq /config/bt/
cp /system/etc/firmware/bt/370b000600020d0e00.sfi /config/bt/
chmod 777 /config/bt/370b10060002220e00.sfi
chmod 777 /config/bt/370b00060002280d00.seq
chmod 777 /config/bt/370b000600020d0e00.sfi
chown system:system /config/bt/370b10060002220e00.sfi
chown system:system /config/bt/370b00060002280d00.seq
chown system:system /config/bt/370b000600020d0e00.sfi
