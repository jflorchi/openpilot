#!/usr/bin/bash

# Cap battery float voltage at 4200mV for longevity. Writing voltage_max in sysfs is a
# silent no-op on this kernel: CONFIG_MACH_COMMA stubs out smbchg_battery_set_property
# (qpnp-smbcharger.c:5730) so VOLTAGE_MAX never reaches hardware, while is_writeable
# still advertises it. Program FV_CFG over SPMI instead. 0x23 = 4200mV (0x05 = 3600mV,
# 20mV steps); stock is 0x2B = 4350mV. FV_CFG is secured, so unlock SEC_ACCESS first.
# Survives charger insert/remove; lost on reboot, hence doing it here.
SPMI=/sys/kernel/debug/spmi/spmi-0
if [ -w "$SPMI/address" ]; then
  echo 0x210D0 > $SPMI/address; echo 0xA5 > $SPMI/data
  echo 0x210F4 > $SPMI/address; echo 0x19 > $SPMI/data
fi

export FINGERPRINT="GLEN"
export SKIP_FW_QUERY="True"
export NOSENSOR="True"

export PASSIVE="0"
exec ./launch_chffrplus.sh

