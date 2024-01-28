#!/usr/bin/bash

#echo 800 > /sys/class/power_supply/bms/temp_warm

export FINGERPRINT="GLEN"
export SKIP_FW_QUERY="True"
export NOSENSOR="True"

export PASSIVE="0"
exec ./launch_chffrplus.sh

