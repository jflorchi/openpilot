#!/usr/bin/bash

export FINGERPRINT="TOYOTA COROLLA 2010"
export SKIP_FW_QUERY="True"

echo $$ > /dev/cpuset/app/tasks
echo $PPID > /dev/cpuset/app/tasks

export PASSIVE="0"
exec ./launch_chffrplus.sh

