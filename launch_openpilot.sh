#!/usr/bin/bash

export FINGERPRINT="GLEN"
export SKIP_FW_QUERY="True"
export NOSENSOR="True"
#export STARTED="True"

echo $$ > /dev/cpuset/app/tasks
echo $PPID > /dev/cpuset/app/tasks

export PASSIVE="0"
exec ./launch_chffrplus.sh

