#!/bin/bash
# ============================================================================
# Copyright 2015 BRAIN Corporation. All rights reserved. This software is
# provided to you under BRAIN Corporation's Beta License Agreement and
# your use of the software is governed by the terms of that Beta License
# Agreement, found at http://www.braincorporation.com/betalicense.
# ============================================================================

# usage: flash_and_reboot.sh <firmware *.bin file>

# directory of bash script
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# try to flash 3 times
flash_result=1
for i in `seq 1 3`;
do
    ${DIR}/stlink/st-flash --reset write $1 0x8000000
    if [ $? -eq 0 ]; then
        flash_result=0
        break
    fi
    sleep 3
done
if [ ${flash_result} -ne 0 ]; then
    exit ${flash_result}
fi
