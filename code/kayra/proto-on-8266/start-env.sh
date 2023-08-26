#! /usr/bin/bash
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )

if [[ -n "${BASH_SOURCE}" && ( "${BASH_SOURCE[0]}" == "${0}" ) ]]; then
    echo "This script should be sourced, not executed. Try: "
    echo "	. ${BASH_SOURCE[0]}"
    exit 1
fi

echo "Configuring the environment and launching VSCode..."
echo "..."
echo "..."
export IDF_PATH=$HOME/esp/ESP8266_RTOS_SDK
. ${IDF_PATH}/export.sh
code $parent_path/
