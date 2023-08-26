# FYI

Info about my setup:
    I've placed the sdk at $HOME/esp/ESP8266_RTOS_SDK then ran install.sh.
    I've also written a small shell script to set the environment variables both for the SDK & VSCode.

Notice on env. variables:
    In order for the toolchain to work, path to ESP8266_RTOS_SDK must be set as an env. variable w/export IDF_PATH=path/to/ESP8266_RTOS_SDK or equivalent BEFORE running VSCode.

    !IMPORTANT! 
    Setting IDF_PATH to ESP8266_RTOS_SDK may break ESP-IDF & should be set to esp_idf's path removed before using ESP-IDF.
