# Rumblepack

This just takes Zephyr's USB Audio sample and adds an I2S device (NAU8325) to it. The code is horrible and the implementation is extremely crude. Please look away.

## Updating via USB Serial with mcumgr

1. Download mcumgr from https://github.com/vouch-opensource/mcumgr-client/releases
2. Build the firmware or grab a dfu_application.zip file
3. Put the .bin files in the same folder as mcumgr
4. Run `.\mcumgr-client.exe upload .\app_update.bin -s 0`, this updates the application core
5. Then run `.\mcumgr-client.exe upload .\net_core_app_update.bin -s 3`, this updates the net core (for Bluetooth)
6. Then run `.\mcumgr-client.exe reset`, to reboot Rumblejack into its new firwmare.