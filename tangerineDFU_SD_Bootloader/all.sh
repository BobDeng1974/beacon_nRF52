./nrfutil settings generate --family NRF52 --application Tangerine-beacon-nRF52-SDK15.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 settings.hex
./mergehex -m settings.hex Tangerine-beacon-nRF52-SDK15.hex -o Tangerine_beacon_app_settings_s132.hex
./mergehex -m s132_nrf52_6.1.0_softdevice.hex  secure_bootloader_ble_s132_pca10040.hex  Tangerine_beacon_app_settings_s132.hex  -o Tangerine_beacon_all_s132_v42$1.hex
./nrfutil pkg generate --hw-version 52 --application-version $1  --application Tangerine-beacon-nRF52-SDK15.hex --sd-req 0xAF --key-file private.key app_dfu_package_v42$1.zip
