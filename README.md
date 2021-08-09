# Edge Impulse Firmware for TI LAUNCHXL-CC1352P

Edge Impulse enables developers to create the next generation of intelligent device solutions with embedded machine learning. This repository contains the Edge Impulse firmware for TI LAUNCHXL-CC1352P development board. This device supports all of Edge Impulse's device features, including ingestion, remote management and inferencing.

> **Note:** Do you just want to use this development board to collect data with Edge Impulse? No need to build this firmware. View the [getting started guide](https://docs.edgeimpulse.com/docs/ti-launchxl) for a pre-built firmware image and flashing instructions. Or, you can use the [data forwarder](https://docs.edgeimpulse.com/docs/cli-data-forwarder) to capture data from any sensor.

## Requirements

### Hardware

* [TI LAUNCHXL-CC1352P](https://www.ti.com/tool/LAUNCHXL-CC1352P).

#### Optional add-ons:

* [BOOSTXL-SENSORS](https://www.ti.com/tool/BOOSTXL-SENSORS) - contains additional sensors and allows to connect (more) external sensors.
* [CC3200AUDBOOST](https://www.ti.com/tool/CC3200AUDBOOST) - adds audio to your project.

### Software

* [Edge Impulse CLI](https://docs.edgeimpulse.com/docs/cli-installation).
* [TI Simplelink SDK](https://www.ti.com/tool/download/SIMPLELINK-CC13X2-26X2-SDK).
* [UniFlash](https://www.ti.com/tool/UNIFLASH).

## Texas Instruments Simplelink SDK

- Uses the TI Simplelink SDK `simplelink_cc13x2_26x2_sdk_5.20.00.52`
- Set `SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR` in `gcc\makefile` to the path to TI SimpleLink SDK.

## Building the application

1. Build the application by calling `make` as follows:

    ```
    $ cd gcc
    $ make
    ```
1. Connect the board to your computer using USB.
1. Flash the board

    **Windows**

    ```
    $ dslite.bat -c tools\user_files\configs\cc1352p1f3.ccxml -l tools\user_files\settings\generated.ufsettings -e -f -v gcc\build\firmware-ti-launchxl.out
    ```

    **Linux, macOS**

    ```
    $ dslite.sh -c tools/user_files/configs/cc1352p1f3.ccxml -l tools/user_files/settings/generated.ufsettings -e -f -v gcc/build/firmware-ti-launchxl.out
    ```

    *Note*: the above expects [UniFlash](https://www.ti.com/tool/UNIFLASH) is installed and `PATH` is set up properly to find `dslite.sh`.

### Or build with Docker

1. Build the Docker image:
    ```
    $ docker build -t ti-build .
    ```
1. Build the application by running the container as follows:

    **Windows**

    ```
    $ docker run --rm -it -v "%cd%":/app ti-build /bin/bash -c "cd gcc && make"
    ```

    **Linux, macOS**

    ```
    $ docker run --rm -it -v $PWD:/app:delegated ti-build /bin/bash -c "cd gcc && make"
    ```

1. Connect the board to your computer using USB.
1. Flash the board:

    ```
    $ dslite.sh -c tools/user_files/configs/cc1352p1f3.ccxml -l tools/user_files/settings/generated.ufsettings -e -f -v gcc/build/firmware-ti-launchxl.out
    ```

## Troubleshooting

**Flashing**

If during flashing you encounter flashing issue. Then..

Ensure:

1. your device is properly connected and/or your cable is not damaged.
2. you have the proper permissions to access the USB device.

If on Linux you may want to try copying `tools/71-ti-permissions.rules` to `/etc/udev/rules.d/`. Then re-attach the USB cable and try again.
