# Edge Impulse Firmware for TI LAUNCHXL-CC1352P

Edge Impulse enables developers to create the next generation of intelligent device solutions with embedded machine learning. This repository contains the Edge Impulse firmware for TI LAUNCHXL-CC1352P development board. This device supports all of Edge Impulse's device features, including ingestion, remote management and inferencing.

> **Note:** Do you just want to use this development board to collect data with Edge Impulse? No need to build this firmware. View the [getting started guide](https://docs.edgeimpulse.com/docs/ti-launchxl) for a pre-built firmware image and flashing instructions. Or, you can use the [data forwarder](https://docs.edgeimpulse.com/docs/cli-data-forwarder) to capture data from any sensor.

## Requirements

### Hardware

* [TI LAUNCHXL-CC1352P](https://www.ti.com/tool/LAUNCHXL-CC1352P).
* [BOOSTXL-SENSORS](https://www.ti.com/tool/BOOSTXL-SENSORS) - contains additional sensors and allows to connect (more) external sensors.
* [CC3200AUDBOOST](https://www.ti.com/tool/CC3200AUDBOOST) - adds audio to your project.

#### Hardware Setup

The add-on boards can be stacked on each other with the CC3200AUDBOOST stacked
on the BOOSTXL-SENSOR add-on.  However hardware modifications
are required to get the CC3200AUDBOOST to work for CC1352P.

See **Hardware Setup > Codec
Setup - CC3200AUDBOOST** in [Quick Start
Guide](https://dev.ti.com/tirex/explore/content/simplelink_audio_plugin_3_30_00_06/docs/Quick_Start_Guide.html)
and follow steps for **CC13x2**.


### Software

* [Edge Impulse CLI](https://docs.edgeimpulse.com/docs/cli-installation).
* Install the following tools separately or install [TI CCSTUDIO](https://www.ti.com/tool/CCSTUDIO) which comes bundled with:
    * [XDCTools](http://downloads.ti.com/dsps/dsps_public_sw/sdo_sb/targetcontent/rtsc/)
    * [SYSCONFIG](https://www.ti.com/tool/SYSCONFIG)
    * [ARM GCC Toolchain 7](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads/7-2017-q4-major-1-1) 
* Install [UniFlash](https://www.ti.com/tool/UNIFLASH).
* Download [TI Simplelink SDK](https://www.ti.com/tool/download/SIMPLELINK-CC13X2-26X2-SDK).
* Download [FreeRTOS 202107.00](https://github.com/FreeRTOS/FreeRTOS/releases/download/202107.00/FreeRTOSv202107.00.zip).

## Texas Instruments Simplelink SDK

- Uses the TI Simplelink SDK `simplelink_cc13x2_26x2_sdk_5.20.00.52`.
- Set `SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR` in `gcc/makefile` to the path to TI SimpleLink SDK, referred to as `<TI SDK FOLDER>`.
- Set the following variables in `<TI SDK FOLDER>/imports.mak`:
    - `SYSCONFIG_TOOL` e.g. `/home/user/ti/sysconfig_1.8.2/sysconfig_cli.sh`
    - `GCC_ARMCOMPILER` e.g. `/home/user/ti/ccs1040/ccs/tools/compiler/gcc-arm-none-eabi-7-2017-q4-major`:
    - `FREERTOS_INSTALL_DIR` e.g. `/home/user/FreeRTOS/`

## Building the application

> The FreeRTOS kernel **must** be built before the applicaiton can be built. To do so,
>
>- Replace the `FreeRTOSConfig.h` in `<TI SDK FOLDER>/kernel/freertos/builds/cc13x2_cc26x2/release` with `gcc/FreeRTOSConfig.h` and
> then in a terminal:
>
> ```
>    $ cd <TI SDK FOLDER>/kernel/freertos
>    $ make
> ```
> Or do all steps by:
>
> ```
>    $ cd gcc
>    $ make build-kernel
> ```

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
    $ export PATH=/home/user/ti/:${PATH}
    $ dslite.sh -c tools/user_files/configs/cc1352p1f3.ccxml -l tools/user_files/settings/generated.ufsettings -e -f -v gcc/build/firmware-ti-launchxl.out
    ```

    *Note*: the above expects [UniFlash](https://www.ti.com/tool/UNIFLASH) is installed and `dslite.sh/dslite.bat` to be in the system path.

### Or build with Docker

1. Build the Docker image:
    ```
    $ docker build -t ti-build .
    ```
1. Build the application by running the container as follows:

    **Windows**

    ```
    $ docker run --rm -it -v "%cd%":/app ti-build /bin/bash -c "cd gcc && make build-kernel && make"
    ```

    **Linux, macOS**

    ```
    $ docker run --rm -it -v $PWD:/app:delegated ti-build /bin/bash -c "cd gcc && make build-kernel && make"
    ```

1. Connect the board to your computer using USB.
1. Flash the board:

    ```
    $ dslite.sh -c tools/user_files/configs/cc1352p1f3.ccxml -l tools/user_files/settings/generated.ufsettings -e -f -v gcc/build/firmware-ti-launchxl.out
    ```
## Troubleshooting

**Flashing**

If during flashing you encounter flashing issue.

Then ensure:

1. your device is properly connected and/or your cable is not damaged.
2. you have the proper permissions to access the USB device.

If on Linux you may want to try copying `tools/71-ti-permissions.rules` to `/etc/udev/rules.d/`. Then re-attach the USB cable and try again.

**Out of (RAM) Memory: (-1002)**

TI LAUNCHXL-CC1352P has 80kB of RAM which may not always be enough to run your
model. If this is the case try reducing the RAM usage of your model.
