FROM ubuntu:20.04

WORKDIR /app

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
        wget \
        build-essential \
        zip \
    && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# GCC ARM
RUN mkdir -p /opt/gcc && \
    cd /opt/gcc && \
    wget https://cdn.edgeimpulse.com/build-system/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 && \
    tar xjf gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 && \
    rm -rf /opt/gcc/*.tar.bz2

ENV PATH="/opt/gcc/gcc-arm-none-eabi-9-2019-q4-major/bin:${PATH}"

# SimpleLink SDK
RUN mkdir -p /opt/ti && \
    cd /opt/ti && \
    wget https://cdn.edgeimpulse.com/build-system/simplelink_cc13x2_26x2_sdk_5_20_00_52.run -O simplelink.run && \
    chmod u+x simplelink.run && \
    ./simplelink.run --mode unattended --prefix /opt/ti && \
    rm -rf simplelink.run

# FreeRTOS
RUN mkdir -p /opt/FreeRTOS && \
    cd /opt/FreeRTOS && \
    wget https://github.com/FreeRTOS/FreeRTOS/releases/download/202107.00/FreeRTOSv202107.00.zip -O freertos.zip && \
    unzip -q freertos.zip && \
    rm -rf freertos.zip && mv FreeRTOS* FreeRTOS

ENV SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR="/opt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52"

RUN sed -i -e "s:/ccs1030/ccs/utils/sysconfig_1.8.0/sysconfig_cli.sh:/sysconfig_1.8.2/sysconfig_cli.sh:g" ${SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR}/imports.mak && \
    sed -i -e "s:/home/username/FreeRTOSv10.2.1:/opt/FreeRTOS/FreeRTOS:g" ${SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR}/imports.mak && \
    sed -i -e "s:/home/username/ti/ccs1030/ccs/tools/compiler/ti-cgt-arm_20.2.4.LTS::g" ${SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR}/imports.mak && \
    sed -i -e "s:/home/username/ti/ccs1030/ccs/tools/compiler/1.2.1.STS::g" ${SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR}/imports.mak && \
    sed -i -e "s:/home/username/ti/ccs1030/ccs/tools/compiler/9.2019.q4.major:/opt/gcc/gcc-arm-none-eabi-9-2019-q4-major:g" ${SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR}/imports.mak && \
    sed -i -e "s:/home/username/ti/:/opt/ti/:g" ${SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR}/imports.mak
