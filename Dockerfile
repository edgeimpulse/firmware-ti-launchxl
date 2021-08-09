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
    wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2019q4/RC2.1/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 && \
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

ENV SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR="/opt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52"

RUN sed -i -e "s:/ccs1030/ccs/utils/sysconfig_1.8.0/sysconfig_cli.sh:/sysconfig_1.8.2/sysconfig_cli.sh:g" /opt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52/imports.mak && \
    sed -i -e "s:/home/username/FreeRTOSv10.2.1::g" /opt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52/imports.mak && \
    sed -i -e "s:/home/username/ti/ccs1030/ccs/tools/compiler/ti-cgt-arm_20.2.4.LTS::g" /opt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52/imports.mak && \
    sed -i -e "s:/home/username/ti/ccs1030/ccs/tools/compiler/1.2.1.STS::g" /opt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52/imports.mak && \
    sed -i -e "s:/home/username/ti/ccs1030/ccs/tools/compiler/9.2019.q4.major:/opt/gcc/gcc-arm-none-eabi-9-2019-q4-major:g" /opt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52/imports.mak && \
    sed -i -e "s:/home/username/ti/:/opt/ti/:g" /opt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52/imports.mak
