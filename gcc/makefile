BUILD := build
SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR ?= ../../../../../../ti/simplelink_cc13x2_26x2_sdk_5_10_00_48

include $(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/imports.mak

KERNEL_BUILD := $(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/kernel/freertos/builds/cc13x2_cc26x2/release

CC = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc"
CXX = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-g++"
LNK = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc"

srctree					:= ..

NAME = firmware-ti-launchxl

OBJECTS = $(NAME).obj $(BUILD)/ti_devices_config.obj $(BUILD)/ti_drivers_config.obj

# Application flags
APPFLAGS += \
	-DEI_PORTING_TI=1 \
	-DEIDSP_USE_CMSIS_DSP \
	-DEIDSP_QUANTIZE_FILTERBANK=0 \
	-DEI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=1 \
	-DEI_CLASSIFIER_TFLITE_LOAD_CMSIS_NN_SOURCES=1 \
	-DARM_MATH_LOOPUNROLL \
	-DEIDSP_LOAD_CMSIS_DSP_SOURCES=1 \

COMPILE_FLAGS = -I.. \
    -I. \
    -I$(BUILD) \
    -DDeviceFamily_CC13X2 \
    "-I$(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/source" \
    "-I$(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/source/ti/posix/gcc" \
    "-I$(FREERTOS_INSTALL_DIR)/FreeRTOS/Source/include" \
    "-I$(FREERTOS_INSTALL_DIR)/FreeRTOS/Source/portable/GCC/ARM_CM4F" \
    "-I$(KERNEL_BUILD)" \
    -mcpu=cortex-m4 \
    -march=armv7e-m \
    -mthumb \
    -mfloat-abi=hard \
    -mfpu=fpv4-sp-d16 \
    -ffunction-sections \
    -fdata-sections \
    -g \
	-Os \
    -gstrict-dwarf \
    -Wall \
    "-I$(GCC_ARMCOMPILER)/arm-none-eabi/include/newlib-nano" \
    "-I$(GCC_ARMCOMPILER)/arm-none-eabi/include" \
	$(APPFLAGS) \

CFLAGS += $(COMPILE_FLAGS)\
	-std=c99 \

CXXFLAGS += $(COMPILE_FLAGS) \
	-std=c++11 \
	-fno-rtti \
	-fno-use-cxa-atexit \
	-fno-inline-functions \

LIBSTDC = "${shell "$(CC)" $(CFLAGS) -print-file-name=libstdc++.a}"

LFLAGS = -Wl,-T,../gcc/cc13x2_cc26x2_freertos.lds \
    "-Wl,-Map,$(BUILD)/$(NAME).map" \
    "-L$(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/source" \
    ti_utils_build_linker.cmd.genlibs \
    "-L$(KERNEL_BUILD)/gcc" \
    -l:freertos.lib \
    -l:ti/devices/cc13x2_cc26x2/driverlib/bin/gcc/driverlib.lib \
    -march=armv7e-m \
    -mthumb \
	-eresetISR \
    -mfloat-abi=hard \
    -mfpu=fpv4-sp-d16 \
    -nostartfiles \
    -static \
    -Wl,--gc-sections \
    "-L$(GCC_ARMCOMPILER)/arm-none-eabi/lib/thumb/v7e-m/hard" \
    -lgcc \
    -lc \
    -lm \
    -lnosys \
	-Os \
	$(LIBSTDC) \
    --specs=nano.specs -u _printf_float

INC_APP += \
	-I$(BUILD) \
	-I $(srctree)/edge_impulse \
	-I $(srctree)/edge_impulse/ingestion-sdk-platform/ti-launchxl \
	-I $(srctree)/edge_impulse/ingestion-sdk-c \
	-I $(srctree)/edge_impulse/repl \
	-I $(srctree)/edge_impulse/mbedtls_hmac_sha256_sw \
	-I $(srctree)/edge_impulse/edge-impulse-sdk \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/classifier \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/porting \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/dsp \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/dsp/kissfft \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/dsp/dct \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/third_party/flatbuffers/include/flatbuffers \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/third_party/gemmlowp \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/third_party/ruy \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Include/ \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/Core/Include/ \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/ \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/micro \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/kernels \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/scheme \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/c \
	-I $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/core/api/ \
	-I $(srctree)/edge_impulse/model-parameters \
	-I $(srctree)/edge_impulse/tflite-model \
	-I $(srctree)/sensors \
	-I $(srctree)/boostxl_sensors \
	-I $(srctree)/audio_codec \

SRC_C += \
	$(wildcard $(srctree)/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/TransformFunctions/*fft*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/CommonTables/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/TransformFunctions/*bit*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/MatrixFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/BasicMathFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/StatisticsFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/SupportFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ActivationFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/BasicMathFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ConcatenationFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ConvolutionFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/FullyConnectedFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/NNSupportFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/PoolingFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ReshapeFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/SoftmaxFunctions/*.c) \
	$(wildcard $(srctree)/edge_impulse/mbedtls_hmac_sha256_sw/mbedtls/src/*.c) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/c/*.c) \
	$(wildcard $(srctree)/edge_impulse/firmware-sdk/QCBOR/src/*.c) \
	$(wildcard $(srctree)/boostxl_sensors/*.c) \
	$(wildcard $(srctree)/audio_codec/*.c) \

SRC_CXX += \
	$(wildcard $(srctree)/*.cpp) \
	$(wildcard $(srctree)/edge_impulse/ingestion-sdk-platform/ti-launchxl/*.cpp) \
	$(wildcard $(srctree)/edge_impulse/ingestion-sdk-c/*.cpp) \
	$(wildcard $(srctree)/edge_impulse/repl/*.cpp) \
	$(wildcard $(srctree)/edge_impulse/firmware-sdk/*.cpp) \
	$(wildcard $(srctree)/edge_impulse/firmware-sdk/sensor-aq/*.cpp) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/porting/ti/*.cpp) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/dsp/dct/*.cpp) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/dsp/kissfft/*.cpp) \
	$(wildcard $(srctree)/edge_impulse/tflite-model/*.cpp) \
	$(wildcard $(srctree)/sensors/*.cpp) \

SRC_CC += \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/kernels/*.cc) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/*.cc) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/*.cc) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/kernels/*.cc) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/memory_planner/*.cc) \
	$(wildcard $(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/core/api/*.cc) \

VPATH += $(srctree)/ \
	$(srctree)/edge_impulse/edge-impulse-sdk/porting/ti/ \
	$(srctree)/edge_impulse/ingestion-sdk-platform/ti-launchxl \
	$(srctree)/edge_impulse/ingestion-sdk-c \
	$(srctree)/edge_impulse/repl \
	$(srctree)/edge_impulse/firmware-sdk \
	$(srctree)/edge_impulse/firmware-sdk/QCBOR/src \
	$(srctree)/edge_impulse/firmware-sdk/sensor-aq \
	$(srctree)/edge_impulse/mbedtls_hmac_sha256_sw/mbedtls/src \
	$(srctree)/edge_impulse/edge-impulse-sdk/dsp/dct \
	$(srctree)/edge_impulse/edge-impulse-sdk/dsp/kissfft \
	$(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/kernels \
	$(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/kernels/internal \
	$(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/micro \
	$(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/kernels \
	$(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/memory_planner \
	$(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/c/ \
	$(srctree)/edge_impulse/edge-impulse-sdk/tensorflow/lite/core/api/ \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/TransformFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/CommonTables \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/MatrixFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/StatisticsFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/SupportFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/BasicMathFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ActivationFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/BasicMathFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ConcatenationFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ConvolutionFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/FullyConnectedFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/NNSupportFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/PoolingFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ReshapeFunctions \
	$(srctree)/edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/SoftmaxFunctions \
	$(srctree)/edge_impulse/tflite-model \
	$(srctree)/sensors \
	$(srctree)/boostxl_sensors \
	$(srctree)/audio_codec \

OBJ = $(SRC_C:$(srctree)/%.c=$(BUILD)/%.obj)
OBJ += $(SRC_CXX:$(srctree)/%.cpp=$(BUILD)/%.obj)
OBJ += $(SRC_CC:$(srctree)/%.cc=$(BUILD)/%.obj)
OBJECTS += $(filter-out $(BUILD)/$(NAME).obj, $(OBJ))

all: $(BUILD)/$(NAME).out

$(KERNEL_BUILD)/gcc/freertos.lib:
	@ $(ECHOBLANKLINE)
	@ echo $@ is not built.
	@ echo You can build it by issuing $(MAKE) in $(dir $@).
	@ $(ECHOBLANKLINE)

.INTERMEDIATE: syscfg
ti_devices_config.c ti_drivers_config.c ti_utils_build_linker.cmd.genlibs ti_drivers_config.h: syscfg
	@ echo generation complete

syscfg: ../$(NAME).syscfg
	@ echo Generating configuration files...
	@ $(SYSCONFIG_TOOL) --compiler gcc --product $(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/.metadata/product.json --output $(@D) $<

$(BUILD)/ti_devices_config.obj: ti_devices_config.c ti_drivers_config.h
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -c -o $@

$(BUILD)/ti_drivers_config.obj: ti_drivers_config.c ti_drivers_config.h
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -c -o $@

$(BUILD)/%.obj: %.c | $(BUILD)
	@mkdir -p $(@D)
	@$(CC) $(CFLAGS) $(INC_APP) -c -o $@ $<
	@echo $<

$(BUILD)/%.obj: %.cpp
	@mkdir -p $(@D)
	@$(CXX) $(CXXFLAGS) $(INC_APP) -c -o $@ $<
	@echo $<

$(BUILD)/%.obj: %.cc
	@mkdir -p $(@D)
	@$(CXX) $(CXXFLAGS) $(INC_APP) -c -o $@ $<
	@echo $<

$(BUILD)/$(NAME).out: $(BUILD)/$(OBJECTS) $(KERNEL_BUILD)/gcc/freertos.lib
	@ echo linking $@
	@ $(LNK)  $(BUILD)/$(OBJECTS)  $(LFLAGS) -o $(BUILD)/$(NAME).out

$(BUILD): syscfg
	mkdir -p $(BUILD)
	@ echo Creating output folder

test:
	@ echo $(SRC_C) $(OBJECTS) $(LIBSTDC)

build-kernel:
	@ echo Copying FreeRTOSConfig.h
	@ cp FreeRTOSConfig.h $(KERNEL_BUILD)
	@ echo Building kernel..
	@ cd $(KERNEL_BUILD)/../../../ && make

clean-kernel:
	@ echo Cleaning kernel..
	@ cd $(KERNEL_BUILD)/../../../ && make clean

clean:
	@ echo Cleaning...
	@ $(RM) -rf $(BUILD)
	@ $(RM) ti_devices_config.c ti_drivers_config.c ti_utils_build_linker.cmd.genlibs ti_drivers_config.h > $(DEVNULL) 2>&1
