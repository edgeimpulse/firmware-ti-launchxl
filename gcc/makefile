BUILD := build
SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR ?= ../../../../../../ti/simplelink_cc13x2_26x2_sdk_5_10_00_48


include $(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/imports.mak

CC = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc"
CXX = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-g++"
LNK = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc"

NAME = firmware-ti-launchxl

OBJECTS = $(NAME).obj $(BUILD)/ti_devices_config.obj $(BUILD)/ti_drivers_config.obj

# Application flags
APPFLAGS += \
	-DEI_SENSOR_AQ_STREAM=FILE \
	-DEI_PORTING_TI=1 \
	-DEIDSP_USE_CMSIS_DSP \
	-DEIDSP_QUANTIZE_FILTERBANK=0 \
	-DEI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN \
	-DARM_MATH_LOOPUNROLL \
	-DEIDSP_LOAD_CMSIS_DSP_SOURCES=1 \

CFLAGS = -I.. \
    -I. \
    -I$(BUILD) \
    -DDeviceFamily_CC13X2 \
    "-I$(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/source" \
    "-I$(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/kernel/nortos" \
    "-I$(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/kernel/nortos/posix" \
    -mcpu=cortex-m4 \
    -march=armv7e-m \
    -mthumb \
    -mfloat-abi=hard \
    -mfpu=fpv4-sp-d16 \
    -ffunction-sections \
    -fdata-sections \
    -g \
	-O3 \
    -gstrict-dwarf \
    -Wall \
    "-I$(GCC_ARMCOMPILER)/arm-none-eabi/include/newlib-nano" \
    "-I$(GCC_ARMCOMPILER)/arm-none-eabi/include" \
	$(APPFLAGS) \

CXXFLAGS += $(CFLAGS) \
	-std=gnu++11 \
	-fno-rtti \
	-fno-use-cxa-atexit \
	-fno-inline-functions \

LIBSTDC = "${shell "$(CC)" $(CFLAGS) -print-file-name=libstdc++.a}"

LFLAGS = -Wl,-T,../gcc/cc13x2_cc26x2_nortos.lds \
    "-Wl,-Map,$(BUILD)/$(NAME).map" \
    "-L$(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/source" \
    -eresetISR \
    "-L$(SIMPLELINK_CC13X2_26X2_SDK_INSTALL_DIR)/kernel/nortos" \
    ti_utils_build_linker.cmd.genlibs \
    -l:ti/devices/cc13x2_cc26x2/driverlib/bin/gcc/driverlib.lib \
    -march=armv7e-m \
    -mthumb \
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
	-I ../edge_impulse \
	-I ../edge_impulse/ingestion-sdk-platform/ti-launchxl \
	-I ../edge_impulse/ingestion-sdk-c \
	-I ../edge_impulse/repl \
	-I ../edge_impulse/QCBOR/inc \
	-I ../edge_impulse/mbedtls_hmac_sha256_sw \
	-I ../edge_impulse/edge-impulse-sdk \
	-I ../edge_impulse/edge-impulse-sdk/classifier \
	-I ../edge_impulse/edge-impulse-sdk/porting \
	-I ../edge_impulse/edge-impulse-sdk/dsp \
	-I ../edge_impulse/edge-impulse-sdk/dsp/kissfft \
	-I ../edge_impulse/edge-impulse-sdk/dsp/dct \
	-I ../edge_impulse/edge-impulse-sdk/third_party/flatbuffers/include/flatbuffers \
	-I ../edge_impulse/edge-impulse-sdk/third_party/gemmlowp \
	-I ../edge_impulse/edge-impulse-sdk/third_party/ruy \
	-I ../edge_impulse/edge-impulse-sdk/CMSIS/DSP/Include/ \
	-I ../edge_impulse/edge-impulse-sdk/CMSIS/Core/Include/ \
	-I ../edge_impulse/edge-impulse-sdk/tensorflow/ \
	-I ../edge_impulse/edge-impulse-sdk/tensorflow/lite \
	-I ../edge_impulse/edge-impulse-sdk/tensorflow/lite/micro \
	-I ../edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/kernels \
	-I ../edge_impulse/edge-impulse-sdk/tensorflow/lite/scheme \
	-I ../edge_impulse/edge-impulse-sdk/tensorflow/lite/c \
	-I ../edge_impulse/edge-impulse-sdk/tensorflow/lite/core/api/ \
	-I ../edge_impulse/model-parameters \
	-I ../edge_impulse/tflite-model \
	-I ../sensors \
	-I ../boostxl_sensors \

SRC_C += \
	$(notdir $(wildcard ../*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/TransformFunctions/*fft*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/CommonTables/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/TransformFunctions/*bit*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/MatrixFunctions/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/StatisticsFunctions/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ActivationFunctions/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/BasicMathFunctions/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ConcatenationFunctions/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ConvolutionFunctions/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/FullyConnectedFunctions/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/NNSupportFunctions/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/PoolingFunctions/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ReshapeFunctions/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/SoftmaxFunctions/*.c)) \
	$(notdir $(wildcard ../edge_impulse/mbedtls_hmac_sha256_sw/mbedtls/src/*.c)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/tensorflow/lite/c/*.c)) \
	$(notdir $(wildcard ../edge_impulse/QCBOR/src/*.c)) \
	$(notdir $(wildcard ../boostxl_sensors/*.c)) \

SRC_CXX += \
	$(notdir $(wildcard ../*.cpp)) \
	$(notdir $(wildcard ../edge_impulse/ingestion-sdk-platform/ti-launchxl/*.cpp)) \
	$(notdir $(wildcard ../edge_impulse/ingestion-sdk-c/*.cpp)) \
	$(notdir $(wildcard ../edge_impulse/repl/*.cpp)) \
	$(notdir $(wildcard ../edge_impulse/firmware-sdk/*.cpp)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/porting/ti/*.cpp)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/dsp/dct/*.cpp)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/dsp/kissfft/*.cpp)) \
	$(notdir $(wildcard ../edge_impulse/tflite-model/*.cpp)) \
	$(notdir $(wildcard ../sensors/*.cpp)) \

SRC_CC += \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/tensorflow/lite/kernels/*.cc)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/*.cc)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/*.cc)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/kernels/*.cc)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/memory_planner/*.cc)) \
	$(notdir $(wildcard ../edge_impulse/edge-impulse-sdk/tensorflow/lite/core/api/*.cc)) \

VPATH += ../ \
	../edge_impulse/edge-impulse-sdk/porting/ti/ \
	../edge_impulse/ingestion-sdk-platform/ti-launchxl \
	../edge_impulse/ingestion-sdk-c \
	../edge_impulse/repl \
	../edge_impulse/QCBOR/src \
	../edge_impulse/mbedtls_hmac_sha256_sw/mbedtls/src \
	../edge_impulse/firmware-sdk \
	../edge_impulse/edge-impulse-sdk/dsp/dct \
	../edge_impulse/edge-impulse-sdk/dsp/kissfft \
	../edge_impulse/edge-impulse-sdk/tensorflow/lite/kernels \
	../edge_impulse/edge-impulse-sdk/tensorflow/lite/kernels/internal \
	../edge_impulse/edge-impulse-sdk/tensorflow/lite/micro \
	../edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/kernels \
	../edge_impulse/edge-impulse-sdk/tensorflow/lite/micro/memory_planner \
	../edge_impulse/edge-impulse-sdk/tensorflow/lite/c/ \
	../edge_impulse/edge-impulse-sdk/tensorflow/lite/core/api/ \
	../edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/TransformFunctions \
	../edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/CommonTables \
	../edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/MatrixFunctions \
	../edge_impulse/edge-impulse-sdk/CMSIS/DSP/Source/StatisticsFunctions \
	../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ActivationFunctions \
	../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/BasicMathFunctions \
	../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ConcatenationFunctions \
	../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ConvolutionFunctions \
	../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/FullyConnectedFunctions \
	../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/NNSupportFunctions \
	../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/PoolingFunctions \
	../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/ReshapeFunctions \
	../edge_impulse/edge-impulse-sdk/CMSIS/NN/Source/SoftmaxFunctions \
	../edge_impulse/tflite-model \
	../sensors \
	../boostxl_sensors \

OBJ = $(addprefix $(BUILD)/, $(SRC_C:.c=.obj))
OBJ += $(addprefix $(BUILD)/, $(SRC_CXX:.cpp=.obj))
OBJ += $(addprefix $(BUILD)/, $(SRC_CC:.cc=.obj))
OBJECTS += $(filter-out $(BUILD)/$(NAME).obj, $(OBJ))

all: $(BUILD)/$(NAME).out

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
	@$(CC) $(CFLAGS) $(INC_APP) -c -o $@ $<
	@echo $<

$(BUILD)/%.obj: %.cpp
	@$(CXX) $(CXXFLAGS) $(INC_APP) -c -o $@ $<
	@echo $<

$(BUILD)/%.obj: %.cc
	@$(CXX) $(CXXFLAGS) $(INC_APP) -c -o $@ $<
	@echo $<

$(BUILD)/$(NAME).out: $(BUILD)/$(OBJECTS)
	@ echo linking $@
	@ $(LNK)  $(BUILD)/$(OBJECTS)  $(LFLAGS) -o $(BUILD)/$(NAME).out

$(BUILD): syscfg
	mkdir -p $(BUILD)
	@ echo Creating output folder

test:
	@ echo $(SRC_C) $(OBJECTS) $(LIBSTDC)

clean:
	@ echo Cleaning...
	@ $(RM) -rf $(BUILD)
	@ $(RM) ti_devices_config.c ti_drivers_config.c ti_utils_build_linker.cmd.genlibs ti_drivers_config.h > $(DEVNULL) 2>&1
