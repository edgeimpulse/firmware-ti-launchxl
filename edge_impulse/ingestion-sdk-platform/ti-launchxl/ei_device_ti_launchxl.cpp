/* Edge Impulse ingestion SDK
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Include ----------------------------------------------------------------- */
#include "ei_device_ti_launchxl.h"
#include "ei_ti_launchxl_fs_commands.h"
#include "ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/ei_utils.h"
#include "ei_inertialsensor.h"
#include "ei_microphone.h"
#include "repl.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_fcfg1.h)

/** Max size for device id array */
#define DEVICE_ID_MAX_SIZE 32

/** Memory location for the processor device address */
#define DEVICE_ID_LSB_ADDR  ((uint32_t)(FCFG1_BASE + FCFG1_O_MAC_BLE_0))
#define DEVICE_ID_MSB_ADDR  ((uint32_t)(FCFG1_BASE + FCFG1_O_MAC_BLE_1))

/** Sensors */
typedef enum
{
    MICROPHONE = 0,
    ACCELEROMETER
} used_sensors_t;


/** Data Output Baudrate */
const ei_device_data_output_baudrate_t ei_dev_max_data_output_baudrate = {
    ei_xstr(MAX_BAUD),
    MAX_BAUD,
};

const ei_device_data_output_baudrate_t ei_dev_default_data_output_baudrate = {
    ei_xstr(DEFAULT_BAUD),
    DEFAULT_BAUD,
};

/* TI extern declared */
extern "C" void Serial_Out(char *string, int length);
extern "C" uint8_t Serial_In(void);
extern "C" void Serial_set_baudrate(uint32_t baud);

/** Device type */
static const char *ei_device_type = "TI_LAUNCH_XL  ";

/** Device id array */
static char ei_device_id[DEVICE_ID_MAX_SIZE];

/** Device object, for this class only 1 object should exist */
EiDeviceTiLaunchXl EiDevice;

static EiState ei_program_state = eiStateIdle;


/* Private function declarations ------------------------------------------- */
static int get_id_c(uint8_t out_buffer[32], size_t *out_size);
static int set_id_c(char *device_id);
static int get_type_c(uint8_t out_buffer[32], size_t *out_size);
static bool get_wifi_connection_status_c(void);
static bool get_wifi_present_status_c(void);
// static void timer_callback(void *arg);
static bool read_sample_buffer(size_t begin, size_t length, void (*data_fn)(uint8_t *, size_t));
static int get_data_output_baudrate_c(ei_device_data_output_baudrate_t *baudrate);

extern "C" void Led_control(uint32_t led, bool on_off);

/* Public functions -------------------------------------------------------- */

EiDeviceInfo* EiDeviceInfo::get_device() { return &EiDevice; }

EiDeviceTiLaunchXl::EiDeviceTiLaunchXl(void)
{
    uint32_t *id_msb = (uint32_t *)DEVICE_ID_MSB_ADDR;
    uint32_t *id_lsb = (uint32_t *)DEVICE_ID_LSB_ADDR;

    /* Setup device ID */
    snprintf(&ei_device_id[0], DEVICE_ID_MAX_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X"
        ,(uint8_t)((*id_msb >> 8) & 0xFF)
        ,(uint8_t)((*id_msb >> 0) & 0xFF)
        ,(uint8_t)((*id_lsb >> 24)& 0xFF)
        ,(uint8_t)((*id_lsb >> 16)& 0xFF)
        ,(uint8_t)((*id_lsb >> 8) & 0xFF)
        ,(uint8_t)((*id_lsb >> 0) & 0xFF)
        );
}

/**
 * @brief      For the device ID, the BLE mac address is used.
 *             The mac address string is copied to the out_buffer.
 *
 * @param      out_buffer  Destination array for id string
 * @param      out_size    Length of id string
 *
 * @return     0
 */
int EiDeviceTiLaunchXl::get_id(uint8_t out_buffer[32], size_t *out_size)
{
    return get_id_c(out_buffer, out_size);
}

/**
 * @brief      Gets the identifier pointer.
 *
 * @return     The identifier pointer.
 */
const char *EiDeviceTiLaunchXl::get_id_pointer(void)
{
    return (const char *)ei_device_id;
}

/**
 * @brief      Copy device type in out_buffer & update out_size
 *
 * @param      out_buffer  Destination array for device type string
 * @param      out_size    Length of string
 *
 * @return     -1 if device type string exceeds out_buffer
 */
int EiDeviceTiLaunchXl::get_type(uint8_t out_buffer[32], size_t *out_size)
{
    return get_type_c(out_buffer, out_size);
}

/**
 * @brief      Gets the type pointer.
 *
 * @return     The type pointer.
 */
const char *EiDeviceTiLaunchXl::get_type_pointer(void)
{
    return (const char *)ei_device_type;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceTiLaunchXl::get_wifi_connection_status(void)
{
    return false;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceTiLaunchXl::get_wifi_present_status(void)
{
    return false;
}

/**
 * @brief      Create sensor list with sensor specs
 *             The studio and daemon require this list
 * @param      sensor_list       Place pointer to sensor list
 * @param      sensor_list_size  Write number of sensors here
 *
 * @return     False if all went ok
 */
bool EiDeviceTiLaunchXl::get_sensor_list(
    const ei_device_sensor_t **sensor_list,
    size_t *sensor_list_size)
{
    /* Calculate number of bytes available on flash for sampling, reserve 1 block for header + overhead */
    uint32_t available_bytes = (ei_ti_launchxl_fs_get_n_available_sample_blocks() - 1) *
        ei_ti_launchxl_fs_get_block_size();

    sensors[MICROPHONE].name = "Microphone";
    sensors[MICROPHONE].start_sampling_cb = &ei_microphone_sample_start;
    sensors[MICROPHONE].max_sample_length_s = available_bytes / (8000 * 2);
    sensors[MICROPHONE].frequencies[0] = 16000.0f;

    sensors[ACCELEROMETER].name = "Accelerometer";
    sensors[ACCELEROMETER].start_sampling_cb = &ei_inertial_setup_data_sampling;

    sensors[ACCELEROMETER].frequencies[0] = 62.5f;
    sensors[ACCELEROMETER].frequencies[1] = 100.0f;
    sensors[ACCELEROMETER].frequencies[2] = 250.0f;
    sensors[ACCELEROMETER].max_sample_length_s = available_bytes / (sensors[ACCELEROMETER].frequencies[2] * SIZEOF_N_AXIS_SAMPLED * EI_SAMPLING_OVERHEAD_PER_SAMPLE);

    *sensor_list = sensors;
    *sensor_list_size = EI_DEVICE_N_SENSORS;

    return false;
}

/**
 * @brief      Create resolution list for snapshot setting
 *             The studio and daemon require this list
 * @param      snapshot_list       Place pointer to resolution list
 * @param      snapshot_list_size  Write number of resolutions here
 *
 * @return     False if all went ok
 */
bool EiDeviceTiLaunchXl::get_snapshot_list(
    const ei_device_snapshot_resolutions_t **snapshot_list,
    size_t *snapshot_list_size,
    const char **color_depth)
{
    return true;
}

/**
 * @brief      Device specific delay ms implementation
 *
 * @param[in]  milliseconds  The milliseconds
 */
void EiDeviceTiLaunchXl::delay_ms(uint32_t milliseconds)
{
    ei_sleep(milliseconds);
}

void EiDeviceTiLaunchXl::setup_led_control(void)
{

}

void EiDeviceTiLaunchXl::set_state(EiState state)
{
    ei_program_state = state;

    if (state == eiStateFinished) {
        Led_control(LEDALL, 0);
        EiDevice.delay_ms(100);
        Led_control(LED1, 1);
        EiDevice.delay_ms(100);
        Led_control(LED2, 1);
        EiDevice.delay_ms(100);
        Led_control(LEDALL, 0);
        EiDevice.delay_ms(200);
        Led_control(LEDALL, 1);
        EiDevice.delay_ms(200);
        Led_control(LEDALL, 0);

        ei_program_state = eiStateIdle;
    }
}

/**
 * @brief      Get the data output baudrate
 *
 * @param      baudrate    Baudrate used to output data
 *
 * @return     0
 */
int EiDeviceTiLaunchXl::get_data_output_baudrate(ei_device_data_output_baudrate_t *baudrate)
{
    return get_data_output_baudrate_c(baudrate);
}

/**
 * @brief      Set output baudrate to max
 *
 */
void EiDeviceTiLaunchXl::set_max_data_output_baudrate()
{
    Serial_set_baudrate(MAX_BAUD);
}

/**
 * @brief      Set output baudrate to default
 *
 */
void EiDeviceTiLaunchXl::set_default_data_output_baudrate()
{
    Serial_set_baudrate(DEFAULT_BAUD);
}

void EiDeviceTiLaunchXl::init_device_id(void)
{
    
}

/**
 * @brief      Get a C callback for the get_id method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceTiLaunchXl::get_id_function(void)
{
    return &get_id_c;
}

/**
 * @brief      Get a C callback for the set_id method
 *
 * @return     Pointer to c get function
 */
c_callback_set_id EiDeviceTiLaunchXl::set_id_function(void)
{
    return &set_id_c;
}

/**
 * @brief      Set the device ID
 *
 * @param      device_id   MAC address
 *
 * @return     0
 */
int EiDeviceTiLaunchXl::set_id(char *device_id)
{
    return set_id_c(device_id);
}

/**
 * @brief      Get a C callback for the get_type method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceTiLaunchXl::get_type_function(void)
{
    return &get_type_c;
}

/**
 * @brief      Get a C callback for the get_wifi_connection_status method
 *
 * @return     Pointer to c get function
 */
c_callback_status EiDeviceTiLaunchXl::get_wifi_connection_status_function(void)
{
    return &get_wifi_connection_status_c;
}

/**
 * @brief      Get a C callback for the wifi present method
 *
 * @return     The wifi present status function.
 */
c_callback_status EiDeviceTiLaunchXl::get_wifi_present_status_function(void)
{
    return &get_wifi_present_status_c;
}

/**
 * @brief      Get a C callback to the read sample buffer function
 *
 * @return     The read sample buffer function.
 */
c_callback_read_sample_buffer EiDeviceTiLaunchXl::get_read_sample_buffer_function(void)
{
    return &read_sample_buffer;
}

/**
 * @brief      Get characters for uart pheripheral and send to repl
 */
void ei_command_line_handle(void)
{
    uint8_t data;

    data = Serial_In();

    if(data != 0) {
        rx_callback(data);
    }
}

/**
 * @brief      Setup the serial port
 */
void ei_serial_setup(void)
{

}

/**
 * @brief      Write serial data with length to Serial output
 *
 * @param      data    The data
 * @param[in]  length  The length
 */
void ei_write_string(char *data, int length)
{
    Serial_Out(data, length);
}

/**
 * @brief Period LED control. Called every 200 ms
 *
 */
extern "C" void timer_led_callback(void)
{
    static char toggle = 0;

    if (toggle) {
        switch (ei_program_state) {
        case eiStateErasingFlash:
            Led_control(LED1, 1);
            break;
        case eiStateSampling:
            Led_control(LED2, 1);
            break;
        case eiStateUploading:
            Led_control(LEDALL, 1);
            break;
        default:
            break;
        }
    }
    else {
        if (ei_program_state != eiStateFinished) {
            Led_control(LEDALL, 0);
        }
    }
    toggle ^= 1;
}


char ei_getchar()
{
    return (char)Serial_In();

}


static int get_id_c(uint8_t out_buffer[32], size_t *out_size)
{
    size_t length = strlen(ei_device_id);

    if (length < 32) {
        memcpy(out_buffer, ei_device_id, length);

        *out_size = length;
        return 0;
    }

    else {
        *out_size = 0;
        return -1;
    }
}

static int set_id_c(char *device_id)
{
    size_t length = strlen(ei_device_id);
    if (length > 31) {
        return -1;
    }

    memcpy(ei_device_id, device_id, strlen(device_id) + 1);

    return 0;
}

static int get_type_c(uint8_t out_buffer[32], size_t *out_size)
{
    size_t length = strlen(ei_device_type);

    if (length < 32) {
        memcpy(out_buffer, ei_device_type, length);

        *out_size = length;
        return 0;
    }

    else {
        *out_size = 0;
        return -1;
    }
}

static bool get_wifi_connection_status_c(void)
{
    return false;
}

static bool get_wifi_present_status_c(void)
{
    return false;
}

static int get_data_output_baudrate_c(ei_device_data_output_baudrate_t *baudrate)
{
    size_t length = strlen(ei_dev_max_data_output_baudrate.str);

    if (length < 32) {
        memcpy(baudrate, &ei_dev_max_data_output_baudrate, sizeof(ei_device_data_output_baudrate_t));
        return 0;
    }
    else {
        return -1;
    }
}

/**
 * @brief      Read samples from sample memory and send to data_fn function
 *
 * @param[in]  begin    Start address
 * @param[in]  length   Length of samples in bytes
 * @param[in]  data_fn  Callback function for sample data
 *
 * @return     false on flash read function
 */
static bool read_sample_buffer(size_t begin, size_t length, void (*data_fn)(uint8_t *, size_t))
{
    size_t pos = begin;
    size_t bytes_left = length;
    bool retVal;

    EiDevice.set_state(eiStateUploading);

    // we're encoding as base64 in AT+READFILE, so this needs to be divisable by 3
    uint8_t buffer[513];
    while (1) {
        size_t bytes_to_read = sizeof(buffer);
        if (bytes_to_read > bytes_left) {
            bytes_to_read = bytes_left;
        }
        if (bytes_to_read == 0) {
            retVal = true;
            break;
        }

        int r = ei_ti_launchxl_fs_read_sample_data(buffer, pos, bytes_to_read);
        if (r != 0) {
            retVal = false;
            break;
        }
        data_fn(buffer, bytes_to_read);

        pos += bytes_to_read;
        bytes_left -= bytes_to_read;
    }

    ei_ti_launchxl_fs_close_sample_file();
    EiDevice.set_state(eiStateFinished);

    return retVal;
}
