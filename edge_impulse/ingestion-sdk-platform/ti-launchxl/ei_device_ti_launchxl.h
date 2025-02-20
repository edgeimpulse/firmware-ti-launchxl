/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EI_DEVICE_TI_LAUNCHXL
#define EI_DEVICE_TI_LAUNCHXL

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_interface.h"

#define DEFAULT_BAUD 115200
#define MAX_BAUD 921600


/** Number of sensors used */
#define EI_DEVICE_N_SENSORS 2

#define EI_RESOLUTIONS_BASE 4
#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA
#define EI_DEVICE_N_RESOLUTIONS     EI_RESOLUTIONS_BASE+1
#else
#define EI_DEVICE_N_RESOLUTIONS     EI_RESOLUTIONS_BASE
#endif

#define EI_DEVICE_N_RESIZE_RESOLUTIONS      4

/** Led definition */
typedef enum
{
    LED1 = 0, LED2, LEDALL
} tEiLeds;

/** C Callback types */
typedef int (*c_callback)(uint8_t out_buffer[32], size_t *out_size);
typedef int (*c_callback_set_id)(char *device_id);
typedef bool (*c_callback_status)(void);
typedef bool (*c_callback_read_sample_buffer)(
    size_t begin,
    size_t length,
    void (*data_fn)(uint8_t *, size_t));

/**
 * @brief      Class description and implementation of device specific
 * 			   characteristics
 */
class EiDeviceTiLaunchXl : public EiDeviceInfo {
private:
    ei_device_sensor_t sensors[EI_DEVICE_N_SENSORS];
    ei_device_snapshot_resolutions_t snapshot_resolutions[EI_DEVICE_N_RESOLUTIONS];
public:
    EiDeviceTiLaunchXl(void);

    int get_id(uint8_t out_buffer[32], size_t *out_size);
    const char *get_id_pointer(void);
    int set_id(char *device_id);
    int get_type(uint8_t out_buffer[32], size_t *out_size);
    const char *get_type_pointer(void);
    bool get_wifi_connection_status(void);
    bool get_wifi_present_status();
    bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size);
    bool get_snapshot_list(const ei_device_snapshot_resolutions_t **resolution_list, size_t *resolution_list_size,
                        const char **color_depth);
    void delay_ms(uint32_t milliseconds);
    void setup_led_control(void);
    void set_state(EiState state);
	int get_data_output_baudrate(ei_device_data_output_baudrate_t *baudrate);
	void set_default_data_output_baudrate() override;
	void set_max_data_output_baudrate() override;
    void init_device_id(void) override;

    c_callback get_id_function(void);
	c_callback_set_id set_id_function(void);
    c_callback get_type_function(void);
    c_callback_status get_wifi_connection_status_function(void);
    c_callback_status get_wifi_present_status_function(void);
    c_callback_read_sample_buffer get_read_sample_buffer_function(void);
};

/* Reference to object for external usage ---------------------------------- */
extern EiDeviceTiLaunchXl EiDevice;

void set_max_data_output_baudrate_c();
void set_default_data_output_baudrate_c();

#endif
