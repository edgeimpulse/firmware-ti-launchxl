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

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "firmware-sdk/ei_config_types.h"
#include "ei_inertialsensor.h"
#include "ei_device_ti_launchxl.h"
#include "firmware-sdk/sensor-aq/sensor_aq.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

#define ACC_SAMPLE_TIME_MS  0.724f
#define FLASH_WRITE_TIME_MS 1.5731f

extern ei_config_t *ei_config_get_config();
extern EI_CONFIG_ERROR ei_config_set_sample_interval(float interval);

extern "C" int bmi160_getData(float *acc);

/* Private variables ------------------------------------------------------- */
static uint32_t samplerate_divider;
static float imu_data[N_AXIS_SAMPLED];

sampler_callback  cb_sampler;


/**
 * @brief      Get data from sensor, convert and call callback to handle
 */
int ei_inertial_read_data(void)
{
    float acc_data[3];
    volatile uint32_t div_sample_count;

    for (div_sample_count = 0; div_sample_count < samplerate_divider; div_sample_count++) {

        if(bmi160_getData(acc_data)) {
            return -1;
        }
    }

    imu_data[0] = acc_data[0] * CONVERT_G_TO_MS2;
    imu_data[1] = acc_data[1] * CONVERT_G_TO_MS2;
    imu_data[2] = acc_data[2] * CONVERT_G_TO_MS2;

    cb_sampler((const void *)&imu_data[0], SIZEOF_N_AXIS_SAMPLED);

    return 0;
}

/**
 * @brief      Setup timing and data handle callback function
 *
 * @param[in]  callsampler         Function to handle the sampled data
 * @param[in]  sample_interval_ms  The sample interval milliseconds
 *
 * @return     true
 */
bool ei_inertial_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
    cb_sampler = callsampler;

    samplerate_divider = (int)((sample_interval_ms-FLASH_WRITE_TIME_MS) / ACC_SAMPLE_TIME_MS);

    EiDevice.set_state(eiStateSampling);

    return true;
}

/**
 * @brief      Setup payload header
 *
 * @return     true
 */
bool ei_inertial_setup_data_sampling(void)
{
    bool sample_start;

    if (ei_config_get_config()->sample_interval_ms < 0.001f) {
        ei_config_set_sample_interval(1.f / 62.5f);
    }

    sensor_aq_payload_info payload = {
        // Unique device ID (optional), set this to e.g. MAC address or device EUI **if** your device has one
        EiDevice.get_id_pointer(),
        // Device type (required), use the same device type for similar devices
        EiDevice.get_type_pointer(),
        // How often new data is sampled in ms. (100Hz = every 10 ms.)
        ei_config_get_config()->sample_interval_ms,
        // The axes which you'll use. The units field needs to comply to SenML units (see https://www.iana.org/assignments/senml/senml.xhtml)
        { { "accX", "m/s2" }, { "accY", "m/s2" }, { "accZ", "m/s2" } },
    };

    EiDevice.set_state(eiStateErasingFlash);
    sample_start = ei_sampler_start_sampling(&payload, &ei_inertial_sample_start, SIZEOF_N_AXIS_SAMPLED);
    EiDevice.set_state(eiStateIdle);

    return sample_start;
}