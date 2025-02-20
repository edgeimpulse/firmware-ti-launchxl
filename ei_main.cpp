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
// #include "ei_run_classifier.h"
#include "ei_device_ti_launchxl.h"
#include "ei_run_impulse.h"
#include "ei_ti_launchxl_fs_commands.h"
#include "ei_classifier_porting.h"
#include "numpy.hpp"
#include "at_cmds.h"

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>

void *ei_command_line_thread(void *arg0)
{
    while (1) {
        ei_command_line_handle();
    }
    return NULL;
}

/**
 * @brief Init sensors, load config and run command handler
 * 
 * @return int 
 */
extern "C" int ei_main() {

    ei_printf("Hello from Edge Impulse Device SDK.\r\n"
        "Compiled on %s %s\r\n", __DATE__, __TIME__);

    if(ei_ti_launchxl_fs_init()) {
        ei_printf("ERR: Failed to initialize file system\r\n");
    }

    /* Intialize configuration */
    static ei_config_ctx_t config_ctx = {0};
    config_ctx.get_device_id = EiDevice.get_id_function();
    config_ctx.set_device_id = EiDevice.set_id_function();
    config_ctx.get_device_type = EiDevice.get_type_function();
    config_ctx.wifi_connection_status = EiDevice.get_wifi_connection_status_function();
    config_ctx.wifi_present = EiDevice.get_wifi_present_status_function();
    config_ctx.load_config = &ei_ti_launchxl_fs_load_config;
    config_ctx.save_config = &ei_ti_launchxl_fs_save_config;
    config_ctx.list_files = NULL;
    config_ctx.read_buffer = EiDevice.get_read_sample_buffer_function();
    // config_ctx.take_snapshot = &ei_camera_take_snapshot_output_on_serial;
    // config_ctx.start_snapshot_stream = &ei_camera_start_snapshot_stream;

    EI_CONFIG_ERROR cr = ei_config_init(&config_ctx);

    if (cr != EI_CONFIG_OK) {
        ei_printf("Failed to initialize configuration (%d)\n", cr);
    } else {
        ei_printf("Loaded configuration\n");
    }

    /* Setup the command line commands */
    ei_at_register_generic_cmds();
    ei_at_cmd_register("RUNIMPULSE", "Run the impulse", run_nn_normal);
    ei_at_cmd_register("RUNIMPULSECONT", "Run the impulse", run_nn_continuous_normal);
    ei_at_cmd_register("RUNIMPULSEDEBUG", "Run the impulse with extra debug output", run_nn_debug);
    ei_printf("Type AT+HELP to see a list of commands.\r\n> ");

    EiDevice.set_state(eiStateFinished);

    pthread_t           thread0;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    /* Set priority and stack size attributes */
    pthread_attr_init(&attrs);
    priParam.sched_priority = 1;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    pthread_attr_setschedparam(&attrs, &priParam);

    retc |= pthread_attr_setstacksize(&attrs, 3072);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    /* Create receive thread */
    retc = pthread_create(&thread0, &attrs, ei_command_line_thread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }

    return 0;
}
