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
// #include "ei_run_classifier.h"
#include "ei_device_ti_launchxl.h"
#include "ei_run_impulse.h"
#include "ei_ti_launchxl_fs_commands.h"
#include "ei_classifier_porting.h"
#include "numpy.hpp"
#include "at_cmds.h"

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
    // ei_at_cmd_register("RUNIMPULSECONT", "Run the impulse", run_nn_continuous_normal);
    ei_at_cmd_register("RUNIMPULSEDEBUG", "Run the impulse with extra debug output", run_nn_debug);
    ei_printf("Type AT+HELP to see a list of commands.\r\n> ");

    EiDevice.set_state(eiStateFinished);

    while (1) {
        ei_command_line_handle();
    }
}
