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

#ifndef EI_CLASSIFIER_INFERENCING_ENGINE_MEMRYX_H
#define EI_CLASSIFIER_INFERENCING_ENGINE_MEMRYX_H

#if (EI_CLASSIFIER_INFERENCING_ENGINE == EI_CLASSIFIER_MEMRYX)

/**
 * @brief we are forcing SOFTWARE inference (simulation),
 *        beacuse use of hardware is not ready
 *
 */
#ifndef EI_CLASSIFIER_USE_MEMRYX_SOFTWARE
#define EI_CLASSIFIER_USE_MEMRYX_HARDWARE 1
#endif

/**
 * @brief Memryx accelerator can leverage up to four MX3 chips for inference.
 *        Specify here the number of chips to be used for acceleration,
 *        e.g. set to 4 in order to use all four chips of the M3X board.
 */
#ifndef EI_CLASSIFIER_USE_MEMRYX_CHIPS_COUNT
#define EI_CLASSIFIER_USE_MEMRYX_CHIPS_COUNT 1
#endif

#include "model-parameters/model_metadata.h"
#if EI_CLASSIFIER_HAS_MODEL_VARIABLES == 1
#include "model-parameters/model_variables.h"
#endif

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "tensorflow-lite/tensorflow/lite/kernels/internal/reference/softmax.h"
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>
#include <math.h>
#ifdef EI_CLASSIFIER_USE_MEMRYX_SOFTWARE
#include "pybind11/embed.h"
#include "pybind11/numpy.h"
#include "pybind11/stl.h"
#else
#include "memx/memx.h"
#endif
/* Headers below help us bundle the DFP model with EIM in single binary */
#include "memryx-model/memryx-model.h"
#include "utils/model_header_utils.h"

/* Result delivered by memryx simulator contains 3 fields, indexes for print */
#define MX_SIM_RES_OUTPUTS 0
#define MX_SIM_RES_LATENCY 1
#define MX_SIM_RES_FPS     2

std::stringstream engine_info;

static bool memryx_initialized = false;

#ifdef EI_CLASSIFIER_USE_MEMRYX_SOFTWARE
/* brings in the `_a` literals to set args to python API */
using namespace pybind11::literals;
namespace py = pybind11;
/* PyBind variables for EIM with Simulator */
static py::module_ memryx;
static py::module_ np;
static py::object zeroes;
static py::object Simulator;
static py::object model;
static py::object device;
static std::vector<size_t> vec;
#endif

#ifdef EI_CLASSIFIER_USE_MEMRYX_HARDWARE
/* Variables for EIM with Hardware */
const uint8_t flow_id = 0; // flow port 0
const uint8_t model_id = 0; // model 0
const uint8_t group_id = 0; // MPU device group 0
const int timeout = 0; // was 200 ms
int argmax = 0; // index with maximum score
#endif

/* We need a workaround for softmax because
 * the MX3+ is not coming out this year, and
 * the MX3 does not support the SoftMax layer
 */
static tflite::RuntimeShape softmax_shape;
static tflite::SoftmaxParams dummy_params;

static bool verbose_debug = 0;

bool init_memryx(bool debug, const ei_impulse_t *impulse)
{
    /* Unpack DFP model to file system */
    std::string project_file_path = "/tmp/" + std::string(impulse->project_name) + "-" + std::to_string(impulse->project_id) + "-" + std::to_string(impulse->deploy_version);
    create_project_if_not_exists(project_file_path, model_h_files, model_h_files_len);

    std::string proj_model_path = project_file_path + "/memryx_trained.dfp";
    const char * model_file_path = proj_model_path.c_str();
#if (defined(EI_CLASSIFIER_USE_MEMRYX_HARDWARE) && (EI_CLASSIFIER_USE_MEMRYX_HARDWARE == 1))
#warning "Building EIM for use with MemryX Hardware"
    memx_status status = MEMX_STATUS_OK;
    // 1. Bind MPU device group 0 as MX3:Cascade to model 0.
    status = memx_open(model_id, group_id, MEMX_DEVICE_CASCADE);
    if(memx_status_error(status)) {
        return false;
    }
    ei_printf("Memryx device opened.\n");

    // 2. Download model from a DFP file to MPU device group, input and
    // output feature map shape is auto, configured after download complete.
    status = memx_download_model(model_id, model_file_path, 0, // model_idx = 0
                                 MEMX_DOWNLOAD_TYPE_WTMEM_AND_MODEL);
    if(memx_status_error(status)) {
        return false;
    }
    ei_printf("Memryx model downloaded.\n");

    // 3. Enable data transfer of this model to device. Set to no wait here
    // since driver will go to data transfer state eventually.
    status = memx_set_stream_enable(model_id, 0);
    if(memx_status_error(status)) {
        return false;
    }
    ei_printf("Data streaming to and from the MX3 board is enabled\n");
#elif (defined(EI_CLASSIFIER_USE_MEMRYX_SOFTWARE) && (EI_CLASSIFIER_USE_MEMRYX_SOFTWARE == 1))
#warning "MEMRYX model will be run in SIMULATION mode (not on real hardware)!"
    py::list path;
    // import Python's memryx module
    try {
        memryx = py::module_::import("memryx");
        if(debug) printf("Memryx PyModule init\n");
    }
    catch (py::error_already_set &e) {
        ei_printf("ERR: Importing 'memryx' library failed:\n%s\n", e.what());
        return false;
    }

    Simulator = memryx.attr("Simulator");
    if(debug) printf("Simulator API init\n");

    // load model
    try {
        model = Simulator("dfp"_a = model_file_path);
        if(debug) printf("Model API init\n");
    }
    catch (py::error_already_set &e) {
        ei_printf("ERR: Can't load model file from %s\n", model_file_path);
        return false;
    }
#else
#error "Neither EI_CLASSIFIER_USE_MEMRYX_HARDWARE or EI_CLASSIFIER_USE_MEMRYX_SOFTWARE are defined or set to 1"
#endif

    // clear info
    engine_info.str("");

    return true;
}


/**
 * @brief      Do neural network inferencing over the processed feature matrix
 *
 * @param      impulse  Struct describing impulse architecture
 * @param      fmatrix  Processed matrix
 * @param      result   Output classifier results
 * @param[in]  debug    Debug output enable
 *
 * @return     The ei impulse error.
 */
#if (defined(EI_CLASSIFIER_USE_MEMRYX_HARDWARE) && (EI_CLASSIFIER_USE_MEMRYX_HARDWARE == 1))
EI_IMPULSE_ERROR run_nn_inference(
    const ei_impulse_t *impulse,
    ei_feature_t *fmatrix,
    uint32_t learn_block_index,
    uint32_t* input_block_ids,
    uint32_t input_block_ids_size,
    ei_impulse_result_t *result,
    void *config_ptr,
    bool debug = false)
{
    ei_learning_block_config_tflite_graph_t *block_config = (ei_learning_block_config_tflite_graph_t*)config_ptr;

    memx_status status = MEMX_STATUS_OK;
    int32_t ifmap_height, ifmap_width, ifmap_channel_number, ifmap_format;
    int32_t ofmap_height, ofmap_width, ofmap_channel_number, ofmap_format;
    int32_t z;
    uint64_t ctx_start_us = 0;
    uint64_t ctx_end_us = 0;

    // check if we've initialized the interpreter and device?
    if (memryx_initialized == false) {
        if(init_memryx(debug, impulse) == false) {
            return EI_IMPULSE_MEMRYX_ERROR;
        }
        memryx_initialized = true;
    }

    /* 4. get input shape - Not needed during runtime, available only for debugging */
    if(verbose_debug) {
        status = memx_get_ifmap_size(model_id, flow_id, &ifmap_height, &ifmap_width, &z, &ifmap_channel_number, &ifmap_format);
        ei_printf("status = %d, ifmap shape = (%d, %d, %d), format = %d\n",
                   status, ifmap_height, ifmap_width, ifmap_channel_number, ifmap_format);
    }

    // 5. get output shape
    status = memx_get_ofmap_size(model_id, flow_id, &ofmap_height, &ofmap_width, &z, &ofmap_channel_number, &ofmap_format);
    if(debug) {
        ei_printf("status = %d, ofmap shape = (%d, %d, %d), format = %d\n",
                  status, ofmap_height, ofmap_width, ofmap_channel_number, ofmap_format);
    }
    if(memx_status_error(status)) {
        return EI_IMPULSE_MEMRYX_ERROR;
    }

    // 6. Prepare input and output buffers
    float* ofmap = new float [ofmap_width * ofmap_height * ofmap_channel_number];

#if EI_CLASSIFIER_SINGLE_FEATURE_INPUT == 0
    size_t mtx_size = impulse->dsp_blocks_size;
    ei::matrix_t* matrix = NULL;

    size_t combined_matrix_size = get_feature_size(fmatrix, input_block_ids_size, input_block_ids, mtx_size);
    ei::matrix_t combined_matrix(1, combined_matrix_size);

    uint32_t buf_pos = 0;

    for (size_t i = 0; i < input_block_ids_size; i++) {
        size_t cur_mtx = input_block_ids[i];

        if (!find_mtx_by_idx(fmatrix, &matrix, cur_mtx, mtx_size)) {
            ei_printf("ERR: Cannot find matrix with id %zu\n", cur_mtx);
            return EI_IMPULSE_INVALID_SIZE;
        }

        for (size_t ix = 0; ix < matrix->rows * matrix->cols; ix++) {
            combined_matrix.buffer[buf_pos++] = matrix->buffer[ix];
        }
    }
    matrix = &combined_matrix;
#else
    ei::matrix_t* matrix = fmatrix[0].matrix;
#endif

    float* ifmap = (float*)matrix->buffer;

    if(verbose_debug) {
        for(int fidx = 0; fidx < (ofmap_width*ofmap_height); fidx++) {
            ei_printf("%f\t", matrix->buffer[fidx]);
            if(!(fidx % ofmap_width)) ei_printf("\n");
        }
    }

    // TODO stream_ifmap only copies buffer to MX3 board,
    // we need a different approach to measure latency
    ctx_start_us = ei_read_timer_us();
    // 7. Stream inputs to device and start inference.
    status = memx_stream_ifmap(model_id, 0, ifmap, timeout);
    ctx_end_us = ei_read_timer_us();
    if(memx_status_error(status)) {
        return EI_IMPULSE_MEMRYX_ERROR;
    }

    result->timing.classification_us = ctx_end_us - ctx_start_us;
    result->timing.classification = (int)(result->timing.classification_us / 1000);

    engine_info.str("");
    engine_info << "Inferences per second: " << (1000000 / result->timing.classification_us);

    // 6. Stream output results from device after inference
    status = memx_stream_ofmap(model_id, 0, ofmap, timeout);
    if(debug) {
        ei_printf(" memx_stream_ofmap (status=%d)\n", status);
    }
    if(memx_status_error(status)) {
        return EI_IMPULSE_MEMRYX_ERROR;
    }

    // init softmax shape
    std::vector<size_t> output_shape = {static_cast<size_t>(ofmap_height),static_cast<size_t>(ofmap_width),
                                        static_cast<size_t>(ofmap_channel_number)};
    softmax_shape.BuildFrom(output_shape);
    // dumy beta parameter for softmax purposes
    dummy_params.beta = 1;

    // apply softmax, becuase MX3 does not support this operation
    tflite::reference_ops::Softmax(dummy_params, softmax_shape, ofmap, softmax_shape, ofmap);

    size_t output_size = ofmap_width * ofmap_height * ofmap_channel_number;

    result->_raw_outputs[learn_block_index].matrix = new matrix_t(1, output_size);
    result->_raw_outputs[learn_block_index].blockId = block_config->block_id;

    for (size_t i = 0; i < output_size; i++) {
        result->_raw_outputs[learn_block_index].matrix->buffer[i] = ofmap[i];
    }

    delete[] ofmap;
    // Device is closed only at EIM exit, therefore we do not use memx_close()
    return EI_IMPULSE_OK;
}

#elif (defined(EI_CLASSIFIER_USE_MEMRYX_SOFTWARE) && (EI_CLASSIFIER_USE_MEMRYX_SOFTWARE == 1))
EI_IMPULSE_ERROR run_nn_inference(
    const ei_impulse_t *impulse,
    ei_feature_t *fmatrix,
    uint32_t learn_block_index,
    uint32_t* inputBlockIds,
    ei_impulse_result_t *result,
    void *config_ptr,
    bool debug = false)
{
    ei_learning_block_config_tflite_graph_t *block_config = (ei_learning_block_config_tflite_graph_t*)config_ptr;
    ei_config_memryx_graph_t *graph_config = (ei_config_memryx_graph_t*)block_config->graph_config;

    // init Python embedded interpreter (should be called once!)
    static py::scoped_interpreter guard{};

    // check if we've initialized the interpreter and device?
    if (memryx_initialized == false) {
        if(init_memryx(debug, impulse) == false) {
            return EI_IMPULSE_MEMRYX_ERROR;
        }
        memryx_initialized = true;
    }

    std::vector<size_t> input_shape = {1, impulse->input_width, impulse->input_height, 3};
    py::array_t<float> input_data(input_shape); // = zeroes(input_shape, 0);

    printf("impulse->w=%d h=%d\n", impulse->input_width, impulse->input_height);

    /*
     * convert features data to the expected shape (4dim)
     * For images RGB shape is (width, height, colors)
     * For images BW shape is (width, height, 1)
     * For Audio shape is (width, height, 1) - spectrogram
     */
    auto r = input_data.mutable_unchecked<4>();

    for (size_t i = 0; i < input_block_ids_size; i++) {
        uint16_t cur_mtx = input_block_ids[i];
#if EI_CLASSIFIER_SINGLE_FEATURE_INPUT == 0
        ei::matrix_t* matrix = NULL;

        if (!find_mtx_by_idx(fmatrix, &matrix, cur_mtx, mtx_size)) {
            ei_printf("ERR: Cannot find matrix with id %zu\n", cur_mtx);
            return EI_IMPULSE_INVALID_SIZE;
        }
#else
        ei::matrix_t* matrix = fmatrix[0].matrix;
#endif
        for (py::ssize_t x = 0; x < r.shape(1); x++) {
            for (py::ssize_t y = 0; y < r.shape(2); y++) {
                for(py::ssize_t z = 0; z < r.shape(3); z++) {
                r(0, x, y, z) = (float)(fmatrix.buffer[x * r.shape(2) * r.shape(3) + y * r.shape(3) + z]);
                }
            }
        }
    }

    py::object runmodel = model.attr("run");
    // result from mx_sim is {np array, float, float}
    py::tuple args = py::make_tuple(py::none(), 0.00, 0.00);
    // run inference in sumualtor
    printf("start inference\n");
    uint64_t ctx_start_us = ei_read_timer_us();
    args = runmodel("inputs"_a=input_data,"frames"_a=1);
    uint64_t ctx_end_us = ei_read_timer_us();
    printf("end of inference\n");

    result->timing.classification_us = ctx_end_us - ctx_start_us;
    result->timing.classification = (int)(result->timing.classification_us / 1000);

    engine_info.str("");
    engine_info << "Inferences per second: " << (1000000 / result->timing.classification_us);

    py::array outputs = py::list(args[0]);
    py::array_t<float> potentials;
    std::vector<float> potentials_v;

    potentials = outputs.squeeze().cast<py::array_t<float>>();

    if (graph_config->object_detection_last_layer == EI_CLASSIFIER_LAST_LAYER_UNKNOWN) {
        potentials_v = outputs.squeeze().cast<std::vector<float>>();
    }
    else {
        auto q = potentials.unchecked<>();
        for (py::ssize_t x = 0; x < q.shape(0); x++) {
            for (py::ssize_t y = 0; y < q.shape(1); y++) {
                for(py::ssize_t z = 0; z < q.shape(2); z++) {
                    potentials_v.push_back(q(x, y, z));
                }
            }
        }
    }

    if(debug) {
        std::string ret_str = py::str(potentials).cast<std::string>();
        ei_printf("Memryx raw output:\n%s\n", ret_str.c_str());
    }

    size_t output_size = potentials_v.size();

    result->_raw_outputs[learn_block_index].matrix = new matrix_t(1, output_size);
    result->_raw_outputs[learn_block_index].blockId = block_config->block_id;

    for (size_t i = 0; i < output_size; i++) {
        result->_raw_outputs[learn_block_index].matrix->buffer[i] = potentials_v[i];
    }

    return EI_IMPULSE_OK;
}
#else
#error "Neither EI_CLASSIFIER_USE_MEMRYX_HARDWARE or EI_CLASSIFIER_USE_MEMRYX_SOFTWARE are defined or set to 1"
#endif // USE_HARDWARE
#endif // EI_CLASSIFIER_INFERENCING_ENGINE == EI_CLASSIFIER_MEMRYX
#endif /* EI_CLASSIFIER_INFERENCING_ENGINE_MEMRYX_H */
