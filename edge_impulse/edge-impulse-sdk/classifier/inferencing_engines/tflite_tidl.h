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

#ifndef _EI_CLASSIFIER_INFERENCING_ENGINE_TFLITE_TIDL_H_
#define _EI_CLASSIFIER_INFERENCING_ENGINE_TFLITE_TIDL_H_

#if (EI_CLASSIFIER_INFERENCING_ENGINE == EI_CLASSIFIER_TFLITE_TIDL)

#include "model-parameters/model_metadata.h"

#include <thread>
#include "tensorflow-lite/tensorflow/lite/c/common.h"
#include "tensorflow-lite/tensorflow/lite/interpreter.h"
#include "tensorflow-lite/tensorflow/lite/kernels/register.h"
#include "tensorflow-lite/tensorflow/lite/model.h"
#include "tensorflow-lite/tensorflow/lite/optional_debug_tools.h"
#include "edge-impulse-sdk/classifier/ei_fill_result_struct.h"
#include "edge-impulse-sdk/classifier/ei_model_types.h"

#include "itidl_rt.h"
#if ARMNN_ENABLE
#include "DelegateOptions.hpp"
#include "armnn_delegate.hpp"
#endif

#include <dlfcn.h>

// old models don't have this, add this here
#ifndef EI_CLASSIFIER_TFLITE_OUTPUT_DATA_TENSOR
#define EI_CLASSIFIER_TFLITE_OUTPUT_DATA_TENSOR 0
#endif // not defined EI_CLASSIFIER_TFLITE_OUTPUT_DATA_TENSOR

#include "tflite-model/tidl-model.h"
#include "utils/model_header_utils.h"

void *in_ptrs[16] = {NULL};
void *out_ptrs[16] = {NULL};

EI_IMPULSE_ERROR run_nn_inference(
    const ei_impulse_t *impulse,
    ei_feature_t *fmatrix,
    uint32_t learn_block_index,
    uint32_t* input_block_ids,
    uint32_t input_block_ids_size,
    ei_impulse_result_t *result,
    void *config_ptr,
    bool debug)
{
    ei_learning_block_config_tflite_graph_t *block_config = (ei_learning_block_config_tflite_graph_t*)config_ptr;

    static std::unique_ptr<tflite::FlatBufferModel> model = nullptr;
    static std::unique_ptr<tflite::Interpreter> interpreter = nullptr;
    static std::vector<int> inputs;
    static std::vector<int> outputs;

    if (!model) {

        std::string proj_artifacts_path = "/tmp/" + std::string(impulse->project_name) + "-" + std::to_string(impulse->project_id) + "-" + std::to_string(impulse->deploy_version);

        create_project_if_not_exists(proj_artifacts_path, model_h_files, model_h_files_len);

        std::string proj_model_path = proj_artifacts_path + "/trained.tflite";

        model = tflite::FlatBufferModel::BuildFromFile(proj_model_path.c_str());
        if (!model) {
            ei_printf("Failed to build TFLite model from buffer\n");
            return EI_IMPULSE_TFLITE_ERROR;
        }

        tflite::ops::builtin::BuiltinOpResolver resolver;
        tflite::InterpreterBuilder builder(*model, resolver);
        builder(&interpreter);

        if (!interpreter) {
            ei_printf("Failed to construct interpreter\n");
            return EI_IMPULSE_TFLITE_ERROR;
        }

        /* This part creates the dlg_ptr */
        ei_printf("TIDL delegate mode\n");
        typedef TfLiteDelegate *(*tflite_plugin_create_delegate)(char **, char **, size_t, void (*report_error)(const char *));
        tflite_plugin_create_delegate tflite_plugin_dlg_create;
        char *keys[] = {(char *)"artifacts_folder", (char *)"num_tidl_subgraphs", (char *)"debug_level"};
        char *values[] = {(char *)proj_artifacts_path.c_str(), (char *)"16", (char *)"0"};
        void *lib = dlopen("libtidl_tfl_delegate.so", RTLD_NOW);
        assert(lib);
        tflite_plugin_dlg_create = (tflite_plugin_create_delegate)dlsym(lib, "tflite_plugin_create_delegate");
        TfLiteDelegate *dlg_ptr = tflite_plugin_dlg_create(keys, values, 3, NULL);
        interpreter->ModifyGraphWithDelegate(dlg_ptr);
        ei_printf("ModifyGraphWithDelegate - Done \n");


        if (interpreter->AllocateTensors() != kTfLiteOk) {
            ei_printf("AllocateTensors failed\n");
            return EI_IMPULSE_TFLITE_ERROR;
        }

        int hw_thread_count = (int)std::thread::hardware_concurrency();
        hw_thread_count -= 1; // leave one thread free for the other application
        if (hw_thread_count < 1) {
            hw_thread_count = 1;
        }

        if (interpreter->SetNumThreads(hw_thread_count) != kTfLiteOk) {
            ei_printf("SetNumThreads failed\n");
            return EI_IMPULSE_TFLITE_ERROR;
        }
    }

    inputs = interpreter->inputs();
    outputs = interpreter->outputs();

    ei_printf("device mem enabled\n");
    for (uint32_t i = 0; i < inputs.size(); i++)
    {
        const TfLiteTensor *tensor = interpreter->input_tensor(i);
        in_ptrs[i] = TIDLRT_allocSharedMem(tflite::kDefaultTensorAlignment, tensor->bytes);
        if (in_ptrs[i] == NULL)
        {
            ei_printf("Could not allocate Memory for input: %s\n", tensor->name);
        }
        interpreter->SetCustomAllocationForTensor(inputs[i], {in_ptrs[i], tensor->bytes});
    }
    for (uint32_t i = 0; i < outputs.size(); i++)
    {
        const TfLiteTensor *tensor = interpreter->output_tensor(i);
        out_ptrs[i] = TIDLRT_allocSharedMem(tflite::kDefaultTensorAlignment, tensor->bytes);
        if (out_ptrs[i] == NULL)
        {
            ei_printf("Could not allocate Memory for ouput: %s\n", tensor->name);
        }
        interpreter->SetCustomAllocationForTensor(outputs[i], {out_ptrs[i], tensor->bytes});
    }

    // Obtain pointers to the model's input and output tensors.
#if EI_CLASSIFIER_QUANTIZATION_ENABLED == 1
    int8_t* input = interpreter->typed_input_tensor<int8_t>(0);
#else
    float* input = interpreter->typed_input_tensor<float>(0);
#endif

    if (!input) {
        return EI_IMPULSE_INPUT_TENSOR_WAS_NULL;
    }

    size_t mtx_size = impulse->dsp_blocks_size + impulse->learning_blocks_size;

    for (size_t i = 0; i < input_block_ids_size; i++) {
#if EI_CLASSIFIER_SINGLE_FEATURE_INPUT == 0
        uint16_t cur_mtx = input_block_ids[i];
        ei::matrix_t* matrix = NULL;

        if (!find_mtx_by_idx(fmatrix, &matrix, cur_mtx, mtx_size)) {
            ei_printf("ERR: Cannot find matrix with id %zu\n", cur_mtx);
            return EI_IMPULSE_INVALID_SIZE;
        }
#else
        ei::matrix_t* matrix = fmatrix[0].matrix;
#endif

        for (uint32_t ix = 0; ix < matrix->rows * matrix->cols; ix++) {
            if (block_config->object_detection) {
#if EI_CLASSIFIER_QUANTIZATION_ENABLED == 1
                float pixel = (float)matrix->buffer[ix];
                input[ix] = static_cast<uint8_t>((pixel / input->tflite_input_scale) + input->tflite_input_zeropoint);
#else
                input[ix] = matrix->buffer[ix];
#endif
            }
            else {
#if EI_CLASSIFIER_QUANTIZATION_ENABLED == 1
                input[ix] = static_cast<int8_t>(round(matrix->buffer[ix] / input->tflite_input_scale) + input->tflite_input_zeropoint);
#else
                input[ix] = matrix->buffer[ix];
#endif
            }
        }
    }

    uint64_t ctx_start_us = ei_read_timer_us();

    interpreter->Invoke();

    uint64_t ctx_end_us = ei_read_timer_us();

    result->timing.classification_us = ctx_end_us - ctx_start_us;
    result->timing.classification = (int)(result->timing.classification_us / 1000);

#if EI_CLASSIFIER_QUANTIZATION_ENABLED == 1
    int8_t* out_data = interpreter->typed_output_tensor<int8_t>(block_config->output_data_tensor);
#else
    float* out_data = interpreter->typed_output_tensor<float>(block_config->output_data_tensor);
#endif

    if (debug) {
        ei_printf("LOG_INFO tensors size: %ld \n", interpreter->tensors_size());
        ei_printf("LOG_INFO nodes size: %ld\n", interpreter->nodes_size());
        ei_printf("LOG_INFO number of inputs: %ld\n", inputs.size());
        ei_printf("LOG_INFO number of outputs: %ld\n", outputs.size());
        ei_printf("LOG_INFO input(0) name: %s\n", interpreter->GetInputName(0));

        int t_size = interpreter->tensors_size();
        for (int i = 0; i < t_size; i++)
        {
            if (interpreter->tensor(i)->name) {
                ei_printf("LOG_INFO %d: %s,%ld,%d,%f,%d,size(", i, interpreter->tensor(i)->name,
                            interpreter->tensor(i)->bytes,
                            interpreter->tensor(i)->type,
                            interpreter->tensor(i)->params.scale,
                            interpreter->tensor(i)->params.zero_point);

                for (int k=0; k < interpreter->tensor(i)->dims->size; k++) {
                    if (k == interpreter->tensor(i)->dims->size - 1) {
                        ei_printf("%d", interpreter->tensor(i)->dims->data[k]);
                    } else {
                        ei_printf("%d,", interpreter->tensor(i)->dims->data[k]);
                    }
                }
                ei_printf(")\n");
            }
        }
    }

    if (!out_data) {
        return EI_IMPULSE_OUTPUT_TENSOR_WAS_NULL;
    }

    if (debug) {
        ei_printf("Predictions (time: %d ms.):\n", result->timing.classification);
    }

    EI_IMPULSE_ERROR fill_res = EI_IMPULSE_OK;

    if (block_config->object_detection) {
        switch (block_config->object_detection_last_layer) {
            case EI_CLASSIFIER_LAST_LAYER_FOMO: {
                if (block_config->quantized == 1) {
#if EI_CLASSIFIER_QUANTIZATION_ENABLED == 1
                    fill_res = fill_result_struct_i8_fomo(
                        impulse,
                        block_config,
                        result,
                        out_data,
                        out_data->tflite_output_zeropoint,
                        out_data->tflite_output_scale,
                        impulse->fomo_output_size,
                        impulse->fomo_output_size);
#endif
                }
                else {
                    fill_res = fill_result_struct_f32_fomo(
                        impulse,
                        block_config,
                        result,
                        out_data,
                        impulse->fomo_output_size,
                        impulse->fomo_output_size);
                }
                break;
            }
            case EI_CLASSIFIER_LAST_LAYER_SSD: {
                float *scores_tensor = interpreter->typed_output_tensor<float>(block_config->output_score_tensor);
                float *label_tensor = interpreter->typed_output_tensor<float>(block_config->output_labels_tensor);
                if (!scores_tensor) {
                    return EI_IMPULSE_SCORE_TENSOR_WAS_NULL;
                }
                if (!label_tensor) {
                    return EI_IMPULSE_LABEL_TENSOR_WAS_NULL;
                }
                if (block_config->quantized == 1) {
                    ei_printf("ERR: MobileNet SSD does not support quantized inference\n");
                    return EI_IMPULSE_UNSUPPORTED_INFERENCING_ENGINE;
                }
                else {
                    fill_res = fill_result_struct_f32_object_detection(
                        impulse,
                        block_config,
                        result,
                        out_data,
                        scores_tensor,
                        label_tensor,
                        debug);
                }
                break;
            }
            case EI_CLASSIFIER_LAST_LAYER_YOLOV5:
            case EI_CLASSIFIER_LAST_LAYER_YOLOV5_V5_DRPAI: {
                if (block_config->quantized == 1) {
                    ei_printf("ERR: YOLOv5 does not support quantized inference\n");
                    return EI_IMPULSE_UNSUPPORTED_INFERENCING_ENGINE;
                }
                else {
                    int version = block_config->object_detection_last_layer == EI_CLASSIFIER_LAST_LAYER_YOLOV5_V5_DRPAI ?
                        5 : 6;
                    fill_res = fill_result_struct_f32_yolov5(
                        impulse,
                        block_config,
                        result,
                        version,
                        out_data,
                        impulse->tflite_output_features_count,
                        debug);
                }
                break;
            }
            case EI_CLASSIFIER_LAST_LAYER_YOLOX: {
                if (block_config->quantized == 1) {
                    ei_printf("ERR: YOLOX does not support quantized inference\n");
                    return EI_IMPULSE_UNSUPPORTED_INFERENCING_ENGINE;
                }
                else {
                    fill_res = fill_result_struct_f32_yolox(
                        impulse,
                        block_config,
                        result,
                        out_data,
                        impulse->tflite_output_features_count,
                        debug);
                }
                break;
            }
            case EI_CLASSIFIER_LAST_LAYER_YOLOV7: {
                if (block_config->quantized == 1) {
                    ei_printf("ERR: YOLOV7 does not support quantized inference\n");
                    return EI_IMPULSE_UNSUPPORTED_INFERENCING_ENGINE;
                }
                else {
                    TfLiteTensor *output = interpreter->output_tensor(0);
                    size_t output_feature_count = 1;
                    for (int ix = 0; ix < output->dims->size; ix++) {
                        output_feature_count *= output->dims->data[ix];
                    }
                    fill_res = fill_result_struct_f32_yolov7(
                        impulse,
                        block_config,
                        result,
                        output->data.f,
                        output_feature_count);
                }
                break;
            }
            default: {
                ei_printf("ERR: Unsupported object detection last layer (%d)\n",
                    block_config->object_detection_last_layer);
                break;
            }
        }
    }
    else {
#if EI_CLASSIFIER_QUANTIZATION_ENABLED == 1
        fill_res = fill_result_struct_i8(impulse, result, out_data, out_data->tflite_output_zeropoint, out_data->tflite_output_scale, debug);
#else
        fill_res = fill_result_struct_f32(impulse, result, out_data, debug);
#endif
    }

    for (uint32_t i = 0; i < inputs.size(); i++)
    {
        if (in_ptrs[i])
        {
            TIDLRT_freeSharedMem(in_ptrs[i]);
        }
    }
    for (uint32_t i = 0; i < outputs.size(); i++)
    {
        if (out_ptrs[i])
        {
            TIDLRT_freeSharedMem(out_ptrs[i]);
        }
    }

    if (fill_res != EI_IMPULSE_OK) {
        return fill_res;
    }

    // on Linux we're not worried about free'ing (for now)

    return EI_IMPULSE_OK;
}

#endif // (EI_CLASSIFIER_INFERENCING_ENGINE == EI_CLASSIFIER_TFLITE_TIDL)
#endif // _EI_CLASSIFIER_INFERENCING_ENGINE_TFLITE_TIDL_H_
