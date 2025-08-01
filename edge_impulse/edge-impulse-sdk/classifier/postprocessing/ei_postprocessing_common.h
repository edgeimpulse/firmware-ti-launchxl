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

#ifndef EI_POSTPROCESSING_COMMON_H
#define EI_POSTPROCESSING_COMMON_H

#include "model-parameters/model_metadata.h"
#include "edge-impulse-sdk/classifier/ei_model_types.h"
#include "edge-impulse-sdk/classifier/ei_classifier_types.h"
#include "edge-impulse-sdk/classifier/ei_nms.h"
#include "edge-impulse-sdk/dsp/ei_vector.h"
#include <string>

#ifndef EI_HAS_OBJECT_DETECTION
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_SSD)
    #define EI_HAS_SSD 1
    #endif
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_FOMO)
    #define EI_HAS_FOMO 1
    #endif
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_YOLOV5) || (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_YOLOV5_V5_DRPAI)
    #define EI_HAS_YOLOV5 1
    #endif
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_YOLOX)
    #define EI_HAS_YOLOX 1
    #endif
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_YOLOV7)
    #define EI_HAS_YOLOV7 1
    #endif
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_TAO_RETINANET) || (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_TAO_SSD)
    #define EI_HAS_TAO_DECODE_DETECTIONS 1
    #endif
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_TAO_YOLOV3) || (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_TAO_YOLOV4)
    #define EI_HAS_TAO_YOLO 1
    #endif
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_TAO_YOLOV3)
    #define EI_HAS_TAO_YOLOV3 1
    #endif
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_TAO_YOLOV4)
    #define EI_HAS_TAO_YOLOV4 1
    #endif
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_YOLOV2)
    #define EI_HAS_YOLOV2 1
    #endif
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_YOLO_PRO)
    #define EI_HAS_YOLO_PRO 1
    #endif
    #if (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_YOLOV11) || (EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER == EI_CLASSIFIER_LAST_LAYER_YOLOV11_ABS)
    #define EI_HAS_YOLOV11 1
    #endif
#endif

int16_t get_block_number(ei_impulse_handle_t *handle, void *init_func)
{
    for (size_t i = 0; i < handle->impulse->postprocessing_blocks_size; i++) {
        if (handle->impulse->postprocessing_blocks[i].init_fn == init_func) {
            return i;
        }
    }
    return -1;
}

typedef struct cube {
    uint32_t x;
    uint32_t y;
    uint32_t width;
    uint32_t height;
    float confidence;
    const char *label;
} ei_classifier_cube_t;

typedef struct {
    float threshold;
} ei_fill_result_object_detection_threshold_config_t;

typedef struct {
    float threshold;
    uint8_t version;
    uint32_t object_detection_count;
    uint32_t output_features_count;
    ei_object_detection_nms_config_t nms_config;
} ei_fill_result_object_detection_f32_config_t;

typedef struct {
    float threshold;
    uint8_t version;
    uint32_t object_detection_count;
    uint32_t output_features_count;
    float zero_point;
    float scale;
    ei_object_detection_nms_config_t nms_config;
} ei_fill_result_object_detection_i8_config_t;

typedef struct {
    float zero_point;
    float scale;
} ei_fill_result_classification_i8_config_t;

typedef struct {
    float threshold;
    uint16_t out_width;
    uint16_t out_height;
    uint32_t object_detection_count;
} ei_fill_result_fomo_f32_config_t;

typedef struct {
    float threshold;
    uint16_t out_width;
    uint16_t out_height;
    uint32_t object_detection_count;
    float zero_point;
    float scale;
} ei_fill_result_fomo_i8_config_t;

typedef struct {
    float threshold;
    uint16_t grid_size_x;
    uint16_t grid_size_y;
} ei_fill_result_visual_ad_f32_config_t;

/**
 * Checks whether a new section overlaps with a cube,
 * and if so, will **update the cube**
 */
__attribute__((unused)) static bool ei_cube_check_overlap(ei_classifier_cube_t *c, uint32_t x, uint32_t y, uint32_t width, uint32_t height, float confidence) {
    bool is_overlapping = !(c->x + c->width < x || c->y + c->height < y || c->x > x + width || c->y > y + height);
    if (!is_overlapping) return false;

    // if we overlap, but the x of the new box is lower than the x of the current box
    if (x < c->x) {
        // update x to match new box and make width larger (by the diff between the boxes)
        c->x = x;
        c->width += c->x - x;
    }
    // if we overlap, but the y of the new box is lower than the y of the current box
    if (y < c->y) {
        // update y to match new box and make height larger (by the diff between the boxes)
        c->y = y;
        c->height += c->y - y;
    }
    // if we overlap, and x+width of the new box is higher than the x+width of the current box
    if (x + width > c->x + c->width) {
        // just make the box wider
        c->width += (x + width) - (c->x + c->width);
    }
    // if we overlap, and y+height of the new box is higher than the y+height of the current box
    if (y + height > c->y + c->height) {
        // just make the box higher
        c->height += (y + height) - (c->y + c->height);
    }
    // if the new box has higher confidence, then override confidence of the whole box
    if (confidence > c->confidence) {
        c->confidence = confidence;
    }
    return true;
}

__attribute__((unused)) static void ei_handle_cube(std::vector<ei_classifier_cube_t*> *cubes, uint32_t x, uint32_t y, float vf, const char *label, float detection_threshold) {
    if (vf < detection_threshold) return;

    bool has_overlapping = false;
    uint32_t width = 1;
    uint32_t height = 1;

    for (auto c : *cubes) {
        // not cube for same class? continue
        if (strcmp(c->label, label) != 0) continue;

        if (ei_cube_check_overlap(c, x, y, width, height, vf)) {
            has_overlapping = true;
            break;
        }
    }

    if (!has_overlapping) {
        ei_classifier_cube_t *cube = new ei_classifier_cube_t();
        cube->x = x;
        cube->y = y;
        cube->width = 1;
        cube->height = 1;
        cube->confidence = vf;
        cube->label = label;
        cubes->push_back(cube);
    }
}

__attribute__((unused)) static void process_cubes(ei_impulse_result_t *result, std::vector<ei_classifier_cube_t*> *cubes, uint32_t out_width_factor, uint32_t object_detection_count) {
    std::vector<ei_classifier_cube_t*> bbs;
    static std::vector<ei_impulse_result_bounding_box_t> results;
    uint32_t added_boxes_count = 0;
    results.clear();

    for (auto sc : *cubes) {
        bool has_overlapping = false;

        uint32_t x = sc->x;
        uint32_t y = sc->y;
        uint32_t width = sc->width;
        uint32_t height = sc->height;
        const char *label = sc->label;
        float vf = sc->confidence;

        for (auto c : bbs) {
            // not cube for same class? continue
            if (strcmp(c->label, label) != 0) continue;

            if (ei_cube_check_overlap(c, x, y, width, height, vf)) {
                has_overlapping = true;
                break;
            }
        }

        if (has_overlapping) {
            continue;
        }

        bbs.push_back(sc);

        ei_impulse_result_bounding_box_t tmp = {
            .label = sc->label,
            .x = (uint32_t)(sc->x * out_width_factor),
            .y = (uint32_t)(sc->y * out_width_factor),
            .width = (uint32_t)(sc->width * out_width_factor),
            .height = (uint32_t)(sc->height * out_width_factor),
            .value = sc->confidence
        };

        results.push_back(tmp);
        added_boxes_count++;
    }

    // if we didn't detect min required objects, fill the rest with fixed value
    if (added_boxes_count < object_detection_count) {
        results.resize(object_detection_count);
        for (size_t ix = added_boxes_count; ix < object_detection_count; ix++) {
            results[ix].value = 0.0f;
        }
    }

    for (auto c : *cubes) {
        delete c;
    }

    result->bounding_boxes = results.data();
    result->bounding_boxes_count = added_boxes_count;
}

/**
 * Fill the result structure from an unquantized output tensor
 */
EI_IMPULSE_ERROR process_classification_f32(ei_impulse_handle_t *handle,
                                uint32_t block_index,
                                uint32_t input_block_id,
                                ei_impulse_result_t *result,
                                void *config_ptr,
                                void *state)
{
    const ei_impulse_t *impulse = handle->impulse;

#ifdef EI_DSP_RESULT_OVERRIDE
    uint32_t stop_count = EI_DSP_RESULT_OVERRIDE;
#else
    uint32_t stop_count = impulse->label_count;
#endif

    ei::matrix_t* raw_output_mtx = NULL;
    bool status = find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);
    if (!status) {
        return EI_IMPULSE_POSTPROCESSING_ERROR;
    }

    for (uint32_t ix = 0; ix < stop_count; ix++) {
        float value = raw_output_mtx->buffer[ix];

#if EI_LOG_LEVEL == EI_LOG_LEVEL_DEBUG
        ei_printf("%s:\t", impulse->categories[ix]);
        ei_printf_float(value);
        ei_printf("\n");
#endif

// For testing purposes, we will have more values than labels
#ifndef EI_DSP_RESULT_OVERRIDE
        result->classification[ix].label = impulse->categories[ix];
#endif
        result->classification[ix].value = value;
    }

    return EI_IMPULSE_OK;
}

/**
 * Fill the result structure from a quantized output tensor
 */
__attribute__((unused)) static EI_IMPULSE_ERROR process_classification_i8(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_classification_i8_config_t *config = (ei_fill_result_classification_i8_config_t*)config_ptr;

    ei::matrix_i8_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    for (uint32_t ix = 0; ix < impulse->label_count; ix++) {
        float value = static_cast<float>(raw_output_mtx->buffer[ix] - config->zero_point) * config->scale;

#if EI_LOG_LEVEL == EI_LOG_LEVEL_DEBUG
        ei_printf("%s:\t", impulse->categories[ix]);
        ei_printf_float(value);
        ei_printf("\n");
#endif

        result->classification[ix].label = impulse->categories[ix];
        result->classification[ix].value = value;
    }



    return EI_IMPULSE_OK;
}

/**
 * Fill the result structure from a quantized output tensor
 */
__attribute__((unused)) static EI_IMPULSE_ERROR process_classification_u8(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_classification_i8_config_t *config = (ei_fill_result_classification_i8_config_t*)config_ptr;

    // legacy unsigned quantized output
    ei::matrix_u8_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    for (uint32_t ix = 0; ix < impulse->label_count; ix++) {
        float value = static_cast<float>(raw_output_mtx->buffer[ix] - config->zero_point) * config->scale;

#if EI_LOG_LEVEL == EI_LOG_LEVEL_DEBUG
        ei_printf("%s:\t", impulse->categories[ix]);
        ei_printf_float(value);
        ei_printf("\n");
#endif

        result->classification[ix].label = impulse->categories[ix];
        result->classification[ix].value = value;
    }

    return EI_IMPULSE_OK;
}

/**
 * Fill the result structure from a quantized output tensor
 */
__attribute__((unused)) static EI_IMPULSE_ERROR process_anomaly(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
    const ei_impulse_t *impulse = handle->impulse;

    ei::matrix_t* raw_output_mtx = NULL;
    bool status = find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);
    if (!status) {
        return EI_IMPULSE_POSTPROCESSING_ERROR;
    }

    result->anomaly = raw_output_mtx->buffer[0];

    return EI_IMPULSE_OK;
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_fomo_f32(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
#ifdef EI_HAS_FOMO
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_fomo_f32_config_t *config = (ei_fill_result_fomo_f32_config_t*)config_ptr;

    std::vector<ei_classifier_cube_t*> cubes;

    int out_width_factor = impulse->input_width / config->out_width;

    ei::matrix_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    for (size_t y = 0; y < config->out_width; y++) {
        for (size_t x = 0; x < config->out_height; x++) {
            size_t loc = ((y * config->out_height) + x) * (impulse->label_count + 1);

            for (size_t ix = 1; ix < impulse->label_count + 1; ix++) {
                float vf = raw_output_mtx->buffer[loc+ix];

                ei_handle_cube(&cubes, x, y, vf, impulse->categories[ix - 1], config->threshold);
            }
        }
    }

    process_cubes(result, &cubes, out_width_factor, config->object_detection_count);

    return EI_IMPULSE_OK;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_fomo_i8(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
#ifdef EI_HAS_FOMO
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_fomo_i8_config_t *config = (ei_fill_result_fomo_i8_config_t*)config_ptr;

    std::vector<ei_classifier_cube_t*> cubes;

    int out_width_factor = impulse->input_width / config->out_width;

    ei::matrix_i8_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    for (size_t y = 0; y < config->out_width; y++) {
        for (size_t x = 0; x < config->out_height; x++) {
            size_t loc = ((y * config->out_height) + x) * (impulse->label_count + 1);

            for (size_t ix = 1; ix < impulse->label_count + 1; ix++) {
                int8_t v = raw_output_mtx->buffer[loc+ix];
                float vf = static_cast<float>(v - config->zero_point) * config->scale;

                ei_handle_cube(&cubes, x, y, vf, impulse->categories[ix - 1], config->threshold);
            }
        }
    }

    process_cubes(result, &cubes, out_width_factor, config->object_detection_count);

    return EI_IMPULSE_OK;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif
}

/**
 * Fill the visual anomaly result structures from an unquantized output tensor
 */
__attribute__((unused)) static EI_IMPULSE_ERROR process_visual_ad_f32(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_visual_ad_f32_config_t *config = (ei_fill_result_visual_ad_f32_config_t*)config_ptr;

    float max_val = 0;
    float sum_val = 0;

    ei::matrix_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    for (uint32_t ix = 0; ix < config->grid_size_x * config->grid_size_y; ix++) {
        float value = raw_output_mtx->buffer[ix];
        sum_val += value;
        if (value > max_val) {
            max_val = value;
        }
    }

    result->visual_ad_result.mean_value = sum_val / (config->grid_size_x * config->grid_size_y);
    result->visual_ad_result.max_value = max_val;

    static ei_vector<ei_impulse_result_bounding_box_t> results;

    results.clear();

    for (uint16_t x = 0; x <= config->grid_size_x - 1; x++) {
        for (uint16_t y = 0; y <= config->grid_size_y - 1; y++) {
            if (raw_output_mtx->buffer[x * config->grid_size_x + y] >= config->threshold) {
                ei_impulse_result_bounding_box_t tmp = {
                    .label = "anomaly",
                    .x = static_cast<uint32_t>(y * (static_cast<float>(impulse->input_height) / config->grid_size_y)),
                    .y = static_cast<uint32_t>(x * (static_cast<float>(impulse->input_width) / config->grid_size_x)),
                    .width = (impulse->input_width / config->grid_size_x),
                    .height = (impulse->input_height / config->grid_size_y),
                    .value = raw_output_mtx->buffer[x * config->grid_size_x + y]
                };

                results.push_back(tmp);
            }
        }
    }

    // result->classification[0].value = result->visual_ad_result.max_value;

    result->visual_ad_grid_cells = results.data();
    result->visual_ad_count = results.size();

#endif // EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    return EI_IMPULSE_OK;
}



__attribute__((unused)) static EI_IMPULSE_ERROR process_ssd_f32(ei_impulse_handle_t *handle,
                                                                             uint32_t block_index,
                                                                             uint32_t input_block_id,
                                                                             ei_impulse_result_t *result,
                                                                             void *config_ptr,
                                                                             void *state) {
#ifdef EI_HAS_SSD
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_f32_config_t *config = (ei_fill_result_object_detection_f32_config_t*)config_ptr;

    static std::vector<ei_impulse_result_bounding_box_t> results;
    int added_boxes_count = 0;
    results.clear();
    results.resize(config->object_detection_count);

    ei::matrix_t* data_mtx = NULL;
    ei::matrix_t* scores_mtx = NULL;
    ei::matrix_t* labels_mtx = NULL;

    find_mtx_by_idx(result->_raw_outputs, &data_mtx, input_block_id + 1, impulse->learning_blocks_size + 3);
    find_mtx_by_idx(result->_raw_outputs, &scores_mtx, input_block_id + 0, impulse->learning_blocks_size + 3);
    find_mtx_by_idx(result->_raw_outputs, &labels_mtx, input_block_id + 3, impulse->learning_blocks_size + 3);

    for (size_t ix = 0; ix < config->object_detection_count; ix++) {
        float score = scores_mtx->buffer[ix];
        float label = labels_mtx->buffer[ix];

        if (score >= config->threshold) {
            float ystart = data_mtx->buffer[(ix * 4) + 0];
            float xstart = data_mtx->buffer[(ix * 4) + 1];
            float yend = data_mtx->buffer[(ix * 4) + 2];
            float xend = data_mtx->buffer[(ix * 4) + 3];

            if (xstart < 0) xstart = 0;
            if (xstart > 1) xstart = 1;
            if (ystart < 0) ystart = 0;
            if (ystart > 1) ystart = 1;
            if (yend < 0) yend = 0;
            if (yend > 1) yend = 1;
            if (xend < 0) xend = 0;
            if (xend > 1) xend = 1;
            if (xend < xstart) xend = xstart;
            if (yend < ystart) yend = ystart;

            results[ix].label = impulse->categories[(uint32_t)label];
            results[ix].x = static_cast<uint32_t>(xstart * static_cast<float>(impulse->input_width));
            results[ix].y = static_cast<uint32_t>(ystart * static_cast<float>(impulse->input_height));
            results[ix].width = static_cast<uint32_t>((xend - xstart) * static_cast<float>(impulse->input_width));
            results[ix].height = static_cast<uint32_t>((yend - ystart) * static_cast<float>(impulse->input_height));
            results[ix].value = score;

            added_boxes_count++;
        } else {
            results[ix].value = 0.0f;
        }
    }

    result->bounding_boxes = results.data();
    result->bounding_boxes_count = added_boxes_count;

    return EI_IMPULSE_OK;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_yolov5_f32(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
#ifdef EI_HAS_YOLOV5
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_f32_config_t *config = (ei_fill_result_object_detection_f32_config_t*)config_ptr;

    ei::matrix_t* raw_output_mtx = NULL;
    bool status = find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);
    if (!status) {
        return EI_IMPULSE_POSTPROCESSING_ERROR;
    }

    static std::vector<ei_impulse_result_bounding_box_t> results;
    results.clear();

    size_t col_size = 5 + impulse->label_count;
    size_t row_count = config->output_features_count / col_size;

    for (size_t ix = 0; ix < row_count; ix++) {
        size_t base_ix = ix * col_size;
        float xc = raw_output_mtx->buffer[base_ix + 0];
        float yc = raw_output_mtx->buffer[base_ix + 1];
        float w = raw_output_mtx->buffer[base_ix + 2];
        float h = raw_output_mtx->buffer[base_ix + 3];
        float x = xc - (w / 2.0f);
        float y = yc - (h / 2.0f);
        if (x < 0) {
            x = 0;
        }
        if (y < 0) {
            y = 0;
        }
        if (x + w > impulse->input_width) {
            w = impulse->input_width - x;
        }
        if (y + h > impulse->input_height) {
            h = impulse->input_height - y;
        }

        if (w < 0 || h < 0) {
            continue;
        }

        float score = raw_output_mtx->buffer[base_ix + 4];

        uint32_t label = 0;
        float highest_value = 0.0f;
        for (size_t lx = 0; lx < impulse->label_count; lx++) {
            float l = raw_output_mtx->buffer[base_ix + 5 + lx];
            if (l > highest_value) {
                label = lx;
                highest_value = l;
            }
        }

        if (score >= config->threshold && score <= 1.0f) {
            ei_impulse_result_bounding_box_t r;
            r.label = impulse->categories[label];

            if (config->version != 5) {
                x *= static_cast<float>(impulse->input_width);
                y *= static_cast<float>(impulse->input_height);
                w *= static_cast<float>(impulse->input_width);
                h *= static_cast<float>(impulse->input_height);
            }

            r.x = static_cast<uint32_t>(x);
            r.y = static_cast<uint32_t>(y);
            r.width = static_cast<uint32_t>(w);
            r.height = static_cast<uint32_t>(h);
            r.value = score;
            results.push_back(r);
        }
    }

    EI_IMPULSE_ERROR nms_res = ei_run_nms(impulse, &config->nms_config, &results);
    if (nms_res != EI_IMPULSE_OK) {
        return nms_res;
    }

    // if we didn't detect min required objects, fill the rest with fixed value
    size_t added_boxes_count = results.size();
    size_t min_object_detection_count = config->object_detection_count;
    if (added_boxes_count < min_object_detection_count) {
        results.resize(min_object_detection_count);
        for (size_t ix = added_boxes_count; ix < min_object_detection_count; ix++) {
            results[ix].value = 0.0f;
        }
    }

    result->bounding_boxes = results.data();
    result->bounding_boxes_count = added_boxes_count;

    return EI_IMPULSE_OK;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_yolov5_i8(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
#ifdef EI_HAS_YOLOV5
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_i8_config_t *config = (ei_fill_result_object_detection_i8_config_t*)config_ptr;

    // yolov5 is the only exception that uses legacy unsigned quantized output
    ei::matrix_u8_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    static std::vector<ei_impulse_result_bounding_box_t> results;
    results.clear();

    size_t col_size = 5 + impulse->label_count;
    size_t row_count = config->output_features_count / col_size;

    for (size_t ix = 0; ix < row_count; ix++) {
        size_t base_ix = ix * col_size;
        float xc = (raw_output_mtx->buffer[base_ix + 0] - config->zero_point) * config->scale;
        float yc = (raw_output_mtx->buffer[base_ix + 1] - config->zero_point) * config->scale;
        float w = (raw_output_mtx->buffer[base_ix + 2] - config->zero_point) * config->scale;
        float h = (raw_output_mtx->buffer[base_ix + 3] - config->zero_point) * config->scale;
        float x = xc - (w / 2.0f);
        float y = yc - (h / 2.0f);
        if (x < 0) {
            x = 0;
        }
        if (y < 0) {
            y = 0;
        }
        if (x + w > impulse->input_width) {
            w = impulse->input_width - x;
        }
        if (y + h > impulse->input_height) {
            h = impulse->input_height - y;
        }

        if (w < 0 || h < 0) {
            continue;
        }

        float score = (raw_output_mtx->buffer[base_ix + 4] - config->zero_point) * config->scale;

        uint32_t label = 0;
        float highest_value = 0.0f;
        for (size_t lx = 0; lx < impulse->label_count; lx++) {
            float l = raw_output_mtx->buffer[base_ix + 5 + lx];
            if (l > highest_value) {
                label = lx;
                highest_value = l;
            }
        }

        if (score >= config->threshold && score <= 1.0f) {
            ei_impulse_result_bounding_box_t r;
            r.label = impulse->categories[label];

            if (config->version != 5) {
                x *= static_cast<float>(impulse->input_width);
                y *= static_cast<float>(impulse->input_height);
                w *= static_cast<float>(impulse->input_width);
                h *= static_cast<float>(impulse->input_height);
            }

            r.x = static_cast<uint32_t>(x);
            r.y = static_cast<uint32_t>(y);
            r.width = static_cast<uint32_t>(w);
            r.height = static_cast<uint32_t>(h);
            r.value = score;
            results.push_back(r);
        }
    }

    EI_IMPULSE_ERROR nms_res = ei_run_nms(impulse, &config->nms_config, &results);
    if (nms_res != EI_IMPULSE_OK) {
        return nms_res;
    }

    // if we didn't detect min required objects, fill the rest with fixed value
    size_t added_boxes_count = results.size();
    size_t min_object_detection_count = config->object_detection_count;
    if (added_boxes_count < min_object_detection_count) {
        results.resize(min_object_detection_count);
        for (size_t ix = added_boxes_count; ix < min_object_detection_count; ix++) {
            results[ix].value = 0.0f;
        }
    }

    result->bounding_boxes = results.data();
    result->bounding_boxes_count = added_boxes_count;

    return EI_IMPULSE_OK;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_yolox_f32(ei_impulse_handle_t *handle,
                                                                             uint32_t block_index,
                                                                             uint32_t input_block_id,
                                                                             ei_impulse_result_t *result,
                                                                             void *config_ptr,
                                                                             void *state) {
#ifdef EI_HAS_YOLOX
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_f32_config_t *config = (ei_fill_result_object_detection_f32_config_t*)config_ptr;

    ei::matrix_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    static std::vector<ei_impulse_result_bounding_box_t> results;
    results.clear();

    // START: def yolox_postprocess()

    // if not p6:
    //     strides = [8, 16, 32]
    // else:
    //     strides = [8, 16, 32, 64]
    const std::vector<int> strides { 8, 16, 32 };

    // hsizes = [img_size[0] // stride for stride in strides]
    // wsizes = [img_size[1] // stride for stride in strides]
    std::vector<int> hsizes(strides.size());
    std::vector<int> wsizes(strides.size());
    for (int ix = 0; ix < (int)strides.size(); ix++) {
        hsizes[ix] = (int)floor((float)impulse->input_width / (float)strides[ix]);
        wsizes[ix] = (int)floor((float)impulse->input_height / (float)strides[ix]);
    }

    // for hsize, wsize, stride in zip(hsizes, wsizes, strides):
    //      grid = np.stack((xv, yv), 2).reshape(1, -1, 2)
    //      grids.append(grid)
    //      shape = grid.shape[:2]
    //      expanded_strides.append(np.full((*shape, 1), stride))
    std::vector<ei::matrix_i32_t*> grids;
    std::vector<ei::matrix_i32_t*> expanded_strides;

    for (int ix = 0; ix < (int)strides.size(); ix++) {
        int hsize = hsizes.at(ix);
        int wsize = wsizes.at(ix);
        int stride = strides.at(ix);

        // xv, yv = np.meshgrid(np.arange(wsize), np.arange(hsize))
        // grid = np.stack((xv, yv), 2).reshape(1, -1, 2)
        ei::matrix_i32_t *grid = new ei::matrix_i32_t(hsize * wsize, 2);
        int grid_ix = 0;
        for (int h = 0; h < hsize; h++) {
            for (int w = 0; w < wsize; w++) {
                grid->buffer[grid_ix + 0] = w;
                grid->buffer[grid_ix + 1] = h;
                grid_ix += 2;
            }
        }
        grids.push_back(grid);

        // shape = grid.shape[:2]
        // expanded_strides.append(np.full((*shape, 1), stride))
        ei::matrix_i32_t *expanded_stride = new ei::matrix_i32_t(hsize * wsize, 1);
        for (int ix = 0; ix < hsize * wsize; ix++) {
            expanded_stride->buffer[ix] = stride;
        }
        expanded_strides.push_back(expanded_stride);
    }

    // grids = np.concatenate(grids, 1)
    int total_grid_rows = 0;
    for (auto g : grids) {
        total_grid_rows += g->rows;
    }
    ei::matrix_i32_t c_grid(total_grid_rows, 2);
    int c_grid_ix = 0;
    for (auto g : grids) {
        for (int row = 0; row < (int)g->rows; row++) {
            c_grid.buffer[c_grid_ix + 0] = g->buffer[(row * 2) + 0];
            c_grid.buffer[c_grid_ix + 1] = g->buffer[(row * 2) + 1];
            c_grid_ix += 2;
        }
        delete g;
    }

    // expanded_strides = np.concatenate(expanded_strides, 1)
    int total_stride_rows = 0;
    for (auto g : expanded_strides) {
        total_stride_rows += g->rows;
    }
    ei::matrix_i32_t c_expanded_strides(total_stride_rows, 1);
    int c_expanded_strides_ix = 0;
    for (auto g : expanded_strides) {
        for (int row = 0; row < (int)g->rows; row++) {
            c_expanded_strides.buffer[c_expanded_strides_ix + 0] = g->buffer[(row * 1) + 0];
            c_expanded_strides_ix += 1;
        }
        delete g;
    }

    const int output_rows = config->output_features_count / (5 + impulse->label_count);
    ei::matrix_t outputs(output_rows, 5 + impulse->label_count, raw_output_mtx->buffer);
    for (int row = 0; row < (int)outputs.rows; row++) {
        float v0 = outputs.buffer[(row * outputs.cols) + 0];
        float v1 = outputs.buffer[(row * outputs.cols) + 1];
        float v2 = outputs.buffer[(row * outputs.cols) + 2];
        float v3 = outputs.buffer[(row * outputs.cols) + 3];

        float cgrid0 = (float)c_grid.buffer[(row * c_grid.cols) + 0];
        float cgrid1 = (float)c_grid.buffer[(row * c_grid.cols) + 1];

        float stride = (float)c_expanded_strides.buffer[row];

        // outputs[..., :2] = (outputs[..., :2] + grids) * expanded_strides
        outputs.buffer[(row * outputs.cols) + 0] = (v0 + cgrid0) * stride;
        outputs.buffer[(row * outputs.cols) + 1] = (v1 + cgrid1) * stride;

        // outputs[..., 2:4] = np.exp(outputs[..., 2:4]) * expanded_strides
        outputs.buffer[(row * outputs.cols) + 2] = exp(v2) * stride;
        outputs.buffer[(row * outputs.cols) + 3] = exp(v3) * stride;
    }

    // END: def yolox_postprocess()

    // boxes = predictions[:, :4]
    ei::matrix_t boxes(outputs.rows, 4);
    for (int row = 0; row < (int)outputs.rows; row++) {
        boxes.buffer[(row * boxes.cols) + 0] = outputs.buffer[(row * outputs.cols) + 0];
        boxes.buffer[(row * boxes.cols) + 1] = outputs.buffer[(row * outputs.cols) + 1];
        boxes.buffer[(row * boxes.cols) + 2] = outputs.buffer[(row * outputs.cols) + 2];
        boxes.buffer[(row * boxes.cols) + 3] = outputs.buffer[(row * outputs.cols) + 3];
    }

    // scores = predictions[:, 4:5] * predictions[:, 5:]
    ei::matrix_t scores(outputs.rows, impulse->label_count);
    for (int row = 0; row < (int)outputs.rows; row++) {
        float confidence = outputs.buffer[(row * outputs.cols) + 4];
        for (int cc = 0; cc < impulse->label_count; cc++) {
            scores.buffer[(row * scores.cols) + cc] = confidence * outputs.buffer[(row * outputs.cols) + (5 + cc)];
        }
    }

    // iterate through scores to see if we have anything with confidence
    for (int row = 0; row < (int)scores.rows; row++) {
        for (int col = 0; col < (int)scores.cols; col++) {
            float confidence = scores.buffer[(row * scores.cols) + col];

            if (confidence >= config->threshold && confidence <= 1.0f) {
                ei_impulse_result_bounding_box_t r;
                r.label = impulse->categories[col];
                r.value = confidence;

                // now find the box...
                float xcenter = boxes.buffer[(row * boxes.cols) + 0];
                float ycenter = boxes.buffer[(row * boxes.cols) + 1];
                float width = boxes.buffer[(row * boxes.cols) + 2];
                float height = boxes.buffer[(row * boxes.cols) + 3];

                int x = (int)(xcenter - (width / 2.0f));
                int y = (int)(ycenter - (height / 2.0f));

                if (x < 0) {
                    x = 0;
                }
                if (x > (int)impulse->input_width) {
                    x = impulse->input_width;
                }
                if (y < 0) {
                    y = 0;
                }
                if (y > (int)impulse->input_height) {
                    y = impulse->input_height;
                }

                r.x = x;
                r.y = y;
                r.width = (int)round(width);
                r.height = (int)round(height);

                results.push_back(r);
            }
        }
    }

    EI_IMPULSE_ERROR nms_res = ei_run_nms(impulse, &config->nms_config, &results);
    if (nms_res != EI_IMPULSE_OK) {
        return nms_res;
    }

    // if we didn't detect min required objects, fill the rest with fixed value
    size_t added_boxes_count = results.size();
    size_t min_object_detection_count = config->object_detection_count;
    if (added_boxes_count < min_object_detection_count) {
        results.resize(min_object_detection_count);
        for (size_t ix = added_boxes_count; ix < min_object_detection_count; ix++) {
            results[ix].value = 0.0f;
        }
    }

    result->bounding_boxes = results.data();
    result->bounding_boxes_count = added_boxes_count;

    return EI_IMPULSE_OK;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // EI_HAS_YOLOX
}

/**
 * Clarification for difference between process_yolox and process_yolox_detect.
 * process_yolox_detect is for YOLOX models with detect layer
*/

__attribute__((unused)) static EI_IMPULSE_ERROR process_yolox_detect_f32(ei_impulse_handle_t *handle,
                                                                             uint32_t block_index,
                                                                             uint32_t input_block_id,
                                                                             ei_impulse_result_t *result,
                                                                             void *config_ptr,
                                                                             void *state) {
#ifdef EI_HAS_YOLOX
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_f32_config_t *config = (ei_fill_result_object_detection_f32_config_t*)config_ptr;

    ei::matrix_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    static std::vector<ei_impulse_result_bounding_box_t> results;
    results.clear();

    // expected format [xmin ymin xmax ymax score label]
    const int output_rows = config->output_features_count / 6;
    ei::matrix_t outputs(output_rows, 6, raw_output_mtx->buffer);

    // iterate through scores to see if we have anything with confidence
    for (int row = 0; row < (int)outputs.rows; row++) {
        float confidence = outputs.buffer[(row * outputs.cols) + 4];
        int class_idx = (int)outputs.buffer[(row * outputs.cols) + 5];

        if (confidence >= config->threshold && confidence <= 1.0f) {
            ei_impulse_result_bounding_box_t r;
            r.label = impulse->categories[class_idx];
            r.value = confidence;

            // now find the box...
            float xmin = outputs.buffer[(row * outputs.cols) + 0];
            float ymin = outputs.buffer[(row * outputs.cols) + 1];
            float xmax = outputs.buffer[(row * outputs.cols) + 2];
            float ymax = outputs.buffer[(row * outputs.cols) + 3];

            float width  = xmax - xmin;
            float height = ymax - ymin;

            int x = (int)xmin;
            int y = (int)ymin;

            if (x < 0) {
                x = 0;
            }
            if (x > (int)impulse->input_width) {
                x = impulse->input_width;
            }
            if (y < 0) {
                y = 0;
            }
            if (y > (int)impulse->input_height) {
                y = impulse->input_height;
            }

            r.x = x;
            r.y = y;
            r.width = (int)round(width);
            r.height = (int)round(height);

            results.push_back(r);
        }
    }

    result->bounding_boxes = results.data();
    result->bounding_boxes_count = results.size();

    return EI_IMPULSE_OK;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // EI_HAS_YOLOX
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_yolov7_f32(ei_impulse_handle_t *handle,
                                                                             uint32_t block_index,
                                                                             uint32_t input_block_id,
                                                                             ei_impulse_result_t *result,
                                                                             void *config_ptr,
                                                                             void *state) {
#ifdef EI_HAS_YOLOV7
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_f32_config_t *config = (ei_fill_result_object_detection_f32_config_t*)config_ptr;

    ei::matrix_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    static std::vector<ei_impulse_result_bounding_box_t> results;
    results.clear();

    size_t col_size = 7;
    // yolov7 is quite special case here, since it has variable output size
    size_t row_count = raw_output_mtx->cols / col_size;

    // output is:
    // batch_id, xmin, ymin, xmax, ymax, cls_id, score
    for (size_t ix = 0; ix < row_count; ix++) {
        size_t base_ix = ix * col_size;
        float xmin = raw_output_mtx->buffer[base_ix + 1];
        float ymin = raw_output_mtx->buffer[base_ix + 2];
        float xmax = raw_output_mtx->buffer[base_ix + 3];
        float ymax = raw_output_mtx->buffer[base_ix + 4];
        uint32_t label = (uint32_t)raw_output_mtx->buffer[base_ix + 5];
        float score = raw_output_mtx->buffer[base_ix + 6];

        if (score >= config->threshold && score <= 1.0f) {
            ei_impulse_result_bounding_box_t r;
            r.label = impulse->categories[label];

            r.x = static_cast<uint32_t>(xmin);
            r.y = static_cast<uint32_t>(ymin);
            r.width = static_cast<uint32_t>(xmax - xmin);
            r.height = static_cast<uint32_t>(ymax - ymin);
            r.value = score;
            results.push_back(r);
        }
    }

    // if we didn't detect min required objects, fill the rest with fixed value
    size_t added_boxes_count = results.size();
    size_t min_object_detection_count = config->object_detection_count;
    if (added_boxes_count < min_object_detection_count) {
        results.resize(min_object_detection_count);
        for (size_t ix = added_boxes_count; ix < min_object_detection_count; ix++) {
            results[ix].value = 0.0f;
        }
    }

    result->bounding_boxes = results.data();
    result->bounding_boxes_count = added_boxes_count;

    return EI_IMPULSE_OK;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_YOLOV7
}

__attribute__((unused)) inline float sigmoid(float a) {
    return 1.0f / (1.0f + exp(-a));
}

#ifdef EI_HAS_YOLOV2
// based on akida_models-1.2.0/detection/processing.py
// input is "2D" array with shape [grid_h * grid_w * nb_box, nb_classes]
__attribute__((unused)) static void softmax(std::vector<float>& input, const size_t nb_classes)
{
    const float max = *std::max_element(input.begin(), input.end());
    const float min = *std::min_element(input.begin(), input.end());
    const float t = -100.0f;

    // x = x - np.max(x)
    std::transform(input.begin(), input.end(), input.begin(),
                   [max](float x) { return x - max; });

    // if np.min(x) < t: x = x / np.min(x) * t
    std::transform(input.begin(), input.end(), input.begin(),
                   [min, t](float x) { return x < t ? (x / min * t): x; });

    // e_x = np.exp(x)
    // do it in place as we don't need raw the input anymore
    std::transform(input.begin(), input.end(), input.begin(),
                   [](float x) { return std::exp(x); });

    // e_x / e_x.sum(axis, keepdims=True)
    // calculated for each 'row', across nb_classes
    for(auto it = input.begin(); it != input.end(); it += nb_classes) {
        float sum = 0.0f;
        // e_x.sum(axis, keepdims=True)
        for(auto it2 = it; it2 != it + nb_classes; it2++) {
            sum += *it2;
        }
        // e_x / e_x.sum(axis, keepdims=True)
        std::transform(it, it + nb_classes, it,
                       [sum](float ex) { return ex / sum; });
    }
}

class BoundingBox {
public:
    float x1, y1, x2, y2, confidence;
    std::vector<float> classes;

    BoundingBox(float x1, float y1, float x2, float y2, float confidence, const std::vector<float>& classes)
        : x1(x1), y1(y1), x2(x2), y2(y2), confidence(confidence), classes(classes) {}

    float get_score() const {
        return confidence;
    }

    int get_label() const {
        auto maxElementIndex = std::max_element(classes.begin(), classes.end()) - classes.begin();
        return maxElementIndex;
    }

    float _interval_overlap(float x1, float x2, float x3, float x4) const {
        if(x3 < x1) {
            if(x4 < x1) {
                return 0;
            }
            return std::min(x2, x4) - x1;
        }
        if(x2 < x3) {
            return 0;
        }
        return std::min(x2, x4) - x3;
    }


    float iou(const BoundingBox& other) const {
        // Implementation of the Intersection over Union calculation
        float intersect_w = this->_interval_overlap(this->x1, this->x2, other.x1, other.x2);
        float intersect_h = this->_interval_overlap(this->y1, this->y2, other.y1, other.y2);

        float intersect = intersect_w * intersect_h;

        float w1 = this->x2 - this->x1;
        float h1 = this->y2 - this->y1;
        float w2 = other.x2 - other.x1;
        float h2 = other.y2 - other.y1;

        float un = w1 * h1 + w2 * h2 - intersect;

        return float(intersect) / un;
    }
};
#endif // EI_HAS_YOLOV2

__attribute__((unused)) static EI_IMPULSE_ERROR process_yolov2_f32(ei_impulse_handle_t *handle,
                                                                             uint32_t block_index,
                                                                             uint32_t input_block_id,
                                                                             ei_impulse_result_t *result,
                                                                             void *config_ptr,
                                                                             void *state) {
#ifdef EI_HAS_YOLOV2
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_f32_config_t *config = (ei_fill_result_object_detection_f32_config_t*)config_ptr;

    ei::matrix_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    static std::vector<ei_impulse_result_bounding_box_t> results;
    results.clear();

    // Example output shape: (7, 7, 5, 7)
    // TODO: calculate grid_h, grid_w, nb_box from output_features_count or get as a param
    // grid_h, grid_w, nb_box = output.shape[:3]
    const size_t grid_h = 7;
    const size_t grid_w = 7;
    const size_t nb_box = 5;
    const std::vector<std::pair<float, float>> anchors = {{0.56594, 1.05012}, {1.0897, 2.03908}, {2.37823, 3.00376}, {2.4593, 4.913}, {5.15981, 5.56699}};

    const size_t nb_classes = impulse->label_count;
    const float obj_threshold = 0.5;
    const float nms_threshold = 0.5;
    std::vector<float> output;
    const int stride = 4 + 1 + nb_classes;

    output.assign(raw_output_mtx->buffer, raw_output_mtx->buffer + config->output_features_count);

    // boxes = []
    std::vector<BoundingBox> boxes;

    // equivalent to: classes_confidences = output[..., 5:]
    std::vector<float> classes_confidences;
    const size_t dim = 5;
    for(auto it = output.begin() + dim; it <= output.end(); it += (dim + nb_classes)) {
        classes_confidences.insert(classes_confidences.end(), it, it + nb_classes);
    }
    // calculate softmax for later use, we need to calculate it across the whole input data so operate on a sliced output
    softmax(classes_confidences, nb_classes);

    for (size_t row = 0; row < grid_h; ++row) {
        for (size_t col = 0; col < grid_w; ++col) {
            for (size_t b = 0; b < nb_box; ++b) {
                size_t idx = row * grid_w * nb_box * stride + col * nb_box * stride + b * stride;
                size_t classes_idx = row * grid_w * nb_box * nb_classes + col * nb_box * nb_classes + b * nb_classes;

                // Apply sigmoid to the 4th element
                // output[..., 4] = _sigmoid(output[..., 4])
                float sigmoid_val = sigmoid(output[idx + 4]);
                output[idx + 4] = sigmoid_val;

                // classes = output[row, col, b, 5:]
                std::vector<float> classes(classes_confidences.begin() + classes_idx, classes_confidences.begin() + classes_idx + nb_classes);

                // output[..., 5:] = output[..., 4][..., np.newaxis] * _softmax(output[..., 5:])
                // output[..., 5:] *= output[..., 5:] > obj_threshold
                std::transform(classes.begin(), classes.end(), classes.begin(),
                               [sigmoid_val, obj_threshold](float c) { c *= sigmoid_val; return c > obj_threshold ? c : 0.0f; });

                // if np.sum(classes) > 0:
                float sum = 0.0f;
                for(auto it = classes.begin(); it != classes.end(); it++) {
                    sum += *it;
                }
                if(sum > 0.0f) {
                    // x, y, w, h = output[row, col, b, :4]
                    float x = output[idx + 0];
                    float y = output[idx + 1];
                    float w = output[idx + 2];
                    float h = output[idx + 3];

                    // x = (col + _sigmoid(x)) / grid_w  # center position, unit: image width
                    x = (col + sigmoid(x)) / grid_w;
                    // y = (row + _sigmoid(y)) / grid_h  # center position, unit: image height
                    y = (row + sigmoid(y)) / grid_h;
                    // w = anchors[b][0] * np.exp(w) / grid_w  # unit: image width
                    w = anchors[b].first * std::exp(w) / grid_w;
                    // h = anchors[b][1] * np.exp(h) / grid_h  # unit: image height
                    h = anchors[b].second * std::exp(h) / grid_h;

                    // confidence = output[row, col, b, 4]
                    float confidence = output[idx + 4];

                    // x1 = max(x - w / 2, 0)
                    float x1 = std::max(x - w / 2, 0.0f);
                    // y1 = max(y - h / 2, 0)
                    float y1 = std::max(y - h / 2, 0.0f);
                    // x2 = min(x + w / 2, grid_w)
                    float x2 = std::min(x + w / 2, static_cast<float>(grid_w));
                    // y2 = min(y + h / 2, grid_h)
                    float y2 = std::min(y + h / 2, static_cast<float>(grid_h));

                    boxes.emplace_back(x1, y1, x2, y2, confidence, classes);
                }
            }
        }
    }

    // Non-maximal suppression (on boxes)
    for (size_t c = 0; c < nb_classes; ++c) {
        std::vector<std::pair<float, int>> sorted_indices;
        for (size_t i = 0; i < boxes.size(); ++i) {
            sorted_indices.emplace_back(boxes[i].classes[c], i);
        }

        std::sort(sorted_indices.begin(), sorted_indices.end(),
                  [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                      return a.first > b.first;
                  });

        for (size_t i = 0; i < sorted_indices.size(); ++i) {
            int index_i = sorted_indices[i].second;
            if (boxes[index_i].classes[c] == 0)
                continue;

            for (size_t j = i + 1; j < sorted_indices.size(); ++j) {
                int index_j = sorted_indices[j].second;

                if ((boxes[index_i].iou(boxes[index_j]) >= nms_threshold) &&
                    (boxes[index_i].get_label() == (int)c) &&
                    (boxes[index_j].get_label() == (int)c)) {
                    boxes[index_j].confidence = 0;
                }
            }
        }
    }

    // remove the boxes which are less likely than a obj_threshold
    boxes.erase(std::remove_if(boxes.begin(), boxes.end(),
                               [obj_threshold](const BoundingBox& box) {
                                   return box.get_score() <= obj_threshold;
                               }), boxes.end());

    // sort boxes by box.get_score()
    std::sort(boxes.begin(), boxes.end(),
                [](const BoundingBox& a, const BoundingBox& b) {
                return a.get_score() > b.get_score();
                });

    // convert relative coordinates to absolute coordinates
    for(auto & box: boxes) {
        ei_impulse_result_bounding_box_t res;
        res.label = impulse->categories[box.get_label()];
        res.x = ceil(box.x1 * impulse->input_width);
        res.y = ceil(box.y1 * impulse->input_height);
        res.width = ceil((box.x2 - box.x1) * impulse->input_width);
        res.height = ceil((box.y2 - box.y1) * impulse->input_height);
        res.value = box.get_score();
        results.push_back(res);
    }

    // if we didn't detect min required objects, fill the rest with fixed value
    size_t added_boxes_count = results.size();
    size_t min_object_detection_count = config->object_detection_count;
    if (added_boxes_count < min_object_detection_count) {
        results.resize(min_object_detection_count);
        for (size_t ix = added_boxes_count; ix < min_object_detection_count; ix++) {
            results[ix].value = 0.0f;
        }
    }

    result->bounding_boxes = results.data();
    result->bounding_boxes_count = added_boxes_count;

    return EI_IMPULSE_OK;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_YOLOV2
}

#if (EI_HAS_TAO_DECODE_DETECTIONS == 1) || (EI_HAS_TAO_YOLO == 1) || (EI_HAS_YOLO_PRO == 1) || (EI_HAS_YOLOV11 == 1)

__attribute__((unused)) static void prepare_nms_results_common(size_t object_detection_count,
                                                               ei_impulse_result_t *result,
                                                               std::vector<ei_impulse_result_bounding_box_t> *results) {
    #define EI_CLASSIFIER_OBJECT_DETECTION_KEEP_TOPK 200

    // if we didn't detect min required objects, fill the rest with fixed value
    size_t added_boxes_count = results->size();
    if (added_boxes_count < object_detection_count) {
        results->resize(object_detection_count);
        for (size_t ix = added_boxes_count; ix < object_detection_count; ix++) {
            (*results)[ix].value = 0.0f;
        }
    }

    // we sort in reverse order across all classes,
    // since results for each class are pushed to the end.
    std::sort(results->begin(), results->end(), [ ]( const ei_impulse_result_bounding_box_t& lhs, const ei_impulse_result_bounding_box_t& rhs )
    {
        return lhs.value > rhs.value;
    });

    // keep topK
    if (results->size() > EI_CLASSIFIER_OBJECT_DETECTION_KEEP_TOPK) {
        results->erase(results->begin() + EI_CLASSIFIER_OBJECT_DETECTION_KEEP_TOPK, results->end());
    }

    result->bounding_boxes = results->data();
    result->bounding_boxes_count = added_boxes_count;
}

#endif

#ifdef EI_HAS_TAO_DECODE_DETECTIONS
template<typename T>
__attribute__((unused)) static EI_IMPULSE_ERROR process_tao_decode_detections_common(const ei_impulse_t *impulse,
                                                                                     ei_impulse_result_t *result,
                                                                                     T *data,
                                                                                     float zero_point,
                                                                                     float scale,
                                                                                     size_t output_features_count,
                                                                                     float threshold,
                                                                                     size_t object_detection_count,
                                                                                     ei_object_detection_nms_config_t nms_config) {

    size_t col_size = 12 + impulse->label_count + 1;
    size_t row_count = output_features_count / col_size;

    static std::vector<ei_impulse_result_bounding_box_t> results;
    static std::vector<ei_impulse_result_bounding_box_t> class_results;
    results.clear();

    for (size_t cls_idx = 1; cls_idx < (size_t)(impulse->label_count + 1); cls_idx++)  {

        std::vector<float> boxes;
        std::vector<float> scores;
        std::vector<int> classes;
        class_results.clear();

        for (size_t ix = 0; ix < row_count; ix++) {

            float score = (static_cast<float>(data[ix * col_size + cls_idx]) - zero_point) * scale;

            if ((score < threshold) || (score > 1.0f)) {
                continue;
            }

            // # 1. calculate boxes location
            size_t base_ix = ix * col_size + col_size; // references the end of the row

            float r_12 = (static_cast<float>(data[base_ix - 12]) - zero_point) * scale;
            float r_11 = (static_cast<float>(data[base_ix - 11]) - zero_point) * scale;
            float r_10 = (static_cast<float>(data[base_ix - 10]) - zero_point) * scale;
            float r_9  = (static_cast<float>(data[base_ix -  9]) - zero_point) * scale;
            float r_8  = (static_cast<float>(data[base_ix -  8]) - zero_point) * scale;
            float r_7  = (static_cast<float>(data[base_ix -  7]) - zero_point) * scale;
            float r_6  = (static_cast<float>(data[base_ix -  6]) - zero_point) * scale;
            float r_5  = (static_cast<float>(data[base_ix -  5]) - zero_point) * scale;
            float r_4  = (static_cast<float>(data[base_ix -  4]) - zero_point) * scale;
            float r_3  = (static_cast<float>(data[base_ix -  3]) - zero_point) * scale;
            float r_2  = (static_cast<float>(data[base_ix -  2]) - zero_point) * scale;
            float r_1  = (static_cast<float>(data[base_ix -  1]) - zero_point) * scale;

            // cx_pred = y_pred[..., -12]
            // cy_pred = y_pred[..., -11]
            // w_pred = y_pred[..., -10]
            // h_pred = y_pred[..., -9]
            float cx_pred = r_12;
            float cy_pred = r_11;
            float w_pred  = r_10;
            float h_pred  = r_9;

            // w_anchor = y_pred[..., -6] - y_pred[..., -8]
            // h_anchor = y_pred[..., -5] - y_pred[..., -7]
            float w_anchor = r_6 - r_8;
            float h_anchor = r_5 - r_7;

            // cx_anchor = tf.truediv(y_pred[..., -6] + y_pred[..., -8], 2.0)
            // cy_anchor = tf.truediv(y_pred[..., -5] + y_pred[..., -7], 2.0)
            float cx_anchor = (r_6 + r_8) / 2.0f;
            float cy_anchor = (r_5 + r_7) / 2.0f;

            // cx_variance = y_pred[..., -4]
            // cy_variance = y_pred[..., -3]
            float cx_variance = r_4;
            float cy_variance = r_3;

            // variance_w = y_pred[..., -2]
            // variance_h = y_pred[..., -1]
            float variance_w = r_2;
            float variance_h = r_1;

            // # Convert anchor box offsets to image offsets.
            // cx = cx_pred * cx_variance * w_anchor + cx_anchor
            // cy = cy_pred * cy_variance * h_anchor + cy_anchor
            // w = tf.exp(w_pred * variance_w) * w_anchor
            // h = tf.exp(h_pred * variance_h) * h_anchor
            float cx = cx_pred * cx_variance * w_anchor + cx_anchor;
            float cy = cy_pred * cy_variance * h_anchor + cy_anchor;
            float w = exp(w_pred * variance_w) * w_anchor;
            float h = exp(h_pred * variance_h) * h_anchor;

            // # Convert 'centroids' to 'corners'.
            float xmin = cx - (w / 2.0f);
            float ymin = cy - (h / 2.0f);
            float xmax = cx + (w / 2.0f);
            float ymax = cy + (h / 2.0f);

            xmin *= impulse->input_width;
            ymin *= impulse->input_height;
            xmax *= impulse->input_width;
            ymax *= impulse->input_height;

            boxes.push_back(ymin);
            boxes.push_back(xmin);
            boxes.push_back(ymax);
            boxes.push_back(xmax);
            scores.push_back(score);
            classes.push_back((int)(cls_idx-1));
        }

        size_t nr_boxes = scores.size();
        EI_IMPULSE_ERROR nms_res = ei_run_nms(impulse,
                                              &class_results,
                                              boxes.data(),
                                              scores.data(),
                                              classes.data(),
                                              nr_boxes,
                                              true /*clip_boxes*/,
                                              &nms_config);

        if (nms_res != EI_IMPULSE_OK) {
            return nms_res;
        }

        for (auto bb: class_results) {
            results.push_back(bb);
        }
    }

    prepare_nms_results_common(object_detection_count, result, &results);

    return EI_IMPULSE_OK;
}
#endif // #ifdef EI_HAS_TAO_DETECT_DETECTIONS

#ifdef EI_HAS_TAO_YOLOV3
template<typename T>
__attribute__((unused)) static EI_IMPULSE_ERROR  process_tao_yolov3_common(const ei_impulse_t *impulse,
                                                                                     ei_impulse_result_t *result,
                                                                                     T *data,
                                                                                     float zero_point,
                                                                                     float scale,
                                                                                     size_t output_features_count,
                                                                                     float threshold,
                                                                                     size_t object_detection_count,
                                                                                     ei_object_detection_nms_config_t nms_config) {
    // # x: 3-D tensor. Last dimension is
    //          (cy, cx, ph, pw, step_y, step_x, pred_y, pred_x, pred_h, pred_w, object, cls...)
    size_t col_size = 11 + impulse->label_count;
    size_t row_count = output_features_count / col_size;

    static std::vector<ei_impulse_result_bounding_box_t> results;
    static std::vector<ei_impulse_result_bounding_box_t> class_results;

    results.clear();
    for (size_t cls_idx = 0; cls_idx < (size_t)impulse->label_count; cls_idx++)  {

        std::vector<float> boxes;
        std::vector<float> scores;
        std::vector<int> classes;
        class_results.clear();

        for (size_t ix = 0; ix < row_count; ix++) {
            size_t data_ix = ix * col_size;
            float r_0  = (static_cast<float>(data[data_ix +  0]) - zero_point) * scale;
            float r_1  = (static_cast<float>(data[data_ix +  1]) - zero_point) * scale;
            float r_2  = (static_cast<float>(data[data_ix +  2]) - zero_point) * scale;
            float r_3  = (static_cast<float>(data[data_ix +  3]) - zero_point) * scale;
            float r_4  = (static_cast<float>(data[data_ix +  4]) - zero_point) * scale;
            float r_5  = (static_cast<float>(data[data_ix +  5]) - zero_point) * scale;
            float r_6  = (static_cast<float>(data[data_ix +  6]) - zero_point) * scale;
            float r_7  = (static_cast<float>(data[data_ix +  7]) - zero_point) * scale;
            float r_8  = (static_cast<float>(data[data_ix +  8]) - zero_point) * scale;
            float r_9  = (static_cast<float>(data[data_ix +  9]) - zero_point) * scale;
            float r_10 = (static_cast<float>(data[data_ix + 10]) - zero_point) * scale;

            float cls = (static_cast<float>(data[data_ix + 11 + cls_idx]) - zero_point) * scale;
            float score = sigmoid(cls) * sigmoid(r_10);

            if ((score < threshold) || (score > 1.0f)) {
                continue;
            }

            float by = r_0 + sigmoid(r_6) * r_4;
            float bx = r_1 + sigmoid(r_7) * r_5;
            float bh = r_2 * exp(r_8);
            float bw = r_3 * exp(r_9);

            float ymin = by - 0.5 * bh;
            float xmin = bx - 0.5 * bw;
            float ymax = by + 0.5 * bh;
            float xmax = bx + 0.5 * bw;

            // from relative to absolute
            ymin *= impulse->input_height;
            xmin *= impulse->input_width;
            ymax *= impulse->input_height;
            xmax *= impulse->input_width;

            boxes.push_back(ymin);
            boxes.push_back(xmin);
            boxes.push_back(ymax);
            boxes.push_back(xmax);
            scores.push_back(score);
            classes.push_back((int)cls_idx);
        }

        size_t nr_boxes = scores.size();
        EI_IMPULSE_ERROR nms_res = ei_run_nms(impulse,
                                              &class_results,
                                              boxes.data(),
                                              scores.data(),
                                              classes.data(),
                                              nr_boxes,
                                              true /*clip_boxes*/,
                                              &nms_config);
        if (nms_res != EI_IMPULSE_OK) {
            return nms_res;
        }

        for (auto bb: class_results) {
            results.push_back(bb);
        }
    }

    prepare_nms_results_common(object_detection_count, result, &results);
    return EI_IMPULSE_OK;
}
#endif // #ifdef EI_HAS_TAO_YOLOV3

#ifdef EI_HAS_TAO_YOLOV4
template<typename T>
__attribute__((unused)) static EI_IMPULSE_ERROR process_tao_yolov4_common(const ei_impulse_t *impulse,
                                                                          ei_impulse_result_t *result,
                                                                          T *data,
                                                                          float zero_point,
                                                                          float scale,
                                                                          size_t output_features_count,
                                                                          float threshold,
                                                                          size_t object_detection_count,
                                                                          ei_object_detection_nms_config_t nms_config) {
    // # x: 3-D tensor. Last dimension is
    //          (cy, cx, ph, pw, step_y, step_x, pred_y, pred_x, pred_h, pred_w, object, cls...)
    size_t col_size = 11 + impulse->label_count;
    size_t row_count = output_features_count / col_size;

    static std::vector<ei_impulse_result_bounding_box_t> results;
    static std::vector<ei_impulse_result_bounding_box_t> class_results;
    results.clear();

    const float grid_scale_xy = 1.0f;

    for (size_t cls_idx = 0; cls_idx < (size_t)impulse->label_count; cls_idx++)  {

        std::vector<float> boxes;
        std::vector<float> scores;
        std::vector<int> classes;
        class_results.clear();

        for (size_t ix = 0; ix < row_count; ix++) {

            float r_0  = (static_cast<float>(data[ix * col_size +  0]) - zero_point) * scale;
            float r_1  = (static_cast<float>(data[ix * col_size +  1]) - zero_point) * scale;
            float r_2  = (static_cast<float>(data[ix * col_size +  2]) - zero_point) * scale;
            float r_3  = (static_cast<float>(data[ix * col_size +  3]) - zero_point) * scale;
            float r_4  = (static_cast<float>(data[ix * col_size +  4]) - zero_point) * scale;
            float r_5  = (static_cast<float>(data[ix * col_size +  5]) - zero_point) * scale;
            float r_6  = (static_cast<float>(data[ix * col_size +  6]) - zero_point) * scale;
            float r_7  = (static_cast<float>(data[ix * col_size +  7]) - zero_point) * scale;
            float r_8  = (static_cast<float>(data[ix * col_size +  8]) - zero_point) * scale;
            float r_9  = (static_cast<float>(data[ix * col_size +  9]) - zero_point) * scale;
            float r_10 = (static_cast<float>(data[ix * col_size + 10]) - zero_point) * scale;

            float cls = (static_cast<float>(data[ix * col_size + 11 + cls_idx]) - zero_point) * scale;
            float score = sigmoid(cls) * sigmoid(r_10);

            if ((score < threshold) || (score > 1.0f)) {
                continue;
            }

            float pred_y = sigmoid(r_6) * grid_scale_xy - (grid_scale_xy - 1.0f) / 2.0f;
            float pred_x = sigmoid(r_7) * grid_scale_xy - (grid_scale_xy - 1.0f) / 2.0f;
            float pred_h = exp(std::min(r_8, 8.0f));
            float pred_w = exp(std::min(r_9, 8.0f));

            r_6 = pred_y;
            r_7 = pred_x;
            r_8 = pred_h;
            r_9 = pred_w;

            float by = r_0 + r_6 * r_4;
            float bx = r_1 + r_7 * r_5;
            float bh = r_2 * r_8;
            float bw = r_3 * r_9;

            float ymin = by - 0.5 * bh;
            float xmin = bx - 0.5 * bw;
            float ymax = by + 0.5 * bh;
            float xmax = bx + 0.5 * bw;

            // from relative to absolute
            ymin *= impulse->input_height;
            xmin *= impulse->input_width;
            ymax *= impulse->input_height;
            xmax *= impulse->input_width;

            boxes.push_back(ymin);
            boxes.push_back(xmin);
            boxes.push_back(ymax);
            boxes.push_back(xmax);
            scores.push_back(score);
            classes.push_back((int)cls_idx);
        }

        size_t nr_boxes = scores.size();
        EI_IMPULSE_ERROR nms_res = ei_run_nms(impulse,
                                              &class_results,
                                              boxes.data(),
                                              scores.data(),
                                              classes.data(),
                                              nr_boxes,
                                              true /*clip_boxes*/,
                                              &nms_config);
        if (nms_res != EI_IMPULSE_OK) {
            return nms_res;
        }

        for (auto bb: class_results) {
            results.push_back(bb);
        }
    }

    prepare_nms_results_common(object_detection_count, result, &results);
    return EI_IMPULSE_OK;
}
#endif // #ifdef EI_HAS_TAO_YOLOV4

__attribute__((unused)) static EI_IMPULSE_ERROR process_tao_detection_i8(ei_impulse_handle_t *handle,
                                                                         uint32_t block_index,
                                                                         uint32_t input_block_id,
                                                                         ei_impulse_result_t *result,
                                                                         void *config_ptr,
                                                                         void *state) {
#ifdef EI_HAS_TAO_DECODE_DETECTIONS
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_i8_config_t *config = (ei_fill_result_object_detection_i8_config_t*)config_ptr;

    ei::matrix_i8_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    EI_IMPULSE_ERROR res = process_tao_decode_detections_common(impulse,
                                                               result,
                                                               raw_output_mtx->buffer,
                                                               config->zero_point,
                                                               config->scale,
                                                               config->output_features_count,
                                                               config->threshold,
                                                               config->object_detection_count,
                                                               config->nms_config);

    return res;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_TAO_DETECT_DETECTIONS
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_tao_detection_f32(ei_impulse_handle_t *handle,
                                                                             uint32_t block_index,
                                                                             uint32_t input_block_id,
                                                                             ei_impulse_result_t *result,
                                                                             void *config_ptr,
                                                                             void *state) {
#ifdef EI_HAS_TAO_DECODE_DETECTIONS
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_f32_config_t *config = (ei_fill_result_object_detection_f32_config_t*)config_ptr;

    ei::matrix_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    EI_IMPULSE_ERROR res = process_tao_decode_detections_common(impulse,
                                                               result,
                                                               raw_output_mtx->buffer,
                                                               0.0f,
                                                               1.0f,
                                                               config->output_features_count,
                                                               config->threshold,
                                                               config->object_detection_count,
                                                               config->nms_config);
    return res;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_TAO_DETECT_DETECTIONS
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_tao_yolov3_f32(ei_impulse_handle_t *handle,
                                                                                uint32_t block_index,
                                                                                uint32_t input_block_id,
                                                                                ei_impulse_result_t *result,
                                                                                void *config_ptr,
                                                                                void *state) {
#ifdef EI_HAS_TAO_YOLOV3
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_f32_config_t *config = (ei_fill_result_object_detection_f32_config_t*)config_ptr;

    ei::matrix_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    EI_IMPULSE_ERROR res = process_tao_yolov3_common(impulse,
                                                     result,
                                                     raw_output_mtx->buffer,
                                                     0.0f,
                                                     1.0f,
                                                     config->output_features_count,
                                                     config->threshold,
                                                     config->object_detection_count,
                                                     config->nms_config);
    return res;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_TAO_YOLOV3
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_tao_yolov3_i8(ei_impulse_handle_t *handle,
                                                                                uint32_t block_index,
                                                                                uint32_t input_block_id,
                                                                                ei_impulse_result_t *result,
                                                                                void *config_ptr,
                                                                                void *state) {
#ifdef EI_HAS_TAO_YOLOV3
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_i8_config_t *config = (ei_fill_result_object_detection_i8_config_t*)config_ptr;

    ei::matrix_i8_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    EI_IMPULSE_ERROR res = process_tao_yolov3_common(impulse,
                                                     result,
                                                     raw_output_mtx->buffer,
                                                     config->zero_point,
                                                     config->scale,
                                                     config->output_features_count,
                                                     config->threshold,
                                                     config->object_detection_count,
                                                     config->nms_config);
    return res;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_TAO_YOLOV3
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_tao_yolov4_f32(ei_impulse_handle_t *handle,
                                                                                uint32_t block_index,
                                                                                uint32_t input_block_id,
                                                                                ei_impulse_result_t *result,
                                                                                void *config_ptr,
                                                                                void *state) {
#ifdef EI_HAS_TAO_YOLOV4
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_f32_config_t *config = (ei_fill_result_object_detection_f32_config_t*)config_ptr;

    ei::matrix_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    EI_IMPULSE_ERROR res = process_tao_yolov4_common(impulse,
                                                     result,
                                                     raw_output_mtx->buffer,
                                                     0.0f,
                                                     1.0f,
                                                     config->output_features_count,
                                                     config->threshold,
                                                     config->object_detection_count,
                                                     config->nms_config);
    return res;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_TAO_YOLOV4
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_tao_yolov4_i8(ei_impulse_handle_t *handle,
                                                                                uint32_t block_index,
                                                                                uint32_t input_block_id,
                                                                                ei_impulse_result_t *result,
                                                                                void *config_ptr,
                                                                                void *state) {
#ifdef EI_HAS_TAO_YOLOV4
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_i8_config_t *config = (ei_fill_result_object_detection_i8_config_t*)config_ptr;

    ei::matrix_i8_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    EI_IMPULSE_ERROR res = process_tao_yolov4_common(impulse,
                                                     result,
                                                     raw_output_mtx->buffer,
                                                     config->zero_point,
                                                     config->scale,
                                                     config->output_features_count,
                                                     config->threshold,
                                                     config->object_detection_count,
                                                     config->nms_config);
    return res;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_TAO_YOLOV4
}

#ifdef EI_HAS_YOLO_PRO
template<typename T>
__attribute__((unused)) static EI_IMPULSE_ERROR fill_result_struct_yolo_pro_common(const ei_impulse_t *impulse,
                                                                                    ei_impulse_result_t *result,
                                                                                    T *data,
                                                                                    float zero_point,
                                                                                    float scale,
                                                                                    size_t output_features_count,
                                                                                    float threshold,
                                                                                    size_t object_detection_count,
                                                                                    ei_object_detection_nms_config_t nms_config) {
    size_t col_size = 4 + impulse->label_count;
    size_t row_count = output_features_count / col_size;

    static std::vector<ei_impulse_result_bounding_box_t> results;
    static std::vector<ei_impulse_result_bounding_box_t> class_results;
    results.clear();

    // (xmin, ymin, xmax, ymax, cls...)
    for (size_t cls_idx = 0; cls_idx < (size_t)impulse->label_count; cls_idx++)  {

        std::vector<float> boxes;
        std::vector<float> scores;
        std::vector<int> classes;
        class_results.clear();


        for (size_t ix = 0; ix < row_count; ix++) {
            size_t base_ix = ix * col_size;
            float xmin  = (static_cast<float>(data[base_ix + 0]) - zero_point) * scale;
            float ymin  = (static_cast<float>(data[base_ix + 1]) - zero_point) * scale;
            float xmax  = (static_cast<float>(data[base_ix + 2]) - zero_point) * scale;
            float ymax  = (static_cast<float>(data[base_ix + 3]) - zero_point) * scale;
            float score = (static_cast<float>(data[base_ix + 4 + cls_idx]) - zero_point) * scale;

            if (xmin < 0) xmin = 0;
            if (xmin > 1) xmin = 1;
            if (ymin < 0) ymin = 0;
            if (ymin > 1) ymin = 1;
            if (ymax < 0) ymax = 0;
            if (ymax > 1) ymax = 1;
            if (xmax < 0) xmax = 0;
            if (xmax > 1) xmax = 1;
            if (xmax < xmin) xmax = xmin;
            if (ymax < ymin) ymax = ymin;

#if EI_LOG_LEVEL == EI_LOG_LEVEL_DEBUG
                ei_printf("%s (", impulse->categories[(uint32_t)cls_idx]);
                ei_printf_float(cls_idx);
                ei_printf("): ");
                ei_printf_float(score);
                ei_printf(" [ ");
                ei_printf_float(xmin);
                ei_printf(", ");
                ei_printf_float(ymin);
                ei_printf(", ");
                ei_printf_float(xmax);
                ei_printf(", ");
                ei_printf_float(ymax);
                ei_printf(" ]\n");
#endif

            if (score >= threshold && score <= 1.0f) {
                ymin *= static_cast<float>(impulse->input_height);
                xmin *= static_cast<float>(impulse->input_width);
                ymax *= static_cast<float>(impulse->input_height);
                xmax *= static_cast<float>(impulse->input_width);

                boxes.push_back(ymin);
                boxes.push_back(xmin);
                boxes.push_back(ymax);
                boxes.push_back(xmax);
                scores.push_back(score);
                classes.push_back((int)cls_idx);
            }
        }

        size_t nr_boxes = scores.size();

        EI_IMPULSE_ERROR nms_res = ei_run_nms(impulse,
                                            &class_results,
                                            boxes.data(),
                                            scores.data(),
                                            classes.data(),
                                            nr_boxes,
                                            true /*clip_boxes*/,
                                            &nms_config);

        if (nms_res != EI_IMPULSE_OK) {
            return nms_res;
        }

        for (auto bb: class_results) {
            results.push_back(bb);
        }
    }

    prepare_nms_results_common(object_detection_count, result, &results);
    return EI_IMPULSE_OK;
}
#endif // #ifdef EI_HAS_YOLO_PRO

__attribute__((unused)) static EI_IMPULSE_ERROR process_yolo_pro_f32(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
#ifdef EI_HAS_YOLO_PRO
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_f32_config_t *config = (ei_fill_result_object_detection_f32_config_t*)config_ptr;

    ei::matrix_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);
    EI_IMPULSE_ERROR res = fill_result_struct_yolo_pro_common(impulse,
                                                                result,
                                                                raw_output_mtx->buffer,
                                                                0.0f,
                                                                1.0f,
                                                                config->output_features_count,
                                                                config->threshold,
                                                                config->object_detection_count,
                                                                config->nms_config);
    return res;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_YOLO_PRO
}

__attribute__((unused)) static EI_IMPULSE_ERROR process_yolo_pro_i8(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
#ifdef EI_HAS_YOLO_PRO
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_i8_config_t *config = (ei_fill_result_object_detection_i8_config_t*)config_ptr;

    ei::matrix_i8_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    EI_IMPULSE_ERROR res = fill_result_struct_yolo_pro_common(impulse,
                                                    result,
                                                    raw_output_mtx->buffer,
                                                    config->zero_point,
                                                    config->scale,
                                                    config->output_features_count,
                                                    config->threshold,
                                                    config->object_detection_count,
                                                    config->nms_config);
    return res;
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_YOLO_PRO
}

#ifdef EI_HAS_YOLOV11

#define EI_YOLOV11_COORD_ABSOLUTE 0
#define EI_YOLOV11_COORD_NORMALIZED 1

template<typename T>
__attribute__((unused)) static EI_IMPULSE_ERROR fill_result_struct_yolov11_common(const ei_impulse_t *impulse,
                                                                                   ei_impulse_result_t *result,
                                                                                   bool is_coord_normalized,
                                                                                   T *data,
                                                                                   float zero_point,
                                                                                   float scale,
                                                                                   size_t output_features_count,
                                                                                   float threshold,
                                                                                   size_t object_detection_count,
                                                                                   ei_object_detection_nms_config_t nms_config) {
    size_t row_count = 4 + impulse->label_count;
    size_t col_size = output_features_count / row_count;

    static std::vector<ei_impulse_result_bounding_box_t> results;
    static std::vector<ei_impulse_result_bounding_box_t> class_results;
    results.clear();

    // output shape: (num_classes + 4, num_detections) e.g. (5, 189)
    //  [0] -> (xcenter, ycenter, width, height, cls...)
    for (size_t cls_idx = 0; cls_idx < (size_t)impulse->label_count; cls_idx++)  {
        std::vector<float> boxes;
        std::vector<float> scores;
        std::vector<int> classes;
        class_results.clear();

        for (size_t det_idx = 0; det_idx < col_size; det_idx++) {

            float xcenter = (static_cast<float>(data[0 * col_size + det_idx]) - zero_point) * scale;
            float ycenter = (static_cast<float>(data[1 * col_size + det_idx]) - zero_point) * scale;
            float width   = (static_cast<float>(data[2 * col_size + det_idx]) - zero_point) * scale;
            float height  = (static_cast<float>(data[3 * col_size + det_idx]) - zero_point) * scale;

            // xywh -> xyxy
            float xmin  = xcenter - (width / 2.0f);
            float ymin  = ycenter - (height / 2.0f);
            float xmax  = xcenter + (width / 2.0f);
            float ymax  = ycenter + (height / 2.0f);

            if (is_coord_normalized) {
                ymin *= static_cast<float>(impulse->input_height);
                xmin *= static_cast<float>(impulse->input_width);
                ymax *= static_cast<float>(impulse->input_height);
                xmax *= static_cast<float>(impulse->input_width);
            }

            if (xmin < 0) {
                xmin = 0;
            }
            if (xmin > impulse->input_width) {
                xmin = impulse->input_width;
            }
            if (ymin < 0) {
                ymin = 0;
            }
            if (ymin > impulse->input_height) {
                ymin = impulse->input_height;
            }

            if (xmax < 0) {
                xmax = 0;
            }
            if (xmax > impulse->input_width) {
                xmax = impulse->input_width;
            }
            if (ymax < 0) {
                ymax = 0;
            }
            if (ymax > impulse->input_height) {
                ymax = impulse->input_height;
            }

            float score = (static_cast<float>(data[(4+cls_idx) * col_size + det_idx]) - zero_point) * scale;

#if EI_LOG_LEVEL == EI_LOG_LEVEL_DEBUG
                ei_printf("%s (", impulse->categories[(uint32_t)cls_idx]);
                ei_printf_float(cls_idx);
                ei_printf("): ");
                ei_printf_float(score);
                ei_printf(" [ ");
                ei_printf_float(xmin);
                ei_printf(", ");
                ei_printf_float(ymin);
                ei_printf(", ");
                ei_printf_float(xmax);
                ei_printf(", ");
                ei_printf_float(ymax);
                ei_printf(" ]\n");
#endif

            if (score >= threshold && score <= 1.0f) {
                boxes.push_back(ymin);
                boxes.push_back(xmin);
                boxes.push_back(ymax);
                boxes.push_back(xmax);
                scores.push_back(score);
                classes.push_back((int)cls_idx);
            }
        }

        size_t nr_boxes = scores.size();

        EI_IMPULSE_ERROR nms_res = ei_run_nms(impulse,
                                            &class_results,
                                            boxes.data(),
                                            scores.data(),
                                            classes.data(),
                                            nr_boxes,
                                            true /*clip_boxes*/,
                                            &nms_config);

        if (nms_res != EI_IMPULSE_OK) {
            return nms_res;
        }

        for (auto bb: class_results) {
            results.push_back(bb);
        }
    }

    prepare_nms_results_common(object_detection_count, result, &results);
    return EI_IMPULSE_OK;
}
#endif // #ifdef EI_HAS_YOLOV11

/**
  * Fill the result structure from an unquantized output tensor
  */
__attribute__((unused)) static EI_IMPULSE_ERROR process_yolov11_f32(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
#ifdef EI_HAS_YOLOV11
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_f32_config_t *config = (ei_fill_result_object_detection_f32_config_t*)config_ptr;

    ei::matrix_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    return fill_result_struct_yolov11_common(impulse,
                                             result,
                                             config->version == EI_YOLOV11_COORD_NORMALIZED,
                                             raw_output_mtx->buffer,
                                             0.0f,
                                             1.0f,
                                             config->output_features_count,
                                             config->threshold,
                                             config->object_detection_count,
                                             config->nms_config);
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_YOLOV11
}

/**
 * Fill the result structure from a quantized output tensor
*/
template<typename T>
__attribute__((unused)) static EI_IMPULSE_ERROR process_yolov11_i8(ei_impulse_handle_t *handle,
                                                                    uint32_t block_index,
                                                                    uint32_t input_block_id,
                                                                    ei_impulse_result_t *result,
                                                                    void *config_ptr,
                                                                    void *state) {
#ifdef EI_HAS_YOLOV11
    const ei_impulse_t *impulse = handle->impulse;
    const ei_fill_result_object_detection_i8_config_t *config = (ei_fill_result_object_detection_i8_config_t*)config_ptr;

    ei::matrix_i8_t* raw_output_mtx = NULL;
    find_mtx_by_idx(result->_raw_outputs, &raw_output_mtx, input_block_id, impulse->learning_blocks_size);

    return fill_result_struct_yolov11_common(impulse,
                                             result,
                                             config->version == EI_YOLOV11_COORD_NORMALIZED,
                                             raw_output_mtx->buffer,
                                             config->zero_point,
                                             config->scale,
                                             config->output_features_count,
                                             config->threshold,
                                             config->object_detection_count,
                                             config->nms_config);
#else
    return EI_IMPULSE_LAST_LAYER_NOT_AVAILABLE;
#endif // #ifdef EI_HAS_YOLOV11
}

EI_IMPULSE_ERROR set_threshold_postprocessing(int16_t block_number, void* block_config, uint8_t type, float threshold) {

    switch (type) {
        case EI_CLASSIFIER_MODE_OBJECT_DETECTION: {
            ei_fill_result_object_detection_threshold_config_t *config = (ei_fill_result_object_detection_threshold_config_t*)block_config;
            config->threshold = threshold;
            break;
        }
        case EI_CLASSIFIER_MODE_VISUAL_ANOMALY: {
            ei_fill_result_visual_ad_f32_config_t *config = (ei_fill_result_visual_ad_f32_config_t*)block_config;
            config->threshold = threshold;
            break;
        }
        default: {
            return EI_IMPULSE_POSTPROCESSING_ERROR;
        }
    }

    return EI_IMPULSE_OK;
}

EI_IMPULSE_ERROR get_threshold_postprocessing(std::string* type_str, std::string* threshold_name_str, void* block_config, uint8_t type, float* threshold) {

    switch (type) {
        case EI_CLASSIFIER_MODE_OBJECT_DETECTION: {
            ei_fill_result_object_detection_threshold_config_t *config = (ei_fill_result_object_detection_threshold_config_t*)block_config;
            *threshold = config->threshold;
            *type_str = "object_detection";
            *threshold_name_str = "min_score";
            break;
        }
        case EI_CLASSIFIER_MODE_VISUAL_ANOMALY: {
            ei_fill_result_visual_ad_f32_config_t *config = (ei_fill_result_visual_ad_f32_config_t*)block_config;
            *threshold = config->threshold;
            *type_str = "anomaly_gmm";
            *threshold_name_str = "min_anomaly_score";
            break;
        }
        default: {
            return EI_IMPULSE_POSTPROCESSING_ERROR;
        }
    }

    return EI_IMPULSE_OK;
}

#endif // EI_POSTPROCESSING_COMMON_H
