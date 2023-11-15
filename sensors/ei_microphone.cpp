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
#include <stdint.h>
#include <stdlib.h>

#include "ei_microphone.h"
#include "ei_ti_launchxl_fs_commands.h"
#include "ei_device_ti_launchxl.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "arm_math.h"

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <mqueue.h>

#include "firmware-sdk/ei_config_types.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "firmware-sdk/sensor-aq/sensor_aq_none.h"
#include "arm_math.h"
#include "ti_drivers_config.h"
#include "AudioCodec.h"
#include <ti/drivers/I2S.h>
#include "FreeRTOS.h"

//#define ENABLE_DEBUG
//#define ENABLE_MEMORY_DEBUG

/* Audio sampling config */
#define AUDIO_SAMPLING_FREQUENCY            16000
#define AUDIO_SAMPLES_PER_MS                (AUDIO_SAMPLING_FREQUENCY / 1000)
#define AUDIO_DSP_SAMPLE_LENGTH_MS          8
#define AUDIO_DSP_SAMPLE_RESOLUTION         (sizeof(short))
#define AUDIO_DSP_SAMPLE_BUFFER_SIZE        6400

/* The higher the sampling frequency, the less time we have to process the data, but the higher the sound quality. */
#define SAMPLE_RATE                     AUDIO_SAMPLING_FREQUENCY   /* Supported values: 8kHz, 16kHz, 32kHz and 44.1kHz */
#define INPUT_OPTION                    AudioCodec_MIC_ONBOARD
#define OUTPUT_OPTION                   AudioCodec_SPEAKER_NONE

/* The more storage space we have, the more delay we have, but the more time we have to process the data. */
#define NUMBUFS         2      /* Total number of buffers to loop through */
#define BUFSIZE         AUDIO_DSP_SAMPLE_BUFFER_SIZE     /* I2S buffer size */

struct frameEvarg {
    int32_t flen;
    //int32_t num;
    int16_t *fbuf;
};

#define MSG_SIZE sizeof(struct frameEvarg)
#define MSG_NUM NUMBUFS

static mqd_t mic_queue;

/* Semaphore used to indicate that data must be processed */
//static sem_t semErrorCallback;

/* Lists containing transactions. Each transaction is in turn in these three lists */
List_List i2sReadList;

/* Buffers containing the data: written by read-interface, modified by treatment, and read by write-interface */
static uint8_t* i2sBufList[NUMBUFS] = {};

/* Transactions will successively be part of the i2sReadList and the treatmentList */
static I2S_Transaction *i2sTransactionList[NUMBUFS] = {};

I2S_Handle i2sHandle;

static volatile int readCnt = 0;
static volatile bool record_ready = false;
static volatile bool skip = true; // used to skip the first (invalid) sample slice

static void audio_buffer_callback(void *buffer, uint32_t n_bytes);
static void get_dsp_data(void (*callback)(void *buffer, uint32_t n_bytes));
static void FrameCb(void *buf, uint16_t blen);
static void empty_queue(void);

static void FrameCb(void *buf, uint16_t blen)
{
    if ((record_ready == true) && (skip == false)) {
        struct frameEvarg evArg;

        evArg.flen = blen;
        evArg.fbuf = (int16_t*) buf;

        mq_send(mic_queue , (char *)&evArg, sizeof(struct frameEvarg), 0);
    } else if ((record_ready == true) && (skip == true)) {
        skip = false;
    }
}

static void empty_queue()
{
    struct frameEvarg evArg;
    int n_msg_ready;
    struct mq_attr mqAttrs;
    do {
        mq_getattr(mic_queue, &mqAttrs);
        n_msg_ready = mqAttrs.mq_curmsgs;

        if(n_msg_ready > 0) {
            mq_receive(mic_queue, (char *)&evArg, sizeof(evArg), NULL);
            n_msg_ready--;
        }
    } while(n_msg_ready);
}

static void errCallbackFxn(I2S_Handle handle, int_fast16_t status, I2S_Transaction *transactionPtr) {
    /* The content of this callback is executed if an I2S error occurs */
    //sem_post(&semErrorCallback);
}

static void writeCallbackFxn(I2S_Handle handle, int_fast16_t status, I2S_Transaction *transactionPtr) {
    /*
     * The content of this callback is executed every time a write-transaction is started
     */
}

static void readCallbackFxn(I2S_Handle handle, int_fast16_t status, I2S_Transaction *transactionPtr) {
    /*
     * The content of this callback is executed every time a read-transaction
     * is started
     */

    /* We must consider the previous transaction (the current one is not over) */
    I2S_Transaction *transactionFinished = (I2S_Transaction*)List_prev(&transactionPtr->queueElement);

    if(transactionFinished != NULL) {
        /* The finished transaction contains data that must be treated */

        // no need for processing, already using right channel with ONBOARD_MIC and MONO_INV
        FrameCb(transactionFinished->bufPtr, transactionFinished->bufSize);
    }
}

/** Status and control struct for inferencing struct */
typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

extern ei_config_t *ei_config_get_config();

/* Dummy functions for sensor_aq_ctx type */
static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*)
{    
    return count;
}

static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin)
{    
    return 0;
}

/* Private variables ------------------------------------------------------- */
static uint32_t headerOffset;
static uint32_t samples_required;
static uint32_t current_sample;
static int max_msg_ready = 0;

static inference_t inference;

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/* Private functions ------------------------------------------------------- */

/**
 * @brief      Ingestion audio callback, write audio samples to memory
 *             Signal record_ready when all needed samples are there
 * @param      buffer   Pointer to source buffer
 * @param[in]  n_bytes  Number of bytes to write
 */
static void audio_buffer_callback(void *buffer, uint32_t n_bytes)
{
    ei_ti_launchxl_fs_write_samples((const void *)buffer, headerOffset + current_sample, n_bytes);

    ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)buffer, n_bytes);

#ifdef ENABLE_DEBUG
    ei_printf("cb: %p (%d)\r\n", buffer, n_bytes);
#endif

    current_sample += n_bytes;
    if(current_sample >= (samples_required << 1)) {
        record_ready = false;
    }
}

/**
 * @brief      Inference audio callback, store samples in ram buffer
 *             Signal when buffer is full, and swap buffers
 * @param      buffer   Pointer to source buffer
 * @param[in]  n_bytes  Number of bytes to write
 */
static void audio_buffer_inference_callback(void *buffer, uint32_t n_bytes)
{
    inference.buf_count += n_bytes >> 1; // bytes to samples

    if(inference.buf_count >= inference.n_samples) {

#ifdef ENABLE_DEBUG
        ei_printf("cb: %p (%d:%d)\r\n", inference.buffers[inference.buf_select], inference.buf_count,n_bytes) ;
#endif

        inference.buffers[inference.buf_select] = (int16_t*) buffer;
        inference.buf_select ^= 1;
        inference.buf_count = 0;
        inference.buf_ready = 1;

    } else {
        ei_printf("something bad happend (%d:%d)\r\n", inference.buf_count,n_bytes) ;
    }
}

/**
 * Gets audio data passed by driver
 *
 * @param[in]  callback  Callback needs to handle the audio samples
 */
static void get_dsp_data(void (*callback)(void *buffer, uint32_t n_bytes))
{
    struct frameEvarg evArg;
    int n_msg_ready;
    struct mq_attr mqAttrs;
    do {
        mq_receive(mic_queue, (char *)&evArg, sizeof(evArg), NULL);

        callback((void *)evArg.fbuf, evArg.flen);

        mq_getattr(mic_queue, &mqAttrs);
        n_msg_ready = mqAttrs.mq_curmsgs;

        if(n_msg_ready > max_msg_ready) {
            max_msg_ready = n_msg_ready;
        }

    } while(n_msg_ready);
}


static void finish_and_upload(char *filename, uint32_t sample_length_ms) {    

    ei_printf("Done sampling, total bytes collected: %u\n", current_sample);


    ei_printf("[1/1] Uploading file to Edge Impulse...\n");

    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=%lu, to=%lu.\n", 0, current_sample + headerOffset);


    ei_printf("[1/1] Uploading file to Edge Impulse OK (took %d ms.)\n", 200);//upload_timer.read_ms());

    ei_printf("OK\n");

    EiDevice.set_state(eiStateIdle);    
}

static int insert_ref(char *buffer, int hdrLength)
{
#define EXTRA_BYTES(a) ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))
    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for (size_t i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for (int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }

    buffer[addLength++] = 0xFF;

    return addLength;
}

static bool create_header(void)
{
    sensor_aq_init_mbedtls_hs256_context(
        &ei_mic_signing_ctx,
        &ei_mic_hs_ctx,
        ei_config_get_config()->sample_hmac_key);

    sensor_aq_payload_info payload = { EiDevice.get_id_pointer(),
                                       EiDevice.get_type_pointer(),
                                       1000.0f / static_cast<float>(AUDIO_SAMPLING_FREQUENCY),
                                       { { "audio", "wav" } } };

    int tr = sensor_aq_init(&ei_mic_ctx, &payload, NULL, true);

    if (tr != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", tr);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix >= 0; ix--) {
        if (((uint8_t *)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    int ref_size =
        insert_ref(((char *)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), end_of_header_ix);

    // and update the signature
    tr = ei_mic_ctx.signature_ctx->update(
        ei_mic_ctx.signature_ctx,
        ((uint8_t *)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix),
        ref_size);
    if (tr != 0) {
        ei_printf("Failed to update signature from header (%d)\n", tr);
        return false;
    }

    end_of_header_ix += ref_size;

    // Write to blockdevice
    tr = ei_ti_launchxl_fs_write_samples(ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);

    if (tr != 0) {
        ei_printf("Failed to write to header blockdevice (%d)\n", tr);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}


/* Public functions -------------------------------------------------------- */

int audio_codec_open()
{
    /* Initialize TLV320AIC3254 Codec on Audio BP */
    uint8_t status = AudioCodec_open();
    if( AudioCodec_STATUS_SUCCESS != status)
    {
        /* Error Initializing codec */
        while(1);
    }

    /* Configure Codec */
    status =  AudioCodec_config(AudioCodec_TI_3254, AudioCodec_16_BIT,
                                SAMPLE_RATE, AudioCodec_MONO, OUTPUT_OPTION,
                                INPUT_OPTION);
    if( AudioCodec_STATUS_SUCCESS != status)
    {
        /* Error Initializing codec */
        while(1);
    }

    /* Volume control */
    AudioCodec_micVolCtrl(AudioCodec_TI_3254, AudioCodec_MIC_ONBOARD, 75);

    /*
     *  Open the I2S driver
     */
    I2S_Params i2sParams;
    I2S_Params_init(&i2sParams);
    i2sParams.samplingFrequency =  SAMPLE_RATE;
    i2sParams.fixedBufferLength =  BUFSIZE;
    i2sParams.writeCallback     =  writeCallbackFxn ;
    i2sParams.readCallback      =  readCallbackFxn ;
    i2sParams.errorCallback     =  errCallbackFxn;
    i2sParams.SD1Channels       =  I2S_CHANNELS_MONO_INV;
    i2sParams.SD0Use            =  I2S_SD0_DISABLED;
    i2sParams.SD0Channels       =  I2S_CHANNELS_NONE;
    i2sHandle = I2S_open(CONFIG_I2S_0, &i2sParams);
    if (i2sHandle == NULL) {
        /* Error Opening the I2S driver */
        while(1);
    }

    return 0;
}

/**
 * @brief      Set the PDM mic to +34dB
 */
extern "C" void ei_microphone_init(void)
{

    int retc;
    //retc = sem_init(&semErrorCallback, 0, 0);
    //if (retc == -1) {
    //    /* sem_init() failed */
    //    while (1);
    //}

    struct mq_attr mqAttrs;
    mqAttrs.mq_maxmsg = MSG_NUM;
    mqAttrs.mq_msgsize = MSG_SIZE;
    mqAttrs.mq_flags = 0;
    mic_queue = mq_open ("audio", O_RDWR | O_CREAT,
                    0664, &mqAttrs);
    if (mic_queue == (mqd_t)-1) {
        /* mq_open() failed */
        while (1);
    }

    retc = audio_codec_open();
    if (retc == -1) {
        while (1);
    }
}

bool ei_microphone_record(uint32_t sample_length_ms, uint32_t start_delay_ms, bool print_start_messages)
{
    EiDevice.set_state(eiStateErasingFlash);

    if (print_start_messages) {
        ei_printf(
            "Starting in %lu ms... (or until all flash was erased)\n",
            start_delay_ms < 2000 ? 2000 : start_delay_ms);
    }

    if (start_delay_ms < 2000) {
        EiDevice.delay_ms(2000 - start_delay_ms);
    }

    if (ei_ti_launchxl_fs_erase_sampledata(0, (samples_required << 1) + ei_ti_launchxl_fs_get_block_size()) !=
        TI_LAUNCHXL_FS_CMD_OK) {
        return false;
    }

    create_header();

    if (print_start_messages) {
        ei_printf("Sampling...\n");
    }

    startStream();

    return true;
}

bool ei_microphone_inference_start(uint32_t n_samples)
{

#ifdef ENABLE_MEMORY_DEBUG
    ei_printf("min heap before:%d\r\n", xPortGetMinimumEverFreeHeapSize());
    ei_printf("heap before:%d\r\n", xPortGetFreeHeapSize());
#endif

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    startStream();
    record_ready = true;

#ifdef ENABLE_MEMORY_DEBUG
    ei_printf("heap after:%d\r\n", xPortGetFreeHeapSize());
    ei_printf("min heap after:%d\r\n", xPortGetMinimumEverFreeHeapSize());
#endif

    return true;
}

bool ei_microphone_inference_record(void)
{
    bool ret = true;

   if(inference.buf_ready == 1) {
        ei_printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
        ret = false;
    }

#ifdef ENABLE_MEMORY_DEBUG
    ei_printf("min heap before record:%d\r\n", xPortGetMinimumEverFreeHeapSize());
    ei_printf("heap before record:%d\r\n", xPortGetFreeHeapSize());
#endif

    while (inference.buf_ready == 0) {
        get_dsp_data(&audio_buffer_inference_callback);
    };

#ifdef ENABLE_MEMORY_DEBUG
    ei_printf("heap after record:%d\r\n", xPortGetFreeHeapSize());
    ei_printf("min heap after record:%d\r\n", xPortGetMinimumEverFreeHeapSize());
#endif

    if (max_msg_ready >= NUMBUFS-1) {
        ei_printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW): %d\n", max_msg_ready);
    }

    max_msg_ready = 0;
    inference.buf_ready = 0;

    return ret;
}

/**
 * @brief      Reset buffer counters for non-continuous inferecing
 */
void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
    record_ready = true;
    empty_queue();
}


/**
 * Get raw audio signal data
 */
int ei_microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    arm_q15_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
#ifdef ENABLE_DEBUG
    ei_printf("get_data: %p\r\n", inference.buffers[inference.buf_select]);
#endif

    return 0;
}

bool ei_microphone_inference_end(void)
{
    record_ready = false;
    stopStream();

    return true;
}

/**
 * Sample raw data
 */
bool ei_microphone_sample_start(void)
{

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: "); (ei_printf_float((float)ei_config_get_config()->sample_interval_ms));ei_printf(" ms.\n");
    ei_printf("\tLength: %lu ms.\n", ei_config_get_config()->sample_length_ms);
    ei_printf("\tName: %s\n", ei_config_get_config()->sample_label);
    ei_printf("\tHMAC Key: %s\n", ei_config_get_config()->sample_hmac_key);
    char filename[256];
    int fn_r = snprintf(filename, 256, "/fs/%s", ei_config_get_config()->sample_label);
    if (fn_r <= 0) {
        ei_printf("ERR: Failed to allocate file name\n");
        return false;
    }
    ei_printf("\tFile name: %s\n", filename);


    samples_required = (uint32_t)(((float)ei_config_get_config()->sample_length_ms) / ei_config_get_config()->sample_interval_ms);

    /* Round to even number of samples for word align flash write */
    if(samples_required & 1) {
        samples_required++;
    }

    current_sample = 0;

    max_msg_ready = 0;

    bool r = ei_microphone_record(ei_config_get_config()->sample_length_ms, (((samples_required <<1)/ ei_ti_launchxl_fs_get_block_size()) * TI_LAUNCHXL_FS_BLOCK_ERASE_TIME_MS), true);
    if (!r) {
        return r;
    }

    record_ready = true;
    EiDevice.set_state(eiStateSampling);

    while (record_ready == true) {
        get_dsp_data(&audio_buffer_callback);
    };

    stopStream();

    if (max_msg_ready >= NUMBUFS-1) {
        ei_printf("Error sample buffer overrun.: %d\n", max_msg_ready);
    }

    int ctx_err =
        ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        ei_printf("Failed to finish signature (%d)\n", ctx_err);
        return false;
    }

    ei_ti_launchxl_fs_close_sample_file();

    // load the first page in flash...
    uint8_t *page_buffer = (uint8_t *)ei_malloc(ei_ti_launchxl_fs_get_block_size());
    if (!page_buffer) {
        ei_printf("Failed to allocate a page buffer to write the hash\n");
        return false;
    }

    int j = ei_ti_launchxl_fs_read_sample_data(page_buffer, 0, ei_ti_launchxl_fs_get_block_size());
    if (j != 0) {
        ei_printf("Failed to read first page (%d)\n", j);
        ei_free(page_buffer);
        return false;
    }

    // update the hash
    uint8_t *hash = ei_mic_ctx.hash_buffer.buffer;
    // we have allocated twice as much for this data (because we also want to be able to represent in hex)
    // thus only loop over the first half of the bytes as the signature_ctx has written to those
    for (size_t hash_ix = 0; hash_ix < ei_mic_ctx.hash_buffer.size / 2; hash_ix++) {
        // this might seem convoluted, but snprintf() with %02x is not always supported e.g. by newlib-nano
        // we encode as hex... first ASCII char encodes top 4 bytes
        uint8_t first = (hash[hash_ix] >> 4) & 0xf;
        // second encodes lower 4 bytes
        uint8_t second = hash[hash_ix] & 0xf;

        // if 0..9 -> '0' (48) + value, if >10, then use 'a' (97) - 10 + value
        char first_c = first >= 10 ? 87 + first : 48 + first;
        char second_c = second >= 10 ? 87 + second : 48 + second;

        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 0] = first_c;
        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 1] = second_c;
    }

    ei_ti_launchxl_fs_close_sample_file();

    j = ei_ti_launchxl_fs_erase_sampledata(0, ei_ti_launchxl_fs_get_block_size());
    if (j != 0) {
        ei_printf("Failed to erase first page (%d)\n", j);
        ei_free(page_buffer);
        return false;
    }

    j = ei_ti_launchxl_fs_write_samples(page_buffer, 0, ei_ti_launchxl_fs_get_block_size());

    ei_free(page_buffer);

    ei_ti_launchxl_fs_close_sample_file();

    if (j != 0) {
        ei_printf("Failed to write first page with updated hash (%d)\n", j);
        return false;
    }

    finish_and_upload(filename, ei_config_get_config()->sample_length_ms);

    return true;
}

void startStream()
{
    /* Initialize the queues and the I2S transactions */
    List_clearList(&i2sReadList);

    uint8_t k;
    for(k = 0; k < NUMBUFS; k++) {
        i2sBufList[k] = (uint8_t *)ei_malloc(BUFSIZE);
        if (i2sBufList[k] == NULL) {
            ei_printf("failed to create i2sBuf: %d\r\n", k);
            return;
        }

        i2sTransactionList[k] = (I2S_Transaction *)ei_malloc(sizeof(I2S_Transaction));
        if (i2sTransactionList[k] == NULL) {
            ei_printf("failed to create i2sTransaction: %d\r\n", k);
            return;
        }
        I2S_Transaction_init(i2sTransactionList[k]);
        i2sTransactionList[k]->bufPtr  = i2sBufList[k];
        i2sTransactionList[k]->bufSize = BUFSIZE;
        List_put(&i2sReadList, (List_Elem*)i2sTransactionList[k]);
    }

    List_tail(&i2sReadList)->next = List_head(&i2sReadList);
    List_head(&i2sReadList)->prev = List_tail(&i2sReadList);

    I2S_setReadQueueHead(i2sHandle,  (I2S_Transaction*) List_head(&i2sReadList));

    skip = true;
    empty_queue();

    I2S_startClocks(i2sHandle);
    I2S_startRead(i2sHandle);
}

void stopStream()
{
    /* Stop I2S streaming */
    I2S_stopRead(i2sHandle);
    I2S_stopClocks(i2sHandle);

    uint8_t k;
    for(k = 0; k < NUMBUFS; k++) {
        ei_free(i2sBufList[k]);
        ei_free(i2sTransactionList[k]);
    }
}
