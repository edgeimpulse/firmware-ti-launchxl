
/* Include ----------------------------------------------------------------- */
#include "ei_ti_launchxl_fs_commands.h"
#include "ei_device_ti_launchxl.h"

#include <ti/drivers/NVS.h>
#include "ti_drivers_config.h"

#define SERIAL_FLASH 0
#define MICRO_SD     1
#define RAM          2

#define SAMPLE_MEMORY SERIAL_FLASH

#define SIZE_RAM_BUFFER 0x1000
#define RAM_BLOCK_SIZE	512
#define RAM_N_BLOCKS    (SIZE_RAM_BUFFER / RAM_BLOCK_SIZE)

/* Private variables ------------------------------------------------------- */
static uint32_t device_size, device_erase_block_size;
static NVS_Handle nvsHandle;

#if (SAMPLE_MEMORY == RAM)
static uint8_t ram_memory[SIZE_RAM_BUFFER];
#endif

/** 32-bit align write buffer size */
#define WORD_ALIGN(a) ((a & 0x3) ? (a & ~0x3) + 0x4 : a)

/**
 * @brief Call TI memory driver
 *
 * @return int 0 if ok
 */
int ei_ti_launchxl_fs_init(void)
{
#if (SAMPLE_MEMORY == SERIAL_FLASH)

    NVS_Attrs regionAttrs;
    NVS_Params nvsParams;

    NVS_init();

    NVS_Params_init(&nvsParams);
    nvsHandle = NVS_open(CONFIG_NVSEXTERNAL, &nvsParams);

    if (nvsHandle == NULL) {
        return -1;
    }

    /*
     * This will populate a NVS_Attrs structure with properties specific
     * to a NVS_Handle such as region base address, region size,
     * and sector size.
     */
    NVS_getAttrs(nvsHandle, &regionAttrs);
    device_size = regionAttrs.regionSize;
    device_erase_block_size = regionAttrs.sectorSize;

    return 0;

#endif
}

/**
 * @brief      Copy configuration data to config pointer
 *
 * @param      config       Destination pointer for config
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_ti_launchxl_ret_t enum
 */
int ei_ti_launchxl_fs_load_config(uint32_t *config, uint32_t config_size)
{
    int retVal = TI_LAUNCHXL_FS_CMD_OK;

    if (config == NULL) {
        return TI_LAUNCHXL_FS_CMD_NULL_POINTER;
    }

#if (SAMPLE_MEMORY == RAM)

    return retVal;

#elif (SAMPLE_MEMORY == SERIAL_FLASH)

    if(NVS_read(nvsHandle, 0, (void *) config, config_size)) {
        retVal = TI_LAUNCHXL_FS_CMD_READ_ERROR;
    }

    return retVal;
#endif
}

/**
 * @brief      Write config to Flash
 *
 * @param[in]  config       Pointer to configuration data
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_ti_launchxl_ret_t enum
 */
int ei_ti_launchxl_fs_save_config(const uint32_t *config, uint32_t config_size)
{
    int retVal = TI_LAUNCHXL_FS_CMD_OK;

    if (config == NULL) {
        return TI_LAUNCHXL_FS_CMD_NULL_POINTER;
    }

#if (SAMPLE_MEMORY == RAM)

    return retVal;

#elif (SAMPLE_MEMORY == SERIAL_FLASH)

    if(NVS_erase(nvsHandle, 0, device_erase_block_size)) {
        retVal = TI_LAUNCHXL_FS_CMD_ERASE_ERROR;
    }
    else if(NVS_write(nvsHandle, 0, (void *) config, config_size, 0)) {
            retVal = TI_LAUNCHXL_FS_CMD_WRITE_ERROR;
    }

    return retVal;
#endif
}

/**
 * @brief      Erase blocks in sample data space
 *
 * @param[in]  start_block  The start block
 * @param[in]  end_address  The end address
 *
 * @return     ei_ti_launchxl_ret_t
 */
int ei_ti_launchxl_fs_erase_sampledata(uint32_t start_block, uint32_t end_address)
{
#if (SAMPLE_MEMORY == RAM)
    return TI_LAUNCHXL_FS_CMD_OK;
#elif (SAMPLE_MEMORY == SERIAL_FLASH)

    uint32_t end_block_address;

    if(end_address <= 1) {
        return TI_LAUNCHXL_FS_CMD_OK;
    }

    /* Round up to block address */
    end_block_address = ((end_address - 1) / device_erase_block_size) + 1;
    end_block_address *= device_erase_block_size;

    if(NVS_erase(nvsHandle, device_erase_block_size + (device_erase_block_size * start_block), end_block_address)) {
        return TI_LAUNCHXL_FS_CMD_ERASE_ERROR;
    }
    else {
        return TI_LAUNCHXL_FS_CMD_OK;
    }

#endif
}

/**
 * @brief      Write sample data
 *
 * @param[in]  sample_buffer   The sample buffer
 * @param[in]  address_offset  The address offset
 * @param[in]  n_samples       The n samples
 *
 * @return     ei_ti_launchxl_ret_t
 */
int ei_ti_launchxl_fs_write_samples(const void *sample_buffer, uint32_t address_offset, uint32_t n_samples)
{

#if (SAMPLE_MEMORY == RAM)

    uint32_t n_word_samples = WORD_ALIGN(n_samples);

    if ((address_offset + n_word_samples) > SIZE_RAM_BUFFER) {
        return TI_LAUNCHXL_FS_CMD_WRITE_ERROR;
    }
    else if (sample_buffer == 0) {
        return TI_LAUNCHXL_FS_CMD_NULL_POINTER;
    }

    for (int i = 0; i < n_word_samples; i++) {
        ram_memory[address_offset + i] = *((char *)sample_buffer + i);
    }
    return TI_LAUNCHXL_FS_CMD_OK;

#elif (SAMPLE_MEMORY == SERIAL_FLASH)

    NVS_write(nvsHandle, device_erase_block_size + address_offset, (void *) sample_buffer, n_samples, 0);
    return TI_LAUNCHXL_FS_CMD_OK;
#endif
}

/**
 * @brief      Read sample data
 *
 * @param      sample_buffer   The sample buffer
 * @param[in]  address_offset  The address offset
 * @param[in]  n_read_bytes    The n read bytes
 *
 * @return     ei_ti_launchxl_ret_t
 */
int ei_ti_launchxl_fs_read_sample_data(void *sample_buffer, uint32_t address_offset, uint32_t n_read_bytes)
{
#if (SAMPLE_MEMORY == RAM)
    if ((address_offset + n_read_bytes) > SIZE_RAM_BUFFER) {
        return TI_LAUNCHXL_FS_CMD_READ_ERROR;
    }
    else if (sample_buffer == 0) {
        return TI_LAUNCHXL_FS_CMD_NULL_POINTER;
    }

    for (int i = 0; i < n_read_bytes; i++) {
        *((char *)sample_buffer + i) = ram_memory[address_offset + i];
    }
    return TI_LAUNCHXL_FS_CMD_OK;

#elif (SAMPLE_MEMORY == SERIAL_FLASH)

    if(NVS_read(nvsHandle, device_erase_block_size + address_offset, (void *)sample_buffer, n_read_bytes)) {
        return TI_LAUNCHXL_FS_CMD_READ_ERROR;
    }
    else {
        return TI_LAUNCHXL_FS_CMD_OK;
    }
#endif
}

/**
 * @brief Close file on file system
 *
 */
void ei_ti_launchxl_fs_close_sample_file(void)
{

}

/**
 * @brief      Get block size (Smallest erasble block). 
 *
 * @return     Length of 1 block
 */
uint32_t ei_ti_launchxl_fs_get_block_size(void)
{
#if (SAMPLE_MEMORY == RAM)
    return RAM_BLOCK_SIZE;
#else
    return device_erase_block_size;
#endif
}

/**
 * @brief      Get available sample blocks
 *
 * @return     Sample memory size / block size
 */
uint32_t ei_ti_launchxl_fs_get_n_available_sample_blocks(void)
{
#if (SAMPLE_MEMORY == RAM)
    return RAM_N_BLOCKS;
#else
    return (device_size - device_erase_block_size) / device_erase_block_size;
#endif
}