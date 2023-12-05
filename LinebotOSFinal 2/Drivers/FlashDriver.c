#include <avr/io.h>
#include <avr/pgmspace.h>

void writeMEMSDataToFlash(MEMS_Data* data, uint32_t address) {
    // Erase the flash page
    NVM.CMD = NVM_CMD_ERASE_APP_PAGE_gc;
    NVM.ADDR0 = address & 0xFF;
    NVM.ADDR1 = (address >> 8) & 0xFF;
    NVM.ADDR2 = (address >> 16) & 0xFF;
    CCP = CCP_SPM_gc;
    NVM.CTRLA = NVM_CMDEX_bm;

    // Wait for the erase operation to complete
    while (NVM.STATUS & NVM_NVMBUSY_bm);

    // Write the MEMS_Data struct to flash
    NVM.CMD = NVM_CMD_WRITE_APP_PAGE_gc;
    for (int i = 0; i < sizeof(MEMS_Data); i++) {
        *((uint8_t*)address + i) = ((uint8_t*)data)[i];
    }
    CCP = CCP_SPM_gc;
    NVM.CTRLA = NVM_CMDEX_bm;

    // Wait for the write operation to complete
    while (NVM.STATUS & NVM_NVMBUSY_bm);
}

#define FLASH_PAGE_SIZE 256
#define MEMS_DATA_SIZE 12
#define MEMS_DATA_PER_PAGE (FLASH_PAGE_SIZE / MEMS_DATA_SIZE)

MEMS_Data memsDataBuffer[MEMS_DATA_PER_PAGE];
uint8_t memsDataCount = 0;

void storeMEMSData(MEMS_Data* data) {
    // Add the new data to the buffer
    memsDataBuffer[memsDataCount] = *data;
    memsDataCount++;

    // If the buffer is full, write it to flash
    if (memsDataCount == MEMS_DATA_PER_PAGE) {
        writeMEMSDataToFlash(memsDataBuffer, flashAddress);
        memsDataCount = 0;
    }
}