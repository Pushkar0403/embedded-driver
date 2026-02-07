#include "device_registers.h"
#include <string.h>
#include <stdio.h>

void reg_init(register_file_t *rf) {
    if (!rf) return;
    memset(rf->regs, 0, sizeof(rf->regs));
}

uint32_t reg_read(register_file_t *rf, uint32_t offset) {
    if (!rf || offset >= REGISTER_FILE_SIZE) {
        return 0xFFFFFFFF;  /* Invalid read returns all 1s */
    }
    return rf->regs[offset / 4];
}

void reg_write(register_file_t *rf, uint32_t offset, uint32_t value) {
    if (!rf || offset >= REGISTER_FILE_SIZE) {
        return;
    }
    rf->regs[offset / 4] = value;
}

void reg_set_bits(register_file_t *rf, uint32_t offset, uint32_t bits) {
    if (!rf || offset >= REGISTER_FILE_SIZE) {
        return;
    }
    rf->regs[offset / 4] |= bits;
}

void reg_clear_bits(register_file_t *rf, uint32_t offset, uint32_t bits) {
    if (!rf || offset >= REGISTER_FILE_SIZE) {
        return;
    }
    rf->regs[offset / 4] &= ~bits;
}
