#ifndef DEVICE_REGISTERS_H
#define DEVICE_REGISTERS_H

#include <stdint.h>

/* Simulated hardware register addresses (offsets) */
#define REG_MOTOR_CTRL      0x00
#define REG_MOTOR_STATUS    0x04
#define REG_MOTOR_SPEED     0x08
#define REG_MOTOR_POSITION  0x0C
#define REG_SENSOR_CTRL     0x10
#define REG_SENSOR_DATA     0x14
#define REG_SENSOR_STATUS   0x18
#define REG_IRQ_STATUS      0x1C
#define REG_IRQ_ENABLE      0x20

/* Motor control register bits */
#define MOTOR_CTRL_ENABLE   (1 << 0)
#define MOTOR_CTRL_DIR_CW   (1 << 1)
#define MOTOR_CTRL_BRAKE    (1 << 2)
#define MOTOR_CTRL_RESET    (1 << 7)

/* Motor status register bits */
#define MOTOR_STATUS_RUNNING    (1 << 0)
#define MOTOR_STATUS_FAULT      (1 << 1)
#define MOTOR_STATUS_STALL      (1 << 2)
#define MOTOR_STATUS_OVERHEAT   (1 << 3)

/* Sensor control register bits */
#define SENSOR_CTRL_ENABLE      (1 << 0)
#define SENSOR_CTRL_CONTINUOUS  (1 << 1)
#define SENSOR_CTRL_TRIGGER     (1 << 2)

/* Sensor status register bits */
#define SENSOR_STATUS_READY     (1 << 0)
#define SENSOR_STATUS_OVERFLOW  (1 << 1)
#define SENSOR_STATUS_ERROR     (1 << 2)

/* IRQ bits */
#define IRQ_MOTOR_FAULT     (1 << 0)
#define IRQ_MOTOR_STALL     (1 << 1)
#define IRQ_SENSOR_READY    (1 << 2)
#define IRQ_SENSOR_ERROR    (1 << 3)

/* Register file size */
#define REGISTER_FILE_SIZE  0x24

/* Simulated register file */
typedef struct {
    uint32_t regs[REGISTER_FILE_SIZE / 4];
} register_file_t;

/* Register I/O functions */
void reg_init(register_file_t *rf);
uint32_t reg_read(register_file_t *rf, uint32_t offset);
void reg_write(register_file_t *rf, uint32_t offset, uint32_t value);
void reg_set_bits(register_file_t *rf, uint32_t offset, uint32_t bits);
void reg_clear_bits(register_file_t *rf, uint32_t offset, uint32_t bits);

#endif /* DEVICE_REGISTERS_H */
