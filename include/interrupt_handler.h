#ifndef INTERRUPT_HANDLER_H
#define INTERRUPT_HANDLER_H

#include "device_registers.h"
#include "motor_controller.h"
#include "sensor_array.h"
#include <signal.h>
#include <stdbool.h>

/* Interrupt sources */
typedef enum {
    INT_MOTOR_FAULT = 0,
    INT_MOTOR_STALL,
    INT_SENSOR_READY,
    INT_SENSOR_ERROR,
    INT_TIMER,
    INT_COUNT
} interrupt_source_t;

/* Interrupt handler callback type */
typedef void (*irq_handler_t)(interrupt_source_t source, void *context);

/* Interrupt controller context */
typedef struct {
    register_file_t *regs;
    motor_controller_t *motor;
    sensor_array_t *sensors;
    irq_handler_t handlers[INT_COUNT];
    void *handler_contexts[INT_COUNT];
    uint32_t pending_irqs;
    uint32_t enabled_irqs;
    volatile sig_atomic_t signal_received;
} interrupt_controller_t;

/* Global interrupt controller (needed for signal handlers) */
extern interrupt_controller_t *g_irq_controller;

/* Interrupt controller functions */
int irq_init(interrupt_controller_t *ic, register_file_t *regs,
             motor_controller_t *motor, sensor_array_t *sensors);
void irq_cleanup(interrupt_controller_t *ic);

/* Handler registration */
int irq_register_handler(interrupt_controller_t *ic, interrupt_source_t source,
                         irq_handler_t handler, void *context);
int irq_unregister_handler(interrupt_controller_t *ic, interrupt_source_t source);

/* IRQ enable/disable */
int irq_enable(interrupt_controller_t *ic, interrupt_source_t source);
int irq_disable(interrupt_controller_t *ic, interrupt_source_t source);
void irq_enable_all(interrupt_controller_t *ic);
void irq_disable_all(interrupt_controller_t *ic);

/* Trigger and process interrupts */
void irq_trigger(interrupt_controller_t *ic, interrupt_source_t source);
int irq_process_pending(interrupt_controller_t *ic);

/* Check pending interrupts */
bool irq_is_pending(interrupt_controller_t *ic, interrupt_source_t source);
uint32_t irq_get_pending_mask(interrupt_controller_t *ic);
void irq_clear(interrupt_controller_t *ic, interrupt_source_t source);

/* Signal-based interrupt simulation */
int irq_setup_signal_handler(interrupt_controller_t *ic);
void irq_signal_handler(int signum);

#endif /* INTERRUPT_HANDLER_H */
