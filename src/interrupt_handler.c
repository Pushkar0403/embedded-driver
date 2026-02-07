#include "interrupt_handler.h"
#include <string.h>
#include <stdlib.h>

/* Global interrupt controller pointer for signal handler */
interrupt_controller_t *g_irq_controller = NULL;

int irq_init(interrupt_controller_t *ic, register_file_t *regs,
             motor_controller_t *motor, sensor_array_t *sensors) {
    if (!ic || !regs) return -1;

    memset(ic, 0, sizeof(interrupt_controller_t));
    ic->regs = regs;
    ic->motor = motor;
    ic->sensors = sensors;
    ic->enabled_irqs = 0;
    ic->pending_irqs = 0;
    ic->signal_received = 0;

    /* Clear IRQ registers */
    reg_write(regs, REG_IRQ_STATUS, 0);
    reg_write(regs, REG_IRQ_ENABLE, 0);

    /* Set global pointer for signal handler */
    g_irq_controller = ic;

    return 0;
}

void irq_cleanup(interrupt_controller_t *ic) {
    if (!ic) return;

    irq_disable_all(ic);
    g_irq_controller = NULL;

    /* Restore default signal handlers */
    signal(SIGUSR1, SIG_DFL);
    signal(SIGUSR2, SIG_DFL);
}

int irq_register_handler(interrupt_controller_t *ic, interrupt_source_t source,
                         irq_handler_t handler, void *context) {
    if (!ic || source >= INT_COUNT) return -1;

    ic->handlers[source] = handler;
    ic->handler_contexts[source] = context;

    return 0;
}

int irq_unregister_handler(interrupt_controller_t *ic, interrupt_source_t source) {
    if (!ic || source >= INT_COUNT) return -1;

    ic->handlers[source] = NULL;
    ic->handler_contexts[source] = NULL;

    return 0;
}

int irq_enable(interrupt_controller_t *ic, interrupt_source_t source) {
    if (!ic || source >= INT_COUNT) return -1;

    ic->enabled_irqs |= (1 << source);
    reg_set_bits(ic->regs, REG_IRQ_ENABLE, (1 << source));

    return 0;
}

int irq_disable(interrupt_controller_t *ic, interrupt_source_t source) {
    if (!ic || source >= INT_COUNT) return -1;

    ic->enabled_irqs &= ~(1 << source);
    reg_clear_bits(ic->regs, REG_IRQ_ENABLE, (1 << source));

    return 0;
}

void irq_enable_all(interrupt_controller_t *ic) {
    if (!ic) return;

    ic->enabled_irqs = (1 << INT_COUNT) - 1;
    reg_write(ic->regs, REG_IRQ_ENABLE, ic->enabled_irqs);
}

void irq_disable_all(interrupt_controller_t *ic) {
    if (!ic) return;

    ic->enabled_irqs = 0;
    reg_write(ic->regs, REG_IRQ_ENABLE, 0);
}

void irq_trigger(interrupt_controller_t *ic, interrupt_source_t source) {
    if (!ic || source >= INT_COUNT) return;

    /* Only trigger if enabled */
    if (ic->enabled_irqs & (1 << source)) {
        ic->pending_irqs |= (1 << source);
        reg_set_bits(ic->regs, REG_IRQ_STATUS, (1 << source));
    }
}

int irq_process_pending(interrupt_controller_t *ic) {
    if (!ic) return -1;

    int processed = 0;

    /* Handle signal-triggered interrupts */
    if (ic->signal_received) {
        ic->signal_received = 0;
        irq_trigger(ic, INT_TIMER);
    }

    /* Process all pending interrupts */
    for (int i = 0; i < INT_COUNT; i++) {
        if ((ic->pending_irqs & (1 << i)) && ic->handlers[i]) {
            ic->handlers[i]((interrupt_source_t)i, ic->handler_contexts[i]);
            processed++;
        }
    }

    /* Clear processed interrupts */
    ic->pending_irqs = 0;
    reg_write(ic->regs, REG_IRQ_STATUS, 0);

    return processed;
}

bool irq_is_pending(interrupt_controller_t *ic, interrupt_source_t source) {
    if (!ic || source >= INT_COUNT) return false;
    return (ic->pending_irqs & (1 << source)) != 0;
}

uint32_t irq_get_pending_mask(interrupt_controller_t *ic) {
    return ic ? ic->pending_irqs : 0;
}

void irq_clear(interrupt_controller_t *ic, interrupt_source_t source) {
    if (!ic || source >= INT_COUNT) return;

    ic->pending_irqs &= ~(1 << source);
    reg_clear_bits(ic->regs, REG_IRQ_STATUS, (1 << source));
}

void irq_signal_handler(int signum) {
    if (!g_irq_controller) return;

    /* Mark that we received a signal - actual processing happens in main loop */
    g_irq_controller->signal_received = 1;

    /* Map signals to interrupt sources */
    if (signum == SIGUSR1) {
        g_irq_controller->pending_irqs |= (1 << INT_MOTOR_FAULT);
    } else if (signum == SIGUSR2) {
        g_irq_controller->pending_irqs |= (1 << INT_SENSOR_READY);
    }
}

int irq_setup_signal_handler(interrupt_controller_t *ic) {
    if (!ic) return -1;

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = irq_signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;

    if (sigaction(SIGUSR1, &sa, NULL) < 0) return -1;
    if (sigaction(SIGUSR2, &sa, NULL) < 0) return -1;

    return 0;
}
