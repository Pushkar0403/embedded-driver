#include "motor_controller.h"
#include <string.h>
#include <stdlib.h>

#define MAX_SPEED 10000
#define SPEED_RAMP_RATE 500

int motor_init(motor_controller_t *mc, register_file_t *regs) {
    if (!mc || !regs) return -1;

    memset(mc, 0, sizeof(motor_controller_t));
    mc->regs = regs;
    mc->state = MOTOR_STATE_IDLE;
    mc->fault_code = MOTOR_FAULT_NONE;

    /* Clear motor registers */
    reg_write(regs, REG_MOTOR_CTRL, 0);
    reg_write(regs, REG_MOTOR_STATUS, 0);
    reg_write(regs, REG_MOTOR_SPEED, 0);
    reg_write(regs, REG_MOTOR_POSITION, 0);

    return 0;
}

int motor_start(motor_controller_t *mc, uint32_t speed, motor_direction_t dir) {
    if (!mc) return -1;
    if (mc->state == MOTOR_STATE_FAULT) return -2;
    if (speed > MAX_SPEED) speed = MAX_SPEED;

    mc->target_speed = speed;
    mc->direction = dir;
    mc->state = MOTOR_STATE_STARTING;

    uint32_t ctrl = MOTOR_CTRL_ENABLE;
    if (dir == MOTOR_DIR_CW) {
        ctrl |= MOTOR_CTRL_DIR_CW;
    }
    reg_write(mc->regs, REG_MOTOR_CTRL, ctrl);

    return 0;
}

int motor_stop(motor_controller_t *mc) {
    if (!mc) return -1;
    if (mc->state == MOTOR_STATE_IDLE) return 0;

    mc->target_speed = 0;
    mc->state = MOTOR_STATE_STOPPING;

    reg_clear_bits(mc->regs, REG_MOTOR_CTRL, MOTOR_CTRL_ENABLE);

    return 0;
}

int motor_brake(motor_controller_t *mc) {
    if (!mc) return -1;

    mc->target_speed = 0;
    mc->current_speed = 0;
    mc->state = MOTOR_STATE_IDLE;

    reg_set_bits(mc->regs, REG_MOTOR_CTRL, MOTOR_CTRL_BRAKE);
    reg_clear_bits(mc->regs, REG_MOTOR_CTRL, MOTOR_CTRL_ENABLE);
    reg_write(mc->regs, REG_MOTOR_SPEED, 0);
    reg_clear_bits(mc->regs, REG_MOTOR_STATUS, MOTOR_STATUS_RUNNING);

    return 0;
}

int motor_set_speed(motor_controller_t *mc, uint32_t speed) {
    if (!mc) return -1;
    if (mc->state == MOTOR_STATE_FAULT) return -2;
    if (speed > MAX_SPEED) speed = MAX_SPEED;

    mc->target_speed = speed;
    return 0;
}

int motor_reset(motor_controller_t *mc) {
    if (!mc) return -1;

    reg_write(mc->regs, REG_MOTOR_CTRL, MOTOR_CTRL_RESET);
    reg_write(mc->regs, REG_MOTOR_STATUS, 0);
    reg_write(mc->regs, REG_MOTOR_SPEED, 0);

    mc->state = MOTOR_STATE_IDLE;
    mc->fault_code = MOTOR_FAULT_NONE;
    mc->current_speed = 0;
    mc->target_speed = 0;

    reg_clear_bits(mc->regs, REG_MOTOR_CTRL, MOTOR_CTRL_RESET);

    return 0;
}

int motor_update(motor_controller_t *mc) {
    if (!mc) return -1;

    /* Check for faults in status register */
    uint32_t status = reg_read(mc->regs, REG_MOTOR_STATUS);
    if (status & (MOTOR_STATUS_FAULT | MOTOR_STATUS_STALL | MOTOR_STATUS_OVERHEAT)) {
        if (mc->state != MOTOR_STATE_FAULT) {
            mc->state = MOTOR_STATE_FAULT;
            if (status & MOTOR_STATUS_STALL) mc->fault_code = MOTOR_FAULT_STALL;
            else if (status & MOTOR_STATUS_OVERHEAT) mc->fault_code = MOTOR_FAULT_OVERHEAT;
            else mc->fault_code = MOTOR_FAULT_OVERCURRENT;
        }
        return 0;
    }

    /* State machine */
    switch (mc->state) {
        case MOTOR_STATE_IDLE:
            /* Nothing to do */
            break;

        case MOTOR_STATE_STARTING:
            /* Ramp up speed */
            if (mc->current_speed < mc->target_speed) {
                mc->current_speed += SPEED_RAMP_RATE;
                if (mc->current_speed >= mc->target_speed) {
                    mc->current_speed = mc->target_speed;
                    mc->state = MOTOR_STATE_RUNNING;
                }
            } else {
                mc->state = MOTOR_STATE_RUNNING;
            }
            reg_write(mc->regs, REG_MOTOR_SPEED, mc->current_speed);
            reg_set_bits(mc->regs, REG_MOTOR_STATUS, MOTOR_STATUS_RUNNING);
            break;

        case MOTOR_STATE_RUNNING:
            /* Adjust speed if target changed */
            if (mc->current_speed < mc->target_speed) {
                mc->current_speed += SPEED_RAMP_RATE;
                if (mc->current_speed > mc->target_speed) {
                    mc->current_speed = mc->target_speed;
                }
            } else if (mc->current_speed > mc->target_speed) {
                mc->current_speed -= SPEED_RAMP_RATE;
                if (mc->current_speed < mc->target_speed) {
                    mc->current_speed = mc->target_speed;
                }
            }
            reg_write(mc->regs, REG_MOTOR_SPEED, mc->current_speed);

            /* Update position */
            if (mc->direction == MOTOR_DIR_CW) {
                mc->position += (int32_t)(mc->current_speed / 100);
            } else {
                mc->position -= (int32_t)(mc->current_speed / 100);
            }
            reg_write(mc->regs, REG_MOTOR_POSITION, (uint32_t)mc->position);
            break;

        case MOTOR_STATE_STOPPING:
            /* Ramp down speed */
            if (mc->current_speed > SPEED_RAMP_RATE) {
                mc->current_speed -= SPEED_RAMP_RATE;
            } else {
                mc->current_speed = 0;
                mc->state = MOTOR_STATE_IDLE;
                reg_clear_bits(mc->regs, REG_MOTOR_STATUS, MOTOR_STATUS_RUNNING);
            }
            reg_write(mc->regs, REG_MOTOR_SPEED, mc->current_speed);
            break;

        case MOTOR_STATE_FAULT:
            /* Stay in fault until reset */
            break;

        case MOTOR_STATE_RECOVERY:
            /* Transition back to idle after recovery */
            mc->state = MOTOR_STATE_IDLE;
            break;
    }

    return 0;
}

motor_state_t motor_get_state(motor_controller_t *mc) {
    return mc ? mc->state : MOTOR_STATE_FAULT;
}

motor_fault_t motor_get_fault(motor_controller_t *mc) {
    return mc ? mc->fault_code : MOTOR_FAULT_NONE;
}

uint32_t motor_get_speed(motor_controller_t *mc) {
    return mc ? mc->current_speed : 0;
}

int32_t motor_get_position(motor_controller_t *mc) {
    return mc ? mc->position : 0;
}

bool motor_is_running(motor_controller_t *mc) {
    if (!mc) return false;
    return (mc->state == MOTOR_STATE_RUNNING || mc->state == MOTOR_STATE_STARTING);
}

void motor_inject_fault(motor_controller_t *mc, motor_fault_t fault) {
    if (!mc) return;

    mc->fault_code = fault;
    mc->state = MOTOR_STATE_FAULT;

    switch (fault) {
        case MOTOR_FAULT_STALL:
            reg_set_bits(mc->regs, REG_MOTOR_STATUS, MOTOR_STATUS_STALL);
            break;
        case MOTOR_FAULT_OVERHEAT:
            reg_set_bits(mc->regs, REG_MOTOR_STATUS, MOTOR_STATUS_OVERHEAT);
            break;
        case MOTOR_FAULT_OVERCURRENT:
            reg_set_bits(mc->regs, REG_MOTOR_STATUS, MOTOR_STATUS_FAULT);
            break;
        default:
            break;
    }
}

int motor_clear_fault(motor_controller_t *mc) {
    if (!mc) return -1;
    if (mc->state != MOTOR_STATE_FAULT) return 0;

    mc->fault_code = MOTOR_FAULT_NONE;
    mc->state = MOTOR_STATE_RECOVERY;
    reg_write(mc->regs, REG_MOTOR_STATUS, 0);

    return 0;
}
