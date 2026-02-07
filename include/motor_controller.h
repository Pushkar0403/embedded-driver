#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "device_registers.h"
#include <stdbool.h>

/* Motor states */
typedef enum {
    MOTOR_STATE_IDLE,
    MOTOR_STATE_STARTING,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_STOPPING,
    MOTOR_STATE_FAULT,
    MOTOR_STATE_RECOVERY
} motor_state_t;

/* Motor direction */
typedef enum {
    MOTOR_DIR_CCW = 0,
    MOTOR_DIR_CW = 1
} motor_direction_t;

/* Motor fault codes */
typedef enum {
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_STALL,
    MOTOR_FAULT_OVERHEAT,
    MOTOR_FAULT_OVERCURRENT
} motor_fault_t;

/* Motor controller context */
typedef struct {
    register_file_t *regs;
    motor_state_t state;
    motor_fault_t fault_code;
    uint32_t target_speed;
    uint32_t current_speed;
    int32_t position;
    motor_direction_t direction;
} motor_controller_t;

/* Motor controller functions */
int motor_init(motor_controller_t *mc, register_file_t *regs);
int motor_start(motor_controller_t *mc, uint32_t speed, motor_direction_t dir);
int motor_stop(motor_controller_t *mc);
int motor_brake(motor_controller_t *mc);
int motor_set_speed(motor_controller_t *mc, uint32_t speed);
int motor_reset(motor_controller_t *mc);

/* State machine update (call periodically) */
int motor_update(motor_controller_t *mc);

/* Status functions */
motor_state_t motor_get_state(motor_controller_t *mc);
motor_fault_t motor_get_fault(motor_controller_t *mc);
uint32_t motor_get_speed(motor_controller_t *mc);
int32_t motor_get_position(motor_controller_t *mc);
bool motor_is_running(motor_controller_t *mc);

/* Fault injection for testing */
void motor_inject_fault(motor_controller_t *mc, motor_fault_t fault);
int motor_clear_fault(motor_controller_t *mc);

#endif /* MOTOR_CONTROLLER_H */
