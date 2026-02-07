#include "sensor_array.h"
#include <string.h>
#include <stdlib.h>

int sensor_array_init(sensor_array_t *sa, register_file_t *regs) {
    if (!sa || !regs) return -1;

    memset(sa, 0, sizeof(sensor_array_t));
    sa->regs = regs;

    /* Initialize sensors with default config */
    sa->sensors[0].type = SENSOR_TYPE_POSITION;
    sa->sensors[0].min_value = -10000;
    sa->sensors[0].max_value = 10000;

    sa->sensors[1].type = SENSOR_TYPE_VELOCITY;
    sa->sensors[1].min_value = 0;
    sa->sensors[1].max_value = 10000;

    sa->sensors[2].type = SENSOR_TYPE_TEMPERATURE;
    sa->sensors[2].min_value = -40;
    sa->sensors[2].max_value = 125;

    sa->sensors[3].type = SENSOR_TYPE_CURRENT;
    sa->sensors[3].min_value = 0;
    sa->sensors[3].max_value = 5000;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        sa->sensors[i].state = SENSOR_STATE_DISABLED;
    }

    /* Clear sensor registers */
    reg_write(regs, REG_SENSOR_CTRL, 0);
    reg_write(regs, REG_SENSOR_DATA, 0);
    reg_write(regs, REG_SENSOR_STATUS, 0);

    return 0;
}

int sensor_array_enable(sensor_array_t *sa) {
    if (!sa) return -1;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        sa->sensors[i].state = SENSOR_STATE_IDLE;
    }

    reg_set_bits(sa->regs, REG_SENSOR_CTRL, SENSOR_CTRL_ENABLE);
    reg_set_bits(sa->regs, REG_SENSOR_STATUS, SENSOR_STATUS_READY);

    return 0;
}

int sensor_array_disable(sensor_array_t *sa) {
    if (!sa) return -1;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        sa->sensors[i].state = SENSOR_STATE_DISABLED;
    }

    reg_clear_bits(sa->regs, REG_SENSOR_CTRL, SENSOR_CTRL_ENABLE);
    reg_clear_bits(sa->regs, REG_SENSOR_STATUS, SENSOR_STATUS_READY);

    return 0;
}

int sensor_array_trigger(sensor_array_t *sa) {
    if (!sa) return -1;
    if (!(reg_read(sa->regs, REG_SENSOR_CTRL) & SENSOR_CTRL_ENABLE)) return -2;

    reg_set_bits(sa->regs, REG_SENSOR_CTRL, SENSOR_CTRL_TRIGGER);

    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sa->sensors[i].state == SENSOR_STATE_IDLE) {
            sa->sensors[i].state = SENSOR_STATE_SAMPLING;
            sa->sensors[i].sample_count++;
        }
    }

    return 0;
}

int sensor_array_set_continuous(sensor_array_t *sa, bool enable) {
    if (!sa) return -1;

    sa->continuous_mode = enable;
    if (enable) {
        reg_set_bits(sa->regs, REG_SENSOR_CTRL, SENSOR_CTRL_CONTINUOUS);
    } else {
        reg_clear_bits(sa->regs, REG_SENSOR_CTRL, SENSOR_CTRL_CONTINUOUS);
    }

    return 0;
}

int32_t sensor_read(sensor_array_t *sa, uint8_t sensor_id) {
    if (!sa || sensor_id >= SENSOR_COUNT) return 0;
    return sa->sensors[sensor_id].value;
}

int sensor_read_all(sensor_array_t *sa, int32_t *values, uint8_t count) {
    if (!sa || !values) return -1;
    if (count > SENSOR_COUNT) count = SENSOR_COUNT;

    for (uint8_t i = 0; i < count; i++) {
        values[i] = sa->sensors[i].value;
    }

    return count;
}

int sensor_buffer_push(sensor_array_t *sa, int32_t value) {
    if (!sa) return -1;

    uint8_t next = (sa->buffer_head + 1) % SENSOR_BUFFER_SIZE;
    if (next == sa->buffer_tail) {
        reg_set_bits(sa->regs, REG_SENSOR_STATUS, SENSOR_STATUS_OVERFLOW);
        return -2;  /* Buffer full */
    }

    sa->buffer[sa->buffer_head] = value;
    sa->buffer_head = next;

    return 0;
}

int sensor_buffer_pop(sensor_array_t *sa, int32_t *value) {
    if (!sa || !value) return -1;

    if (sa->buffer_head == sa->buffer_tail) {
        return -2;  /* Buffer empty */
    }

    *value = sa->buffer[sa->buffer_tail];
    sa->buffer_tail = (sa->buffer_tail + 1) % SENSOR_BUFFER_SIZE;

    return 0;
}

int sensor_buffer_count(sensor_array_t *sa) {
    if (!sa) return 0;

    if (sa->buffer_head >= sa->buffer_tail) {
        return sa->buffer_head - sa->buffer_tail;
    }
    return SENSOR_BUFFER_SIZE - sa->buffer_tail + sa->buffer_head;
}

void sensor_buffer_clear(sensor_array_t *sa) {
    if (!sa) return;
    sa->buffer_head = 0;
    sa->buffer_tail = 0;
    reg_clear_bits(sa->regs, REG_SENSOR_STATUS, SENSOR_STATUS_OVERFLOW);
}

sensor_state_t sensor_get_state(sensor_array_t *sa, uint8_t sensor_id) {
    if (!sa || sensor_id >= SENSOR_COUNT) return SENSOR_STATE_ERROR;
    return sa->sensors[sensor_id].state;
}

bool sensor_is_ready(sensor_array_t *sa) {
    if (!sa) return false;
    return (reg_read(sa->regs, REG_SENSOR_STATUS) & SENSOR_STATUS_READY) != 0;
}

bool sensor_has_error(sensor_array_t *sa) {
    if (!sa) return true;
    return (reg_read(sa->regs, REG_SENSOR_STATUS) & SENSOR_STATUS_ERROR) != 0;
}

int sensor_array_update(sensor_array_t *sa) {
    if (!sa) return -1;

    /* Process sensors in sampling state */
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sa->sensors[i].state == SENSOR_STATE_SAMPLING) {
            /* Simulate sampling complete */
            sa->sensors[i].state = SENSOR_STATE_IDLE;

            /* Clamp value to valid range */
            if (sa->sensors[i].value < sa->sensors[i].min_value) {
                sa->sensors[i].value = sa->sensors[i].min_value;
            }
            if (sa->sensors[i].value > sa->sensors[i].max_value) {
                sa->sensors[i].value = sa->sensors[i].max_value;
            }

            /* Push to buffer if in continuous mode */
            if (sa->continuous_mode) {
                sensor_buffer_push(sa, sa->sensors[i].value);
            }
        }
    }

    /* Clear trigger bit after processing */
    reg_clear_bits(sa->regs, REG_SENSOR_CTRL, SENSOR_CTRL_TRIGGER);

    /* Auto-trigger in continuous mode */
    if (sa->continuous_mode && sensor_is_ready(sa)) {
        sensor_array_trigger(sa);
    }

    return 0;
}

void sensor_set_simulated_value(sensor_array_t *sa, uint8_t sensor_id, int32_t value) {
    if (!sa || sensor_id >= SENSOR_COUNT) return;
    sa->sensors[sensor_id].value = value;
}
