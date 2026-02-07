#ifndef SENSOR_ARRAY_H
#define SENSOR_ARRAY_H

#include "device_registers.h"
#include <stdbool.h>

#define SENSOR_COUNT        4
#define SENSOR_BUFFER_SIZE  16

/* Sensor types */
typedef enum {
    SENSOR_TYPE_POSITION,
    SENSOR_TYPE_VELOCITY,
    SENSOR_TYPE_TEMPERATURE,
    SENSOR_TYPE_CURRENT
} sensor_type_t;

/* Sensor states */
typedef enum {
    SENSOR_STATE_DISABLED,
    SENSOR_STATE_IDLE,
    SENSOR_STATE_SAMPLING,
    SENSOR_STATE_ERROR
} sensor_state_t;

/* Individual sensor */
typedef struct {
    sensor_type_t type;
    sensor_state_t state;
    int32_t value;
    int32_t min_value;
    int32_t max_value;
    uint32_t sample_count;
} sensor_t;

/* Sensor array context */
typedef struct {
    register_file_t *regs;
    sensor_t sensors[SENSOR_COUNT];
    int32_t buffer[SENSOR_BUFFER_SIZE];
    uint8_t buffer_head;
    uint8_t buffer_tail;
    bool continuous_mode;
} sensor_array_t;

/* Sensor array functions */
int sensor_array_init(sensor_array_t *sa, register_file_t *regs);
int sensor_array_enable(sensor_array_t *sa);
int sensor_array_disable(sensor_array_t *sa);
int sensor_array_trigger(sensor_array_t *sa);
int sensor_array_set_continuous(sensor_array_t *sa, bool enable);

/* Read sensor values */
int32_t sensor_read(sensor_array_t *sa, uint8_t sensor_id);
int sensor_read_all(sensor_array_t *sa, int32_t *values, uint8_t count);

/* Buffer operations */
int sensor_buffer_push(sensor_array_t *sa, int32_t value);
int sensor_buffer_pop(sensor_array_t *sa, int32_t *value);
int sensor_buffer_count(sensor_array_t *sa);
void sensor_buffer_clear(sensor_array_t *sa);

/* Status functions */
sensor_state_t sensor_get_state(sensor_array_t *sa, uint8_t sensor_id);
bool sensor_is_ready(sensor_array_t *sa);
bool sensor_has_error(sensor_array_t *sa);

/* Update function (call periodically) */
int sensor_array_update(sensor_array_t *sa);

/* Simulate sensor values for testing */
void sensor_set_simulated_value(sensor_array_t *sa, uint8_t sensor_id, int32_t value);

#endif /* SENSOR_ARRAY_H */
