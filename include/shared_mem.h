#ifndef SHARED_MEM_H
#define SHARED_MEM_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#define SHM_NAME "/motor_driver_shm"
#define SHM_SIZE 4096

/* Command types for IPC */
typedef enum {
    CMD_NONE = 0,
    CMD_MOTOR_START,
    CMD_MOTOR_STOP,
    CMD_MOTOR_SET_SPEED,
    CMD_SENSOR_READ,
    CMD_GET_STATUS,
    CMD_RESET
} command_type_t;

/* Response status */
typedef enum {
    RESP_OK = 0,
    RESP_ERROR,
    RESP_BUSY,
    RESP_INVALID_CMD
} response_status_t;

/* Shared memory structure */
typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t cmd_ready;
    pthread_cond_t resp_ready;

    /* Command section */
    command_type_t cmd;
    uint32_t cmd_param1;
    uint32_t cmd_param2;
    bool cmd_pending;

    /* Response section */
    response_status_t resp_status;
    int32_t resp_data[8];
    bool resp_ready_flag;

    /* Status section */
    uint32_t motor_state;
    uint32_t motor_speed;
    int32_t motor_position;
    int32_t sensor_values[4];
    uint32_t fault_code;

    /* Control flags */
    bool shutdown_requested;
} shared_mem_t;

/* Shared memory functions */
shared_mem_t* shm_create(void);
shared_mem_t* shm_open_existing(void);
void shm_destroy(shared_mem_t *shm);
void shm_close(shared_mem_t *shm);

/* Thread-safe operations */
int shm_send_command(shared_mem_t *shm, command_type_t cmd,
                     uint32_t param1, uint32_t param2);
int shm_wait_response(shared_mem_t *shm, response_status_t *status,
                      int32_t *data, int data_count);
int shm_get_command(shared_mem_t *shm, command_type_t *cmd,
                    uint32_t *param1, uint32_t *param2);
int shm_try_get_command(shared_mem_t *shm, command_type_t *cmd,
                        uint32_t *param1, uint32_t *param2);
int shm_send_response(shared_mem_t *shm, response_status_t status,
                      int32_t *data, int data_count);

/* Status update */
int shm_update_status(shared_mem_t *shm, uint32_t motor_state,
                      uint32_t motor_speed, int32_t motor_position,
                      int32_t *sensor_values, uint32_t fault_code);

/* Utility */
bool shm_is_shutdown_requested(shared_mem_t *shm);
void shm_request_shutdown(shared_mem_t *shm);

#endif /* SHARED_MEM_H */
