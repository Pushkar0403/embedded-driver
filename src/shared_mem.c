#include "shared_mem.h"
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef _WIN32
/* Windows implementation using heap memory for simplicity */
static shared_mem_t *g_shared_mem = NULL;

shared_mem_t* shm_create(void) {
    if (g_shared_mem) {
        return g_shared_mem;
    }

    g_shared_mem = (shared_mem_t*)calloc(1, sizeof(shared_mem_t));
    if (!g_shared_mem) return NULL;

    pthread_mutexattr_t mutex_attr;
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutex_init(&g_shared_mem->mutex, &mutex_attr);
    pthread_mutexattr_destroy(&mutex_attr);

    pthread_condattr_t cond_attr;
    pthread_condattr_init(&cond_attr);
    pthread_cond_init(&g_shared_mem->cmd_ready, &cond_attr);
    pthread_cond_init(&g_shared_mem->resp_ready, &cond_attr);
    pthread_condattr_destroy(&cond_attr);

    return g_shared_mem;
}

shared_mem_t* shm_open_existing(void) {
    return g_shared_mem;
}

void shm_destroy(shared_mem_t *shm) {
    if (!shm || shm != g_shared_mem) return;

    pthread_mutex_destroy(&shm->mutex);
    pthread_cond_destroy(&shm->cmd_ready);
    pthread_cond_destroy(&shm->resp_ready);

    free(shm);
    g_shared_mem = NULL;
}

void shm_close(shared_mem_t *shm) {
    (void)shm;  /* Nothing to do for heap-based implementation */
}

#else
/* POSIX implementation */
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

shared_mem_t* shm_create(void) {
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) return NULL;

    if (ftruncate(fd, sizeof(shared_mem_t)) < 0) {
        close(fd);
        shm_unlink(SHM_NAME);
        return NULL;
    }

    shared_mem_t *shm = mmap(NULL, sizeof(shared_mem_t),
                             PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (shm == MAP_FAILED) {
        shm_unlink(SHM_NAME);
        return NULL;
    }

    /* Initialize mutex with process-shared attribute */
    pthread_mutexattr_t mutex_attr;
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&shm->mutex, &mutex_attr);
    pthread_mutexattr_destroy(&mutex_attr);

    /* Initialize condition variables */
    pthread_condattr_t cond_attr;
    pthread_condattr_init(&cond_attr);
    pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
    pthread_cond_init(&shm->cmd_ready, &cond_attr);
    pthread_cond_init(&shm->resp_ready, &cond_attr);
    pthread_condattr_destroy(&cond_attr);

    shm->cmd = CMD_NONE;
    shm->cmd_pending = false;
    shm->resp_ready_flag = false;
    shm->shutdown_requested = false;

    return shm;
}

shared_mem_t* shm_open_existing(void) {
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd < 0) return NULL;

    shared_mem_t *shm = mmap(NULL, sizeof(shared_mem_t),
                             PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (shm == MAP_FAILED) return NULL;

    return shm;
}

void shm_destroy(shared_mem_t *shm) {
    if (!shm) return;

    pthread_mutex_destroy(&shm->mutex);
    pthread_cond_destroy(&shm->cmd_ready);
    pthread_cond_destroy(&shm->resp_ready);

    munmap(shm, sizeof(shared_mem_t));
    shm_unlink(SHM_NAME);
}

void shm_close(shared_mem_t *shm) {
    if (shm) {
        munmap(shm, sizeof(shared_mem_t));
    }
}
#endif

int shm_send_command(shared_mem_t *shm, command_type_t cmd,
                     uint32_t param1, uint32_t param2) {
    if (!shm) return -1;

    pthread_mutex_lock(&shm->mutex);

    /* Wait for previous command to be processed */
    while (shm->cmd_pending) {
        pthread_cond_wait(&shm->resp_ready, &shm->mutex);
    }

    shm->cmd = cmd;
    shm->cmd_param1 = param1;
    shm->cmd_param2 = param2;
    shm->cmd_pending = true;
    shm->resp_ready_flag = false;

    pthread_cond_signal(&shm->cmd_ready);
    pthread_mutex_unlock(&shm->mutex);

    return 0;
}

int shm_wait_response(shared_mem_t *shm, response_status_t *status,
                      int32_t *data, int data_count) {
    if (!shm) return -1;

    pthread_mutex_lock(&shm->mutex);

    while (!shm->resp_ready_flag) {
        pthread_cond_wait(&shm->resp_ready, &shm->mutex);
    }

    if (status) *status = shm->resp_status;
    if (data && data_count > 0) {
        int count = data_count > 8 ? 8 : data_count;
        memcpy(data, shm->resp_data, count * sizeof(int32_t));
    }

    shm->resp_ready_flag = false;
    pthread_cond_signal(&shm->resp_ready);

    pthread_mutex_unlock(&shm->mutex);

    return 0;
}

int shm_get_command(shared_mem_t *shm, command_type_t *cmd,
                    uint32_t *param1, uint32_t *param2) {
    if (!shm) return -1;

    pthread_mutex_lock(&shm->mutex);

    while (!shm->cmd_pending && !shm->shutdown_requested) {
        pthread_cond_wait(&shm->cmd_ready, &shm->mutex);
    }

    if (shm->shutdown_requested) {
        pthread_mutex_unlock(&shm->mutex);
        return -2;
    }

    if (cmd) *cmd = shm->cmd;
    if (param1) *param1 = shm->cmd_param1;
    if (param2) *param2 = shm->cmd_param2;

    pthread_mutex_unlock(&shm->mutex);

    return 0;
}

int shm_send_response(shared_mem_t *shm, response_status_t status,
                      int32_t *data, int data_count) {
    if (!shm) return -1;

    pthread_mutex_lock(&shm->mutex);

    shm->resp_status = status;
    if (data && data_count > 0) {
        int count = data_count > 8 ? 8 : data_count;
        memcpy(shm->resp_data, data, count * sizeof(int32_t));
    }

    shm->cmd_pending = false;
    shm->resp_ready_flag = true;
    pthread_cond_broadcast(&shm->resp_ready);

    pthread_mutex_unlock(&shm->mutex);

    return 0;
}

int shm_update_status(shared_mem_t *shm, uint32_t motor_state,
                      uint32_t motor_speed, int32_t motor_position,
                      int32_t *sensor_values, uint32_t fault_code) {
    if (!shm) return -1;

    pthread_mutex_lock(&shm->mutex);

    shm->motor_state = motor_state;
    shm->motor_speed = motor_speed;
    shm->motor_position = motor_position;
    shm->fault_code = fault_code;

    if (sensor_values) {
        memcpy(shm->sensor_values, sensor_values, 4 * sizeof(int32_t));
    }

    pthread_mutex_unlock(&shm->mutex);

    return 0;
}

bool shm_is_shutdown_requested(shared_mem_t *shm) {
    if (!shm) return true;

    pthread_mutex_lock(&shm->mutex);
    bool shutdown = shm->shutdown_requested;
    pthread_mutex_unlock(&shm->mutex);

    return shutdown;
}

void shm_request_shutdown(shared_mem_t *shm) {
    if (!shm) return;

    pthread_mutex_lock(&shm->mutex);
    shm->shutdown_requested = true;
    pthread_cond_broadcast(&shm->cmd_ready);
    pthread_cond_broadcast(&shm->resp_ready);
    pthread_mutex_unlock(&shm->mutex);
}
