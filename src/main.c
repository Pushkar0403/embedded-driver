#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "device_registers.h"
#include "motor_controller.h"
#include "sensor_array.h"
#include "interrupt_handler.h"
#include "shared_mem.h"

/* Global running flag */
static volatile sig_atomic_t g_running = 1;

/* Signal handler for graceful shutdown */
static void shutdown_handler(int signum) {
    (void)signum;
    g_running = 0;
}

/* IRQ callback for motor faults */
static void motor_fault_handler(interrupt_source_t source, void *context) {
    (void)source;
    motor_controller_t *mc = (motor_controller_t*)context;
    printf("[IRQ] Motor fault detected: %d\n", motor_get_fault(mc));
}

/* IRQ callback for sensor ready */
static void sensor_ready_handler(interrupt_source_t source, void *context) {
    (void)source;
    sensor_array_t *sa = (sensor_array_t*)context;
    printf("[IRQ] Sensor data ready, buffer count: %d\n", sensor_buffer_count(sa));
}

/* Process commands from shared memory */
static void process_command(shared_mem_t *shm, motor_controller_t *mc,
                           sensor_array_t *sa, command_type_t cmd,
                           uint32_t param1, uint32_t param2) {
    int32_t resp_data[8] = {0};
    response_status_t status = RESP_OK;

    switch (cmd) {
        case CMD_MOTOR_START:
            if (motor_start(mc, param1, (motor_direction_t)param2) < 0) {
                status = RESP_ERROR;
            }
            break;

        case CMD_MOTOR_STOP:
            if (motor_stop(mc) < 0) {
                status = RESP_ERROR;
            }
            break;

        case CMD_MOTOR_SET_SPEED:
            if (motor_set_speed(mc, param1) < 0) {
                status = RESP_ERROR;
            }
            break;

        case CMD_SENSOR_READ:
            sensor_read_all(sa, resp_data, 4);
            break;

        case CMD_GET_STATUS:
            resp_data[0] = motor_get_state(mc);
            resp_data[1] = (int32_t)motor_get_speed(mc);
            resp_data[2] = motor_get_position(mc);
            resp_data[3] = motor_get_fault(mc);
            break;

        case CMD_RESET:
            motor_reset(mc);
            sensor_buffer_clear(sa);
            break;

        default:
            status = RESP_INVALID_CMD;
            break;
    }

    shm_send_response(shm, status, resp_data, 8);
}

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    printf("Embedded Motor Controller Driver\n");
    printf("================================\n\n");

    /* Setup shutdown signal handler */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = shutdown_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* Initialize components */
    register_file_t regs;
    motor_controller_t motor;
    sensor_array_t sensors;
    interrupt_controller_t irq;

    reg_init(&regs);
    motor_init(&motor, &regs);
    sensor_array_init(&sensors, &regs);
    irq_init(&irq, &regs, &motor, &sensors);

    /* Register interrupt handlers */
    irq_register_handler(&irq, INT_MOTOR_FAULT, motor_fault_handler, &motor);
    irq_register_handler(&irq, INT_MOTOR_STALL, motor_fault_handler, &motor);
    irq_register_handler(&irq, INT_SENSOR_READY, sensor_ready_handler, &sensors);

    /* Enable interrupts */
    irq_enable(&irq, INT_MOTOR_FAULT);
    irq_enable(&irq, INT_MOTOR_STALL);
    irq_enable(&irq, INT_SENSOR_READY);

    /* Setup signal-based interrupt handling */
    irq_setup_signal_handler(&irq);

    /* Create shared memory for IPC */
    shared_mem_t *shm = shm_create();
    if (!shm) {
        fprintf(stderr, "Failed to create shared memory\n");
        return 1;
    }

    /* Enable sensors */
    sensor_array_enable(&sensors);

    printf("Driver initialized. PID: %d\n", getpid());
    printf("Send SIGUSR1 for motor fault, SIGUSR2 for sensor interrupt\n");
    printf("Press Ctrl+C to exit\n\n");

    /* Demo: Start motor */
    printf("Starting motor at 5000 RPM clockwise...\n");
    motor_start(&motor, 5000, MOTOR_DIR_CW);

    /* Set some simulated sensor values */
    sensor_set_simulated_value(&sensors, 0, 100);   /* Position */
    sensor_set_simulated_value(&sensors, 1, 5000);  /* Velocity */
    sensor_set_simulated_value(&sensors, 2, 45);    /* Temperature */
    sensor_set_simulated_value(&sensors, 3, 2500);  /* Current */

    /* Main loop */
    int tick = 0;
    while (g_running && !shm_is_shutdown_requested(shm)) {
        /* Update motor state machine */
        motor_update(&motor);

        /* Trigger sensor sampling periodically */
        if (tick % 10 == 0) {
            sensor_array_trigger(&sensors);
        }
        sensor_array_update(&sensors);

        /* Check for and process interrupts */
        if (motor_get_state(&motor) == MOTOR_STATE_FAULT) {
            irq_trigger(&irq, INT_MOTOR_FAULT);
        }
        irq_process_pending(&irq);

        /* Update shared memory status */
        int32_t sensor_vals[4];
        sensor_read_all(&sensors, sensor_vals, 4);
        shm_update_status(shm, motor_get_state(&motor), motor_get_speed(&motor),
                         motor_get_position(&motor), sensor_vals, motor_get_fault(&motor));

        /* Check for commands (non-blocking check) */
        command_type_t cmd;
        uint32_t p1, p2;
        if (shm_get_command(shm, &cmd, &p1, &p2) == 0 && cmd != CMD_NONE) {
            process_command(shm, &motor, &sensors, cmd, p1, p2);
        }

        /* Print status every 50 ticks */
        if (tick % 50 == 0) {
            printf("Tick %d: State=%d Speed=%u Position=%d Temp=%d\n",
                   tick, motor_get_state(&motor), motor_get_speed(&motor),
                   motor_get_position(&motor), sensor_read(&sensors, 2));
        }

        usleep(10000);  /* 10ms */
        tick++;
    }

    printf("\nShutting down...\n");

    /* Cleanup */
    motor_stop(&motor);
    while (motor_is_running(&motor)) {
        motor_update(&motor);
        usleep(10000);
    }

    sensor_array_disable(&sensors);
    irq_cleanup(&irq);
    shm_destroy(shm);

    printf("Driver stopped.\n");
    return 0;
}
