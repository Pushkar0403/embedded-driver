#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "device_registers.h"
#include "motor_controller.h"
#include "sensor_array.h"
#include "interrupt_handler.h"
#include "shared_mem.h"

/* Test framework macros */
#define TEST_PASS 0
#define TEST_FAIL 1

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        printf("  FAIL: %s:%d - %s\n", __FILE__, __LINE__, #cond); \
        return TEST_FAIL; \
    } \
} while(0)

#define ASSERT_EQ(a, b) do { \
    if ((a) != (b)) { \
        printf("  FAIL: %s:%d - %s != %s (%d != %d)\n", \
               __FILE__, __LINE__, #a, #b, (int)(a), (int)(b)); \
        return TEST_FAIL; \
    } \
} while(0)

#define ASSERT_NE(a, b) do { \
    if ((a) == (b)) { \
        printf("  FAIL: %s:%d - %s == %s\n", __FILE__, __LINE__, #a, #b); \
        return TEST_FAIL; \
    } \
} while(0)

/* Test result tracking */
static int tests_run = 0;
static int tests_passed = 0;

/* IRQ callback counter for testing */
static int irq_callback_count = 0;
static void test_irq_callback(interrupt_source_t source, void *context) {
    (void)source;
    (void)context;
    irq_callback_count++;
}

/*============================================================================
 * REGISTER TESTS (4 tests)
 *===========================================================================*/

int test_reg_init(void) {
    register_file_t rf;
    reg_init(&rf);

    for (int i = 0; i < REGISTER_FILE_SIZE / 4; i++) {
        ASSERT_EQ(rf.regs[i], 0);
    }
    return TEST_PASS;
}

int test_reg_read_write(void) {
    register_file_t rf;
    reg_init(&rf);

    reg_write(&rf, REG_MOTOR_CTRL, 0xDEADBEEF);
    ASSERT_EQ(reg_read(&rf, REG_MOTOR_CTRL), 0xDEADBEEF);

    reg_write(&rf, REG_SENSOR_DATA, 0x12345678);
    ASSERT_EQ(reg_read(&rf, REG_SENSOR_DATA), 0x12345678);

    return TEST_PASS;
}

int test_reg_set_clear_bits(void) {
    register_file_t rf;
    reg_init(&rf);

    reg_set_bits(&rf, REG_MOTOR_CTRL, MOTOR_CTRL_ENABLE);
    ASSERT_EQ(reg_read(&rf, REG_MOTOR_CTRL), MOTOR_CTRL_ENABLE);

    reg_set_bits(&rf, REG_MOTOR_CTRL, MOTOR_CTRL_DIR_CW);
    ASSERT_EQ(reg_read(&rf, REG_MOTOR_CTRL), MOTOR_CTRL_ENABLE | MOTOR_CTRL_DIR_CW);

    reg_clear_bits(&rf, REG_MOTOR_CTRL, MOTOR_CTRL_ENABLE);
    ASSERT_EQ(reg_read(&rf, REG_MOTOR_CTRL), MOTOR_CTRL_DIR_CW);

    return TEST_PASS;
}

int test_reg_invalid_offset(void) {
    register_file_t rf;
    reg_init(&rf);

    /* Reading invalid offset should return 0xFFFFFFFF */
    ASSERT_EQ(reg_read(&rf, 0xFF00), 0xFFFFFFFF);

    /* Writing to invalid offset should not crash */
    reg_write(&rf, 0xFF00, 0x12345678);

    return TEST_PASS;
}

/*============================================================================
 * MOTOR CONTROLLER TESTS (12 tests)
 *===========================================================================*/

int test_motor_init(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    ASSERT_EQ(motor_init(&mc, &rf), 0);
    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_IDLE);
    ASSERT_EQ(motor_get_speed(&mc), 0);
    ASSERT_EQ(motor_get_fault(&mc), MOTOR_FAULT_NONE);

    return TEST_PASS;
}

int test_motor_start(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    motor_init(&mc, &rf);

    ASSERT_EQ(motor_start(&mc, 5000, MOTOR_DIR_CW), 0);
    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_STARTING);
    ASSERT_TRUE(reg_read(&rf, REG_MOTOR_CTRL) & MOTOR_CTRL_ENABLE);

    return TEST_PASS;
}

int test_motor_stop(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    motor_init(&mc, &rf);
    motor_start(&mc, 5000, MOTOR_DIR_CW);

    /* Run until running */
    for (int i = 0; i < 20; i++) motor_update(&mc);
    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_RUNNING);

    motor_stop(&mc);
    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_STOPPING);

    /* Run until stopped */
    for (int i = 0; i < 20; i++) motor_update(&mc);
    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_IDLE);

    return TEST_PASS;
}

int test_motor_brake(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    motor_init(&mc, &rf);
    motor_start(&mc, 5000, MOTOR_DIR_CW);

    for (int i = 0; i < 20; i++) motor_update(&mc);

    motor_brake(&mc);
    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_IDLE);
    ASSERT_EQ(motor_get_speed(&mc), 0);
    ASSERT_TRUE(reg_read(&rf, REG_MOTOR_CTRL) & MOTOR_CTRL_BRAKE);

    return TEST_PASS;
}

int test_motor_speed_ramp(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    motor_init(&mc, &rf);
    motor_start(&mc, 5000, MOTOR_DIR_CW);

    uint32_t prev_speed = 0;
    for (int i = 0; i < 20; i++) {
        motor_update(&mc);
        uint32_t curr_speed = motor_get_speed(&mc);
        ASSERT_TRUE(curr_speed >= prev_speed);  /* Speed should increase */
        prev_speed = curr_speed;
    }

    ASSERT_EQ(motor_get_speed(&mc), 5000);

    return TEST_PASS;
}

int test_motor_direction(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    motor_init(&mc, &rf);

    /* Test CW direction */
    motor_start(&mc, 1000, MOTOR_DIR_CW);
    ASSERT_TRUE(reg_read(&rf, REG_MOTOR_CTRL) & MOTOR_CTRL_DIR_CW);
    motor_stop(&mc);
    for (int i = 0; i < 10; i++) motor_update(&mc);

    /* Test CCW direction */
    motor_start(&mc, 1000, MOTOR_DIR_CCW);
    ASSERT_TRUE((reg_read(&rf, REG_MOTOR_CTRL) & MOTOR_CTRL_DIR_CW) == 0);

    return TEST_PASS;
}

int test_motor_position_update(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    motor_init(&mc, &rf);

    motor_start(&mc, 1000, MOTOR_DIR_CW);
    for (int i = 0; i < 20; i++) motor_update(&mc);

    int32_t pos_cw = motor_get_position(&mc);
    ASSERT_TRUE(pos_cw > 0);  /* CW should increase position */

    motor_reset(&mc);
    motor_start(&mc, 1000, MOTOR_DIR_CCW);
    for (int i = 0; i < 20; i++) motor_update(&mc);

    int32_t pos_ccw = motor_get_position(&mc);
    ASSERT_TRUE(pos_ccw < 0);  /* CCW should decrease position */

    return TEST_PASS;
}

int test_motor_fault_stall(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    motor_init(&mc, &rf);
    motor_start(&mc, 5000, MOTOR_DIR_CW);

    motor_inject_fault(&mc, MOTOR_FAULT_STALL);

    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_FAULT);
    ASSERT_EQ(motor_get_fault(&mc), MOTOR_FAULT_STALL);
    ASSERT_TRUE(reg_read(&rf, REG_MOTOR_STATUS) & MOTOR_STATUS_STALL);

    return TEST_PASS;
}

int test_motor_fault_overheat(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    motor_init(&mc, &rf);

    motor_inject_fault(&mc, MOTOR_FAULT_OVERHEAT);

    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_FAULT);
    ASSERT_EQ(motor_get_fault(&mc), MOTOR_FAULT_OVERHEAT);
    ASSERT_TRUE(reg_read(&rf, REG_MOTOR_STATUS) & MOTOR_STATUS_OVERHEAT);

    return TEST_PASS;
}

int test_motor_fault_recovery(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    motor_init(&mc, &rf);

    motor_inject_fault(&mc, MOTOR_FAULT_STALL);
    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_FAULT);

    motor_clear_fault(&mc);
    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_RECOVERY);
    ASSERT_EQ(motor_get_fault(&mc), MOTOR_FAULT_NONE);

    motor_update(&mc);
    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_IDLE);

    return TEST_PASS;
}

int test_motor_reset(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    motor_init(&mc, &rf);

    motor_start(&mc, 5000, MOTOR_DIR_CW);
    for (int i = 0; i < 20; i++) motor_update(&mc);

    motor_reset(&mc);

    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_IDLE);
    ASSERT_EQ(motor_get_speed(&mc), 0);
    ASSERT_EQ(motor_get_fault(&mc), MOTOR_FAULT_NONE);

    return TEST_PASS;
}

int test_motor_max_speed(void) {
    register_file_t rf;
    motor_controller_t mc;

    reg_init(&rf);
    motor_init(&mc, &rf);

    /* Try to set speed above max */
    motor_start(&mc, 99999, MOTOR_DIR_CW);
    for (int i = 0; i < 50; i++) motor_update(&mc);

    /* Should be clamped to 10000 */
    ASSERT_EQ(motor_get_speed(&mc), 10000);

    return TEST_PASS;
}

/*============================================================================
 * SENSOR ARRAY TESTS (10 tests)
 *===========================================================================*/

int test_sensor_init(void) {
    register_file_t rf;
    sensor_array_t sa;

    reg_init(&rf);
    ASSERT_EQ(sensor_array_init(&sa, &rf), 0);

    for (int i = 0; i < SENSOR_COUNT; i++) {
        ASSERT_EQ(sensor_get_state(&sa, i), SENSOR_STATE_DISABLED);
    }

    return TEST_PASS;
}

int test_sensor_enable_disable(void) {
    register_file_t rf;
    sensor_array_t sa;

    reg_init(&rf);
    sensor_array_init(&sa, &rf);

    sensor_array_enable(&sa);
    ASSERT_TRUE(sensor_is_ready(&sa));
    ASSERT_TRUE(reg_read(&rf, REG_SENSOR_CTRL) & SENSOR_CTRL_ENABLE);

    sensor_array_disable(&sa);
    ASSERT_TRUE(!sensor_is_ready(&sa));

    return TEST_PASS;
}

int test_sensor_trigger(void) {
    register_file_t rf;
    sensor_array_t sa;

    reg_init(&rf);
    sensor_array_init(&sa, &rf);
    sensor_array_enable(&sa);

    ASSERT_EQ(sensor_array_trigger(&sa), 0);
    ASSERT_TRUE(reg_read(&rf, REG_SENSOR_CTRL) & SENSOR_CTRL_TRIGGER);

    return TEST_PASS;
}

int test_sensor_read(void) {
    register_file_t rf;
    sensor_array_t sa;

    reg_init(&rf);
    sensor_array_init(&sa, &rf);
    sensor_array_enable(&sa);

    sensor_set_simulated_value(&sa, 0, 1234);
    sensor_set_simulated_value(&sa, 1, 5678);

    ASSERT_EQ(sensor_read(&sa, 0), 1234);
    ASSERT_EQ(sensor_read(&sa, 1), 5678);

    return TEST_PASS;
}

int test_sensor_read_all(void) {
    register_file_t rf;
    sensor_array_t sa;
    int32_t values[4];

    reg_init(&rf);
    sensor_array_init(&sa, &rf);
    sensor_array_enable(&sa);

    sensor_set_simulated_value(&sa, 0, 100);
    sensor_set_simulated_value(&sa, 1, 200);
    sensor_set_simulated_value(&sa, 2, 300);
    sensor_set_simulated_value(&sa, 3, 400);

    ASSERT_EQ(sensor_read_all(&sa, values, 4), 4);
    ASSERT_EQ(values[0], 100);
    ASSERT_EQ(values[1], 200);
    ASSERT_EQ(values[2], 300);
    ASSERT_EQ(values[3], 400);

    return TEST_PASS;
}

int test_sensor_continuous_mode(void) {
    register_file_t rf;
    sensor_array_t sa;

    reg_init(&rf);
    sensor_array_init(&sa, &rf);
    sensor_array_enable(&sa);

    sensor_array_set_continuous(&sa, true);
    ASSERT_TRUE(reg_read(&rf, REG_SENSOR_CTRL) & SENSOR_CTRL_CONTINUOUS);

    sensor_array_set_continuous(&sa, false);
    ASSERT_TRUE((reg_read(&rf, REG_SENSOR_CTRL) & SENSOR_CTRL_CONTINUOUS) == 0);

    return TEST_PASS;
}

int test_sensor_buffer_push_pop(void) {
    register_file_t rf;
    sensor_array_t sa;
    int32_t value;

    reg_init(&rf);
    sensor_array_init(&sa, &rf);

    ASSERT_EQ(sensor_buffer_push(&sa, 111), 0);
    ASSERT_EQ(sensor_buffer_push(&sa, 222), 0);
    ASSERT_EQ(sensor_buffer_push(&sa, 333), 0);

    ASSERT_EQ(sensor_buffer_count(&sa), 3);

    ASSERT_EQ(sensor_buffer_pop(&sa, &value), 0);
    ASSERT_EQ(value, 111);
    ASSERT_EQ(sensor_buffer_pop(&sa, &value), 0);
    ASSERT_EQ(value, 222);

    return TEST_PASS;
}

int test_sensor_buffer_overflow(void) {
    register_file_t rf;
    sensor_array_t sa;

    reg_init(&rf);
    sensor_array_init(&sa, &rf);

    /* Fill buffer to capacity */
    for (int i = 0; i < SENSOR_BUFFER_SIZE - 1; i++) {
        ASSERT_EQ(sensor_buffer_push(&sa, i), 0);
    }

    /* Next push should fail with overflow */
    ASSERT_EQ(sensor_buffer_push(&sa, 999), -2);
    ASSERT_TRUE(reg_read(&rf, REG_SENSOR_STATUS) & SENSOR_STATUS_OVERFLOW);

    return TEST_PASS;
}

int test_sensor_buffer_clear(void) {
    register_file_t rf;
    sensor_array_t sa;

    reg_init(&rf);
    sensor_array_init(&sa, &rf);

    sensor_buffer_push(&sa, 100);
    sensor_buffer_push(&sa, 200);
    ASSERT_EQ(sensor_buffer_count(&sa), 2);

    sensor_buffer_clear(&sa);
    ASSERT_EQ(sensor_buffer_count(&sa), 0);

    return TEST_PASS;
}

int test_sensor_value_clamping(void) {
    register_file_t rf;
    sensor_array_t sa;

    reg_init(&rf);
    sensor_array_init(&sa, &rf);
    sensor_array_enable(&sa);

    /* Set value beyond max for temperature sensor (max 125) */
    sensor_set_simulated_value(&sa, 2, 9999);
    sensor_array_trigger(&sa);
    sensor_array_update(&sa);

    ASSERT_EQ(sensor_read(&sa, 2), 125);  /* Clamped to max */

    return TEST_PASS;
}

/*============================================================================
 * INTERRUPT HANDLER TESTS (6 tests)
 *===========================================================================*/

int test_irq_init(void) {
    register_file_t rf;
    motor_controller_t mc;
    sensor_array_t sa;
    interrupt_controller_t ic;

    reg_init(&rf);
    motor_init(&mc, &rf);
    sensor_array_init(&sa, &rf);

    ASSERT_EQ(irq_init(&ic, &rf, &mc, &sa), 0);
    ASSERT_EQ(irq_get_pending_mask(&ic), 0);

    irq_cleanup(&ic);
    return TEST_PASS;
}

int test_irq_enable_disable(void) {
    register_file_t rf;
    motor_controller_t mc;
    sensor_array_t sa;
    interrupt_controller_t ic;

    reg_init(&rf);
    motor_init(&mc, &rf);
    sensor_array_init(&sa, &rf);
    irq_init(&ic, &rf, &mc, &sa);

    irq_enable(&ic, INT_MOTOR_FAULT);
    ASSERT_TRUE(reg_read(&rf, REG_IRQ_ENABLE) & (1 << INT_MOTOR_FAULT));

    irq_disable(&ic, INT_MOTOR_FAULT);
    ASSERT_TRUE((reg_read(&rf, REG_IRQ_ENABLE) & (1 << INT_MOTOR_FAULT)) == 0);

    irq_cleanup(&ic);
    return TEST_PASS;
}

int test_irq_trigger(void) {
    register_file_t rf;
    motor_controller_t mc;
    sensor_array_t sa;
    interrupt_controller_t ic;

    reg_init(&rf);
    motor_init(&mc, &rf);
    sensor_array_init(&sa, &rf);
    irq_init(&ic, &rf, &mc, &sa);

    irq_enable(&ic, INT_SENSOR_READY);
    irq_trigger(&ic, INT_SENSOR_READY);

    ASSERT_TRUE(irq_is_pending(&ic, INT_SENSOR_READY));
    ASSERT_TRUE(reg_read(&rf, REG_IRQ_STATUS) & (1 << INT_SENSOR_READY));

    irq_cleanup(&ic);
    return TEST_PASS;
}

int test_irq_handler_callback(void) {
    register_file_t rf;
    motor_controller_t mc;
    sensor_array_t sa;
    interrupt_controller_t ic;

    reg_init(&rf);
    motor_init(&mc, &rf);
    sensor_array_init(&sa, &rf);
    irq_init(&ic, &rf, &mc, &sa);

    irq_callback_count = 0;
    irq_register_handler(&ic, INT_MOTOR_FAULT, test_irq_callback, NULL);
    irq_enable(&ic, INT_MOTOR_FAULT);
    irq_trigger(&ic, INT_MOTOR_FAULT);

    irq_process_pending(&ic);
    ASSERT_EQ(irq_callback_count, 1);

    irq_cleanup(&ic);
    return TEST_PASS;
}

int test_irq_pending_mask(void) {
    register_file_t rf;
    motor_controller_t mc;
    sensor_array_t sa;
    interrupt_controller_t ic;

    reg_init(&rf);
    motor_init(&mc, &rf);
    sensor_array_init(&sa, &rf);
    irq_init(&ic, &rf, &mc, &sa);

    irq_enable_all(&ic);
    irq_trigger(&ic, INT_MOTOR_FAULT);
    irq_trigger(&ic, INT_SENSOR_READY);

    uint32_t mask = irq_get_pending_mask(&ic);
    ASSERT_TRUE(mask & (1 << INT_MOTOR_FAULT));
    ASSERT_TRUE(mask & (1 << INT_SENSOR_READY));

    irq_cleanup(&ic);
    return TEST_PASS;
}

int test_irq_clear(void) {
    register_file_t rf;
    motor_controller_t mc;
    sensor_array_t sa;
    interrupt_controller_t ic;

    reg_init(&rf);
    motor_init(&mc, &rf);
    sensor_array_init(&sa, &rf);
    irq_init(&ic, &rf, &mc, &sa);

    irq_enable(&ic, INT_MOTOR_STALL);
    irq_trigger(&ic, INT_MOTOR_STALL);
    ASSERT_TRUE(irq_is_pending(&ic, INT_MOTOR_STALL));

    irq_clear(&ic, INT_MOTOR_STALL);
    ASSERT_TRUE(!irq_is_pending(&ic, INT_MOTOR_STALL));

    irq_cleanup(&ic);
    return TEST_PASS;
}

/*============================================================================
 * SHARED MEMORY TESTS (3 tests)
 *===========================================================================*/

int test_shm_create_destroy(void) {
    shared_mem_t *shm = shm_create();
    ASSERT_TRUE(shm != NULL);

    shared_mem_t *shm2 = shm_open_existing();
    ASSERT_TRUE(shm2 != NULL);

    shm_destroy(shm);
    return TEST_PASS;
}

int test_shm_status_update(void) {
    shared_mem_t *shm = shm_create();
    ASSERT_TRUE(shm != NULL);

    int32_t sensors[4] = {100, 200, 300, 400};
    ASSERT_EQ(shm_update_status(shm, 2, 5000, 1234, sensors, 0), 0);

    ASSERT_EQ(shm->motor_state, 2);
    ASSERT_EQ(shm->motor_speed, 5000);
    ASSERT_EQ(shm->motor_position, 1234);
    ASSERT_EQ(shm->sensor_values[0], 100);

    shm_destroy(shm);
    return TEST_PASS;
}

int test_shm_shutdown(void) {
    shared_mem_t *shm = shm_create();
    ASSERT_TRUE(shm != NULL);

    ASSERT_TRUE(!shm_is_shutdown_requested(shm));
    shm_request_shutdown(shm);
    ASSERT_TRUE(shm_is_shutdown_requested(shm));

    shm_destroy(shm);
    return TEST_PASS;
}

/*============================================================================
 * INTEGRATION TESTS (2 tests)
 *===========================================================================*/

int test_integration_motor_sensor(void) {
    register_file_t rf;
    motor_controller_t mc;
    sensor_array_t sa;

    reg_init(&rf);
    motor_init(&mc, &rf);
    sensor_array_init(&sa, &rf);
    sensor_array_enable(&sa);

    /* Start motor and simulate sensor readings */
    motor_start(&mc, 3000, MOTOR_DIR_CW);

    for (int i = 0; i < 20; i++) {
        motor_update(&mc);

        /* Update velocity sensor with motor speed */
        sensor_set_simulated_value(&sa, 1, (int32_t)motor_get_speed(&mc));
        sensor_array_trigger(&sa);
        sensor_array_update(&sa);
    }

    /* Verify sensor tracks motor speed */
    ASSERT_EQ(sensor_read(&sa, 1), (int32_t)motor_get_speed(&mc));

    return TEST_PASS;
}

int test_integration_fault_irq(void) {
    register_file_t rf;
    motor_controller_t mc;
    sensor_array_t sa;
    interrupt_controller_t ic;

    reg_init(&rf);
    motor_init(&mc, &rf);
    sensor_array_init(&sa, &rf);
    irq_init(&ic, &rf, &mc, &sa);

    irq_callback_count = 0;
    irq_register_handler(&ic, INT_MOTOR_FAULT, test_irq_callback, NULL);
    irq_enable(&ic, INT_MOTOR_FAULT);

    /* Start motor and inject fault */
    motor_start(&mc, 5000, MOTOR_DIR_CW);
    for (int i = 0; i < 10; i++) motor_update(&mc);

    motor_inject_fault(&mc, MOTOR_FAULT_STALL);

    /* Fault should trigger IRQ */
    irq_trigger(&ic, INT_MOTOR_FAULT);
    irq_process_pending(&ic);

    ASSERT_EQ(irq_callback_count, 1);
    ASSERT_EQ(motor_get_state(&mc), MOTOR_STATE_FAULT);

    irq_cleanup(&ic);
    return TEST_PASS;
}

/*============================================================================
 * TEST RUNNER
 *===========================================================================*/

typedef struct {
    const char *name;
    int (*func)(void);
} test_case_t;

static test_case_t all_tests[] = {
    /* Register tests */
    {"test_reg_init", test_reg_init},
    {"test_reg_read_write", test_reg_read_write},
    {"test_reg_set_clear_bits", test_reg_set_clear_bits},
    {"test_reg_invalid_offset", test_reg_invalid_offset},

    /* Motor controller tests */
    {"test_motor_init", test_motor_init},
    {"test_motor_start", test_motor_start},
    {"test_motor_stop", test_motor_stop},
    {"test_motor_brake", test_motor_brake},
    {"test_motor_speed_ramp", test_motor_speed_ramp},
    {"test_motor_direction", test_motor_direction},
    {"test_motor_position_update", test_motor_position_update},
    {"test_motor_fault_stall", test_motor_fault_stall},
    {"test_motor_fault_overheat", test_motor_fault_overheat},
    {"test_motor_fault_recovery", test_motor_fault_recovery},
    {"test_motor_reset", test_motor_reset},
    {"test_motor_max_speed", test_motor_max_speed},

    /* Sensor array tests */
    {"test_sensor_init", test_sensor_init},
    {"test_sensor_enable_disable", test_sensor_enable_disable},
    {"test_sensor_trigger", test_sensor_trigger},
    {"test_sensor_read", test_sensor_read},
    {"test_sensor_read_all", test_sensor_read_all},
    {"test_sensor_continuous_mode", test_sensor_continuous_mode},
    {"test_sensor_buffer_push_pop", test_sensor_buffer_push_pop},
    {"test_sensor_buffer_overflow", test_sensor_buffer_overflow},
    {"test_sensor_buffer_clear", test_sensor_buffer_clear},
    {"test_sensor_value_clamping", test_sensor_value_clamping},

    /* IRQ tests */
    {"test_irq_init", test_irq_init},
    {"test_irq_enable_disable", test_irq_enable_disable},
    {"test_irq_trigger", test_irq_trigger},
    {"test_irq_handler_callback", test_irq_handler_callback},
    {"test_irq_pending_mask", test_irq_pending_mask},
    {"test_irq_clear", test_irq_clear},

    /* Shared memory tests */
    {"test_shm_create_destroy", test_shm_create_destroy},
    {"test_shm_status_update", test_shm_status_update},
    {"test_shm_shutdown", test_shm_shutdown},

    /* Integration tests */
    {"test_integration_motor_sensor", test_integration_motor_sensor},
    {"test_integration_fault_irq", test_integration_fault_irq},
};

static int run_test(const char *name) {
    int num_tests = sizeof(all_tests) / sizeof(all_tests[0]);

    for (int i = 0; i < num_tests; i++) {
        if (strcmp(all_tests[i].name, name) == 0) {
            printf("Running %s...\n", name);
            int result = all_tests[i].func();
            if (result == TEST_PASS) {
                printf("  PASS\n");
                return 0;
            } else {
                printf("  FAILED\n");
                return 1;
            }
        }
    }

    fprintf(stderr, "Unknown test: %s\n", name);
    return 1;
}

static void run_all_tests(void) {
    int num_tests = sizeof(all_tests) / sizeof(all_tests[0]);

    printf("Running %d tests...\n\n", num_tests);

    for (int i = 0; i < num_tests; i++) {
        tests_run++;
        printf("[%2d/%2d] %s... ", i + 1, num_tests, all_tests[i].name);
        fflush(stdout);

        int result = all_tests[i].func();
        if (result == TEST_PASS) {
            printf("PASS\n");
            tests_passed++;
        } else {
            printf("FAILED\n");
        }
    }

    printf("\n================================\n");
    printf("Results: %d/%d tests passed\n", tests_passed, tests_run);

    if (tests_passed == tests_run) {
        printf("All tests PASSED!\n");
    } else {
        printf("%d tests FAILED\n", tests_run - tests_passed);
    }
}

int main(int argc, char *argv[]) {
    if (argc < 2 || strcmp(argv[1], "all") == 0) {
        run_all_tests();
        return (tests_passed == tests_run) ? 0 : 1;
    }

    return run_test(argv[1]);
}
