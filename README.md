# Embedded Motor Controller Driver

A comprehensive embedded systems driver implementation in C, featuring motor control, sensor management, interrupt handling, and inter-process communication through shared memory.

## Features

- **Motor Controller** - State machine-based motor control with speed ramping, direction control, and fault handling
- **Sensor Array** - Multi-sensor support with buffering, continuous mode, and value clamping
- **Interrupt Handler** - IRQ management with callback registration and signal-based simulation
- **Shared Memory IPC** - POSIX shared memory for inter-process communication
- **Device Registers** - Simulated hardware register file interface

## Project Structure

```
embedded-driver/
├── include/
│   ├── device_registers.h    # Hardware register definitions
│   ├── motor_controller.h    # Motor control interface
│   ├── sensor_array.h        # Sensor management interface
│   ├── interrupt_handler.h   # IRQ handling interface
│   └── shared_mem.h          # Shared memory IPC interface
├── src/
│   ├── device_registers.c    # Register I/O implementation
│   ├── motor_controller.c    # Motor state machine implementation
│   ├── sensor_array.c        # Sensor array implementation
│   ├── interrupt_handler.c   # Interrupt controller implementation
│   ├── shared_mem.c          # Shared memory implementation
│   └── main.c                # Demo application
├── tests/
│   └── test_driver.c         # Comprehensive test suite (37 tests)
├── CMakeLists.txt            # CMake build configuration
└── Makefile                  # Make-based build option
```

## Building

### Using CMake

```bash
mkdir build && cd build
cmake ..
make
```

### Using Make

```bash
make debug    # Build with debug symbols
make release  # Build optimized version
```

## Running

```bash
# Run the motor driver demo
./build/motor_driver

# The driver will:
# - Initialize motor and sensors
# - Start motor at 5000 RPM clockwise
# - Continuously update motor state and sensor readings
# - Print status every 50 ticks
# - Respond to signals (SIGUSR1 for motor fault, SIGUSR2 for sensor interrupt)
# - Press Ctrl+C to exit gracefully
```

## Testing

```bash
# Run all tests
make test

# Run specific test
make test-motor_init
make test-sensor_read

# Run with Valgrind for memory leak detection
make valgrind
```

### Test Coverage

| Module              | Tests |
|---------------------|-------|
| Device Registers    | 4     |
| Motor Controller    | 12    |
| Sensor Array        | 10    |
| Interrupt Handler   | 6     |
| Shared Memory       | 3     |
| Integration         | 2     |
| **Total**           | **37**|

## API Overview

### Motor Controller

```c
motor_controller_t mc;
motor_init(&mc, &regs);
motor_start(&mc, 5000, MOTOR_DIR_CW);  // Start at 5000 RPM clockwise
motor_set_speed(&mc, 3000);             // Change speed
motor_stop(&mc);                        // Graceful stop
motor_brake(&mc);                       // Emergency brake
```

**Motor States:** `IDLE` → `STARTING` → `RUNNING` → `STOPPING` → `IDLE`

**Fault Handling:** Stall, Overheat, Overcurrent detection with recovery

### Sensor Array

```c
sensor_array_t sa;
sensor_array_init(&sa, &regs);
sensor_array_enable(&sa);
sensor_array_trigger(&sa);              // Single sample
sensor_array_set_continuous(&sa, true); // Continuous mode

int32_t value = sensor_read(&sa, 0);    // Read single sensor
int32_t values[4];
sensor_read_all(&sa, values, 4);        // Read all sensors
```

**Sensor Types:** Position, Velocity, Temperature, Current

### Interrupt Handler

```c
interrupt_controller_t ic;
irq_init(&ic, &regs, &motor, &sensors);
irq_register_handler(&ic, INT_MOTOR_FAULT, fault_callback, &motor);
irq_enable(&ic, INT_MOTOR_FAULT);
irq_process_pending(&ic);               // Process triggered interrupts
```

### Shared Memory IPC

```c
shared_mem_t *shm = shm_create();       // Server creates
shared_mem_t *shm = shm_open_existing(); // Client opens

shm_send_command(shm, CMD_MOTOR_START, speed, direction);
shm_wait_response(shm, &status, data, count);
shm_update_status(shm, state, speed, position, sensors, fault);
```

## Requirements

- GCC with C11 support
- POSIX-compliant OS (Linux, macOS)
- pthread library
- CMake 3.10+ (optional)

## License

This project is open source.
