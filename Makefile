# Embedded Device Driver Makefile
CC = gcc
CFLAGS = -Wall -Wextra -Wpedantic -std=c11 -I./include
LDFLAGS = -pthread

# Debug/Release flags
DEBUG_FLAGS = -g -O0 -DDEBUG
RELEASE_FLAGS = -O2

# Directories
SRC_DIR = src
INC_DIR = include
TEST_DIR = tests
BUILD_DIR = build

# Source files
SOURCES = $(SRC_DIR)/device_registers.c \
          $(SRC_DIR)/motor_controller.c \
          $(SRC_DIR)/sensor_array.c \
          $(SRC_DIR)/shared_mem.c \
          $(SRC_DIR)/interrupt_handler.c

OBJECTS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SOURCES))

# Targets
TARGET = $(BUILD_DIR)/motor_driver
TEST_TARGET = $(BUILD_DIR)/test_driver

# Platform-specific
UNAME := $(shell uname 2>/dev/null || echo Windows)
ifeq ($(UNAME),Linux)
    LDFLAGS += -lrt
endif

# Default target
all: debug

# Debug build
debug: CFLAGS += $(DEBUG_FLAGS)
debug: $(TARGET) $(TEST_TARGET)

# Release build
release: CFLAGS += $(RELEASE_FLAGS)
release: $(TARGET) $(TEST_TARGET)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Link main executable
$(TARGET): $(OBJECTS) $(BUILD_DIR)/main.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/main.o: $(SRC_DIR)/main.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Link test executable
$(TEST_TARGET): $(OBJECTS) $(BUILD_DIR)/test_driver.o
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_driver.o: $(TEST_DIR)/test_driver.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Run tests
test: $(TEST_TARGET)
	@echo "Running all tests..."
	@$(TEST_TARGET) all

# Run individual test
test-%: $(TEST_TARGET)
	@$(TEST_TARGET) $*

# Memory check with Valgrind
valgrind: debug
	valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes \
		--error-exitcode=1 $(TEST_TARGET) all

# Memory check for main program (short run)
valgrind-driver: debug
	timeout 5 valgrind --leak-check=full --show-leak-kinds=all \
		$(TARGET) || true

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Install (optional)
install: release
	install -m 755 $(TARGET) /usr/local/bin/

# Uninstall
uninstall:
	rm -f /usr/local/bin/motor_driver

# Help
help:
	@echo "Embedded Device Driver Build System"
	@echo "===================================="
	@echo ""
	@echo "Targets:"
	@echo "  all        - Build debug version (default)"
	@echo "  debug      - Build with debug symbols"
	@echo "  release    - Build optimized version"
	@echo "  test       - Run all tests"
	@echo "  test-NAME  - Run specific test (e.g., test-motor_init)"
	@echo "  valgrind   - Run tests under Valgrind"
	@echo "  clean      - Remove build artifacts"
	@echo "  help       - Show this help"

.PHONY: all debug release test valgrind valgrind-driver clean install uninstall help
