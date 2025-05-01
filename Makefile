# Define the build directory
BUILD_DIR = build

# So 'make' will always execute regardless of file timestamps
.PHONY: build flash run

# Default target to build the project
build:
	cmake --build $(BUILD_DIR)

# Target to flash the project
flash:
	cmake --build $(BUILD_DIR) --target flash

# build and flash
run:
	cmake --build $(BUILD_DIR)
	cmake --build $(BUILD_DIR) --target flash

# Target to start openocd server
openocd:
	cmake --build $(BUILD_DIR) --target openocd

# Target to run debug with gdb
debug:
	arm-none-eabi-gdb build/starplat2.elf --command=.gdbinit

# Target to clean the build directory
clean:
	rm -rf $(BUILD_DIR)

# Target to configure the project
configure:
	cmake -S . -B $(BUILD_DIR)
