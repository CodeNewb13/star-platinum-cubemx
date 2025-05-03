# Star Platinum CubeMX Project
This project is a firmware for an STM32 microcontroller using STM32CubeMX and HAL libraries.
It initializes GPIO, I2C, and TIM peripherals and interfaces with the CH452 LED driver module to control a display.

## Project Overview    
MCU: STM32 (specific model depends on your CubeMX setup)

Framework: STM32 HAL

### Peripherals Used:

GPIO

I2C1

TIM1, TIM3

## Steps to Set Up and Use the Project
Download the Project:
Clone the repository or download the project files to your local machine.

### Open the .ioc File:
The .ioc file is the configuration file for STM32CubeMX, which defines the initialization of peripherals and pin mappings. Open the .ioc file with STM32CubeMX (or CubeIDE) to load the configuration.

### Configure Pins in STM32CubeMX:

After opening the .ioc file, you can see the Pinout & Configuration view.

In this view, you can assign the GPIO pins to various functions such as I2C, TIM, and other peripherals.

Make sure to assign the pins for I2C, TIM, and GPIO correctly according to your hardware setup (e.g., for communication with the CH452 LED driver module).

You can also configure other settings like the GPIO mode (input/output/analog) and pull-up/pull-down resistors.

### Configure Peripherals:

I2C1: Enable and configure I2C1 for communication with the CH452.

TIM1, TIM3: Set up timers to manage timing or PWM signals if needed.

Ensure that all required peripherals are enabled and properly configured.

### Generate the Code:

Once you have finished the configuration in STM32CubeMX, click on Project → Generate Code.

Choose STM32CubeIDE (or Keil if you prefer) as the IDE to generate the necessary project files.

### Open the Project in STM32CubeIDE or Keil:

If you selected STM32CubeIDE during code generation, simply open the generated project file (.ioc will generate a project folder).

Alternatively, if you're using Keil, open the .uvprojx file in Keil to start the project.

### Build the Project:

In STM32CubeIDE: Click Project → Build Project or simply press the build icon (hammer).

In Keil: Click on the Build button or use the shortcut Ctrl+F7 to compile and generate the hex file.

### Flash the Firmware to the STM32:

Connect your STM32 to your PC via a programmer/debugger (e.g., ST-Link).

In STM32CubeIDE: Click on the Run button (green triangle) to load the firmware onto the microcontroller.

In Keil: Use the Download button to flash the firmware.

### Test the Display:
Once the firmware is uploaded, the CH452 LED driver module should be controlling the display based on the configuration and code written.

Notes:
You can always revisit the .ioc file to adjust pin configurations or peripheral settings and regenerate the code.

If you need additional configurations or peripherals, STM32CubeMX offers easy-to-use wizards for adding more components.

## TODO
- [ ] Integrate algorithm into the codebase
- [ ] Implement TT_Motor for flame take
- [ ] Integrate 3 orientation greyscale sensors for Mahony filter recalibration
  (maybe reinitialize quaternion each time it arrived at the checkpoint)
- [ ] With the greyscale sensor, implement rotation (dont rely on mahony filter
for rotation)
