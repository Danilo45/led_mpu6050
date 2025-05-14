# MPU6050 LED Brightness Adjustment (STM32F407)

## Overview

This project uses an **MPU6050 accelerometer** to dynamically adjust the brightness of two LEDs on an **STM32F407** development board. The brightness of each LED is controlled by the tilt angle of the sensor, specifically in relation to the **Y-axis**. As the sensor tilts from **0° to 90°**, one of the LEDs turns on with varying brightness depending on the tilt angle.

### Key Features:
- **MPU6050 Sensor**: Used to detect tilt along the Y-axis.
- **PWM Control**: Adjusts the brightness of two LEDs based on the angle.
- **STM32F407**: The microcontroller used for processing the sensor data and controlling the LEDs.
- **Two LEDs**: Each LED corresponds to a different tilt direction (left/right), and their brightness changes with the angle of the tilt.
  
## How it Works

- The **MPU6050** accelerometer continuously reads the X, Y, and Z accelerometer data to determine the orientation of the sensor.
- The angle of tilt is calculated based on the Y-axis values, and a **Kalman filter** is applied to smooth the data.
- The tilt angle is then mapped to a PWM signal that controls the brightness of the LEDs:
  - If the tilt angle is positive, the first LED (connected to TIM Channel 1) is dimmed or brightened depending on the angle.
  - If the tilt angle is negative, the second LED (connected to TIM Channel 2) adjusts in a similar manner.
  
As the tilt angle approaches 90°, the corresponding LED increases in brightness according to the PWM duty cycle.
