# MPU Automatic Calibration with Simulator (Archived)

**Status**: This repository has been archived and is no longer actively maintained.

---

## Overview

This repository provides an automated approach to calibrating [MPU6050](https://invensense.tdk.com/products/motion-processing/6-axis/mpu-6050/) 6-axis sensors. The system uses a **3-rotational + 1-linear axis** setup to subject multiple MPU6050 devices to known, repeatable motions. A real-time program running on a microcontroller controls the mechanical motion, while an offline C++ program calculates the ideal sensor outputs and then compares them against the actual MPU6050 readings to adjust calibration parameters.

By automating the calibration, you can achieve more accurate sensor readings across multiple sensors under consistent conditions.

---

## Features

- **Hardware-based Automated Motion**  
  A real-time microcontroller program commands the mechanical axes to rotate and translate the MPU6050s in a precisely controlled manner.

- **3 Rotational + 1 Linear Axis**  
  The system can apply rotation about three orthogonal axes and translation along a single linear axis, providing comprehensive coverage of sensor movement.

- **C++ Calibration Utility**  
  A standalone C++ program calculates what the ideal sensor data should be for each motion, then compares it to the actual readings from each MPU6050 to determine calibration offsets and scale factors.

- **Multiple Sensor Support**  
  Designed to calibrate multiple MPU6050s at once, reducing overall time and effort.

- **Archived Repository**  
  This project is no longer under active development, but the code and documentation remain available for reference or forking.

---

## Repository Structure


1. **GimbalV2/**  
   Contains source code for the real-time program. It sends movement commands (e.g., angles, velocities) to the mechanical system, ensuring precise rotational and linear motion.

2. **GimbalProcessor/**  
   Contains the C++ calibration program. It computes the expected sensor outputs for each known motion and compares them with the actual sensor readings to produce calibration constants.

2. **GimbalSimulator/**  
   Contains the C++ simulation program. This emulates the results of the MPU outputs based on the trajectory of motion with the expected added noise and randomness of the physical systems

3. **MPUReader/**  
   Stores sample sensor data logs. This can be used for testing the C++ calibration program.

---

## How It Works

1. **Control the Motion**  
   - The microcontroller runs a program that commands motors or actuators on the 3 rotational axes plus 1 linear axis.  
   - At each step in a test sequence, it sets a specific orientation or linear displacement.

2. **Collect Sensor Data**  
   - The microcontroller simultaneously reads the raw accelerometer and gyroscope data from each MPU6050 and logs it (e.g., over Serial or I2C).

3. **Ideal Sensor Values**  
   - The C++ program calculates what the MPU6050 *should* read, given the known motion. For instance, when rotating at a specific angular velocity, the gyroscope readings should match those rates.

4. **Compare & Calibrate**  
   - The calibration program then compares the real sensor data to the expected data.  
   - It computes offsets, scale corrections, and bias adjustments for accelerometers and gyroscopes.  
   - Updated calibration parameters are saved for future use.

---

## Getting Started

### Hardware Requirements
- A mechanical system capable of:
  - 3-axis rotation
  - 1-axis linear translation
- A microcontroller (e.g., Arduino, STM32, ESP32) with real-time control capabilities, this used a Teensy 3.5
- One or more MPU6050 sensors

### Software Requirements
- C++17 (or later) compiler for the calibration utility
- Arduino IDE (or similar) for the microcontroller code
- A serial logger or data transfer mechanism for sensor data

## Usage

1. **Set Up Motion Test**  
   - Mount your MPU6050(s) onto the mechanical platform.  
   - Configure your microcontroller to run the test routine (rotations and translations).

2. **Record Sensor Data**  
   - The microcontroller logs readings to a CSV or similar format.  
   - Transfer this data file to your PC for calibration.

3. **Run the Calibration**  
   - Use the C++ program to calculate the expected sensor outputs.  
   - Compare actual vs. expected readings.  
   - Collect the final calibration offsets and scale factors.

4. **Apply Calibration**  
   - Integrate the generated calibration parameters into your sensor fusion or data reading pipeline.

---

## License

Distributed under the [MIT License](LICENSE). See `LICENSE` for more details.

---

## Disclaimer

This project has been archived and is provided **as is**, without warranty of any kind. Use it at your own risk, and carefully verify and validate any calibration results. Mechanical, electrical, or software modifications may be necessary for your specific setup.

---

## Acknowledgments

- [Invensense MPU6050 Documentation](https://invensense.tdk.com/products/motion-processing/6-axis/mpu-6050/)
- Contributors to this repository for sharing ideas and code.
