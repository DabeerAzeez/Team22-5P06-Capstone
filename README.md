# Cardiac Catheterization Testing System (CCTA)

## Overview

<img src="./readme_imgs/system_diagram.png" alt="System Schematic" width="700"/>
<p align="center"><em>Figure 1: System schematic showing data and control flow between components.</em></p>

<img src="./readme_imgs/prototype.png" alt="Prototype" width="500"/>
<p align="center"><em>Figure 2: Physical prototype of the Cardiac Catheterization Testing Apparatus (CCTA).</em></p>


This project presents a low-cost, modular cardiac catheterization testing system designed to simulate physiological pressure and flow conditions in a controlled and reproducible in-vitro environment. It replaces the need for expensive commercial simulators or ethically problematic animal testing setups by offering a customizable, software-integrated alternative.

At the core of this system is a MATLAB UI that interfaces with an Arduino microcontroller. The Arduino collects data from pressure and flow sensors and transmits it to MATLAB, where it's processed and visualized in real-time. Based on user-defined targets and modes, MATLAB sends control signals back to the Arduino to adjust a positive displacement pump. The system supports manual, automatic (PID-based), and pulsatile flow control.

---

## Folder Structure

### `/assets`
Contains engineering-related reference files:
- CAD designs of the fluid system
- Electrical schematics for sensor and pump integration
- Excel sheets with calibration curves and technical calculations

### `/data`
Used to store collected experimental data, logs, and exported results from MATLAB sessions.

### `/docs`
Project documentation, including:
- Full system report
- Verification test results
- Bill of Materials (BOM)
- A short video walkthrough of system setup
- Operation manual

### `/src`
Includes the complete codebase:
- **MATLAB App**: Provides the user interface for data visualization, system control, and parameter configuration. Handles serial communication, real-time plotting, setpoint tracking, and exporting functions.
- **Arduino Code**: Acquires sensor data and transmits it to MATLAB via serial. It interprets control signals to adjust pump output depending on the selected mode:
  - **Manual Mode**: Receives and applies a direct PWM value
  - **Auto Mode**: Uses PID control based on provided setpoint and coefficients
  - **Pulsatile Mode**: Generates a sinusoidal PWM signal based on amplitude and BPM settings

See the diagram below for a simplified system data flow:

![System Architecture](./readme_imgs/code_flow_diagram.png)

---

## Contributors

For any questions or inquiries, feel free to reach out to the contributors:

- **Dabeer Abdul-Azeez** (Engineering Physics) – dabeerazeez@gmail.com  
- **Syed Saad Ali** (Engineering Physics & BME)
- **Yousuf Araim** (Mechanical & BME)
- **Owen Johnstone** (Engineering Physics & BME)
- **Aly Pirbay** (Mechanical)
