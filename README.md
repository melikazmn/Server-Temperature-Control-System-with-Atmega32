# Server-Temperature-Control-System-with-Atmega32

## Project Overview
This project implements a Server Temperature Control System using two Atmega32 microcontrollers: one acting as the Master (front-end) and the other as the Slave (control unit). The system controls three motors that regulate server temperature based on sensor readings. It features a password-protected interface and communicates between the master and slave using USART.

## Components:
- Master Unit: Displays information to the user, interacts via keypad and LCD, and handles status LEDs and a speaker for alerts.
- Slave Unit: Reads temperature data, controls motor speeds, and adjusts cooling based on system conditions. It also sends status updates back to the master unit.

## Features
1. User Authentication and Locking System
Upon startup, the system is locked. The user must enter a correct password using the keypad.
After 3 incorrect attempts, the system locks for 30 seconds.
Upon successful login, the system unlocks, displaying the main menu.
3. Temperature-Controlled Motors
The motors' speed (duty cycle) is controlled based on the temperature:
For every 10°C increase in temperature, the duty cycle increases by 10%.
If one or more motors fail, the remaining motors distribute the total duty cycle.
If all motors fail or the system can't be adequately cooled, an error is raised, and an alert sounds.
4. Real-time Monitoring and Control
The master unit displays real-time motor status and temperature data on the LCD.
The system dynamically adjusts motor speeds and raises alarms or displays warnings if cooling fails.
5. Visual and Audio Alerts
- LED Indicators:
  - Green LED: All motors working normally, adequate cooling.
  - Yellow LED: One or two motors failed; cooling still functional but degraded.
  - Red LED: All motors failed, or insufficient cooling. A critical error.
-Speaker Alarm: Activated when a critical error occurs (e.g., all motors fail or motors cannot sufficiently cool the system).

## System Functionality
### Master Unit
The master unit serves as the user interface, allowing for password entry, menu navigation, and monitoring. It features:

- Keypad: Allows the user to enter passwords and navigate menus.
- LCD Screen: Displays password prompts, motor status, and temperature readings.
- LEDs and Speaker: Provide visual and audio alerts in case of motor failure or insufficient cooling.
- User Interface
- After entering the correct password, the user can select from the following menu options:
- Motor Status: View the speed (duty cycle) of each motor.
- Temperature: View the current system temperature.
- Exit: Exit the system safely.

### Slave Unit
The slave unit handles motor control and temperature management. It reads temperature data via an ADC and adjusts the motors' PWM duty cycle based on this data. The motors' duty cycles increase proportionally with rising temperatures:

- One motor failure: The remaining two motors adjust to handle the cooling load.
- Two motor failures: The last working motor takes on the full load, up to its capacity.
- All motors fail: The system triggers a critical error, and the speaker sounds an alarm

## Communication Protocol (USART)
The master and slave units communicate via USART, exchanging commands and data:

- Master to Slave: Sends requests for motor duty cycles or temperature.
  -'o', 'm', 'n': Requests the duty cycle for motor 1, 2, or 3.
  -'t': Requests the current temperature.
  
- Slave to Master: Responds with the requested motor or temperature data, or sends system status updates:
  -Motor Duty Cycles: Sent as percentages, followed by a * character.
  -System Status: Sends Y (warning) if one or two motors fail, or R (critical error) if all motors fail.

## Motor Control Based on Temperature
|Temperature|	Motor 1	|Motor 2|	Motor 3|	Overall Duty Cycle	|System Status|
|-----------|---------|-------|--------|----------------------|-------------|
|30°C|30%|30%|30%|90%|Green|
|30°C (1 motor failure)|45%|45%|Failed|90%|Yellow|
|30°C (2 motor failures)|90%|Failed|Failed|90%|Yellow|
|50°C|50%|50%|50%|150%|Green|
|50°C (1 motor failure)|100%|Failed|Failed|Error|Red|


