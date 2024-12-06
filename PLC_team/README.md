# Project name: MACS(Medicine Auto Caring System)

This Project made at KG kairos 

## Table of Contents
1. [Conveyor_Defect_detection](#Conveyor_Defect_detection)
2. [PC_INV](#PC_INV)
3. [PC_PLC(test)](#PC_PLC(test))
4. [PC_PLC](#PC_PLC)
5. [PLC_INV](#PLC_INV)
6. [Pusher](#Pusher)
7. [Etc](#Etc)

## Webcam Qr Code Reader
This folder

- **file:**
    - 'false_ditect_survo_ardu': This Arduino code controls a servo motor upon receiving a specified value.
    - 'webcam(qr) reader.py': This code is designed to recognize QR codes using a webcam.
    - 'webcam(qr)_ardu_controll.py': 
This code recognizes QR codes using a webcam and communicates with an Arduino via serial communication.


- **Role:**
    - Its role is to recognize QR codes using a webcam and control the Arduino through serial communication based on the recognition results.

## PC_INV
This folder

- **file:**
    - 'Final Python PLC Controll': This is an XG5000 file for configuring the PLC settings required to control the inverter using Python.
    - 'PLC Register Reading.py': This code is for reading register values within a specified range.
    - 'Python Inv Direct Controll.py': 
This code controls the conveyor connected to the inverter by entering a specified value in the terminal after execution.
    - 'Python Inv Websocket Controll Client.py': 
This code recognizes QR codes using a webcam connected to a Raspberry Pi, checks if the address exists in a list, sends a signal to the Arduino, and transmits the signal to a connected WebSocket server.
    - 'Python Inv Websocket Controll Server.py': 
This code controls the conveyor connected to the inverter based on the signals received from a WebSocket client. 

- **Role:**
    - This system connects a Raspberry Pi, PLC, and inverter to perform the following tasks: recognize QR codes using a webcam connected to the Raspberry Pi, identify and classify defects, and control the inverter and the conveyor connected to it.

## PC_PLC(test)
This is a test code for communication between the PC and the PLC.

- **file:**
    - 'test.py': This is a code for transmitting values from Python and receiving values sent by the PLC.
    - 'test3.py': This is a code for transmitting values from Python and receiving values sent by the PLC.
    - 'test5.py': This is a code for reading the coil values from a specific address of the PLC using Python.

- **Role:**
    - This is a test code for reading and writing coil values to control the PLC using Python.

## PC_PLC
All the files in this folder are related to RS-485 communication, and a 2-wire RS-485 to USB adapter was used for the communication.

- **file:**
    - 'Mini PLC Coil Controll with Python': This is an XG5000 file for controlling the coil values of a mini PLC using Python code.
    - 'PLC Coli Controll with Python': This is an XG5000 file for controlling the coil values of the PLC using Python code.
    - 'PLC Trancemit PC Recive': This is an XG5000 file for receiving values transmitted by the PLC through Python code.
    - 'Mini PLC Coil Controll with Python.py': This is a code for controlling the coil values of a mini PLC via RS-485 communication.
    - 'PLC Coil Controll with Python.py': This is a code for controlling the coil values of a PLC via RS-485 communication.
    - 'PLC Recive to Python.py': This is a code for receiving values transmitted by the PLC.
    - 'PLC(XBCH) inverter(M100) menuer.py': This is a manual for configuring the PLC and inverter settings required to control the PLC coil values using Python.
- **Role:**
    - These files are for controlling and reading coil values of a PLC using Python. 

## PLC_INV
This is a file for controlling the conveyor connected to the inverter (M100) via RS-485 communication using the PLC (XBC-D32H) model.

- **file:**
    - 'Conveyer Controll with Button': 
This is an XG5000 file for configuring the PLC to control the conveyor connected to the inverter via external inputs (buttons) connected to the PLC.
    - 'Conveyer Controll with Sensor': 
This is an XG5000 file for configuring the PLC to control the conveyor connected to the inverter via external inputs (sensors) connected to the PLC.

- **Role:**
    - This is a file for controlling the conveyor connected to the inverter via the PLC.

## Pusher
This folder

- **file:**
    - 'stepmotor_test': This is an Arduino test code to control a stepper motor by receiving values via serial communication from Python.
    - 'stepmotor_test.py': This is a test code for sending values through serial communication to control a stepper motor on the Arduino.

- **Role:**
    - This is a test code to check if the stepper motor connected to the Arduino is functioning correctly and if the serial communication between Python and the Arduino is working properly.
    
## Etc
These are files that were developed for the project but are no longer used.

### esp32_ino 

- **file:**
    - 'esp32_camera': 
This is a test code to turn on the camera using the ESP32. Here's an example of how to initialize and use the camera on the ESP32:
    - 'f'alse_ditect_survo_esp32': This is a code to control a servo motor on the ESP32 by entering a predefined value. 
    - 'false_ditect_survo_esp32_websocket': This is a code to control the servo motor connected to the ESP32 using WebSocket communication.

- **Role:**
    - 
This is a code to turn on the camera and control the servo motor using the ESP32.
### esp32_Python

- **file:**
    - 'esp32_to_python_camera.py': This is the code to fetch the camera feed from the web page when running the ESP32 camera test code.
    - 'python_arduino_survomotor_controll.py': 
This is the code to control the servo motor connected to the ESP32 by transmitting values via serial communication.
    - 'python_esp32_websocket.py': 
This is the code to control the servo motor connected to the ESP32 by transmitting values via WebSocket communication.

- **Role:**
    - This is the Python code to send values to control the servo motor connected to the ESP32.