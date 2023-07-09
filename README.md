# whirlwind-combat-SW
Code for a meltybrain combat robot controlled via ESP32 bluetooth link between the robot and its controller

## About
In progress design, I'll fill this out as it gets more usable

## Design
The machine's hardware is based around access to a Markforged X7 3D printer which can print the large outer shell with continuous carbon fiber layers. Other than this (which I have access to through work), the robot uses only cheap off-the-shelf parts, 2D machined components, and simple circuit boards which can be easily procured from JLCPCB or equivalent

Links to the cad files and any PCB designs will be provided here once I have finalized hardware

## Software
This repo contains 2 arduino sketches which can be uploaded through Arduino IDE.

### BLEController
Written for an ESP32-WROOM-32D which I had lying around, this reads from the IO devices in a custom controller and updates the controller with a (soon to be) low overhead command system

TODO:
- Rewrite for ESP32-S3 w/ extrenal antenna
- Write tare value
- Ensure reliable BLE connection logic

### ESC-Run
Goes on the robot - interfaces with an accelerometer to track and update angle, indicate heading with LEDs, and command motor velocities based on the recieved controller data

TODO:
- Rewrite for a SEEED Xiao esp32 S3 (2x cores + external antenna) -> maybe the bluetooth can be split off from heading updates to avoid jitter
- Add tare functionality
- Ensure reliable BLE connection logic, add connection in
- Cleanup code for higher efficiency