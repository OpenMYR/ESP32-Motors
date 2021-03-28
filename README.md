# ESP32-Motors
by [OpenMYR](http://www.OpenMYR.com/) 

OpenMYR WiFi motors can be controled via WiFi, or programed with an Arduino sketch. At the core of the system is an Espressif ESP32.

This project utilizes PlatformIO with Visual Studio Code

Use with [ESP32-Hobby-Servo](https://github.com/OpenMYR/ESP32-Hobby-Servo), [ESP32-Stepper](https://github.com/OpenMYR/ESP32-Stepper), or with your own ESP32 boards.

## Driver Support
* Up to 15 180-degree servo motors
* Stepper motor

## Installation 

1. [Install PlatformIO and Visual Studio Code](https://platformio.org/install/ide?install=vscode)
2. [Clone our repository](https://help.github.com/en/github/creating-cloning-and-archiving-repositories/cloning-a-repository)
3. Open local project folder in Visual Studio Code
4. Using the [PlatformIO Toolbar](https://docs.platformio.org/en/latest/integration/ide/vscode.html#platformio-toolbar) build all environments with the "PlatformIO: Build" command
5. Upload the code and file system using Serial or OTA  
    Upload both code and file system with new devices to ensure proper operation 
