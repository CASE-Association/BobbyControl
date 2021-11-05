## BobbyControl

Control for differential steering of bobby car.

### How it works

The code is running on an ESP32 devkit (wemos lolin32) on the bottom of the bobby car. The ESP32 is soldered to a protoboard, which has all of the connectors to communicate with the various systems. These systems are:

- **VESC:** These are the motor controllers that are located on the underside of the bobby car, marked "Right" and "Left". The ESP32 communicates with one of the VESCs through UART, and commands for the other motor is automatically forwarded via CAN. This way both motors can be controlled (and read) with a single UART line. Power for the ESP32 also comes from the VESC (5V, 1.5A max). 
- **RC:** This is the remote control reciever. It is using some unknown protocol that is read with bitbanging interrupts in the code. It works most of the time, the range is not that good though. 
- **LED1 & LED2:** These ports are for the neopixel strips. 
- **Ratt:** This is the input from the steering wheel and throttle. Both are simple potentiometers. 

The pinout is defined at the top of main.c. The code to handle the different inputs and outputs are implemented as freeRTOS tasks. This makes it a bit different from standard arduino code, but it enables very easy "parallel processing" (actually switches tasks automatically and seamlessly) that is needed to control all systems simultaniously. 

### Improvements

- [ ] Make a PCB for the ESP32 that has all the connectors needed, and also allows for future flexibility.
- [ ] Split the code into multiple files to make it easier to read and to work with. 
- [ ] Add more LED patterns and melodies.
- [ ] Replace the buzzer with a speaker to actually hear it. 
- [ ] Improve the driver for reading the remote.
