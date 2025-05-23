# Microphone Design

This directory holds the electrical design for the microphones. Note, you will need one  PIC32 board and two microphone boards for a single system. The two microphones allows for stereo audio. The PIC32 board is an intermediary that translates the audio signal into a format that the Jetson can read.

**Note**: Soldering the microphones is not easy. You will need solder paste and a reflow oven. 

## Setting up the Microphones
Once you have a soldered PIC32, you must program it with the source code located in the [PIC32 Firmware](./PIC32%20Firmware/) directory. Connect the two microphones to the PIC32 board. Connect the PIC32 board to the Jetson's I2S pins. Then, follow the instructions at https://github.com/satomm1/mattbot_record/tree/noetic for Jetson software.