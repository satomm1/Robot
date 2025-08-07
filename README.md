# Mobile Robot
This repository contains many instructions for setting up and constructing the robot. However, for a comprehensive guide, please visit: [https://satomm1.github.io/mattbot/instructions.html](https://satomm1.github.io/mattbot/index.html).


## Authors
Matthew Sato and Kincho Law<br>
satomm@stanford.edu<br>
Stanford University Department of Civil and Environmental Engineering<br>
Engineering Informatics Lab

## Description
This repo contains mechanical (CAD) designs, electrical schematics, and software for creating a wheeled mobile robot.

![Mobile Robot](./CAD/images/CAD_model.png)
![Mobile Robot](./CAD/images/robot.png)

- The mechanical design is provided in the `./CAD` directory. In this directory, you will find [detailed instructions](./CAD/README.md) for constructing the mobile robot along with relevant STL/DXF files and a bill of materials. The CAD files were generated in Inventor.
- The electrical design for the various components are located in the `./Electrical` directory. In this directory, you will find electrical schematics and the PCB design for the main control board, IMU breakout board, microphone breakout board, and microphone controller board. A [detailed list of the components and instructions](./Electrical/README.md) for each of these boards is included.
- The software design for the main control board is in the `./Software` directory. This software is for the PIC32 located on the main control board and facilitates communication between the PIC32 and Jetson, performs PID control, odometry, etc. Additionally, the software for the microphone control board is included in this directory. Please visit the [README](./Software/README.md) for more detailed instructions.
- Instructions for the Nvidia Jetson Orin Nano aare provided in the `./Jetson` directory. This directory includes instructions for bringing up the Jetson, setting up the WiFi mesh node connectivity, setting up the Docker environments, and setting up the ROS environment/software stack. Please visit the [Jetson README](./Jetson/README.md) for more details.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
