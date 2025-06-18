# CAD

This directory contains the CAD files for the mobile robot. The components for all parts are modeled with Autodesk Inventor and are available in the `Inventor` directory. An older version of the mobile robot design in SolidWorks is available in the `mechanical` zip file.

The remainder of this README provides instructions for creating the mechanical form of the mobile robot for the approximately 1 meter tall robot:

<p align="center">

![Mobile Robot CAD Model](./images/CAD_model.png)
![Real Mobile Robot](./images/robot.png)
</p>

A comprehensive parts list is located in `BOM.xlsx` that will be needed for constructing this mobile robot.

1) **Laser cut the platforms out of a thin, stiff wood.** I use tempered hardwood (Duron) of 1/8"-1/4" thickness. Every file that should be laser cut is listed in the `lasercut.xlsx` file. The corresponding `.dxf` files are located in this directory. For reference, to make sure the files are the correct size/units, the diameter of the circular platforms is 354 mm (14 inches).
2) **Prepare the 3D printed parts.** 3D Print the required parts using PLA plastic. Every file that should be 3D printed is listed in the `3d_print.xlsx` file. The corresponding `.stl` files are located in this directory. To to verify that the 3D printed parts are the correct size/units, the length of the bearing blocks should be 3 inches (76 mm).

> [!NOTE]
> If you use different components (motor, wheels, etc), the laser cut or 3D printed files may need to be modified first. 

> [!IMPORTANT]
> The height of the motor holders and bearing blocks must be adjusted so that the bottom of the wheels are in plane with the ball casters. Adjust the hardboard_thickness in `Inventor/parameters.xlsx` and update the Inventor model. Verify everything is correct and re-export the motor holders and bearing block stl files.

> [!IMPORTANT]
> You should print the correct bearing block depending on whether your ball bearings have flanges. If the bearings don't have flanges, 3D print the bearing block designated with "no_flange."

3) **Insert heat set inserts.** Use a soldering iron to insert the heat set inserts into any 3D printed part that requires heat set inserts. The small end of the heat  set insert should enter the 3D printed part first. I recommend using a low temperature (~400F or 200C). 

![Heat set inserts](./images/heat_set.jpg)

> [!TIP]
> IF you are not familiar with installing heat set inserts, watch this quick video: https://youtu.be/P7nHyI1TwKY?t=189.

4) **Install the motors.** Insert the motors into the motor blocks and secure with screws. **Important**: select the proper length screw (not too long!!!) so that the screw does not go too far and break the motor.

![Motor Blocks](./images/motors.jpg)

5) **Install the bearing blocks.** Insert the ball bearings into the bearing blocks (one bearing per bearing block). They are meant to be press fit, but if they are loose then use some epoxy to secure the ball bearings.

![Bearing Blocks](./images/bearing_blocks.jpg)

6) **Install the wheel blocks.** These wheel blocks must be inserted in the wheel to hold the drive shaft. Use epoxy to secure the blocks into the wheel (insert a block on both sides of the wheel-2 total per wheel). Insert the shaft to ensure the blocks are aligned.

![Wheel](./images/wheel.jpg)

7) **Install all parts on the first platform layer.** 
    - Secure the motor, wheel, bearing blocks, shaft, and spider coupler to the first platform layer. Use M3 screws. The ball bearing should be facing the wheel.
    - Secure the two ball casters to the bottom of the first level using (2) M3 screws per caster.

![Drive Train](./images/drive_train.jpg)
![Casters](./images/casters.png)

8) **Install battery holders, Jetson holder, and IMU onto the second level platform.** 

    - Use (16) M3 screws to attach the 2 battery holders and Nvidia Jetson Holder onto the top of the second level platform. 
    - Use (3) M3 screws and plastic standoffs to attach the IMU to the bottom of the second level platform. Please see the [IMU README](../Electrical/IMU_Breakout/README.md) for details on soldering and installing the IMU board.
    - Use (4) M3 screws to attach 3 inch plastic standoffs to the 2nd level platform.

> [!NOTE]
> Use only 3 screws/standoffs for the IMU to prevent unwanted stresses in the IMU!

![Second Level Top](./images/second_level.png)
![Second Level Bottom](./images/second_level_bottom.png)

9) **Install standoffs on the first layer.** Install the 3 inch standoffs to the first layer platform. Connect the motors and IMU to the PCB, and secure the second layer to the standoffs.

![First Layer Standoffs](./images/first_layer_standoff.jpg)

![Fist and Second Layer Connected](./images/first_and_second.png)

10) **Secure 3 Pronged standoffs to each platform.** Connect the 3 pronged standoff to the top of levels 3, 4, and 5. This means you will insert (3) M3 screws through the bottom of the platform. Make sure the prongs are aligned so that when you connect the other side they are lined up with the empty holes!

![Three Prong Standoffs](./images/three_prong.png)

> [!TIP]
> At this point, I recommend preparing the Jetson and inserting the Jetson into it's platform. Read more instructions [here](../Electrical/README.md) in the `Electrical` directory.

11) **Add Levels 4, 5, and 6.** Add these levels one by one by connecting to the 3 pronged standoffs. This is the most tedious part of the whole process as you will need to attach three M3 screws per standoff. 

![Three Prong All Connected](./images/three_prong_all.png)

12) **Connect the 2nd and 3rd level.** Connect the 2nd and 3rd level using the 3D printed 3 inch standoffs and M3 screws. At this point all the levels are connected!

![All Levels Connected](./images/all_levels.png)

13) **Attach the LiDAR to the platform and to the top level.** Screw the LiDAR sensor into the platform. Then, attach the LiDAR platform to the top level with the 3.5 inch standoffs. 

> [!IMPORTANT] 
> Do not use too long of screws. 

> [!IMPORTANT]
> The LiDAR cable should be facing the front of the robot.

![LiDAR](./images/LiDAR.jpg)

14) **Attach the Astra Camera.** Insert the Astra camera into its mount using an M6 screw. Then, attach the mount to the top level using M3 screws.

![Astra Camera](./images/astra.jpg)

15) **Attach the touch screen.** Attach the touch screen to its legs. The long legs use a screw (provided with the screen) to attach, while the screen just rests in the short legs. Secure the legs to the top platform using M3 screws.

![Touch Screen](./images/touch_screen.jpg)

16) **Attach the USB camera.** Use two M3 screws to attach the camera to the top level. 

![USB Camera](./images/usb_cam.jpg)

17) **Attach the microphones.** 
    - First, the microphones need to be inserted into the housings. To view detailed information on constructing the microphone housing, visit the [microphone README](../Electrical/Microphones/README.md).
    - Second, connect the entire housing to one of the three pronged standoffs using a M3 screw and nut.

![Microphone](./images/microphone.jpg)

18) **Attach the foam bumpers.** Use the zipties to secure the foam bumpers to each level. You may need to cut small sections out of the foam to accomodate the three pronged standoffs. To connect the ends of the foam together, use some masking tape.

![Foam Bumper](./images/bumper.jpg)