# Build `ml_ros` from scratch (optional)

Most setups should pull the pre-built image from GHCR instead. See [Docker Setup](README.md#docker-setup) in the Jetson README.

These steps worked for an NVIDIA Jetson Orin Nano when assembling the image locally before it was published to GHCR. Complete Docker system setup in the [Jetson README](README.md#docker-setup) first (jetson-containers, data root, restart Docker).

1) Choose a base container. I like l4t-ml since it contains cuda enabled pytorch, cuda enabled tensorflow2, and cuda enabled opencv. The full list of containers can be found https://github.com/dusty-nv/jetson-containers/tree/master

    Now, run the container, e.g.:
    ```
	jetson-containers run $(autotag l4t-ml)
    ```

2) Now we can install ROS. For example, for ROS Noetic, follow the instructions here: https://wiki.ros.org/noetic/Installation/Ubuntu

    **IMPORTANT**: Install the **ROS-Base** version. The other version will attempt to modify the opencv, which we cannot do! Thus, install the bare bones version and the selectively install additional packages we may need.

    The following is also useful to put in your `~/.bashrc` file
    ```
    source /workspace/catkin_ws/devel/setup.bash
    export ROS_IP=192.168.xx.xx
    export ROS_MASTER_URI=http://$ROS_IP:11311
    export ROBOT_ID=x
    ```

3) Install any additional ros packages you need, e.g.
    ```
	sudo apt install ros-noetic-PACKAGE
    ```
    I installed the following:
    - tf2-msgs
    - tf
    - gmapping
    - diagnostic-updater
    
    For the camera to work, I find it necessary to first run:
    ```
    apt-get purge -y '*opencv*'
    ```
    Then, follow instructions at https://github.com/satomm1/ros_astra_camera, with the following exception: run these lines outside of the docker container
    ```
    ./scripts/create_udev_rules
    sudo udevadm control --reload && sudo  udevadm trigger
    ```
 
4) Install other python packages you need.

    **IMPORTANT**: Any package which has an opencv-python dependency must be sure not to modify the opencv-python package already installed from the original container. Updating will break the package!

    Some packages I install include:
    - spidev
    - confluent-kafka
    - rospy-message-converter
    - pyignite
    - matplotlib
    - scipy
    - ultralytics****be careful with opencv-python here!

5) Install Cyclone DDS and it’s python binding.

    Follow the directions of https://github.com/eclipse-cyclonedds/cyclonedds to first install cycloneDDS. Be careful! Check what the latest version of the cyclonedds python binding is and match this version!  The install directory should be /path/to/cyclonedds/install. Next:
    ```
    export CYCLONEDDS_HOME="$(pwd)/install"
    ```
    Even better, put this in the `~/.bashrc` file since we need this anytime we use the python binding. Last, we install the python binding via:
    ```
    pip3 install cyclonedds --no-binary cyclonedds
    ```

6) Commit the docker container so you can use it later, e.g.:
	docker commit c3f279d17e0a ml_ros:latest

7) Now, to run the container, we can use the command:
    ```
	jetson-containers run $(autotag ml_ros:latest)
    ```

    You can include any of the usual docker commands with this. For example, my full command is:
    ```
    jetson-containers run -v ~/workspaces/catkin_ws:/workspace/catkin_ws -v ~/gemini_api:/gemini_code -v /dev/bus/usb:/dev/bus/usb -v /dev/video0:/dev/video0 -v /dev/video1:/dev/video1 -i --device=/dev/ttyUSB0 --device=/dev/spidev0.0 --rm --privileged --name ros_noetic $(autotag ml_ros:latest)
    ```
    OR
    ```
    sudo docker run --runtime nvidia --network=host -v ~/workspaces/catkin_ws:/workspace/catkin_ws -v ~/gemini_api:/gemini_code -v /dev/bus/usb:/dev/bus/usb -v /dev/video0:/dev/video0 -v /dev/video1:/dev/video1 -it --device=/dev/ttyUSB0 --device=/dev/spidev0.0 --rm --privileged --name ros_noetic ml_ros:latest
    ```

8) To save the image (for easy loading on a new machine), use the following command:
    ```
	docker save myimage:latest | gzip > myimage_latest.tar.gz
    ```

If everything has gone according to plan, you should now have a docker container which has ROS, cuda enabled pytorch/tensorflow, and everything else you might need!

Return to [Setting Up ROS Workspace](README.md#setting-up-ros-workspace) in the Jetson README.