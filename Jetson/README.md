# Jetson Instructions

We use an 8 GB Nvidia Jetson Orin Nano Developer Kit for high level control of the mobile robot.

## Jetson Bring Up Instructions
1) Flash a microSD card with the Jetpack Image and insert into the Jetson. I have had WiFi card issues with Jetpack 6.x, so I recommend using Jetpack 5.x (latest version as of May 2025 is 5.1.5). Please find the SD card image at this webpage: https://developer.nvidia.com/embedded/jetpack-sdk-515.

    To flash the SD card, you will need to first format the SD card using: https://www.sdcard.org/downloads/formatter/. Then, use [Balena etcher](https://etcher.balena.io/) to flash the SD card.

    Alternatively, you can use the Nvidia SDK Manager to flash the Jetson. If you have a device that is natively running Ubuntu, this will work great. Otherwise, you will need a VM (specifically VMware Workstation). Some tips if using a VM – make sure you allocate plenty of hard disk memory for the VM and change the settings so that the USB connection to the Jetson automatically is sent to the VM.

2) Switch out the default WiFi card. The default WiFi card does not support roaming (which is necessary to switch between mesh nodes). We use the Intel AC9260 WiFi card. The WiFi card is located on the bottom of the Jetson and can be removed with a screwdriver. 

>[!Important]
>Be sure to reconnect the antennas to the new WiFi card!

3) Power on the device while connected to a monitor. Connect to Wi-Fi so that we can SSH into the device.

4) Call: `sudo apt update`. Then, install nano: `sudo apt install nano`.

5) Set up SPI capabilities on headers pins so that we can communicate with the MCU:
    - Navigate to opt/nvidia/jetson-io: `cd /opt/nvidia/jetson-io`
    - Call `sudo python3 jetson-io.py`
        - We need to manullay conifgure the 40 pin header
        - Select options to activate SPI1 and I2S
    - Reboot the Jetson for the new pin functions to take effect

6) Load the spidev module automatically on start up:
	- Edit the file: `sudo nano /etc/modules-load.d/spidev.conf`
 	- write: `spidev` in the file, and save the file. Reboot the Jetson to take effect.

7) If you have issues with `dev/ttyUSB0`, you may need to call `sudo apt remove brltty`

8) Connect the corresponding SPI pins on the header to the SPI pins on the MCU control board. The corresponding pins are listed in the [Electrical README](../Electrical/README.md).

9) Connect the corresponding I2S pins on the header to the I2S pins on the microphone control board. The corresponding pins are listed in the [Microphone README](../Electrical/Microphones/README.md).

## WiFi Roaming Setup Instructions

Configuring the WiFi driver to roam is not straightforward. Roaming lets the Jetson switch access points when the signal is weak, instead of waiting for a full disconnect.

We automate setup with `wifi_roaming_setup.sh` in this directory. The script:

1. **NetworkManager** — Writes `/etc/NetworkManager/NetworkManager.conf` so your WiFi NIC (detected as `wlan*`, e.g. `wlan0`) is unmanaged by NetworkManager.
2. **wpa_supplicant** — Writes `/etc/wpa_supplicant/wpa_supplicant-<iface>.conf` with roaming `bgscan`, then enables **`wpa_supplicant@<iface>.service`** as the sole WiFi controller (and disables the generic `wpa_supplicant.service` to avoid conflicts).
3. **DHCP on boot** — Installs `/usr/local/bin/custom_wifi.sh` (runs `dhclient` only) and **`custom_wifi.service`**, which starts after `wpa_supplicant@<iface>`.

In `bgscan="simple:30:-60:600"`: **-60** = -60 dBm to trigger a scan, **30** = scan every 30 seconds while below -60 dBm, **600** = scan every 600 seconds otherwise.

### Prerequisites

1. Install a roaming-capable WiFi card and driver. We use the **Intel AC9260** (see bring-up step 2).
2. Complete initial Jetson bring-up through step 4 (`sudo apt update`, `nano` installed) so you can SSH in.
3. On the Jetson, install `wpasupplicant` if it is not already present:
    ```
    sudo apt install wpasupplicant
    ```
4. Confirm a `wlan*` interface exists (e.g. `wlan0`):
    ```
    ip link show | grep wlan
    ```

### Copy the setup script to the Jetson

**Option A — Download with `wget`**

On the Jetson:

```
cd ~
wget "https://raw.githubusercontent.com/satomm1/Robot/main/Jetson/wifi_roaming_setup.sh"
chmod +x wifi_roaming_setup.sh
```

**Option B — Create the file with `nano`**

On the Jetson, open the file from this repo in a browser or editor, copy its contents, then:

```
nano ~/wifi_roaming_setup.sh
```

Paste the contents, save (`Ctrl+O`, `Enter`, `Ctrl+X`), then:

```
chmod +x ~/wifi_roaming_setup.sh
```

### Run the setup script

SSH into the Jetson, then:

```
chmod +x ~/wifi_roaming_setup.sh
sudo ~/wifi_roaming_setup.sh
```

The script will:

- Prompt for confirmation (it modifies system network files; backups use a `.bak` suffix).
- Auto-detect the first `wlan*` interface and its MAC address.
- Prompt for your WiFi **SSID** and **password**.
- Ask whether the device operates in the **USA** (`US`) or **South Korea** (`KR`) and set the WiFi regulatory country code accordingly.

When it finishes, **reboot**:

```
sudo reboot
```

### Verify after reboot

```
sudo systemctl status wpa_supplicant@wlan0 custom_wifi.service
iw dev wlan0 link
ip addr show wlan0
```

Replace `wlan0` with your interface name if different.

Optional manual test before reboot:

```
sudo systemctl start custom_wifi.service
```

### Undo / restore previous networking

If you need to revert:

```
sudo systemctl disable --now custom_wifi.service
sudo systemctl disable --now wpa_supplicant@wlan0.service
sudo systemctl enable wpa_supplicant.service
sudo rm /etc/systemd/system/custom_wifi.service /usr/local/bin/custom_wifi.sh
sudo mv /etc/wpa_supplicant/wpa_supplicant-wlan0.conf.bak /etc/wpa_supplicant/wpa_supplicant-wlan0.conf
sudo mv /etc/NetworkManager/NetworkManager.conf.bak /etc/NetworkManager/NetworkManager.conf
sudo reboot
```

Adjust paths if your interface is not `wlan0` or if no `.bak` files were created.

> [!TIP]
> Assign a fixed IP to the Jetson’s WiFi MAC in your router’s DHCP settings so the Jetson keeps the same address on the LAN.

## Docker Setup
Getting a Docker container that does everything we need is not trivial. The following steps worked for my NVIDIA Jetson Orin Nano. If you already have a docker image saved as a .tar.gz file, please look at the end for instructions.

1)  Follow system setup (through at least the “Relocating Docker Data Root” step) from https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md
    
    If you have NVME storage to add, I found this tutorial helpful for setting up the NVME storage: https://www.digitalocean.com/community/tutorials/how-to-partition-and-format-storage-devices-in-linux

	Also, be sure to add the following line to `etc/docker/daemon.json`:
    ```
	"data-root": "/mnt/data"
    ```

	Note: `/data` should be whatever your mount folder name is…

    Then, restart docker:
    ```
    sudo systemctl restart docker
    ```

----
If you already have a docker container image, skip to the [next section](#load-docker-image)!

Docker images can be downloaded from: https://drive.google.com/file/d/1__ZI9WkVhz9b7KRzaHHCewiELtFPr_nl/view?usp=sharing.

----

2)	Choose a base container. I like l4t-ml since it contains cuda enabled pytorch, cuda enabled tensorflow2, and cuda enabled opencv. The full list of containers can be found https://github.com/dusty-nv/jetson-containers/tree/master

    Now, run the container, e.g.:
    ```
	jetson-containers run $(autotag l4t-ml)
    ```

3)	Now we can install ROS. For example, for ROS Noetic, follow the instructions here: https://wiki.ros.org/noetic/Installation/Ubuntu

    **IMPORTANT**: Install the **ROS-Base** version. The other version will attempt to modify the opencv, which we cannot do! Thus, install the bare bones version and the selectively install additional packages we may need.

    The following is also useful to put in your `~/.bashrc` file
    ```
    source /workspace/catkin_ws/devel/setup.bash
    export ROS_IP=192.168.xx.xx
    export ROS_MASTER_URI=http://$ROS_IP:11311
    export ROBOT_ID=x
    ```

4)	Install any additional ros packages you need, e.g.
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
 
5)	Install other python packages you need.

    **IMPORTANT**: Any package which has an opencv-python dependency must be sure not to modify the opencv-python package already installed from the original container. Updating will break the package!

    Some packages I install include:
    - spidev
    - confluent-kafka
    - rospy-message-converter
    - pyignite
    - matplotlib
    - scipy
    - ultralytics****be careful with opencv-python here!

6)	Install Cyclone DDS and it’s python binding.

    Follow the directions of https://github.com/eclipse-cyclonedds/cyclonedds to first install cycloneDDS. Be careful! Check what the latest version of the cyclonedds python binding is and match this version!  The install directory should be /path/to/cyclonedds/install. Next:
    ```
    export CYCLONEDDS_HOME="$(pwd)/install"
    ```
    Even better, put this in the `~/.bashrc` file since we need this anytime we use the python binding. Last, we install the python binding via:
    ```
    pip3 install cyclonedds --no-binary cyclonedds
    ```

7)	Commit the docker container so you can use it later, e.g.:
	docker commit c3f279d17e0a ml_ros:latest

8)	Now, to run the container, we can use the command:
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

9)	To save the image (for easy loading on a new machine), use the following command:
    ```
	docker save myimage:latest | gzip > myimage_latest.tar.gz
    ```

If everything has gone according to plan, you should now have a docker container which has ROS, cuda enabled pytorch/tensorflow, and everything else you might need!

----
<a name="load-docker-image"></a>
### What if I already have a docker image saved as a \*.tar.gz file?

Use this pre-existing docker image: https://drive.google.com/file/d/1__ZI9WkVhz9b7KRzaHHCewiELtFPr_nl/view?usp=sharing

In this case, just set docker up and the call:
```
sudo docker load < your_image.tar.gz
```
----
### Setting Up ROS Workspace
1) Start the ROS docker container. 
    ```
    sudo docker run --runtime nvidia --network=host -v ~/workspaces/catkin_ws:/workspace/catkin_ws -v ~/gemini_api:/gemini_code -v /dev/bus/usb:/dev/bus/usb -v /dev/video0:/dev/video0 -v /dev/video1:/dev/video1 -it --device=/dev/ttyUSB0 --device=/dev/spidev0.0 --rm --privileged --name ros_noetic ml_ros:latest
    ```

    You should source ROS via:
    ```
    source /opt/ros/noetic/setup.bash
    ```

    Within the docker container, navigate to `/workspace/catkin_ws` (you may need to create this directory):
    ```
    cd /workspace/catkin_ws
    ```

2) Now, create the devel and src directories:
    ```
    mkdir devel src
    ```

3) Now, build the workspace:
    ```
    catkin_make
    ```

4) Source the build and go to the `src` directory:
    ```
    source devel/setup.bash
	cd src
    ```

5) Now, clone the following repositories and switch to the noetic branch. To make cloning easier, I have provided a script to automatically clone all the relevant repositories and a few useful files. Copy the `clone_repos.sh` script to your Jetson, make it executable, and then run the script:
    ```
	nano clone_repos.sh
	(copy clone_repos.sh code here)
	ctrl+x, y (to save file)
	chmod +x clone_repos.sh
	. clone_repos.sh
    ```

    The script will clone the following repositories/files from https://github.com/satomm1/
    
    - mattbot_record
    - mattbot_bringup          
    - mattbot_dds         
    - mattbot_image_detection  
    - mattbot_mcl              
    - mattbot_navigation	 
    - mattbot_teleop
    - ros_astra_camera
    - rplidar_ros
    - twist_mux
    - slam_gmapping
    - robot_env.sh (From this repo)
    - startup_script.py (from this repo)

6) Navigate back to the `catkin_ws` directory and build the packages:
    ```
	cd /workspace/catkin_ws
	catkin_make
	source devel/setup.bash
    ```

7) Add important information to the `robot_env.sh` file:
    ```
    nano /workspace/catkin_ws/src/robot_env.sh
    ```
    Please update the file with the Jetson IP address, Robot ID, MCU SPI number, camera type, and robot height.

8) Make sure our ROS environment and robot environment settings are sourced via the `bashrc` file:
    ```
	nano ~/.bashrc
    ```
    Append the following:
    ```
    source /workspace/catkin_ws/devel/setup.bash
    source /workspace/catkin_ws/src/robot_env.sh
    ```
    Source the bashrc file:
    ```
    source ~/.bashrc
    ```

8) Test the setup, call roscore. If you have no errors, this is good!
    ```
	roscore 
    ```

9) Set up the Astra Camera.  You will also need to navigate to the `ros_astra_camera` directory and perform these two commands outside of the docker container:
    ```
    cd ~/workspaces/catkin_ws/src/ros_astra_camera
    ./scripts/create_udev_rules
    sudo udevadm control --reload && sudo udevadm trigger
    ```

10) You should commit the docker container to save any changes. Outside of the docker container (but with the docker container running), run this command:
    ```
    sudo docker commit ros_noetic ml_ros:latest
    ```

----
### Setting up the Gemini Container

1) Download and load the Gemini image:
You can download the image here: https://drive.google.com/file/d/1xQtwj8xyFaPMbaMlJZ36KgxLJ6gVcdOr/view?usp=sharing

    ```
    sudo docker load < gemini_latest.tar.gz
    ```

2) Clone the gemini_api repo:
    ```
    cd ~ && git clone https://github.com/satomm1/gemini_api.git
    ```

3) Start the container:
    ```
    sudo docker run -v ~/gemini_api:/gemini_code -v ~/Desktop/audio:/audio -w /gemini_code -it --rm --privileged -p 5000:5000  --name gemini gemini:latest
    ```

4) To run the needed file, run the `start_scripts.sh` script (which just runs `endpoint.py`):
    ```
    . start_scripts.sh
    ```

----
### Setting up the Display

1) Clone the display repository:
    ```
    git clone https://github.com/satomm1/mattbot_display.git
    cd mattbot_display
    ```

2) Install dependencies:
    ```
    pip3 install pygame
    pip3 install -U pyinstaller
    ```

3) Make the `Desktop/audio` directory:
    ```
    mkdir ~/Desktop/audio
    ```
    Copy the `default.mp3` file to the `audio` directory:
    ```
    cp default.mp3 ~/Desktop/audio
    ```

4) You can run the display script via:

    ```
    python3 display_app.py
    ```
    This will print messages sent to localhost:65432 to the screen. You may need to adjust the line `os.environ['AUDIODEV'] = 'plughw:<card number>,<device number>'` for your specific device/card number for the sound to work. I recommend adjusting the card number between 0 and 2.

5) Create an executable:
    ```
    pyinstaller display_app.spec
    ```
    This executable will be located at `/dist/display_app`.

6) Copy the executable to the Desktop:
    ```
    cp dist/display_app ~/Desktop
    ```

7) Now, the executable can be run by double-clicking the icon on the desktop!
