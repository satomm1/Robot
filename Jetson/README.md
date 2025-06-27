# Jetson Instructions

We use an 8 GB Nvidia Jetson Orin Nano Developer Kit for high level control of the mobile robot.

## Jetson Bring Up Instructions
1) Flash a microSD card with the Jetpack Image and insert into the Jetson. I have had WiFi card issues with Jetpack 6.x, so I recommend using Jetpack 5.x (latest version as of May 2025 is 5.1.5). Please find the SD card image at this webpage: https://developer.nvidia.com/embedded/jetpack-sdk-515. <br> <br>
Alternatively, you can use the Nvidia SDK Manager to flash the Jetson. If you have a device that is natively running Ubuntu, this will work great. Otherwise, you will need a VM (specifically VMware Workstation). Some tips if using a VM – make sure you allocate plenty of hard disk memory for the VM and change the settings so that the USB connection to the Jetson automatically is sent to the VM.

2) Switch out the default WiFi card. The default WiFi card does not support roaming (which is necessary to switch between mesh nodes). We use the Intel AC9260 WiFi card. The WiFi card is located on the bottom of the Jetson and can be removed with a screwdriver. 

>[!Important]
>Be sure to reconnect the antennas to the new WiFi card!

3) Power on the device while connected to a monitor. Connect to Wi-Fi so that we can SSH into the device.

4) Call: `sudo apt update`. Then, install nano: `sudo apt install nano`.

5) Set up SPI capabilities on headers pins so that we can communicate with the MCU:
    - Navigate to opt/nvidia/jetson-io: `cd /opt/nvidia/jetson-io`
    - Call `sudo python3 jetson-io.py`
        - Manually update the pin functions
        - Select options to activate SPI1 and I2S
    - Reboot the Jetson for the new pin functions to take effect
    - Call `sudo modprobe spidev`

6) I0f you have issues with `dev/ttyUSB0`, you may need to call `sudo apt remove brltty`

7) Connect the corresponding SPI pins on the header to the SPI pins on the MCU control board. The corresponding pins are listed in the [Electrical README](../Electrical/README.md).

8) Connect the corresponding I2S pins on the header to the I2S pins on the microphone control board. The corresponding pins are listed in the [Microphone README](../Electrical/Microphones/README.md).

## WiFi Roaming Setup Instructions
Configuring the WiFi driver to roam is not straightforward. But, roaming will allow us to switch access points when connection is poor, rather than waiting for complete loss of connection to switch access points. 

1) Make sure a roaming enabled WiFi card (and driver) is installed. We use Intel AC9260.

2) Disable NetworkManager for your WiFi interface (in our case wlan0) by editing `/etc/NetworkManager/NetworkManager.conf`:

    ```
    sudo nano /etc/NetworkManager/NetworkManager.conf
    ```

    The configuration file should contain the following:

    ```
    [main]
    plugins=ifupdown,keyfile

    [ifupdown]
    managed=false

    [device]
    wifi.scan-rand-mac-address=no

    [keyfile]
    unmanaged-devices=mac:<WiFi Card MAC Address>
    ```

    Your WiFi card MAC address can be determined by calling:

    ```
    ifconfig
    ```
    and looking at the corresponding wlan0 entry.

    Last, restart the NetworkManager: 

    ```
    sudo systemctl restart NetworkManager
    ```

3) Set your wpa_supplicant configuration file at `/etc/wpa_supplicant/wpa_supplicant.conf`:

    ```
    sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
    ```

    The wpa_supplicant configuration file should contain the following:

    ```
    ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
    update_config=1
    country=US
    bgscan="simple:30:-60:600"

    network={
        ssid="<Your WiFi SSID> "
        psk="<Your WiFi Password>"
        key_mgmt=WPA-PSK
    }
    ```

    > [!IMPORTANT]
    > Be sure to replace the placeholdes with your own WiFi SSD and password

    Restart wpa_supplicant: 
    
    ```
    sudo systemctl restart wpa_supplicant
    ```

    > [!NOTE]
    > In bgscan, -60 = -60 dBm to trigger a scan, 30 = scan every 30 seconds while below -60 dBM, 600 = scan every 600 seconds no matter what.

4) Create a shell script for connecting to the WiFi network at `/usr/local/bin/custom_wifi.sh`:

    ```
    sudo nano /usr/local/bin/custom_wifi.sh
    ```

    The script should contain:

    ```
    #!/bin/bash

    INTERFACE="wlan0"
    SSID="<Your WiFI SSID Here>"

    sudo ifconfig $INTERFACE down
    sudo iwconfig $INTERFACE essid $SSID
    sudo ifconfig $INTERFACE up

    sudo wpa_supplicant -B -i $INTERFACE -c /etc/wpa_supplicant/wpa_supplicant.conf

    sudo dhclient $INTERFACE
    ```

    Make the script executable: 
    ```
    sudo chmod +x /usr/local/bin/custom_wifi.sh
    ```

5) Run the shell script automatically upon start up.

    - Create a systemmd service: 
       
        ```
        sudo nano /etc/systemd/system/custom_wifi.service
        ```
        
        The service should contain:
        ```
        [Unit]
        Description=Custom WiFi Setup
        After=network.target

        [Service]
        ExecStart=/usr/local/bin/custom_wifi.sh
        RemainAfterExit=yes

        [Install]
        WantedBy=multi-user.target
        ```
    - Reload systemctl
        ```
        sudo systemctl daemon-reload
        ```
    - Enable systemctl
        ```
        sudo systemctl enable custom_wifi.service
        ```
    - [Optional] Test the service
        ```
        sudo systemctl start custom_wifi.service
        ```

6) Reboot the Jetson for everything to take effect.