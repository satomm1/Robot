# Please replace with your values below
export ROS_IP=<Your Robot IP Here>  # IP Address of this Jetson Device                        
export ROS_MASTER_URI=http://$ROS_IP:11311
export ROBOT_ID=<Robot ID Here (Integer)>  # Select a unique ID for this robot (Align with MCU Robot ID)
export MCU_SPI=<SPI used for MCU comms (either 1 or 3)>
export CAMERA_TYPE=astra_pro_plus  # camera type, i.e. <astra_pro_plus, astra, astra_pro>
export CYCLONEDDS_HOME="/home/cyclonedds/install"