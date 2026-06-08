#!/bin/bash

repositories=(
    "https://github.com/satomm1/mattbot_bringup.git"
    "https://github.com/satomm1/mattbot_dds.git"
    "https://github.com/satomm1/mattbot_record.git"
    "https://github.com/satomm1/mattbot_image_detection.git"
    "https://github.com/satomm1/mattbot_mcl.git"
    "https://github.com/satomm1/mattbot_navigation.git"
    "https://github.com/satomm1/mattbot_teleop.git"
    "https://github.com/satomm1/mattbot_database.git"
    "https://github.com/satomm1/ros_astra_camera.git"
    "https://github.com/satomm1/rplidar_ros.git"
    "https://github.com/satomm1/twist_mux.git"
    "https://github.com/satomm1/slam_gmapping.git"
)

files=(
    "https://raw.githubusercontent.com/satomm1/Robot/main/Jetson/robot_env.sh"
    "https://raw.githubusercontent.com/satomm1/dds_robot_platform/main/robot/startup_script.py"
    "https://raw.githubusercontent.com/satomm1/Robot/main/Jetson/cyclonedds.xml"
)


for repo in "${repositories[@]}"; do
    # Extract the repository name from the URL
    repo_name=$(basename "$repo" .git)
    
    # Check if the repository already exists
    if [ -d "$repo_name" ]; then
        echo "Repository $repo_name already exists. Skipping clone."
    else
        echo "Cloning $repo..."
        git clone -b noetic "$repo"
    fi
done

for file in "${files[@]}"; do
    # Extract the filename from the URL
    filename=$(basename "$file")

    # Check if the file already exists
    if [ -f "$filename" ]; then
        echo "File $filename already exists. Skipping download."
    else
        echo "Downloading $file..."
        wget "$file"
    fi
done