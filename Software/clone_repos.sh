#!/bin/bash

repositories=(
    "https://github.com/satomm1/mattbot_bringup.git"
    "https://github.com/satomm1/mattbot_dds.git"
    "https://github.com/satomm1/mattbot_record.git"
    "https://github.com/satomm1/mattbot_image_detection.git"
    "https://github.com/satomm1/mattbot_mcl.git"
    "https://github.com/satomm1/mattbot_navigation.git"
    "https://github.com/satomm1/mattbot_teleop.git"
    "https://github.com/satomm1/ros_astra_camera.git"
    "https://github.com/satomm1/rplidar_ros.git"
    "https://github.com/satomm1/twist_mux.git"
    "https://github.com/satomm1/slam_gmapping.git"
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