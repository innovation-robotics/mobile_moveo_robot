# mobile_moveo_robot
- Installing Nvidia Drivers
Steps:
Disable the Nouveau Open-Source Driver
sudo apt-get purge "*nvidia*"
sudo apt autoremove
sudo apt update
Create a new blacklist file: 
sudo nano /etc/modprobe.d/blacklist-nouveau.conf
blacklist nouveau
options nouveau modeset=0
sudo update-initramfs -u
sudo reboot
Installation
ubuntu-drivers devices
select the driver with "recommended" keyword for nvidia RTX 3060 I used
sudo apt install nvidia-driver-570
sudo reboot
check that the driver is installed by runing this command
nvidia-smi

- Installing Nvidia Container Toolkit
Steps:
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
Verifying GPU access inside the Docker Container
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi

- Create Ubuntu 22.04 with ROS Humble Docker Image
Steps:
docker build -t my-ubuntu-2204-image .
docker run --name=mobile_moveo_container -it --user ros --network=host --ipc=host -v $PWD/mobile_moveo_docker_ws:/home/ros -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY --gpus all my-ubuntu-2204-image
docker container start -i mobile_moveo_container
cd home
cd ros
mkdir mobile_moveo
cd mobile_moveo
mkdir src
cd src
git clone --branch main https://github.com/innovation-robotics/mobile_moveo_robot.git
vcs import < mobile_moveo_robot/mobile_moveo.repos

git clone --branch main https://github.com/innovation-robotics/AprilTag-ROS-2.git
git clone --branch main https://github.com/innovation-robotics/mobile_moveo_ros2.git
git clone --branch humble https://github.com/innovation-robotics/moveit_resources.git
git clone --branch main https://github.com/innovation-robotics/moveo_arm_arduino.git
git clone --branch ros2 https://github.com/innovation-robotics/realsense-ros.git
git clone --branch ros2 https://github.com/innovation-robotics/rviz_visual_tools.git
git clone --branch ros_world2021 https://github.com/innovation-robotics/stretch_ros2.git
git clone --branch main https://github.com/innovation-robotics/launch_param_builder.git
git clone --branch humble https://github.com/innovation-robotics/moveit2.git
git clone --branch humble https://github.com/innovation-robotics/moveit_task_constructor.git
git clone --branch main https://github.com/innovation-robotics/moveo_diffdrive_arduino.git
git clone --branch feature/add_system_clock https://github.com/innovation-robotics/ros_ign.git
git clone --branch ros2 https://github.com/innovation-robotics/srdfdom.git
git clone --branch main https://github.com/innovation-robotics/tcp_gstreamer_publisher.git
git clone --branch main https://github.com/innovation-robotics/mobile_moveo_robot.git
git clone --branch humble https://github.com/innovation-robotics/moveit2_tutorials.git
git clone --branch ros2 https://github.com/innovation-robotics/moveit_visual_tools.git
git clone --branch main https://github.com/innovation-robotics/moveo_generate_parameter_library.git
git clone --branch ros2 https://github.com/innovation-robotics/rosparam_shortcuts.git
git clone --branch main https://github.com/innovation-robotics/stretch_moveit_plugins.git
