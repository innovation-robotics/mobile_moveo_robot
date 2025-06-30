# mobile_moveo_robot
- Installing Nvidia Drivers<br>
Steps:<br>
Disable the Nouveau Open-Source Driver<br>
sudo apt-get purge "*nvidia*"<br>
sudo apt autoremove<br>
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
download this file
https://github.com/innovation-robotics/mobile_moveo_robot/blob/main/Dockerfile
then build it
docker build -t my-ubuntu-2204-image .
cd
mkdir docker
cd docker
mkdir mobile_moveo_docker_ws
cd mobile_moveo_docker_ws
docker run --name=mobile_moveo_container -it --privileged --device=/dev/video0:/dev/video0 --device=/dev/video1:/dev/video1 --device=/dev/media0:/dev/media0 --user ros --network=host --ipc=host -v $PWD/mobile_moveo_docker_ws:/home/ros -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY --gpus all my-ubuntu-2204-image:with-dep-yolo

- Installing OpenCV with GStreamer
apt list --installed | grep opencv
sudo apt purge libopencv* python3-opencv opencv-data
sudo apt autoremove
sudo apt clean
sudo apt autoclean
sudo apt-get update
sudo apt-get install gstreamer1.0*
sudo apt install ubuntu-restricted-extras
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt-get install -y libopenjpip7 libopenjpip-dec-server libopenjp2-tools
sudo apt-get install libgstreamer1.0–0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
sudo apt-get install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev
sudo apt pkg-config
sudo apt install libgtk-3-dev
sudo apt install libgtk2.0-dev
sudo apt install libdc1394–22-dev
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D PYTHON_EXECUTABLE=$(which python3) \
-D BUILD_opencv_python2=OFF \
-D CMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") \
-D PYTHON3_EXECUTABLE=$(which python3) \
-D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
-D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
-D WITH_GSTREAMER=ON \
-D BUILD_EXAMPLES=OFF ..
sudo make -j16
sudo make install
sudo ldconfig
python3
import cv2
print(cv2.getBuildInformation())

Installing ROS Humble
Steps:
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
exit

- Installing Mobile Moveo
docker container start -i mobile_moveo_container
cd home
cd ros
mkdir mobile_moveo
cd mobile_moveo
mkdir src
cd src
git clone --branch main https://github.com/innovation-robotics/mobile_moveo_robot.git
vcs import < mobile_moveo_robot/mobile_moveo.repos
cd moveit_task_constructor
git submodule update --init --recursive
cd ..
source /opt/ros/humble/setup.bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
sudo apt install python3-vcstool
sudo apt-get install ros-humble-ros-gz
sudo apt install ros-humble-ros-gz-bridge
sudo apt install ros-humble-ros-gz-sim
sudo apt install ros-humble-gazebo-ros* ros-humble-gazebo-dev ignition-fortress
colcon build --packages-select v4l2_camera image_transport_plugins --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
ldd /opt/ros/humble/lib/librviz_default_plugins.so
sudo mv /opt/ros/humble/lib/x86_64-linux-gnu/libimage_transport.so /opt/ros/humble/lib/x86_64-linux-gnu/libimage_transport.so.old
cd src
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ..
export IGNITION_VERSION=fortress
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cd src
git clone https://github.com/innovation-robotics/AprilTag-ROS-2.git
cd ..
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
sudo apt install ros-humble-twist-mux

- Installing Yolo for ROS2
git clone https://github.com/mgonzs13/yolo_ros.git
sudo apt update
sudo apt install python3-pip
pip3 install -r yolo_ros/requirements.txt
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
sudo usermod -a -G video ros
sudo usermod -a -G sudo ros
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0
gst-launch-1.0 -e -v v4l2src device=/dev/video0 ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegdec ! videoconvert ! autovideosink
docker commit -m "Installing yolo and dependencies" mobile_moveo_container my-ubuntu-2204-image:with-dep-yolo
docker exec -it mobile_moveo_container /bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
sudo apt-get install ros-${ROS_DISTRO}-v4l2-camera
ros2 run v4l2_camera v4l2_camera_node
ros2 launch yolo_bringup yolo.launch.py
rviz2

- Preparing Raspberry PI
ssh ubuntu@192.168.1.245
install ros2 humble
git clone https://github.com/Slamtec/rplidar_ros.git
git clone https://github.com/innovation-robotics/robot_teleop_keyboard.git
git clone https://github.com/innovation-robotics/network_to_serial_bridge.git
download arduino mega code for the manipulator then compile and upload to the microcontroller
git clone https://github.com/innovation-robotics/moveo_arm_arduino_mega.git
download arduino uno code for the base then compile and upload to the microcontroller
git clone https://github.com/innovation-robotics/ros_arduino_bridge.git
sudo chmod a+rw /dev/ttyUSB0
sudo chmod a+rw /dev/ttyUSB1
sudo chmod a+rw /dev/ttyUSB2
sudo minicom -b 57600 -o -D /dev/ttyUSB0
e
1 1
if you found this response it means it is connected to the base
to leave the minicom
Ctrl+a
z
x
sudo minicom -b 57600 -o -D /dev/ttyUSB2
if the lidar stopped it means it is connected to this port otherwise it is connected to the arm
in my case i found it to be like this
base 0
arm 1
lidar 2
open repos in vscode
review this file rplidar_a1_launch.py
serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
make sure ttyUSB matches the com port for the lidar in this case it doesn't match so I will change it to ttyUSB2 to be like this
serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB2')
in terminal 
cd /repos/lidar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
build for the new serial port
colcon build --symlink-install
ros2 launch rplidar_ros rplidar_a1_launch.py
if you want to stop the lidar in new terminal create the ssh and source and then run this command 
ros2 service call /stop_motor std_srvs/srv/Empty
in new terminal
sudo gst-launch-1.0 v4l2src ! image/jpeg,width=1280,height=720,framerate=30/1 ! tcpserversink port=7001 host=0.0.0.0 sync=false async=false
for the base open new terminal
cd repos/rbx1_ws/build/network_to_serial_bridge/
./network_to_serial_bridge 7777 /dev/ttyUSB0 57600
for the arm open new terminal
cd repos/rbx1_ws/build/network_to_serial_bridge/
./network_to_serial_bridge 7778 /dev/ttyUSB1 115200

- Running the pick and place demo
docker container start -i mobile_moveo_container
cd home/ros/mobile_moveo
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch moveit_resources_robot_moveit_config demo3.launch.py

TODO
- Please remove ros_ign
- delete meshes folder in the /mobile_moveo_robot/robot_description 