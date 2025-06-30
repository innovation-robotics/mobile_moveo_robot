# mobile_moveo_robot
- Installing Nvidia Drivers<br>
Steps:<br>
Disable the Nouveau Open-Source Driver<br>
sudo apt-get purge "*nvidia*"<br>
sudo apt autoremove<br>
sudo apt update<br>
Create a new blacklist file: <br>
sudo nano /etc/modprobe.d/blacklist-nouveau.conf<br>
blacklist nouveau<br>
options nouveau modeset=0<br>
sudo update-initramfs -u<br>
sudo reboot<br>
Installation<br>
ubuntu-drivers devices<br>
select the driver with "recommended" keyword for nvidia RTX 3060 I used<br>
sudo apt install nvidia-driver-570<br>
sudo reboot<br>
check that the driver is installed by runing this command<br>
nvidia-smi<br>

- Installing Nvidia Container Toolkit<br>
Steps:<br>
sudo apt-get install -y nvidia-container-toolkit<br>
sudo nvidia-ctk runtime configure --runtime=docker<br>
sudo systemctl restart docker<br>
Verifying GPU access inside the Docker Container<br>
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi<br>

- Create Ubuntu 22.04 with ROS Humble Docker Image<br>
Steps:<br>
download this file<br>
https://github.com/innovation-robotics/mobile_moveo_robot/blob/main/Dockerfile<br>
then build it<br>
docker build -t my-ubuntu-2204-image .<br>
cd<br>
mkdir docker<br>
cd docker<br>
mkdir mobile_moveo_docker_ws<br>
cd mobile_moveo_docker_ws<br>
docker run --name=mobile_moveo_container -it --privileged --device=/dev/video0:/dev/video0 --device=/dev/video1:/dev/video1 --device=/dev/media0:/dev/media0 --user ros --network=host --ipc=host -v $PWD/mobile_moveo_docker_ws:/home/ros -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY --gpus all my-ubuntu-2204-image:with-dep-yolo<br>

- Installing OpenCV with GStreamer<br>
apt list --installed | grep opencv<br>
sudo apt purge libopencv* python3-opencv opencv-data<br>
sudo apt autoremove<br>
sudo apt clean<br>
sudo apt autoclean<br>
sudo apt-get update<br>
sudo apt-get install gstreamer1.0*<br>
sudo apt install ubuntu-restricted-extras<br>
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev<br>
sudo apt-get install -y libopenjpip7 libopenjpip-dec-server libopenjp2-tools<br>
sudo apt-get install libgstreamer1.0–0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio<br>
sudo apt-get install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev<br>
sudo apt pkg-config<br>
sudo apt install libgtk-3-dev<br>
sudo apt install libgtk2.0-dev<br>
sudo apt install libdc1394–22-dev<br>
cd opencv<br>
mkdir build<br>
cd build<br>
cmake -D CMAKE_BUILD_TYPE=RELEASE \<br>
-D INSTALL_PYTHON_EXAMPLES=ON \<br>
-D INSTALL_C_EXAMPLES=OFF \<br>
-D PYTHON_EXECUTABLE=$(which python3) \<br>
-D BUILD_opencv_python2=OFF \<br>
-D CMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") \<br>
-D PYTHON3_EXECUTABLE=$(which python3) \<br>
-D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \<br>
-D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \<br>
-D WITH_GSTREAMER=ON \<br>
-D BUILD_EXAMPLES=OFF ..<br>
sudo make -j16<br>
sudo make install<br>
sudo ldconfig<br>
python3<br>
import cv2<br>
print(cv2.getBuildInformation())<br>

Installing ROS Humble<br>
Steps:<br>
locale  # check for UTF-8<br>
sudo apt update && sudo apt install locales<br>
sudo locale-gen en_US en_US.UTF-8<br>
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8<br>
export LANG=en_US.UTF-8<br>
locale  # verify settings<br>
sudo apt install software-properties-common<br>
sudo add-apt-repository universe<br>
sudo apt update && sudo apt install curl -y<br>
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')<br>
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME<br>
sudo dpkg -i /tmp/ros2-apt-source.deb<br>
sudo apt update<br>
sudo apt upgrade<br>
sudo apt install ros-humble-desktop<br>
sudo apt install ros-dev-tools<br>
exit<br>

- Installing Mobile Moveo<br>
docker container start -i mobile_moveo_container<br>
cd home<br>
cd ros<br>
mkdir mobile_moveo<br>
cd mobile_moveo<br>
mkdir src<br>
cd src<br>
git clone --branch main https://github.com/innovation-robotics/mobile_moveo_robot.git<br>
vcs import < mobile_moveo_robot/mobile_moveo.repos<br>
cd moveit_task_constructor<br>
git submodule update --init --recursive<br>
cd ..<br>
source /opt/ros/humble/setup.bash<br>
sudo apt install python3-rosdep<br>
sudo rosdep init<br>
rosdep update<br>
sudo apt update<br>
sudo apt dist-upgrade<br>
sudo apt install python3-colcon-common-extensions<br>
sudo apt install python3-colcon-mixin<br>
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml<br>
colcon mixin update default<br>
sudo apt install python3-vcstool<br>
sudo apt-get install ros-humble-ros-gz<br>
sudo apt install ros-humble-ros-gz-bridge<br>
sudo apt install ros-humble-ros-gz-sim<br>
sudo apt install ros-humble-gazebo-ros* ros-humble-gazebo-dev ignition-fortress<br>
colcon build --packages-select v4l2_camera image_transport_plugins --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF<br>
source install/setup.bash<br>
ldd /opt/ros/humble/lib/librviz_default_plugins.so<br>
cd src<br>
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y<br>
cd ..<br>
export IGNITION_VERSION=fortress<br>
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF<br>
cd src<br>
git clone https://github.com/innovation-robotics/AprilTag-ROS-2.git<br>
cd ..<br>
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF<br>
sudo apt install ros-humble-twist-mux<br>

- Installing Yolo for ROS2<br>
git clone https://github.com/mgonzs13/yolo_ros.git<br>
sudo apt update<br>
sudo apt install python3-pip<br>
pip3 install -r yolo_ros/requirements.txt<br>
cd ~/ros2_ws<br>
rosdep install --from-paths src --ignore-src -r -y<br>
colcon build<br>
sudo usermod -a -G video ros<br>
sudo usermod -a -G sudo ros<br>
v4l2-ctl --list-devices<br>
v4l2-ctl --list-formats-ext -d /dev/video0<br>
gst-launch-1.0 -e -v v4l2src device=/dev/video0 ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegdec ! videoconvert ! autovideosink<br>
docker commit -m "Installing yolo and dependencies" mobile_moveo_container my-ubuntu-2204-image:with-dep-yolo<br>
docker exec -it mobile_moveo_container /bin/bash<br>
source /opt/ros/humble/setup.bash<br>
source install/setup.bash<br>
sudo apt-get install ros-${ROS_DISTRO}-v4l2-camera<br>
ros2 run v4l2_camera v4l2_camera_node<br>
ros2 launch yolo_bringup yolo.launch.py<br>
rviz2<br>

- Preparing Raspberry PI<br>
ssh ubuntu@192.168.1.245<br>
install ros2 humble<br>
git clone https://github.com/Slamtec/rplidar_ros.git<br>
git clone https://github.com/innovation-robotics/robot_teleop_keyboard.git<br>
git clone https://github.com/innovation-robotics/network_to_serial_bridge.git<br>
download arduino mega code for the manipulator then compile and upload to the microcontroller<br>
git clone https://github.com/innovation-robotics/moveo_arm_arduino_mega.git<br>
download arduino uno code for the base then compile and upload to the microcontroller<br>
git clone https://github.com/innovation-robotics/ros_arduino_bridge.git<br>
sudo chmod a+rw /dev/ttyUSB0<br>
sudo chmod a+rw /dev/ttyUSB1<br>
sudo chmod a+rw /dev/ttyUSB2<br>
sudo minicom -b 57600 -o -D /dev/ttyUSB0<br>
e<br>
1 1<br>
if you found this response it means it is connected to the base<br>
to leave the minicom<br>
Ctrl+a<br>
z<br>
x<br>
sudo minicom -b 57600 -o -D /dev/ttyUSB2<br>
if the lidar stopped it means it is connected to this port otherwise it is connected to the arm<br>
in my case i found it to be like this<br>
base 0<br>
arm 1<br>
lidar 2<br>
open repos in vscode<br>
review this file rplidar_a1_launch.py<br>
serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')<br>
make sure ttyUSB matches the com port for the lidar in this case it doesn't match so I will change it to ttyUSB2 to be like this<br>
serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB2')<br>
in terminal <br>
cd /repos/lidar_ws<br>
source /opt/ros/humble/setup.bash<br>
source install/setup.bash<br>
build for the new serial port<br>
colcon build --symlink-install<br>
ros2 launch rplidar_ros rplidar_a1_launch.py<br>
if you want to stop the lidar in new terminal create the ssh and source and then run this command 
ros2 service call /stop_motor std_srvs/srv/Empty<br>
in new terminal<br>
sudo gst-launch-1.0 v4l2src ! image/jpeg,width=1280,height=720,framerate=30/1 ! tcpserversink port=7001 host=0.0.0.0 sync=false async=false<br>
for the base open new terminal<br>
cd repos/rbx1_ws/build/network_to_serial_bridge/<br>
./network_to_serial_bridge 7777 /dev/ttyUSB0 57600<br>
for the arm open new terminal<br>
cd repos/rbx1_ws/build/network_to_serial_bridge/<br>
./network_to_serial_bridge 7778 /dev/ttyUSB1 115200<br>

- Running the pick and place demo<br>
docker container start -i mobile_moveo_container<br>
cd home/ros/mobile_moveo<br>
source /opt/ros/humble/setup.bash<br>
source install/setup.bash<br>
ros2 launch moveit_resources_robot_moveit_config demo3.launch.py<br>

TODO<br>
- Please remove ros_ign<br>
- delete meshes folder in the /mobile_moveo_robot/robot_description <br>