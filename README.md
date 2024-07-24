# Cheongju University Camera(OpenCV) Lecture Notes<br>
  Author : Hwasu Lee<br><br>
  Affiliation : UNICON LAB (Incheon National University, South Korea)<br><br>
  Position : M.A. student<br><br>
  E-mail : leehwasu96@naver.com (or leehwasu9696@inu.ac.kr)<br><br>

#  Course Duration
  Date : 24.07.15 ~ 24.07.17<br><br>

# Camera(OpenCV) Lecture Note Description
  "Camera Lecture Notes (1).pdf is the Camera(OpenCV) lecture notes."<br><br>
  "Camera Lecture Notes (2).pdf is the Camera(YOLO v8) lecture notes."<br><br>

# Commands for practicing OpenCV(Python) based on ROS.

Please note that this practice was conducted in an<br><br> 
Ubuntu 20.04 LTS and ROS(Robot Operating System) 1 Noetic environment.<br><br>

To set up the project, follow these steps:<br><br>

1. Install libraries related to 'OpenCV':
  ```shell
  pip install opencv-python
  ```
  or
  ```shell
  pip3 install opencv-python
  ```
  <br>
  
  ```shell
  pip install numpy
  ```
  <br><br>

2. Create a package for practicing OpenCV in the ROS Noetic environment:
  ```shell
  cd ~/catkin_ws/src
  ```
  ```shell
  catkin_create_pkg opencv_test rospy std_msgs sensor_msgs cv_bridge
  ```
  <br><br>

3. Set up CV bridge for integrating OpenCV with the ROS environment:
  ```shell
  cd opencv_test
  ```
  ```shell
  gedit CMakeLists.txt
  ```
  Add the following commands to the file.
  ```shell
  include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )
  ```
  <br>
  
  ```shell
  gedit package.xml
  ```
  Add the following commands to the file.
  ```shell
  <build_depend>opencv</build_depend>
  <exec_depend>opencv</exec_depend>
  ```
<br><br>

4. Create a 'scripts' folder for practicing OpenCV and build the catkin workspace:
  ```shell
  cd ~/catkin_ws/src/opencv_test
  ```
  ```shell
  mkdir scripts
  ```
  ```shell
  cd scripts
  ```
  Download the Python practice code located in the OpenCV example source code folder within the   Practice source code folder from the GitHub repository to the 'scripts' folder.
  ```shell
  chmod +x *
  ```
  ```shell
  cd ~/catkin_ws/ && catkin_make
  ```
  ```shell
  source devel/setup.bash
  ```
