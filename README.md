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
  Download the Python practice code located in the OpenCV example source code folder within the Practice source code folder in the GitHub repository to the 'scripts' folder.
  ```shell
  chmod +x *
  ```
  ```shell
  cd ~/catkin_ws/ && catkin_make
  ```
  ```shell
  source devel/setup.bash
  ```
  <br><br>

5. Create a 'images' folder for practicing OpenCV:
  ```shell
  cd ~/catkin_ws/src/opencv_test
  ```
  ```shell
  mkdir images
  ```
  ```shell
  cd images
  ```
  Download the files 'image.jpg', 'road1.jpg', and 'road2.jpg' from the images folder within the Practice source code folder in the GitHub repository to the 'images' folder.
  <br><br><br>

6. Perform OpenCV practice in the ROS environment:
  ```shell
  cd ~/catkin_ws/src/opencv_test/scripts
  ```
  <br>
  Note: Ensure to set the 'image_path' variable correctly to the path on your local PC within each Python practice code.<br><br>
  
  6-1. Practice displaying images using OpenCV.
  ```shell
  python3 show_image.py
  ```
  <br>

  6-2. Practice converting images to gray scale using OpenCV.
  ```shell
  python3 show_gray_scale_image.py
  ```
  <br>

  6-3. Practice edge detection in images using OpenCV.
  ```shell
  python3 show_edge_image.py
  ```
  <br>

  6-4. Practice contour detection in images using OpenCV.
  ```shell
  python3 show_contour_image.py
  ```
  <br>

  6-5. Practice image rotation, resizing, and skew transformations using OpenCV.
  ```shell
  python3 show_turn_resize_affine_image.py
  ```
  <br>

  6-6. Practice image filtering using Gaussian blur in OpenCV.
  ```shell
  python3 show_blurred_image.py
  ```
  <br>

  6-7. Practice face detection in images using OpenCV.  
  ```shell
  python3 show_face_detection_image.py
  ```
  <br>
  6-8. Practice lane detection in images using Hough transform in OpenCV.
  
  ```shell
  python3 hough_lane_detection.py
  ```
  <br>

  6-9. Practice lane detection in images using HSV(Hue+Saturation+Value) in OpenCV.
  ```shell
  python3 hsv_lane_detection.py
  ```
  <br>

  6-10. Practice lane detection in images using Hough transform and HSV(Hue+Saturation+Value) in OpenCV.
  
  ```shell
  python3 hough_hsv_lane_detection.py
  ```
  <br><br>


7. Practice lane detection and tracking using the ROS Gazebo simulation environment:
  
  7-1. ROS Gazebo simulation environment setup.
  ```shell
  cd ~/catkin_ws/src/
  ```
  <br>

  ```shell
  git clone https://github.com/gihoonbackend/ackermann_vehicle.git
  ```
  or
  ```shell
  git clone https://github.com/0-keun/ackermann_vehicle.git
  ```
  <br>

  ```shell
  sudo apt install ros-noetic-ackermann-msgs
  ```
  <br>

  ```shell
  cd ..
  ```
  <br>

  ```shell
  rosdep install --from-paths src --ignore-src -r –y
  ```
  <br>

  ```shell
  catkin_make
  ```
  <br>

  ```shell
  source devel/setup.bash
  ```
  <br>

  ```shell
  gedit ~/.bashrc
  ```
  ```shell
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/{your PC name}/catkin_ws/src/ackermann_vehicle/roadmap_generator
  ```
  ```shell
  source ~/.bashrc
  ```
  <br>

  7-2. ~~.
  ```shell
  ~~
  ```
  <br>
