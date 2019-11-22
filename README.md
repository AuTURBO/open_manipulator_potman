# Description 
Open manipulator application pacakges.   
2019 R-BIZ ROBOTIS Open Manipulator Challenge.   
Auturbo team potman   

# HW Block Diagram 
<img src="/picture/1.PNG" width="100%" height="100%">  


# SW Block Diagram 
<img src="/picture/2.PNG" width="100%" height="100%">  


# Environment Setting  

## PC Setting  
### 0.open manipulator setting   
http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_setup/#install-ros-packages   

### 1.D435 realsense package install  
http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_applications/#installation-1   

### 2.JSK package install   
```bash
$ sudo apt-get install ros-kinetic-jsk-recognition
$ sudo apt-get install ros-kinetic-jsk-topic-tools
$ sudo apt-get install ros-kinetic-libsiftfast
$ sudo apt-get install ros-kinetic-laser-assembler
$ sudo apt-get install ros-kinetic-octomap-server
$ sudo apt-get install ros-kinetic-nodelet
$ sudo apt-get install ros-kinetic-depth-image-proc
$ cd ~/catkin_ws/src/
$ git clone https://github.com/jsk-ros-pkg/jsk_common.git
$ catkin_ws && catkin_make
```

### 3. Download this repository code    
```bash
$ cd ~/catkin_ws/src/
$ git clone --recursive https://github.com/minwoominwoominwoo7/darknet_ros.git
$ git clone https://github.com/AuTURBO/open_manipulator_potman.git
$ git clone https://github.com/minwoominwoominwoo7/open_manipulator_gazebo_potman.git
$ catkin_ws && catkin_make
```

## First Phone Setting to voicecommand 
You install the below application to phone.    
https://github.com/minwoominwoominwoo7/ros-app-connectedphone/blob/master/android_pubCommandVoice-debug.apk   

## Second Phone Setting to facetracking   
You install the below two applications to phone.   
https://github.com/minwoominwoominwoo7/ros-app-connectedphone/blob/master/android_pubFaceTracker-debug.apk   
https://github.com/minwoominwoominwoo7/facetracker-app-connectedphone/blob/master/facetracker.apk   
The facetracker.apk performs face detection and broadcasts the recognized face coordinates to the android_pubFaceTracker-debug.apk .    
The android_pubFaceTracker-debug.apk performs ros android.   
The android_pubFaceTracker-debug.apk send face detection data to ROS PC by wifi.


# RUN

## PC Run 
```bash
roslaunch open_manipulator_controller open_manipulator_controller.launch
roslaunch open_manipulator_potman yolo_jsk_pose.launch camera_model:=realsense_d435
roslaunch open_manipulator_potman potman.launch
roslaunch open_manipulator_potman key_command.launch
```
## First Phone Run to voicecommand 
Step1. click icon of android_pubCommandVoice-debug.apk. 
Step2. Enter ip address of PC's ROS core. and Press connection button.   
<img src="/picture/Screenshot_20191121-080800(1).png" width="30%" height="30%">   
Step3. And you can control the operation of the open manipulator by voice or button of application.    
Of course, you can also control the operation of the open manipulator by entering key to terminal of  command.launch on PC, even if you are not using the application   

<img src="/picture/Screenshot_20191121-080820(1).png" width="30%" height="30%">   

## Second Phone Run to facetracking   
Step1. click icon of android_pubFaceTracker-debug.apk    
Step2. Enter ip address of PC's ROS core. and Press connection button.    
Step3. And press the Home key to run the application as BackGround. Never press the back key.   

<img src="/picture/Screenshot_20191121-080914(1).png" width="30%" height="30%">   

Step4. click icon of facetracker.apk   
Only the largest face detected by the application is displayed on the camera preveiw screen.   

<img src="/picture/Screenshot_20191121-081524.png" width="30%" height="30%">   
Step5. Now you can see rostopic about data of detected face on ROS PC by rostopic of rqt.   

## Operation Mode 

There is 9 mode, You can control by PC( terminal of key_command.launch ) or phone ( voice control or button )   
### 0. Start Pour   
We read the fixed joint position coordinates from the saved file and moved Robot Arm(Open manipulator).    
We use Open Manipulator Control Package to control open manipulator by joint control.       
The way to save the joint movement file are located at the bottom of the page.   
Click image to link to YouTube video.   

[![Video Label](http://img.youtube.com/vi/TuQmHOsT_p8/0.jpg)](https://youtu.be/TuQmHOsT_p8?t=0s)    

### 1. Start cheers ( Picking cup & Cup tracking )
Click image to link to YouTube video.   
Used Packages   
. We use Darknet ROS Package to detect cup object. The used model is yolo V3.      
. We use JSK Package to calculation 3D position.     
. We use Open Manipulator Control Package to control open manipulator by inverse kinematic.   
[![Video Label](http://img.youtube.com/vi/rtBZDF8icJw/0.jpg)](https://youtu.be/rtBZDF8icJw?t=0s)   
[![Video Label](http://img.youtube.com/vi/esYlpivZYQo/0.jpg)](https://youtu.be/esYlpivZYQo?t=0s)   

### 2. Stop cheers ( Realease Cup )   

### 3. Serve Food     
We read the fixed joint position coordinates from the saved file and moved Robot Arm(Open manipulator).    
We use Open Manipulator Control Package to control open manipulator by joint control.       
The way to save the joint movement file are located at the bottom of the page.     
Click image to link to YouTube video.   
[![Video Label](http://img.youtube.com/vi/dL8CXf71Wz4/0.jpg)](https://youtu.be/dL8CXf71Wz4?t=0s)   
[![Video Label](http://img.youtube.com/vi/Kr8EuuMiMAU/0.jpg)](https://youtu.be/Kr8EuuMiMAU?t=0s)   

### 4. Start Phone ( PICKING phone & face tracking )    
The face detection algorithm is LargestFaceFocusingProcessor of com.google.android.gms.vision.face.FaceDetector.     
https://developers.google.com/android/reference/com/google/android/gms/vision/face/LargestFaceFocusingProcessor     
Click image to link to YouTube video.   
[![Video Label](http://img.youtube.com/vi/hsZrGeCw8Zo/0.jpg)](https://youtu.be/hsZrGeCw8Zo?t=0s)   

### 5. Stop Phone ( Realease Phone )   
### 6. Stop    
### 7. Restart    
### 8. Panorama Shot    

## Run Gazebo ( PC Side )   
```bash 
roslaunch open_manipulator_gazebo_potman open_manipulator_gazebo_potman.launch
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
roslaunch open_manipulator_potman yolo_jsk_pose.launch camera_model:=realsense_d435 use_platform:=false
roslaunch open_manipulator_potman potman.launch use_platform:=false
roslaunch open_manipulator_potman key_command.launch
``` 
Click image to link to YouTube video.   
[![Video Label](http://img.youtube.com/vi/u4CAUhD-lkU/0.jpg)](https://youtu.be/u4CAUhD-lkU?t=0s)  


# How to make joint movement file   
0. Start Pour and 3. Serve Food  is fixed movement by file.   
0. Start Pour use https://github.com/AuTURBO/open_manipulator_potman/blob/master/cfg/output_pour.txt   
3. Serve Food use https://github.com/AuTURBO/open_manipulator_potman/blob/master/cfg/output_serve.txt   
you can make movement file by open_manipulator_save_and_load   
https://github.com/minwoominwoominwoo7/open_manipulator_save_and_load   

[![Video Label](http://img.youtube.com/vi/PH-7JwGH9uM/0.jpg)](https://youtu.be/PH-7JwGH9uM?t=0s)    

