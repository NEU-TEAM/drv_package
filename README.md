# drv_package
Deep Robot Vision system for NEU household service robot
## 1. Softeware Prerequisites
### 1.1 ROS
We run this package in **indigo**, other versions of ROS may not be supported.
### 1.2 Caffe
We use Caffe to implement our tracking and searching method. More specifically, the tracking method (i.e. GOTURN) use original Caffe, while the searching method (i.e. Faster RCNN) use customized Caffe. To install original Caffe, refer <http://caffe.berkeleyvision.org/installation.html> and follow the 'CMake Build' instruction, as for the caffe-faster-rcnn, refer 1.2.
### 1.3 py-faster-rcnn
This package is not included in the drv_package, you need to get it from <https://github.com/rbgirshick/py-faster-rcnn>, and change the 8th line in file 'process.py' according to where you installed the py-faster-rcnn.
### 1.4 rosserial
Please refer the site <http://wiki.ros.org/rosserial> for using rosserial. We use **rosserial_arduino** to communicate with one Arduino Uno board (may change to Mega 2560 in near future) and an ADXL345 3-axis accelerator. The Arduino is used to control 2 servos, one for pitching the RGBD-camera (ASUS Xtion Pro) and the other for rotating it. The accelerator is used to get the pitch and yaw angles of the camera. The .ino file which is loaded to the Arduino board is provided in folder *arduino_control*, you can load it to your board with Arduino IDE <http://arduino.cc/en/Main/Software>.  
### 1.5 openni_camera or astra_camera
Install openni_camera from binary is recommended. The astra_camera is also supported. You can find the calibration files we use in the *camera_info* folder for reference purpose. When run in ROS, this folder should be put in */.ros*.
### 1.6 GOTURN
While GOTURN itself is not necessary to be compiled to run this program, we still need the trained model tracker.caffemodel to be put in /home/aicrobo/GOTURN/nets/models/pretrained_model. You can get GOTURN form <https://github.com/davheld/GOTURN>. If your route to the caffemodel is different from above, you need to modify the route declarations in **drv_track.cpp** which is in the folder /drv_package/drv_track/.
## 2 Hardware
To run searching and tracking modules smoothly, a workstation with least 2GB of GPU RAM is necessary. This program has been tested on multi-machine configurations, in which the host computer's CPUs run at frequencies exceeding 2.4 GHz. A low frequencies will lead to delay in communication between the workstation and the host pc.
## 3. Installation
1. Clone this repository into catkin_ws/src:
`git clone https://github.com/NEU-TEAM/drv_package.git`
2. First `catkin_make` drv_msgs to generate header file used by other node packages.
3. Then run `catkin_make` to make all the rest packages.
## 4. Usage
1. Run `roscore` first.
2. Run launch file: `roslaunch drv_brain drv.launch` to launch the whole robot vision system. If you use astra_camera, run `roslaunch drv_brain drv_astra.launch` instead. The only difference between the two launch files is the camera_node being launched.
3. If you use DRV on multiple machines, `roslaunch drv_brain drv_host.launch` on host machine (which is on the robot), and `roslaunch drv_brain drv_workstation.launch` on the workstation to control the robot and process data remotely.
4. Set target by setting rosparam as follows: `rosparam set /comm/control/target/label bottle`, `rosparam set /comm/control/target/is_set true` , here 'bottle' refer to the target label and can be changed to 'chair', 'person', etc. Notice that set target function and more useful functions can be easily realized with JARVIS <https://github.com/NEU-TEAM/JARVIS>, which is a Android app for controlling the NEU household robot.
5. With target set, the system will automatically run in *search mode* and find the target in the scene. If some suspected objects were found, the system will call for user input to judge the result and decide whether continue searching the target or tracking the confirmed target. If the target is confirmed, the system will run in *tracking mode* and location of the target in space will be generated for manipulating the target.
6. By setting the target *is_set* param to be *false*, the system will run in *wander mode* and do nothing.
## 5. Trouble Shooting
