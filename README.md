# drv_package
Deep Robot Vision system for NEU household service robot
## 1. Software Prerequisites
### 1.1 ROS
We run this package in **indigo**, other versions of ROS may not be supported.
### 1.2 Caffe
We use Caffe to implement our tracking and searching methods. More specifically, 
the tracking method (GOTURN) use original Caffe, while the searching method (Faster RCNN) use customized Caffe. 
To install original Caffe, refer <http://caffe.berkeleyvision.org/installation.html> and follow the 'CMake Build' instruction, as for the caffe-faster-rcnn, refer 1.2.
### 1.3 py-faster-rcnn
This package is not included in the drv_package, you need to get it from <https://github.com/rbgirshick/py-faster-rcnn>, 
and change the 8th line in file 'process.py' according to where you installed the py-faster-rcnn. The pre-trained model is also needed,
by default, we have *faster_rcnn_test.pt* in $PY_FASTER_RCNN/models/DRV which is the same as the one in  $PY_FASTER_RCNN/models/pascal_voc/VGG16/faster_rcnn_alt_opt/. Besides, we have *VGG16_faster_rcnn_final.caffemodel* in $PY_FASTER_RCNN/model/DRV/ which you can obtain from [Caffe Model Zoo](https://github.com/BVLC/caffe/wiki/Model-Zoo) by executing `./data/scripts/fetch_imagenet_models.sh` in $PY_FASTER_RCNN.
### 1.4 rosserial
Please refer [Arduino IDE Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) for using rosserial. We use **rosserial_arduino** to communicate between one Arduino Uno board (may change to Mega 2560 in near future) and an ADXL345 3-axis accelerator. The Arduino controls 2 servos, one for pitching the RGBD-camera (ASUS Xtion Pro or ASTRA) and the other for rotating it. The accelerator measures the pitch and yaw angles of the camera. The .ino file loaded on the Arduino board is provided in folder *$DRV_DIR/supplements/arduino_control*, you can load it to your board with Arduino IDE <http://arduino.cc/en/Main/Software>.
### 1.5 astra_camera or openni_camera
Using astra_camera is recommended. Install openni_camera from binary is also supported. You can find the calibration files for our device in folder *$DRV_DIR/supplements/camera_info* for reference purpose. When running in ROS, this folder should be put into */.ros*.
### 1.6 GOTURN
While GOTURN itself is not necessary to be compiled to run this program, we still need the trained model tracker.caffemodel to be put in /home/aicrobo/GOTURN/nets/models/pretrained_model. You can get GOTURN form <https://github.com/davheld/GOTURN>. If your route to the caffemodel is different from above, you need to modify the route declarations in **drv_track.cpp** which is in folder *$DRV_DIR/drv_track*.

## 2 Hardware
To run searching and tracking modules smoothly, a workstation with least 2GB of GRAM is necessary. This program has been tested on multi-machine configurations, in which the host computer's CPUs run at frequencies exceeding 2.4 GHz. Low frequencies will lead to a delay in point cloud capturing and communication between the workstation and the host PC.

## 3. Installation
1. Clone this repository into catkin_ws/src:
`git clone https://github.com/NEU-TEAM/drv_package.git`
2. First `catkin_make` drv_msgs to generate header file used by other node packages.
3. Then run `catkin_make` to make all the rest packages.

## 4. Usage
1. Run `roscore` first.
2. Run launch file: `roslaunch drv_brain drv.launch` to launch the whole robot vision system. If you use astra_camera, run `roslaunch drv_brain drv_astra.launch` instead. The only difference between the two launch files is the camera_node being launched.
3. If you use DRV on multiple machines, `roslaunch drv_brain drv_host.launch` on host machine (which is on the robot), and `roslaunch drv_brain drv_workstation.launch` on the workstation to control the robot and process data remotely.
4. Target can be set by setting rosparam as follows: `rosparam set /comm/control/target/label bottle`, `rosparam set /comm/control/target/is_set true`,the setting order should exactly as this one. Here the 'bottle' refer to the target label and can be changed to 'chair', 'person', etc. For the record, setting target function and more useful functions beyond that can be easily realized with JARVIS <https://github.com/NEU-TEAM/JARVIS>, which is an Android app for controlling the NEU household robot. Till now it only supports Android 6.0 and above.
5. With the target set, the system will automatically run in *search mode* and finds the target in the scene. If some suspected objects were found, the system will call for user input to judge the result and decide whether continuing searching the target or tracking the confirmed target. If the target is confirmed, the system will run in *tracking mode* in which the location of the target in 3D space will be generated for manipulating the target.
6. By setting the target *is_set* param to be *false*, the system will run in *wander mode* and do nothing.

## 5. Trouble Shooting
1. If custom message issue occurred when running catkin_make, run `catkin_make --pkg drv_msgs --force-cmake` first to make the msg header files needed, and then run `catkin_make`.
2. If you use astra camera and find the point cloud edge is not quite well, first make sure you get the official source code from <https://github.com/orbbec/ros_astra_camera.git>, clone it to your catkin_ws and try `catkin_make --pkg astra_camera -DFILTER=ON`.
