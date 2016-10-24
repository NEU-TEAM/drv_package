# drv_package
Deep Robot Vision system for NEU household service robot

##1. Prerequisites (dependencies)
###1.1 Caffe
We use Caffe to implement our tracking and searching method. More specifically, the tracking method (i.e. GOTURN) use original Caffe, while the searching method (i.e. Faster RCNN) use customized Caffe. To install original Caffe, refer <http://caffe.berkeleyvision.org/installation.html> and follow the 'CMake Build' instruction, as for the caffe-faster-rcnn, refer 1.2.
###1.2 py-faster-rcnn
This package is not included in the drv_package, you need to get it from <https://github.com/rbgirshick/py-faster-rcnn>, and change the 8th line in file 'process.py' according to where you install the py-faster-rcnn.
###1.3 rosserial
Please refer the site <http://wiki.ros.org/rosserial>. We use **rosserial_arduino** to communicate with one Arduino Uno board and an ADXL345 3-axis accelerator. The control board is used to control 2 servos, one for pitch the RGBD-camera (ASUS Xtion Pro) and the other for yaw the RGBD-camera. The accelerator is used to get the pitch and yaw angle of the camera. 
###1.4 openni_camera
Install openni_camera from binary is recommended.

##2. Installation
1. In your ROS package path (check your environment variable ROS_PACKAGE_PATH) clone this repository:
`git clone https://github.com/NEU-TEAM/drv_package.git`
2. Build all by running `catkin_make` in your workspace root.

##3. Usage
1. Run `roscore` first.
2. Run launch file: `roslaunch drv_brain drv.launch` to launch the whole robot vision system.
3. Set target by send rosmsg as follows: `rostopic pub /recognize/target drv_msgs/target_info '{label: {data: "bottle"}}'` , here 'bottle' refer to the target label and can be changed to 'chair', 'person' etc.
4. While sending target msg, the system will automatically run in *search mode* and find the target in the scene. If some suspected objects were found, the system will call for user input to judge the result and decide whether continue searching the target or tracking the confirmed target. If the target is confirmed, the system will run in *tracking mode* and grasp plan will be generated to manipulate the target.
5. By canceling publishing the target msg, the system will run in *wander mode* and do nothing.