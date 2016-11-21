# drv_package
Deep Robot Vision system for NEU household service robot

##1. Prerequisites (dependencies)
###1.1 Caffe
We use Caffe to implement our tracking and searching method. More specifically, the tracking method (i.e. GOTURN) use original Caffe, while the searching method (i.e. Faster RCNN) use customized Caffe. To install original Caffe, refer <http://caffe.berkeleyvision.org/installation.html> and follow the 'CMake Build' instruction, as for the caffe-faster-rcnn, refer 1.2.
###1.2 py-faster-rcnn
This package is not included in the drv_package, you need to get it from <https://github.com/rbgirshick/py-faster-rcnn>, and change the 8th line in file 'process.py' according to where you installed the py-faster-rcnn.
###1.3 rosserial
Please refer the site <http://wiki.ros.org/rosserial>. We use **rosserial_arduino** to communicate with one Arduino Uno board and an ADXL345 3-axis accelerator. And that is used to control 2 servos, one for pitch the RGBD-camera (ASUS Xtion Pro) and the other for yaw the RGBD-camera. The accelerator is used to get the pitch and yaw angles of the camera. The .ino file which is loaded to the Arduino board is provided.  
###1.4 openni_camera or astra_camera
Install openni_camera from binary is recommended. The astra_camera is also supported.
###1.5 GOTURN
While GOTURN itself is not necessary to be compiled to run this program, we still need the trained model tracker.caffemodel to be put in /home/aicrobo/GOTURN/nets/models/pretrained_model, if your route to the caffemodel is different from this one, you need to modify the route declarations in **drv_track.cpp**.

##2. Installation
1. Clone this repository into catkin_ws/src:
`git clone https://github.com/NEU-TEAM/drv_package.git`
2. Build all by running `catkin_make` in /catkin_ws.

##3. Usage
1. Run `roscore` first.
2. Run launch file: `roslaunch drv_brain drv.launch` to launch the whole robot vision system. If you use astra_camera, run `roslaunch drv_brain drv_astra.launch` instead. The only difference between the two launch files is the camera_node being launched.
3. Set target by set rosparam as follows: `rosparam set /comm/control/target/label bottle`, `rosparam set /comm/control/target/is_set true` , here 'bottle' refer to the target label and can be changed to 'chair', 'person' etc.
4. With target set, the system will automatically run in *search mode* and find the target in the scene. If some suspected objects were found, the system will call for user input to judge the result and decide whether continue searching the target or tracking the confirmed target. If the target is confirmed, the system will run in *tracking mode* and location of the target in space will be generated for manipulating the target.
5. By setting the target *is_set* param to be *false*, the system will run in *wander mode* and do nothing.