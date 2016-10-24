# drv_package
Deep Robot Vision system for NEU household service robot
##1. Prerequisites (dependencies)
###1.1 Caffe
We use Caffe to impeliment our tracking and searching method. More specificlly, the tracking method (i.e. GOTURN) use original Caffe: , while the searching method (i.e. Faster RCNN) use customed Caffe. To install original Caffe, refer <>, as for the caffe-faster-rcnn, refer 1.2.
###1.2 py-faster-rcnn
This package is not included in the drv_package, you need get it from <>, and change the 8th line of file 'process.py' according to where you install the py-faster-rcnn.
