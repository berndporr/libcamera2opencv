# libcamera to openCV library

This is a wrapper around libcamera which hides all its complexity and
makes it as easy as possible to get a humble callback delivering openCV BGR
frames.

The motivation behind this wrapper is that libcamera's raw callback interface
forces the user to understand complex memory mapping, mmap and 
conventions buried deep in its example applications. 
This wrapper here is an attempt to abstract all
this complexity away and instead provide a simple friendly callback delivering openCV BGR frames
so that the coder can focus on their actual task.

Works with:
 - Raspberry PI CSI cameras
 - Webcams delivering MJPEG

Github project page: https://github.com/berndporr/libcamera2opencv
