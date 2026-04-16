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

## Prerequisites

```
apt install libopencv-dev libcamera-dev libturbojpeg0-dev
```

## Compilation and installation

```
cmake .
make
sudo make install
```

## How to use it

 1. Include `libcam2opencv.h` and add `target_link_libraries(yourproj cam2opencv)` to your `CMakeLists.txt`.

 2. Create a class containing a callback receiving the frames:
```
    class MyAI {
	    public:
        void onFrame(const cv::Mat &frame) {
                ai->detect(frame);
        }
    };
```

 3. Create instances of the camera and your application:

```
Libcam2OpenCV camera;
MyAI myAI;
```

 4. Register the callback

```
   camera.registerCallback([&](const cv::Mat &mat, const libcamera::ControlList &meta){ myAI.onFrame(mat); });
```

 5. Create an instance of the cameramanager (only one allowed for all cameras) and start it:
```
libcamera::CameraManager cm;
cm.start();

```

 6. Start the camera delivering frames via the callback

```
camera.start(cm);
```

 7. sleep or run your GUI or wait for a keypress

 8. Stop the camera and the cameramanager

```
camera.stop();
cm.stop();
```

## Examples

### Metadata printer

In the subdirectory `metadataprinter` is a demo which just prints the sensor
metadata from the callback. This is useful to see what
info is available for example the sensor timestamp to
check the framerate.

### QT Image Viewer

The subdirectory `qtviewer` contains a simple QT application which displays the camera on screen. This shows also shows how the callback triggers QT's window update without resorting to message queues as it deals with the heavy lifting itself. The callback returns as quickly as possible.

![alt tag](qtviewer_screenshot.png)

### Dual Camera Viewer

The subdirectory `dualcamviewer` contains a QT application which displays two cameras on screen. The Raspberry PI allows 2 cameras to be connected and you see how two callbacks per camera can be set up. Again, it demos how the image update is handled with QT's upated() command keeping the callback as short as possible and leaving it to QT to do the actual displaying.

## Credits

Based on https://github.com/kbingham/simple-cam, libcamera's qcam and then turned into this library by Bernd Porr. Additional features by Raphael Nekam.
