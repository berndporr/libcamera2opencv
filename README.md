# Raspberry PI libcamera to openCV library with callback

This is a wrapper around libcamera which makes it super easy to establish
a callback containing an openCV matrix.

The raw callback interface of libcamera leaves it to the user to do
complex memory mapping with a lot of implicit conventions 
to access the raw image data. It also leaves it to the user to convert
it to standard RGB. For example, the MJPEG stream from a USB camera
is not decoded but leaves it to the user.
This library is an attempt to abstract all this
complexity away and just provide the user with an BGR openCV matrix
no matter what raw format the camera has.

## Prerequisites

```
apt install libopencv-dev libcamera-dev libjpeg-turbo8-dev libturbojpeg0-dev
```

## Compilation and installation

```
cmake .
make
sudo make install
```

## How to use it

 1. Include `libcam2opencv.h` and add `target_link_libraries(yourproj cam2opencv)` to your `CMakeLists.txt`.

 2. Create a class containing this callback:
```
    struct MyCallback {
        virtual void onFrame(const cv::Mat &frame, const libcamera:ControlList &) {
            if (nullptr != window) {
                window->updateImage(frame);
            }
        }
    };
```

 3. Create instances of the the camera and the callback:

```
Libcam2OpenCV camera;
MyCallback myCallback;
```

 4. Register the callback

```
   camera.registerCallback([&](const cv::Mat &mat, const libcamera::ControlList &meta){ myCallback.onFrame(mat, meta); });

```

 5. Start the camera delivering frames via the callback

```
camera.start();
```

 6. Stop the camera

```
camera.stop();
```

## Examples

### Metadata printer

In the subdirectory `metadataprinter` is a demo which just prints the sensor
metadata from the callback. This is useful to see what
info is available for example the sensor timestamp to
check the framerate.

### QT Image Viewer

The subdirectory `qtviewer` contains a simple QT application
which displays the camera on screen.

![alt tag](qtviewer_screenshot.png)

## Credits

Based on https://github.com/kbingham/simple-cam, libcamera's qcam and then turned into this library by Bernd Porr. Additional features by Raphael Nekam.
