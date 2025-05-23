#ifndef __LIBCAM2OPENCV
#define __LIBCAM2OPENCV

/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Ideas on Board Oy.
 * Copyright (C) 2024, Bernd Porr
 * Copyright (C) 2021, kbarni https://github.com/kbarni/
 */

#include <iomanip>
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <memory>
#include <sys/mman.h>
#include <opencv2/opencv.hpp>

// need to undefine QT defines here as libcamera uses the same expressions (!).
#undef signals
#undef slots
#undef emit
#undef foreach

#include <libcamera/libcamera.h>

/**
 * Settings
 **/
struct Libcam2OpenCVSettings {
    /**
     * Index of the camera used. Default is 0.
     **/
    unsigned int cameraIndex = 0;
    
    /**
     * Width of the video capture. A zero lets libcamera decide the width.
     **/
    unsigned int width = 0;
    
    /**
     * Height of the video capture. A zero lets libcamera decide the height.
     **/
    unsigned int height = 0;

    /**
     * Framerate. A zero lets libcamera decide the framerate.
     **/
    unsigned int framerate = 0;

    /**
     * Brightness
     **/
    float brightness = 0.0;

    /**
     * Contrast
     **/
    float contrast = 1.0;
    
    /**
     * Exposure Time (in microseconds). A zero lets libcamera decide the exposure time.
     **/
    int64_t exposureTime = 0;
    
    /**
     * Exposure Value (EV) adjustment. By convention EV adjusts the exposure as log2. For example EV = [-2, -1, -0.5, 0, 0.5, 1, 2] results in an exposure adjustment of [1/4x, 1/2x, 1/sqrt(2)x, 1x, sqrt(2)x, 2x, 4x].
     **/
    float exposureValue = 0.0;
    
    /**
     * Saturation adjustment. Default is 1.0, 0.0 is greyscale
     **/
    float saturation = 1.0f;

    /**
     * Set the focus position (e.g. for Raspberry Pi Camera Module 3). (Lensposition) 0.0 is closes, 1.0 is furthest. Keep at < 0 for auto
     **/
    float lensPosition = -1.0f;
     
};

class Libcam2OpenCV {
public:
    struct Callback {
	virtual void hasFrame(const cv::Mat &frame, const libcamera::ControlList &metadata) = 0;
	virtual ~Callback() {}
    };

    /**
     * Register the callback for the frame data
     **/
    void registerCallback(Callback* cb) {
	callback = cb;
    }

    /**
     * Starts the camera and the callback at default resolution and framerate
     **/
    void start(Libcam2OpenCVSettings settings = Libcam2OpenCVSettings() );

    /**
     * Stops the camera and the callback
     **/
    void stop();

    ~Libcam2OpenCV() {
	stop();
    }
    
private:
    std::shared_ptr<libcamera::Camera> camera;
    std::map<libcamera::FrameBuffer *, std::vector<libcamera::Span<uint8_t>>> mapped_buffers;
    std::unique_ptr<libcamera::CameraConfiguration> config;
    cv::Mat frame;
    Callback* callback = nullptr;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
    libcamera::Stream *stream = nullptr;
    std::unique_ptr<libcamera::CameraManager> cm;
    std::vector<std::unique_ptr<libcamera::Request>> requests;
    libcamera::ControlList controls;

    std::vector<libcamera::Span<uint8_t>> Mmap(libcamera::FrameBuffer *buffer) const
    {
	auto item = mapped_buffers.find(buffer);
	if (item == mapped_buffers.end())
	    return {};
	return item->second;
    }

    /*
     * --------------------------------------------------------------------
     * Handle RequestComplete
     *
     * For each Camera::requestCompleted Signal emitted from the Camera the
     * connected Slot is invoked.
     */
    void requestComplete(libcamera::Request *request);
};

#endif
