#ifndef __LIBCAM2OPENCV
#define __LIBCAM2OPENCV

/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Ideas on Board Oy.
 * Copyright (C) 2024-2026, Bernd Porr
 * Copyright (C) 2021, kbarni https://github.com/kbarni/
 */

#include "libcam2opencv_format_converter.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <sys/mman.h>

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
   * Exposure Time (in microseconds). A zero lets libcamera decide the
   *exposure time.
   **/
  int64_t exposureTime = 0;

  /**
   * Exposure Value (EV) adjustment. By convention EV adjusts the exposure as
   *log2. For example EV = [-2, -1, -0.5, 0, 0.5, 1, 2] results in an
   *exposure adjustment of [1/4x, 1/2x, 1/sqrt(2)x, 1x, sqrt(2)x, 2x, 4x].
   **/
  float exposureValue = 0.0;

  /**
   * Saturation adjustment. Default is 1.0, 0.0 is greyscale
   **/
  float saturation = 1.0f;

  /**
   * Set the focus position (e.g. for Raspberry Pi Camera Module 3).
   *(Lensposition) 0.0 is closes, 1.0 is furthest. Keep at < 0 for auto
   **/
  float lensPosition = -1.0f;
};

class Libcam2OpenCV {
public:
  /**
   * The callback prototype carrying the current openCV frame.
   * This callback needs to be defined by you and usually as a lambda
   * function:
   * [&](const cv::Mat &mat, const libcamera::ControlList &) {
   * myImageProcessor(mat); }); which is then registered with
   * registerCallback.
   */
  using OnFrame =
      std::function<void(const cv::Mat &, const libcamera::ControlList &)>;

  /**
   * Register the callback for the frame data. This will be usually done with
   *a lambda function connecting Libcam2OpenCV to a class processing the
   *image: registerCallback([&](const cv::Mat &mat, const
   *libcamera::ControlList &){ mystuff.process(mat); });
   **/
  void registerCallback(OnFrame cb) { onFrame = cb; }

  /**
   * Starts the camera and the callback at default resolution and framerate
   **/
  void start(libcamera::CameraManager &cm,
             Libcam2OpenCVSettings settings = Libcam2OpenCVSettings());

  /**
   * Stops the camera and the callback
   **/
  void stop();

  ~Libcam2OpenCV() { stop(); }

private:
  std::shared_ptr<libcamera::Camera> camera;
  std::map<libcamera::FrameBuffer *, std::vector<libcamera::Span<uint8_t>>>
      framebuffer2memory;
  std::unique_ptr<libcamera::CameraConfiguration> config;
  OnFrame onFrame;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
  libcamera::Stream *stream = nullptr;
  std::vector<std::unique_ptr<libcamera::Request>> requests;
  libcamera::ControlList controls;
  FormatConverter formatConverter;
  std::mutex shutdown_mutex;

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
