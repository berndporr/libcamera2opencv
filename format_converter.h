/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 * Copyright (C) 2026, Bernd Porr <mail@berndporr.me.uk>
 *
 * Convert buffer to RGB
 */

#pragma once

#include <stddef.h>

// need to undefine QT defines here as libcamera uses the same expressions (!).
#undef signals
#undef slots
#undef emit
#undef foreach

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>
#include <turbojpeg.h>

#include <vector>
#include <span>

/**
 * Format converter from any libcamera format to openCV's 
 * 
 */
class FormatConverter
{
public:
	void stop();
	void start(const libcamera::PixelFormat format, int width, int height, int stride);
	cv::Mat convert(const std::vector<libcamera::Span<uint8_t>> &srcmem);
	static constexpr libcamera::PixelFormat nativeInputFormat = libcamera::formats::RGB888;
	static constexpr int openCVoutputFormat = CV_8UC3;

private:
	enum FormatFamily {
		NATIVE,
		MJPEG,
		RGB,
		YUVPacked,
		YUVPlanar,
		YUVSemiPlanar,
	};

	void convertJPG(const std::vector<libcamera::Span<uint8_t>> &srcmem);
	void convertRGB(const std::vector<libcamera::Span<uint8_t>> &srcmem);
	void convertYUVPacked(const std::vector<libcamera::Span<uint8_t>> &srcmem);
	void convertYUVPlanar(const std::vector<libcamera::Span<uint8_t>> &srcmem);
	void convertYUVSemiPlanar(const std::vector<libcamera::Span<uint8_t>> &srcmem);

	libcamera::PixelFormat format_;
	unsigned int width_;
	unsigned int height_;
	unsigned int stride_;

	enum FormatFamily formatFamily_;

	/* NV parameters */
	unsigned int horzSubSample_;
	unsigned int vertSubSample_;
	bool nvSwap_;

	/* RGB parameters */
	unsigned int bpp_;
	unsigned int r_pos_;
	unsigned int g_pos_;
	unsigned int b_pos_;

	/* YUV parameters */
	unsigned int y_pos_;
	unsigned int cb_pos_;

	/* JPEG decompressor */
	tjhandle tjInstance = nullptr;

	/* The destination matrix */
	cv::Mat dst;
};
