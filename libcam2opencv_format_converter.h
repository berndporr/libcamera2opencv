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
 */
class FormatConverter
{
public:
	/**
	 * The input format which won't require any conversion. libcamera should be
	 * told to use this format so that ideally no conversion or even memory
	 * allocation is needed here at all.
	 * Note that libcamera::formats::RGB888 actually is BGR (!) so that it's compatible
	 * with openCV's BGR and thus won't need conversion.
	 */
	static constexpr libcamera::PixelFormat nativeInputFormat = libcamera::formats::RGB888;

	/**
	 * The output openCV format no matter what libcamera's native input format is.
	 * We always output 8 bit unsigned BGR, the native openCV format.
	 */
	static constexpr int openCVoutputFormat = CV_8UC3;

	/**
	 * Starts the converter and will allocate memory if needed.
	 *
	 * @param format The libcamera pixel format.
	 * @param width The width of a frame.
	 * @param height The height of a frame.
	 * @param stride The stride between rows.
	 */
	void start(const libcamera::PixelFormat format, int width, int height, int stride);

	/**
	 * Frees up any memory.
	 */
	void stop();

	/**
	 * Destroy the Format Converter object. This also frees up any internal memory.
	 */
	~FormatConverter()
	{
		stop();
	}

	/**
	 * Converts a raw image and returns an openCV image.
	 * @param planes The libcamera planes which contain the raw image.
	 * @return cv::Mat The openCV matrix which contains the converted image.
	 */
	cv::Mat convert(const std::vector<libcamera::Span<uint8_t>> &planes);

private:
	enum FormatFamily
	{
		NATIVE,
		MJPEG,
		RGB,
		YUVPacked,
		YUVPlanar,
		YUVSemiPlanar,
	};

	void yuv_to_rgb(const int y, const int u, const int v, int *r, int *g, int *b) const;

	/**
	 * Motion JPEG decoding.
	 * 
	 * @param planes There is only a single plane per frame.
	 * @return cv::Mat The cv Mat in openCV's format.
	 */
	cv::Mat convertJPG(const std::vector<libcamera::Span<uint8_t>> &planes);

	/**
	 * Converts from to BGR from various RGB/RGBA/BGRA formats. Remember that libcamera's
	 * BGR is actually RGB (!) so needs to be re-shuffled.
	 * 
	 * @param planes The planes containing the RGB data.
	 * @return cv::Mat The openCV BGR Matrix.
	 */
	cv::Mat convertRGB(const std::vector<libcamera::Span<uint8_t>> &planes);

	cv::Mat convertYUVPacked(const std::vector<libcamera::Span<uint8_t>> &planes);
	cv::Mat convertYUVPlanar(const std::vector<libcamera::Span<uint8_t>> &planes);
	cv::Mat convertYUVSemiPlanar(const std::vector<libcamera::Span<uint8_t>> &planes);

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
	tjhandle tjInstance = NULL;

	/* The destination Image matrix */
	cv::Mat dstImage;
};
