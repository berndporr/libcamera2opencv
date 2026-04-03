#include "libcam2opencv.h"
#include <unistd.h>
#include <stdexcept>

void Libcam2OpenCV::requestComplete(libcamera::Request *request)
{
    if (nullptr == request)
        return;
    if (request->status() == libcamera::Request::RequestCancelled)
        return;

    /*
     * When a request has completed, it is populated with a metadata control
     * list that allows an application to determine various properties of
     * the completed request. This can include the timestamp of the Sensor
     * capture, or its gain and exposure values, or properties from the IPA
     * such as the state of the 3A algorithms.
     *
     * ControlValue types have a toString, so to examine each request, print
     * all the metadata for inspection. A custom application can parse each
     * of these items and process them according to its needs.
     */
    const libcamera::ControlList &requestMetadata = request->metadata();

    /*
     * Each buffer has its own FrameMetadata to describe its state, or the
     * usage of each buffer. While in our simple capture we only provide one
     * buffer per request, a request can have a buffer for each stream that
     * is established when configuring the camera.
     *
     * This allows a viewfinder and a still image to be processed at the
     * same time, or to allow obtaining the RAW capture buffer from the
     * sensor along with the image as processed by the ISP.
     */
    const libcamera::Request::BufferMap &buffers = request->buffers();
    for (auto bufferPair : buffers)
    {
        libcamera::FrameBuffer *buffer = bufferPair.second;
        libcamera::StreamConfiguration &streamConfig = config->at(0);
        int vw = streamConfig.size.width;
        int vh = streamConfig.size.height;
        int vstr = streamConfig.stride;
        size_t imageSize = buffer->metadata().planes()[0].bytesused;
        auto mem = mappedPlanes[buffer];
        uint8_t *ptr = mem[0].data();
        cv::Mat frame(vh, vw, CV_8UC3);
        if (streamConfig.pixelFormat == libcamera::formats::MJPEG)
        {
            int jpegSubsamp, jpegColorspace;

            // Read JPEG header
            if (tjDecompressHeader3(tjInstance, ptr, imageSize,
                                    &vw, &vh, &jpegSubsamp, &jpegColorspace) != 0)
            {
                std::cerr << tjGetErrorStr() << std::endl;
            }

            // Decompress to RGB
            if (tjDecompress2(tjInstance, ptr, imageSize,
                              frame.data, vw, 0, vh,
                              TJPF_BGR, TJFLAG_FASTDCT) != 0)
            {
                std::cerr << tjGetErrorStr() << std::endl;
            }
        }
        else
        {
            fprintf(stderr, "%dx%d,%d\n", vw, vh, vstr);
            uint ls = vw * 3;
            for (unsigned int i = 0; i < vh; i++, ptr += vstr)
            {
                memcpy(frame.ptr(i), ptr, ls);
            }
        }
        if (nullptr != callback)
        {
            callback->hasFrame(frame, requestMetadata);
        }
    }

    // in case the request has been cancelled in the meantime
    // this is a hack because libcamera should wait till a request has finisehd but doesn't
    if (nullptr == request)
        return;
    if (request->status() == libcamera::Request::RequestCancelled)
        return;
    /* Re-queue the Request to the camera. */
    request->reuse(libcamera::Request::ReuseBuffers);
    camera->queueRequest(request);
}

void Libcam2OpenCV::start(Libcam2OpenCVSettings settings)
{
    tjInstance = tjInitDecompress();
    if (!tjInstance)
        throw std::runtime_error("Failed to initialize TurboJPEG decompressor.");

    /*
     * --------------------------------------------------------------------
     * Create a Camera Manager.
     *
     * The Camera Manager is responsible for enumerating all the Camera
     * in the system, by associating Pipeline Handlers with media entities
     * registered in the system.
     *
     * The CameraManager provides a list of available Cameras that
     * applications can operate on.
     *
     * When the CameraManager is no longer to be used, it should be deleted.
     * We use a unique_ptr here to manage the lifetime automatically.
     *
     * There can only be a single CameraManager constructed within any
     * process space.
     */
    cm = std::make_unique<libcamera::CameraManager>();
    cm->start();

    /*
     * Just as a test, generate names of the Cameras registered in the
     * system, and list them.
     */
    std::cerr << "Cams:" << std::endl;
    for (auto const &camera : cm->cameras())
        std::cerr << " - " << camera.get()->id() << std::endl;

    /*
     * --------------------------------------------------------------------
     * Camera
     *
     * Camera are entities created by pipeline handlers, inspecting the
     * entities registered in the system and reported to applications
     * by the CameraManager.
     *
     * In general terms, a Camera corresponds to a single image source
     * available in the system, such as an image sensor.
     *
     * Application lock usage of Camera by 'acquiring' them.
     * Once done with it, application shall similarly 'release' the Camera.
     */
    if (cm->cameras().empty())
    {
        std::cerr << "No cameras were identified on the system."
                  << std::endl;
        cm->stop();
        return;
    }

    if (settings.cameraIndex >= cm->cameras().size())
    {
        std::cerr << "Camera index out of range."
                  << std::endl;
        cm->stop();
        return;
    }

    std::string cameraId = cm->cameras()[settings.cameraIndex]->id();
    camera = cm->get(cameraId);
    camera->acquire();

    /*
     * Stream
     *
     * Each Camera supports a variable number of Stream. A Stream is
     * produced by processing data produced by an image source, usually
     * by an ISP.
     *
     *   +-------------------------------------------------------+
     *   | Camera                                                |
     *   |                +-----------+                          |
     *   | +--------+     |           |------> [  Main output  ] |
     *   | | Image  |     |           |                          |
     *   | |        |---->|    ISP    |------> [   Viewfinder  ] |
     *   | | Source |     |           |                          |
     *   | +--------+     |           |------> [ Still Capture ] |
     *   |                +-----------+                          |
     *   +-------------------------------------------------------+
     *
     * The number and capabilities of the Stream in a Camera are
     * a platform dependent property, and it's the pipeline handler
     * implementation that has the responsibility of correctly
     * report them.
     */

    /*
     * --------------------------------------------------------------------
     * Camera Configuration.
     *
     * Camera configuration is tricky! It boils down to assign resources
     * of the system (such as DMA engines, scalers, format converters) to
     * the different image streams an application has requested.
     *
     * Depending on the system characteristics, some combinations of
     * sizes, formats and stream usages might or might not be possible.
     *
     * A Camera produces a CameraConfigration based on a set of intended
     * roles for each Stream the application requires.
     */
    config = camera->generateConfiguration({libcamera::StreamRole::Viewfinder});

    /*
     * The CameraConfiguration contains a StreamConfiguration instance
     * for each StreamRole requested by the application, provided
     * the Camera can support all of them.
     *
     * Each StreamConfiguration has default size and format, assigned
     * by the Camera depending on the Role the application has requested.
     */
    libcamera::StreamConfiguration &streamConfig = config->at(0);

    /*
     * Each StreamConfiguration parameter which is part of a
     * CameraConfiguration can be independently modified by the
     * application.
     *
     * In order to validate the modified parameter, the CameraConfiguration
     * should be validated -before- the CameraConfiguration gets applied
     * to the Camera.
     *
     * The CameraConfiguration validation process adjusts each
     * StreamConfiguration to a valid value.
     */

    /*
     * The Camera configuration procedure fails with invalid parameters.
     */
    if ((settings.width > 0) && (settings.height > 0))
    {
        streamConfig.size.width = settings.width;
        streamConfig.size.height = settings.height;
        int ret = camera->configure(config.get());
        if (ret)
        {
            std::cerr << "CONFIGURATION FAILED!" << std::endl;
            throw;
        }
    }

    // opencv compatible format
    streamConfig.pixelFormat = libcamera::formats::RGB888;

    /*
     * Validating a CameraConfiguration -before- applying it will adjust it
     * to a valid configuration which is as close as possible to the one
     * requested.
     */
    config->validate();

    std::cerr << "Stream configuration adjusted to "
              << streamConfig.toString() << std::endl;

    /*
     * Once we have a validated configuration, we can apply it to the
     * Camera.
     */
    camera->configure(config.get());

    /* Allocate and map buffers. */
    allocator = new libcamera::FrameBufferAllocator(camera);
    for (libcamera::StreamConfiguration &config : *config)
    {
        libcamera::Stream *stream = config.stream();

        int ret = allocator->allocate(stream);
        if (ret < 0)
        {
            std::cerr << "Failed to allocate capture buffers";
            throw;
        }

        for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator->buffers(stream))
        {
            struct MappedBufferInfo
            {
                uint8_t *address = nullptr;
                size_t mapLength = 0;
                size_t dmabufLength = 0;
            };
            std::map<int, MappedBufferInfo> mappedBuffers;

            for (const libcamera::FrameBuffer::Plane &plane : buffer->planes())
            {
                const int fd = plane.fd.get();
                if (mappedBuffers.find(fd) == mappedBuffers.end())
                {
                    const size_t length = lseek(fd, 0, SEEK_END);
                    mappedBuffers[fd] = MappedBufferInfo{nullptr, 0, length};
                }

                const size_t length = mappedBuffers[fd].dmabufLength;

                if (plane.offset > length ||
                    plane.offset + plane.length > length)
                {
                    std::cerr << "plane is out of buffer: buffer length="
                              << length << ", plane offset=" << plane.offset
                              << ", plane length=" << plane.length
                              << std::endl;
                    throw;
                }
                size_t &mapLength = mappedBuffers[fd].mapLength;
                mapLength = std::max(mapLength,
                                     static_cast<size_t>(plane.offset + plane.length));
            }

            for (const libcamera::FrameBuffer::Plane &plane : buffer->planes())
            {
                const int fd = plane.fd.get();
                auto &info = mappedBuffers[fd];
                if (!info.address)
                {
                    void *address = mmap(nullptr, info.mapLength, PROT_READ,
                                         MAP_SHARED, fd, 0);
                    if (address == MAP_FAILED)
                    {
                        int error = -errno;
                        std::cerr << "Failed to mmap plane: "
                                  << strerror(-error) << std::endl;
                        throw;
                    }

                    info.address = static_cast<uint8_t *>(address);
                }
                mappedPlanes[buffer.get()].emplace_back(info.address + plane.offset, plane.length);
            }
        }
    }

    /*
     * --------------------------------------------------------------------
     * Frame Capture
     *
     * libcamera frames capture model is based on the 'Request' concept.
     * For each frame a Request has to be queued to the Camera.
     *
     * A Request refers to (at least one) Stream for which a Buffer that
     * will be filled with image data shall be added to the Request.
     *
     * A Request is associated with a list of Controls, which are tunable
     * parameters (similar to v4l2_controls) that have to be applied to
     * the image.
     *
     * Once a request completes, all its buffers will contain image data
     * that applications can access and for each of them a list of metadata
     * properties that reports the capture parameters applied to the image.
     */
    stream = streamConfig.stream();
    const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = allocator->buffers(stream);
    for (unsigned int i = 0; i < buffers.size(); ++i)
    {
        std::unique_ptr<libcamera::Request> request = camera->createRequest();
        if (!request)
        {
            std::cerr << "Can't create request" << std::endl;
            return;
        }

        const std::unique_ptr<libcamera::FrameBuffer> &buffer = buffers[i];
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0)
        {
            std::cerr << "Can't set buffer for request"
                      << std::endl;
            return;
        }

        requests.push_back(std::move(request));
    }

    /*
     * --------------------------------------------------------------------
     * Signal&Slots
     *
     * libcamera uses a Signal&Slot based system to connect events to
     * callback operations meant to handle them, inspired by the QT graphic
     * toolkit.
     *
     * Signals are events 'emitted' by a class instance.
     * Slots are callbacks that can be 'connected' to a Signal.
     *
     * A Camera exposes Signals, to report the completion of a Request and
     * the completion of a Buffer part of a Request to support partial
     * Request completions.
     *
     * In order to receive the notification for request completions,
     * applications shall connecte a Slot to the Camera 'requestCompleted'
     * Signal before the camera is started.
     */
    camera->requestCompleted.connect(this, &Libcam2OpenCV::requestComplete);

    if (settings.framerate > 0)
    {
        int64_t frame_time = 1000000 / settings.framerate; // in us
        controls.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({frame_time, frame_time}));
    }

    if (settings.lensPosition >= 0)
    {
        controls.set(libcamera::controls::LensPosition, settings.lensPosition);
    }

    if (settings.exposureTime > 0)
    {
        controls.set(libcamera::controls::ExposureTime, settings.exposureTime); // in µs
    }

    if (settings.exposureValue != 0.0f)
    {
        controls.set(libcamera::controls::ExposureValue, settings.exposureValue);
    }

    if (settings.saturation != 1.0f)
    { // Check if saturation is different from the default
        controls.set(libcamera::controls::Saturation, settings.saturation);
    }

    controls.set(libcamera::controls::Brightness, settings.brightness);
    controls.set(libcamera::controls::Contrast, settings.contrast);

    /*
     * --------------------------------------------------------------------
     * Start Capture
     *
     * In order to capture frames the Camera has to be started and
     * Request queued to it. Enough Request to fill the Camera pipeline
     * depth have to be queued before the Camera start delivering frames.
     *
     * For each delivered frame, the Slot connected to the
     * Camera::requestCompleted Signal is called.
     */
    camera->start(&controls);
    for (std::unique_ptr<libcamera::Request> &request : requests)
        camera->queueRequest(request.get());
}

void Libcam2OpenCV::stop()
{
    /*
     * --------------------------------------------------------------------
     * Clean Up
     *
     * Stop the Camera, release resources and stop the CameraManager.
     * libcamera has now released all resources it owned.
     */
    if (camera)
    {
        camera->stop();
        if (allocator)
            allocator->free(stream);
        camera->release();
        camera.reset();
        delete allocator;
        tjDestroy(tjInstance);
    }
    cm->stop();
}
