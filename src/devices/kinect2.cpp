/*
 * kinect2_device_factory.cpp
 *
 *  Created on: Nov 27, 2016
 *      Author: chao
 */

#include "kinect2.h"

#include <chrono>
#include <stdexcept>

#include <libfreenect2/packet_pipeline.h>

namespace device
{
    /************************************
     * Kinect2DeviceFactory Definitions *
     ************************************/

    Kinect2DeviceFactory::Kinect2DeviceFactory() :
            freenect2_ptr(new libfreenect2::Freenect2())
    {
        assert(freenect2_ptr);
    }

    Kinect2DeviceFactory::~Kinect2DeviceFactory()
    {}

    std::vector<std::shared_ptr<Kinect2Device>> Kinect2DeviceFactory::create_devices() const
    {
        assert(freenect2_ptr);

        std::vector<std::shared_ptr<Kinect2Device>> kinect2_devices { };

        size_t num_devices {
                static_cast<size_t>(freenect2_ptr->enumerateDevices()) };
        if(num_devices == 0)
            return kinect2_devices;

        for(size_t i { 0 }; i < num_devices; ++i)
            kinect2_devices.push_back(
                    std::make_shared<Kinect2Device>(freenect2_ptr, i));

        return kinect2_devices;

    }

    /*****************************
     * Kinect2Device Definitions *
     *****************************/

    Kinect2Device::Kinect2Device(
            std::shared_ptr<libfreenect2::Freenect2> freenect2_ptr,
            size_t cam_num, int cam_type, size_t timeout_sec) :
            freenect2_ptr(freenect2_ptr)
    {
        assert(this->freenect2_ptr);

        std::string serial_num { freenect2_ptr->getDeviceSerialNumber(cam_num) };

        std::chrono::high_resolution_clock::time_point start {
                std::chrono::high_resolution_clock::now() };
        while (!device_ptr)
        {
            device_ptr = std::unique_ptr<libfreenect2::Freenect2Device> {
                    freenect2_ptr->openDevice(serial_num,
                            new libfreenect2::OpenCLPacketPipeline()) };

            std::chrono::high_resolution_clock::time_point end {
                    std::chrono::high_resolution_clock::now() };
            long int duration {
                    std::chrono::duration_cast<std::chrono::seconds>(
                            (end - start)).count() };

            if(static_cast<size_t>(duration) > timeout_sec)
                throw DeviceFailureException(
                        "Failed to create Kinect2Device object " + serial_num
                                + " in " + std::to_string(timeout_sec)
                                + " seconds.");
        }

        assert(device_ptr);
        set_frame_listener(cam_type);
    }

    Kinect2Device::~Kinect2Device()
    {}

    std::string Kinect2Device::get_serial_number() const
    {
        assert(device_ptr);
        return device_ptr->getSerialNumber();
    }

    std::string Kinect2Device::get_firmware_version() const
    {
        assert(device_ptr);
        return device_ptr->getFirmwareVersion();
    }

    bool Kinect2Device::start(size_t timeout_sec)
    {
        assert(device_ptr);

        std::chrono::high_resolution_clock::time_point start {
                std::chrono::high_resolution_clock::now() };
        while (!device_ptr->start())
        {
            std::chrono::high_resolution_clock::time_point end {
                    std::chrono::high_resolution_clock::now() };
            long int duration {
                    std::chrono::duration_cast<std::chrono::seconds>(
                            (end - start)).count() };

            if(static_cast<size_t>(duration) > timeout_sec)
                throw DeviceFailureException(
                        "Failed to start Kinect2Device object "
                                + get_serial_number() + " in "
                                + std::to_string(timeout_sec) + " seconds.");
        }

        if(!registration_ptr)
            registration_ptr = std::unique_ptr<libfreenect2::Registration> {
                    new libfreenect2::Registration(
                            device_ptr->getIrCameraParams(),
                            device_ptr->getColorCameraParams()) };
        assert(registration_ptr);

        if(!frame_params_ptr)
            frame_params_ptr = std::unique_ptr<Kinect2FrameParam> {
                    new Kinect2FrameParam(device_ptr->getIrCameraParams()) };
        assert(frame_params_ptr);

        is_running = true;
        return started();
    }

    void Kinect2Device::stop()
    {
        if(device_ptr)
        {
            device_ptr->stop();
            device_ptr->close();
        }

        registration_ptr.reset(nullptr);
        frame_params_ptr.reset(nullptr);

        is_running = false;
    }

    void Kinect2Device::read_cloud(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
            size_t timeout_sec) const
    {
        assert(frame_listener_ptr);
        Kinect2Frame frame { *frame_listener_ptr };

        read_cloud(out_cloud_ptr, frame, timeout_sec);
    }

    void Kinect2Device::read_cloud(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
            cv::Mat & out_rgb_mat, cv::Mat & out_depth_mat,
            size_t timeout_sec) const
    {
        assert(frame_listener_ptr);
        Kinect2Frame frame { *frame_listener_ptr };

        read_cloud(out_cloud_ptr, frame, timeout_sec);

        if(out_cloud_ptr)
        {
            out_rgb_mat = frame.rgb_mat;
            out_depth_mat = frame.depth_mat;
        }
    }

    bool Kinect2Device::read_frame(Kinect2Frame & frame,
            size_t timeout_sec) const
    {
        assert(frame_listener_ptr);
        assert(registration_ptr);

        if(!frame_listener_ptr->waitForNewFrame(frame.frames,
                timeout_sec * MILLISEC_MULT))
            return false;

        registration_ptr->apply(frame.get_rgb_frames(),
                frame.get_depth_frames(), &frame.undistorted_frame,
                &frame.registered_frame);

        return true;
    }

    void Kinect2Device::read_cloud(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
            Kinect2Frame & frame, size_t timeout_sec) const
    {
        if(!out_cloud_ptr)
            out_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>(
                    Kinect2FrameParam::FRAME_WIDTH,
                    Kinect2FrameParam::FRAME_HEIGHT).makeShared();
        else
        {
            out_cloud_ptr->resize(Kinect2FrameParam::FRAME_AREA);
            out_cloud_ptr->width = Kinect2FrameParam::FRAME_WIDTH;
            out_cloud_ptr->height = Kinect2FrameParam::FRAME_HEIGHT;
        }

        if(read_frame(frame, timeout_sec))
            frame_to_cloud(frame, out_cloud_ptr);
    }

    void Kinect2Device::set_frame_listener(int cam_type)
    {
        assert(device_ptr);

        int frame_type { 0 };
        if(cam_type & Kinect2CamType::COLOR)
            frame_type |= libfreenect2::Frame::Color;
        if(cam_type & Kinect2CamType::DEPTH)
            frame_type |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

        frame_listener_ptr = std::unique_ptr<
                libfreenect2::SyncMultiFrameListener>(
                new libfreenect2::SyncMultiFrameListener(frame_type));
        assert(frame_listener_ptr);

        device_ptr->setColorFrameListener(frame_listener_ptr.get());
        device_ptr->setIrAndDepthFrameListener(frame_listener_ptr.get());
    }

    void Kinect2Device::frame_to_cloud(const Kinect2Frame & frame,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr) const
    {
        if(!cloud_ptr)
            return;

        cloud_ptr->is_dense = false;

        for(std::size_t y { 0 }; y < cloud_ptr->height; ++y)
        {
            std::size_t row { y * cloud_ptr->width };

            const float * depth_ptr {
                    reinterpret_cast<const float *>(frame.depth_mat.ptr()) + row };
            const char * rgb_ptr {
                    reinterpret_cast<const char *>(frame.rgb_mat.ptr())
                            + (row << 2) };

            for(std::size_t x { 0 }; x < cloud_ptr->width; ++x)
            {
                pcl::PointXYZRGB * pt_ptr { &cloud_ptr->points.at(row + x) };

                float depth_value { *(depth_ptr + x) / 1000.0f };

                if(!std::isnan(depth_value)
                        && !(std::abs(depth_value) < 0.0001))
                {
                    pt_ptr->x = frame_params_ptr->depth_col_map[x]
                            * depth_value;
                    pt_ptr->y = frame_params_ptr->depth_row_map[y]
                            * depth_value;
                    pt_ptr->z = depth_value;

                    const char * rgb { rgb_ptr + (x << 2) };
                    pt_ptr->r = rgb[2];
                    pt_ptr->g = rgb[1];
                    pt_ptr->b = rgb[0];
                }
                else
                {
                    pt_ptr->x = pt_ptr->y = pt_ptr->z = std::numeric_limits<
                            float>::quiet_NaN();
                    pt_ptr->rgb = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
    }
} /* namespace device */
