/*
 * kinect2_device_factory.cpp
 *
 *  Created on: Nov 27, 2016
 *      Author: chao
 */

#include "kinect2.h"

#include <libfreenect2/packet_pipeline.h>

namespace device
{
    Kinect2DeviceFactory::Kinect2DeviceFactory() :
            freenect2_ptr(new libfreenect2::Freenect2())
    {
        assert(freenect2_ptr);
    }

    Kinect2DeviceFactory::~Kinect2DeviceFactory()
    {
    }

    std::vector<std::unique_ptr<Kinect2Device>> Kinect2DeviceFactory::create_devices() const
    {
        assert(freenect2_ptr);

        std::vector<std::unique_ptr<Kinect2Device>> kinect2_devices { };

        size_t num_devices { (size_t) freenect2_ptr->enumerateDevices() };
        if(num_devices == 0)
            return kinect2_devices;

        for(size_t i { 0 }; i < num_devices; ++i)
        {
            std::string serial_num { freenect2_ptr->getDeviceSerialNumber(i) };

            std::unique_ptr<libfreenect2::Freenect2Device> device {
                    freenect2_ptr->openDevice(serial_num,
                            new libfreenect2::CpuPacketPipeline()) };

            kinect2_devices.push_back(
                    std::unique_ptr<Kinect2Device>(
                            new Kinect2Device(std::move(device))));

        }

        return kinect2_devices;

    }

    Kinect2Device::Kinect2Device(
            std::unique_ptr<libfreenect2::Freenect2Device> device_ptr,
            int cam_type) :
            device_ptr(std::move(device_ptr))
    {
        assert(this->device_ptr);
        set_frame_listener(cam_type);
    }

    Kinect2Device::~Kinect2Device()
    {
    }

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

    bool Kinect2Device::start()
    {
        assert(device_ptr);
        if(!device_ptr->start())
            return false;

        if(!registration_ptr)
            registration_ptr = std::unique_ptr<libfreenect2::Registration>(
                    new libfreenect2::Registration(
                            device_ptr->getIrCameraParams(),
                            device_ptr->getColorCameraParams()));

        frame_params = std::unique_ptr<Kinect2FrameParam>(
                new Kinect2FrameParam(device_ptr->getIrCameraParams()));

        return true;
    }

    void Kinect2Device::stop()
    {
        assert(device_ptr);

        device_ptr->stop();
        device_ptr->close();

        registration_ptr.reset(nullptr);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect2Device::read_cloud(
            size_t timeout_sec) const
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr { new pcl::PointCloud<
                pcl::PointXYZRGB>(Kinect2FrameParam::FRAME_WIDTH,
                Kinect2FrameParam::FRAME_HEIGHT) };
        cloud_ptr->is_dense = false;

        Kinect2Frame frame { *frame_listener_ptr };

        if(read_frame(frame, timeout_sec))
            frame_to_cloud(frame, cloud_ptr);

        return cloud_ptr;
    }

    bool Kinect2Device::read_frame(Kinect2Frame & data,
            size_t timeout_sec) const
    {
        assert(frame_listener_ptr);
        assert(registration_ptr);

        if(!frame_listener_ptr->waitForNewFrame(data.frames,
                timeout_sec * MILLISEC_MULT))
            return false;

        registration_ptr->apply(data.get_rgb_frames(), data.get_depth_frames(),
                &data.undistorted_frame, &data.registered_frame);

        return true;
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
        device_ptr->setColorFrameListener(frame_listener_ptr.get());
        device_ptr->setIrAndDepthFrameListener(frame_listener_ptr.get());
    }

    void Kinect2Device::frame_to_cloud(const Kinect2Frame & frame,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr) const
    {
        assert(cloud_ptr);

        for(std::size_t y { 0 }; y < cloud_ptr->height; ++y)
        {
            std::size_t row { y * cloud_ptr->width };

            float * depth_ptr { ((float *) frame.depth_mat.ptr()) + row };
            char * rgb_ptr { ((char *) frame.rgb_mat.ptr()) + (row << 2) };

            for(std::size_t x { 0 }; x < cloud_ptr->width; ++x)
            {
                pcl::PointXYZRGB * pt_ptr { &cloud_ptr->points.at(row + x) };

                float depth_value { *(depth_ptr + x) / 1000.0f };

                if(!std::isnan(depth_value)
                        && !(std::abs(depth_value) < 0.0001))
                {
                    pt_ptr->x = frame_params->depth_col_map[x] * depth_value;
                    pt_ptr->y = frame_params->depth_row_map[y] * depth_value;
                    pt_ptr->z = depth_value;

                    char * rgb { rgb_ptr + (x << 2) };
                    pt_ptr->r = rgb[2];
                    pt_ptr->g = rgb[1];
                    pt_ptr->b = rgb[0];

                    cloud_ptr->is_dense = true;
                }
                else
                {
                    pt_ptr->x = pt_ptr->y = pt_ptr->z = std::numeric_limits<
                            float>::quiet_NaN();
                    pt_ptr->rgb = std::numeric_limits<float>::quiet_NaN();
                    cloud_ptr->is_dense = false;
                }
            }
        }
    }

} /* namespace device */
