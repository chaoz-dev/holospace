/*
 * kinect2.h
 *
 *  Created on: Nov 27, 2016
 *      Author: chao
 */

#ifndef KINECT2_H
#define KINECT2_H

#include <exception>
#include <memory>
#include <vector>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>

#include <opencv2/opencv.hpp>

#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>

namespace device
{
    enum Kinect2CamType
    {
        COLOR = 1, DEPTH = 1 << 1
    };

    class Kinect2Device;

    class Kinect2DeviceFactory
    {
        public:
            Kinect2DeviceFactory();
            ~Kinect2DeviceFactory();

            std::vector<std::shared_ptr<Kinect2Device>> create_devices() const;

        private:
            std::shared_ptr<libfreenect2::Freenect2> freenect2_ptr;
    };

    class Kinect2Device
    {
        public:
            Kinect2Device(
                    std::shared_ptr<libfreenect2::Freenect2> freenect2_ptr,
                    size_t cam_num,
                    int cam_type = Kinect2CamType::COLOR
                            | Kinect2CamType::DEPTH, size_t timeout_sec = 10);
            virtual ~Kinect2Device();

            std::string get_serial_number() const;
            std::string get_firmware_version() const;

            bool started() const
            {
                return is_running;
            }

            bool start(size_t timeout_sec = 10);
            void stop();
            void read_cloud(
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                    size_t timeout_sec = 10) const;
            void read_cloud(
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                    cv::Mat & out_rgb_mat, cv::Mat & out_depth_mat,
                    size_t timeout_sec = 10) const;

        private:
            struct Kinect2FrameParam
            {
                static constexpr size_t FRAME_HEIGHT { 424 };
                static constexpr size_t FRAME_WIDTH { 512 };
                static constexpr size_t BYTES_PER_PIX { 4 };
                static constexpr size_t FRAME_AREA { FRAME_HEIGHT
                        * FRAME_WIDTH };

                float depth_col_map[FRAME_WIDTH] { };
                float depth_row_map[FRAME_HEIGHT] { };

                Kinect2FrameParam(
                        const libfreenect2::Freenect2Device::IrCameraParams & ir_cam_params)
                {
                    for(size_t i { 0 }; i < FRAME_WIDTH; ++i)
                    {
                        depth_col_map[i] = (i - ir_cam_params.cx + 0.5)
                                / ir_cam_params.fx;
                    }

                    for(size_t j { 0 }; j < FRAME_HEIGHT; ++j)
                    {
                        depth_row_map[j] = (j - ir_cam_params.cy + 0.5)
                                / ir_cam_params.fy;
                    }
                }
            };

            class Kinect2Frame
            {
                public:
                    libfreenect2::FrameMap frames { };

                    libfreenect2::Frame undistorted_frame {
                            Kinect2FrameParam::FRAME_WIDTH,
                            Kinect2FrameParam::FRAME_HEIGHT,
                            Kinect2FrameParam::BYTES_PER_PIX };
                    libfreenect2::Frame registered_frame {
                            Kinect2FrameParam::FRAME_WIDTH,
                            Kinect2FrameParam::FRAME_HEIGHT,
                            Kinect2FrameParam::BYTES_PER_PIX };

                    cv::Mat depth_mat { Kinect2FrameParam::FRAME_HEIGHT,
                            Kinect2FrameParam::FRAME_WIDTH, CV_8UC4,
                            undistorted_frame.data };
                    cv::Mat rgb_mat { Kinect2FrameParam::FRAME_HEIGHT,
                            Kinect2FrameParam::FRAME_WIDTH, CV_8UC4,
                            registered_frame.data };

                    Kinect2Frame(
                            libfreenect2::SyncMultiFrameListener & listener) :
                            frame_listener(listener)
                    {}

                    ~Kinect2Frame()
                    {
                        frame_listener.release(frames);
                    }

                    libfreenect2::Frame * get_rgb_frames()
                    {
                        return frames[libfreenect2::Frame::Color];
                    }

                    libfreenect2::Frame * get_depth_frames()
                    {
                        return frames[libfreenect2::Frame::Depth];
                    }

                private:
                    libfreenect2::SyncMultiFrameListener & frame_listener;
            };

            static constexpr size_t MILLISEC_MULT { 1000 };

            // Need to hold the freenect2 object in memory as long as
            // a libfreenect2::Freenect2Device object exists.
            std::shared_ptr<libfreenect2::Freenect2> freenect2_ptr { };

            std::unique_ptr<libfreenect2::Freenect2Device> device_ptr { };
            std::unique_ptr<libfreenect2::SyncMultiFrameListener> frame_listener_ptr { };

            std::unique_ptr<libfreenect2::Registration> registration_ptr { };
            std::unique_ptr<Kinect2FrameParam> frame_params_ptr { };

            bool is_running { false };

            void set_frame_listener(int cam_type);
            bool read_frame(Kinect2Frame & frame,
                    size_t timeout_sec = 10) const;
            void read_cloud(
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                    Kinect2Frame & frame, size_t timeout_sec) const;
            void frame_to_cloud(const Kinect2Frame & frame,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr) const;
    };

    class DeviceFailureException: public std::runtime_error
    {
        public:
            DeviceFailureException(const std::string & message) :
                    runtime_error(message)
            {}
    };

} /* namespace device */

#endif /* KINECT2_H */
