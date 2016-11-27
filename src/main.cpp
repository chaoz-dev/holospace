/*
 * main.cpp
 *
 *  Created on: Nov 16, 2016
 *      Author: chao
 */

#include <chrono>
#include <cstdio>
#include <csignal>
#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>

static bool keep_running = true;
void sigint_handler(int s)
{
    if (s)
        keep_running = false;
}

static constexpr size_t FRAME_WIDTH { 512 };
static constexpr size_t FRAME_HEIGHT { 424 };
void create_mapping(
        const libfreenect2::Freenect2Device::IrCameraParams & depth_params,
        Eigen::Matrix<float, FRAME_WIDTH, 1> & col_map,
        Eigen::Matrix<float, FRAME_HEIGHT, 1> & row_map)
{
    float * col_ptr { col_map.data() };
    float * row_ptr { row_map.data() };

    for (size_t i = 0; i < FRAME_WIDTH; ++i)
    {
        *col_ptr++ = (i - depth_params.cx + 0.5) / depth_params.fx;
    }

    for (size_t j = 0; j < FRAME_HEIGHT; ++j)
    {
        *row_ptr++ = (j - depth_params.cy + 0.5) / depth_params.fy;
    }
}

int main(void)
{
    signal(SIGINT, sigint_handler);

    libfreenect2::Freenect2 freenect2 { };

    if (freenect2.enumerateDevices() == 0)
    {
        printf("No devices are connected!\n");
        return -1;
    }

    std::string serial { freenect2.getDeviceSerialNumber(0) };
    libfreenect2::PacketPipeline * pipeline =
            new libfreenect2::CpuPacketPipeline;
    libfreenect2::Freenect2Device * device = freenect2.openDevice(serial,
            pipeline);

    int types { libfreenect2::Frame::Color | libfreenect2::Frame::Ir
            | libfreenect2::Frame::Depth };
    libfreenect2::SyncMultiFrameListener listener(types);

    device->setColorFrameListener(&listener);
    device->setIrAndDepthFrameListener(&listener);

    if (!device->start())
    {
        printf("Failed to start device!\n");
        return -1;
    } else
    {
        printf("Serial Number: %s", serial.c_str());
        printf("Firmware Version: %s", device->getFirmwareVersion().c_str());
    }

    libfreenect2::Registration * registration = new libfreenect2::Registration(
            device->getIrCameraParams(), device->getColorCameraParams());
    libfreenect2::Frame undistorted(FRAME_WIDTH, FRAME_HEIGHT, 4), registered(
            FRAME_WIDTH, FRAME_HEIGHT, 4);

    Eigen::Matrix<float, FRAME_WIDTH, 1> col_map { };
    Eigen::Matrix<float, FRAME_HEIGHT, 1> row_map { };
    create_mapping(device->getIrCameraParams(), col_map, row_map);
    pcl::visualization::CloudViewer visualizer { "Viewer" };

    while (keep_running)
    {
        std::chrono::high_resolution_clock::time_point start {
                std::chrono::high_resolution_clock::now() };
        libfreenect2::FrameMap frames { };

        /* Capture data from Kinect v2 */
        if (!listener.waitForNewFrame(frames, 10 * 1000))
        {
            printf("Waiting for frame timed out!\n");
            return -1;
        }

        libfreenect2::Frame * rgb_frames = frames[libfreenect2::Frame::Color];
//        libfreenect2::Frame* ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame * depth_frames = frames[libfreenect2::Frame::Depth];

        registration->apply(rgb_frames, depth_frames, &undistorted,
                &registered);

        /* Convert data into PCL Point Cloud */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr { new pcl::PointCloud<
                pcl::PointXYZRGB>((uint32_t) undistorted.width,
                (uint32_t) undistorted.height) };

        cv::Mat depth_mat { (int) FRAME_HEIGHT, (int) FRAME_WIDTH,
        CV_8UC4, undistorted.data };
        cv::Mat rgb_mat { (int) FRAME_HEIGHT, (int) FRAME_WIDTH,
        CV_8UC4, registered.data };

        for (std::size_t y = 0; y < cloud_ptr->height; ++y)
        {
            std::size_t row { y * cloud_ptr->width };

            float * depth_ptr { ((float *) depth_mat.ptr()) + row };
            char * rgb_ptr { ((char *) rgb_mat.ptr()) + (row << 2) };

            for (std::size_t x = 0; x < cloud_ptr->width; ++x)
            {
                pcl::PointXYZRGB * pt_ptr { &cloud_ptr->points.at(row + x) };

                float depth_value { *(depth_ptr + x) / 1000.0f };

                if (!std::isnan(depth_value)
                        && !(std::abs(depth_value) < 0.0001))
                {
                    pt_ptr->x = col_map(x) * depth_value;
                    pt_ptr->y = row_map(y) * depth_value;
                    pt_ptr->z = depth_value;

                    char * rgb { rgb_ptr + (x << 2) };
                    pt_ptr->r = rgb[2];
                    pt_ptr->g = rgb[1];
                    pt_ptr->b = rgb[0];

                    cloud_ptr->is_dense = true;
                } else
                {
                    pt_ptr->x = pt_ptr->y = pt_ptr->z =
                            std::numeric_limits<float>::quiet_NaN();
                    pt_ptr->rgb = std::numeric_limits<float>::quiet_NaN();
                    cloud_ptr->is_dense = false;
                }
            }
        }
        visualizer.showCloud(cloud_ptr);

        /* Release frames when finished */
        listener.release(frames);

        std::chrono::high_resolution_clock::time_point end {
                std::chrono::high_resolution_clock::now() };
        double duration { std::chrono::duration_cast<std::chrono::microseconds>(
                (end - start)).count() / 1000000.0 };

        printf("Elapsed time: %.3f\n", duration);

    }

    device->stop();
    device->close();

}

