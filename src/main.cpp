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

#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>

#include "kinect2.h"

static bool keep_running = true;
void sigint_handler(int s)
{
    if(s)
        keep_running = false;
}

int main(void)
{
    signal(SIGINT, sigint_handler);

    device::Kinect2DeviceFactory k2_factory { };
    std::vector<std::unique_ptr<device::Kinect2Device>> device_ptrs {
            k2_factory.create_devices() };
    assert(device_ptrs.size() > 0);

    for(auto itr { device_ptrs.begin() }; itr != device_ptrs.end(); ++itr)
    {
        assert(*itr);
        assert((*itr)->start());
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr { new pcl::PointCloud<
            pcl::PointXYZRGB>() };

    std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer {
            new pcl::visualization::PCLVisualizer("Cloud Visualizer") };
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
            cloud_ptr);
    visualizer->addPointCloud(cloud_ptr, rgb, "Cloud_1");
    visualizer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Cloud_1");
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->addCoordinateSystem(1.0);
    visualizer->initCameraParameters();

    while (keep_running)
    {
        std::chrono::high_resolution_clock::time_point start {
                std::chrono::high_resolution_clock::now() };

        device_ptrs.at(0)->read_cloud(cloud_ptr);
        rgb.setInputCloud(cloud_ptr);
        visualizer->updatePointCloud<pcl::PointXYZRGB>(cloud_ptr, rgb,
                "Cloud_1");
        visualizer->spinOnce();

        std::chrono::high_resolution_clock::time_point end {
                std::chrono::high_resolution_clock::now() };
        double duration { std::chrono::duration_cast<std::chrono::microseconds>(
                (end - start)).count() / 1000000.0 };
        printf("Elapsed time: %.3f\n", duration);

    }

    for(auto itr { device_ptrs.begin() }; itr != device_ptrs.end(); ++itr)
    {
        (*itr)->stop();
    }
}

