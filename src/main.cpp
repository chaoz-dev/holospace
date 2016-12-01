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

#include <pcl-1.7/pcl/visualization/cloud_viewer.h>

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
        (*itr)->start();
    }

    pcl::visualization::CloudViewer visualizer { "Viewer" };

    while (keep_running)
    {
        std::chrono::high_resolution_clock::time_point start {
                std::chrono::high_resolution_clock::now() };

        visualizer.showCloud(device_ptrs.at(0)->read_cloud());

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

