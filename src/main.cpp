/*
 * main.cpp
 *
 *  Created on: Nov 16, 2016
 *      Author: chao
 */

#include <chrono>
#include <cstdio>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <boost/program_options.hpp>

#include <pcl-1.8/pcl/filters/filter.h>
#include <pcl-1.8/pcl/features/integral_image_normal.h>
#include <pcl-1.8/pcl/features/fpfh_omp.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>

#include "kinect2.h"

static bool keep_running = true;
static void sigint_handler(int s)
{
    if(s)
        keep_running = false;
}

template<typename PointT>
struct PCLCloud
{
    typename pcl::PointCloud<PointT>::Ptr ptr { new pcl::PointCloud<PointT>() };
    typename pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb { ptr };
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr { new pcl::PointCloud< pcl::Normal> };

    cv::Mat rgb_mat { };
    cv::Mat depth_mat { };

    std::string cloud_name { "PCL Cloud" };
    std::string img_name { "PCL Image" };

};

struct ProgramOptions
{
    bool exit { false };

    std::string calibration_image { "calibration_img.png" };

};

ProgramOptions read_args(int argc, char ** argv)
{
    boost::program_options::options_description desc { "Usage: Qmulus [options...]\n"
                "Options:" };
    desc.add_options()
            ("help", "Shows this help message")
            ("calibration-image", boost::program_options::value<std::string>(),
                    "Calibration image file used to calibrate cameras for registration")
    ;

    boost::program_options::variables_map variables_map { };
    boost::program_options::store(
            boost::program_options::parse_command_line(argc, argv, desc),
            variables_map);
    boost::program_options::notify(variables_map);

    ProgramOptions program_options { };

    if (variables_map.count("help"))
    {
        std::cout << desc << std::endl;
        program_options.exit = true;
    }

    if (variables_map.count("calibration-image"))
        program_options.calibration_image =
                variables_map["calibration-image"].as<std::string>();

    return program_options;
}

int main(int argc, char ** argv)
{
    signal(SIGINT, sigint_handler);
    ProgramOptions program_options { read_args(argc, argv) };

    if (program_options.exit)
        return 1;

    // Kinect2DeviceFactory initialization
    device::Kinect2DeviceFactory k2_factory { };

    std::vector<std::shared_ptr<device::Kinect2Device>> device_ptrs {
            k2_factory.create_devices() };

    size_t num_devices { device_ptrs.size() };
    if(num_devices == 0)
    {
        printf("Failed to detect any connected devices. Exiting.\n");
        return 0;
    }
    else
    {
        printf("Detected %lu devices...\n", device_ptrs.size());
        for(auto device : device_ptrs)
            printf("Device %s\n", device->get_serial_number().c_str());
    }

    std::vector<std::shared_ptr<PCLCloud<pcl::PointXYZRGB>> > pcl_cloud_ptrs {
            num_devices, nullptr };

    for(size_t i { 0 }; i < num_devices; ++i)
    {
        auto device_ptr = device_ptrs.at(i);
        // TODO: Surround with TRY/CATCH
        device_ptr->start();

        if(device_ptr->started())
        {
            std::shared_ptr<PCLCloud<pcl::PointXYZRGB>> pcl_cloud_ptr {
                    std::make_shared<PCLCloud<pcl::PointXYZRGB>>() };
            pcl_cloud_ptr->cloud_name.append(" " + i);
            pcl_cloud_ptr->img_name.append(" " + i);
            pcl_cloud_ptrs.at(i) = pcl_cloud_ptr;
        }
    }


// Pre-processing initialization
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne { };
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);

//    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh { };
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree { new pcl::search::KdTree<
//            pcl::PointXYZRGB> };
//    fpfh.setSearchMethod(tree);
//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs { new pcl::PointCloud<
//            pcl::FPFHSignature33> };
//    fpfh.setRadiusSearch(0.05f);

// Visualizer initialization
    std::vector<int> visualizer_handles(static_cast<int>(num_devices) << 1, 0);
    printf("Visualizer Handle Size %zu\n", visualizer_handles.size());
    std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer {
            new pcl::visualization::PCLVisualizer("Cloud Visualizer") };

    visualizer->addCoordinateSystem(1.0);
    visualizer->initCameraParameters();

    for(size_t i { 0 }; i < num_devices; ++i)
    {
        int & cloud_visualizer_handle { visualizer_handles.at(i) };
        int & img_visualizer_handle { visualizer_handles.at(i + num_devices) };

        visualizer->createViewPort(static_cast<double>(i) / num_devices, 0.5,
                static_cast<double>(i + 1) / num_devices, 1.0,
                cloud_visualizer_handle);
        visualizer->createViewPort(static_cast<double>(i) / num_devices, 0.0,
                static_cast<double>(i + 1) / num_devices, 0.5,
                img_visualizer_handle);
        visualizer->setBackgroundColor(0, 0, 0, cloud_visualizer_handle);
        visualizer->setBackgroundColor(0, 0, 0, img_visualizer_handle);

        std::shared_ptr<PCLCloud<pcl::PointXYZRGB>> pcl_cloud_ptr {
                pcl_cloud_ptrs.at(i) };

        visualizer->addPointCloud(pcl_cloud_ptr->ptr, pcl_cloud_ptr->rgb,
                pcl_cloud_ptr->cloud_name, cloud_visualizer_handle);
        visualizer->addPointCloud(pcl_cloud_ptr->ptr, pcl_cloud_ptr->rgb,
                pcl_cloud_ptr->img_name, img_visualizer_handle);
        visualizer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                pcl_cloud_ptr->cloud_name);
        visualizer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                pcl_cloud_ptr->img_name);
    }

    while (keep_running)
    {
        std::chrono::high_resolution_clock::time_point start {
                std::chrono::high_resolution_clock::now() };

        for(size_t i { 0 }; i < num_devices; ++i)
        {
            std::shared_ptr<device::Kinect2Device> device_ptr { device_ptrs.at(
                    i) };
            std::shared_ptr<PCLCloud<pcl::PointXYZRGB>> pcl_cloud_ptr {
                    pcl_cloud_ptrs.at(i) };

            if(device_ptr->started() && pcl_cloud_ptr)
            {
                device_ptr->read_cloud(pcl_cloud_ptr->ptr,
                        pcl_cloud_ptr->rgb_mat, pcl_cloud_ptr->depth_mat);

//                ne.setInputCloud(cloud_1_ptr);
//                ne.compute(*normals_1);
            }
        }

        // TODO: Figure out how to extract points via indices. Maybe applicable to the voxel filter, which would be very handy.
//        std::vector<int> indices { };
//        pcl::removeNaNNormalsFromPointCloud(*normals_1, *normals_1, indices);

//        for(size_t i = 0; i < normals_1->points.size(); i++)
//        {
//            if(!pcl::isFinite<pcl::Normal>(normals_1->points[i]))
//            {
//                PCL_WARN("normals[%zu] is not finite\n", i);
//                break;
//            }
//        }

//        fpfh.setInputCloud(cloud_1_ptr);
//        fpfh.setInputNormals(normals_1);
//        fpfh.compute(*fpfhs);

        std::chrono::high_resolution_clock::time_point end {
                std::chrono::high_resolution_clock::now() };
        double duration { std::chrono::duration_cast<std::chrono::microseconds>(
                (end - start)).count() / 1000000.0 };
        printf("Elapsed time: %.3f\n", duration);

        for(size_t i { 0 }; i < num_devices; ++i)
        {
            std::shared_ptr<device::Kinect2Device> device_ptr { device_ptrs.at(
                    i) };
            std::shared_ptr<PCLCloud<pcl::PointXYZRGB>> pcl_cloud_ptr {
                    pcl_cloud_ptrs.at(i) };

            if(device_ptr->started() && pcl_cloud_ptr)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr {
                        pcl_cloud_ptr->ptr };
                visualizer->updatePointCloud<pcl::PointXYZRGB>(
                        pcl_cloud_ptr->ptr, pcl_cloud_ptr->rgb,
                        pcl_cloud_ptr->cloud_name);

                for(auto itr { cloud_ptr->points.begin() };
                        itr != cloud_ptr->points.end(); ++itr)
                    itr->z = 0;

                visualizer->updatePointCloud<pcl::PointXYZRGB>(
                        pcl_cloud_ptr->ptr, pcl_cloud_ptr->rgb,
                        pcl_cloud_ptr->img_name);
            }
        }

        visualizer->spinOnce();
    }

    for(auto itr { device_ptrs.begin() }; itr != device_ptrs.end(); ++itr)
    {
        (*itr)->stop();
    }
}

