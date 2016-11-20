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

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

static bool keep_running = true;
void sigint_handler(int s)
{
    if (s)
        keep_running = false;
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
    libfreenect2::PacketPipeline* pipeline = new libfreenect2::CpuPacketPipeline;
    libfreenect2::Freenect2Device* device = freenect2.openDevice(serial,
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

    libfreenect2::Registration* registration = new libfreenect2::Registration(
            device->getIrCameraParams(), device->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    libfreenect2::FrameMap frames { };
    while (keep_running)
    {
        std::chrono::high_resolution_clock::time_point start {
                std::chrono::high_resolution_clock::now() };
        if (!listener.waitForNewFrame(frames, 10 * 1000))
        {
            printf("Waiting for frame timed out!\n");
            return -1;
        }

        libfreenect2::Frame* rgb = frames[libfreenect2::Frame::Color];
//        libfreenect2::Frame* ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered);
        listener.release(frames);
        std::chrono::high_resolution_clock::time_point end {
                std::chrono::high_resolution_clock::now() };

        double duration {
                std::chrono::duration_cast<std::chrono::microseconds>(
                (end - start)).count() / 1000000.0 };

        printf("Elapsed time: %.3f\n", duration);
    }

    device->stop();
    device->close();

}

