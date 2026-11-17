#include <iostream>
#include <opencv2/opencv.hpp>
#include <OpenNI.h>

int main() {
    std::cout << "--- OpenCV & OpenNI2 Detection Test ---" << std::endl;

    // Test OpenCV Video Capture devices
    for (int i = 0; i < 4; ++i) {
        cv::VideoCapture cap(i);
        if (cap.isOpened()) {
            std::cout << "OpenCV VideoCapture index " << i << " OPENED SUCCESSFULLY!" << std::endl;
            std::cout << "  Width: " << cap.get(cv::CAP_PROP_FRAME_WIDTH)
                      << ", Height: " << cap.get(cv::CAP_PROP_FRAME_HEIGHT)
                      << ", FPS: " << cap.get(cv::CAP_PROP_FPS)
                      << ", FOURCC: " << cap.get(cv::CAP_PROP_FOURCC) << std::endl;
            cap.release();
        } else {
            std::cout << "OpenCV VideoCapture index " << i << " failed to open." << std::endl;
        }
    }

    // Test OpenNI2
    openni::Status status = openni::OpenNI::initialize();
    if (status == openni::STATUS_OK) {
        std::cout << "OpenNI2 Initialized Successfully!" << std::endl;
        openni::Array<openni::DeviceInfo> deviceList;
        openni::OpenNI::enumerateDevices(&deviceList);
        std::cout << "Found " << deviceList.getSize() << " OpenNI2 devices." << std::endl;
        for (int i = 0; i < deviceList.getSize(); ++i) {
            std::cout << " Device " << i << ": " << deviceList[i].getName()
                      << " Vendor: " << deviceList[i].getVendor()
                      << " URI: " << deviceList[i].getUri() << std::endl;
        }
        openni::OpenNI::shutdown();
    } else {
        std::cout << "OpenNI2 failed to initialize: " << openni::OpenNI::getExtendedError() << std::endl;
    }

    return 0;
}
