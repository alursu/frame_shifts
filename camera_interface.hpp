#ifndef CAMERA_INTERFACE_H_
#define CAMERA_INTERFACE_H_

#include <stdio.h>
#include <sys/types.h>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include "guidescusb2.h"
#include <unistd.h>
#include <malloc.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
// #include <stdbool.h>
#include <fcntl.h>
#include "sys/time.h"
#include "time.h"
#include <stdlib.h>
#include <chrono>

#define EP_IN 0x81 // Endpoint 1- Image data receive
#define WIDTH 640
#define HEIGHT 512
#define TIMEOUT 1000

// Класс для подключения к камере и получения данных с нее
class CameraInterface
{
public:

    int base();
    void open();
    bool camera_connected();
    cv::Mat get_frame();
    void close();

    bool is_opened_ = false;

private:

    // int serailCallBack(int id,guide_usb_serial_data_t *pSerialData);
    // int connectStatusCallBack(int id,guide_usb_device_status_e deviceStatus);
    // int frameCallBack(int id,guide_usb_frame_data_t *pVideoData);
};

#endif // CAMERA_INTERFACE_H_