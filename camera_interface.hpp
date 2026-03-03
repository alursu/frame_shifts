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

    void open();
    bool camera_connected();
    cv::Mat get_frame();
    void close();
    void shutter_close();
    void shutter_open();

    bool is_opened_ = false;
    bool shutter_is_closed = false;

private:

};

#endif // CAMERA_INTERFACE_H_