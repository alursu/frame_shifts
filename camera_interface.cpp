#include "camera_interface.hpp"

int serialCallBack(int id,guide_usb_serial_data_t *pSerialData);
int connectStatusCallBack(int id,guide_usb_device_status_e deviceStatus);
int frameCallBack(int id,guide_usb_frame_data_t *pVideoData);

bool frame_in_buffer = false;
cv::Mat frame;

int CameraInterface::base()
{
    guide_usb_setloglevel(LOG_LEVEL_INFO);//Setting a Log Level

    int ret = guide_usb_get_devcount();//Quantity of equipment acquired  Device ID No.: 1,2,3,4,...,count
    printf("devices counts:%d \n",ret);

    ret = guide_usb_initial(1);//Initialize device 1
    if(ret < 0)
    {
        printf("Initial device 1 fail:%d \n",ret);
        return ret;
    }
    else
    {
        ret = guide_usb_opencommandcontrol(1,(OnSerialDataReceivedCB)serialCallBack);//Endpoint communication is enabled on device 1
        printf("Initial device 1 success:%d\n",ret);
    }


    ret = guide_usb_openstream_auto(1,(OnFrameDataReceivedCB)frameCallBack,(OnDeviceConnectStatusCB)connectStatusCallBack); //Device 1 Starts the video streaming thread

    if(ret < 0)
    {
       printf("Open 1 fail:%d\n",ret);
       return ret;
    }
    else
    {
        printf("Open 1 return:%d\n",ret);
    }

    int count = 6000000;
    while (count--)
    {
        usleep(100);
        // auto real_time = std::chrono::steady_clock::now();
        // auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(real_time - g_last_frame_time);
        // std::cout << "Time diff: " << diff.count() << std::endl;
    }

    ret = guide_usb_closestream(1);
    printf("close 1 return:%d\n",ret);

    ret = guide_usb_closecommandcontrol(1);
    printf("closecommandcontrol 1 return:%d\n",ret);

    ret = guide_usb_exit(1);
    printf("exit 1 return:%d\n",ret);

    return ret;
}

void CameraInterface::open()
{
    guide_usb_setloglevel(LOG_LEVEL_ERROR);//Setting a Log Level

    int ret = guide_usb_get_devcount();//Quantity of equipment acquired  Device ID No.: 1,2,3,4,...,count

    while (ret < 1) {
        usleep(500000);
        ret = guide_usb_get_devcount();
        std::clog << "No camera found, trying to reconnect...";
    }
    std::clog << "Camera counts: " << ret << std::endl;

    ret = guide_usb_initial(1);//Initialize device 1
    if(ret < 0)
    {
        std::clog << "Initial device 1 fail: " << ret << std::endl;
    }
    else
    {
        ret = guide_usb_opencommandcontrol(1,(OnSerialDataReceivedCB)serialCallBack);//Endpoint communication is enabled on device 1
        std::clog << "Initial device 1 success: " << ret << std::endl;
    }

    ret = guide_usb_openstream_auto(1,(OnFrameDataReceivedCB)frameCallBack,(OnDeviceConnectStatusCB)connectStatusCallBack); //Device 1 Starts the video streaming thread

    if(ret < 0)
    {
        std::clog << "Open 1 fail: " << ret << std::endl;
    }
    else
    {
        std::clog << "Open 1 return: " << ret << std::endl;
        is_opened_ = true;
    }
}

bool CameraInterface::camera_connected()
{
    int count = 30000;
    while (count--)
    {
        usleep(100);
        if (frame_in_buffer){
            frame_in_buffer = false;
            return true;
        }
    }
    close();
    return false;
}

cv::Mat CameraInterface::get_frame()
{
    return frame;
}

void CameraInterface::close()
{
    int ret = guide_usb_closestream(1);
    std::clog << "Close 1 return: " << ret << std::endl;

    ret = guide_usb_closecommandcontrol(1);
    std::clog << "Close command control 1 return: " << ret << std::endl;

    ret = guide_usb_exit(1);
    std::clog << "Exit 1 return: " << ret << std::endl;
}

int serialCallBack(int id,guide_usb_serial_data_t *pSerialData)
{
    return 0;
}

int connectStatusCallBack(int id,guide_usb_device_status_e deviceStatus)
{
    switch (id)
    {
      case 1:
        switch (deviceStatus)
        {
            case DEVICE_CONNECT_OK:
                printf("ID:%d VideoStream Capture start...\n",id);
            break;
            case DEVICE_DISCONNECT_OK:
                printf("ID:%d VideoStream Capture end...\n",id);
            break;
        }
        break;
      case 2:
        switch (deviceStatus)
        {
            case DEVICE_CONNECT_OK:
                printf("ID:%d VideoStream Capture start...\n",id);
            break;
            case DEVICE_DISCONNECT_OK:
                printf("ID:%d VideoStream Capture end...\n",id);
            break;
        }
        break;
    }

    return 0;
}

int frameCallBack(int id,guide_usb_frame_data_t *pVideoData)
{
    switch (id)
    {
      case 1:
        if(pVideoData->frame_yuv_data != NULL)
        {
            cv::Mat yuv16bit(512, 640, CV_16UC1, pVideoData->frame_yuv_data);

            cv::Mat yuv422(512, 640, CV_8UC2, pVideoData->frame_yuv_data);

            cv::Mat bgr;
            cv::cvtColor(yuv422, bgr, cv::COLOR_YUV2BGR_UYVY);
            cv::cvtColor(bgr,bgr,cv::COLOR_BGR2GRAY);
            std::cout << "bgr type: " << bgr.type() << std::endl;

            // 2. Конвертируем YUV422 → BGR
            cv::Mat frame_8;
            cv::cvtColor(yuv422, frame_8, cv::COLOR_YUV2BGR_UYVY);

            cv::normalize(yuv16bit, frame, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            std::cout << "frame_8 type: " << frame_8.type() << std::endl;
            std::cout << "frame cout: " << frame.type() << std::endl;

            frame_in_buffer = true;
            cv::imshow("8_preobr", bgr);
            cv::waitKey(10);
            // cv::imshow("16_1", yuv16bit);
            // cv::waitKey(10);
            // cv::imshow("8_preobr", frame_8);
            // cv::waitKey(10);
            cv::imshow("16_preobr", frame);
            cv::waitKey(10);
        }
        break;
     default:
        break;
    }
    return 0;
}