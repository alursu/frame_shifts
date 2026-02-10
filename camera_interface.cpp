#include "camera_interface.hpp"

int serialCallBack(int id,guide_usb_serial_data_t *pSerialData);
int connectStatusCallBack(int id,guide_usb_device_status_e deviceStatus);
int frameCallBack(int id,guide_usb_frame_data_t *pVideoData);

bool frame_in_buffer = false;
cv::Mat frame;

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
    int count = 50000;
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
            cv::normalize(yuv16bit, frame, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            frame_in_buffer = true;
        }
        break;
     default:
        break;
    }
    return 0;
}