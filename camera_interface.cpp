#include "camera_interface.hpp"

unsigned char close_shutter[12] = {0x55,0xAA,0x07,0xA0,0x02,0x08,0x00,0x00,0x00,0x00,0xAD,0xF0};
unsigned char open_shutter[12] = {0x55,0xAA,0x07,0xA0,0x02,0x08,0x00,0x00,0x00,0x01,0xAC,0xF0};
unsigned char adaptive_compensation[12] = {0x55,0xAA,0x07,0x01,0x00,0x07,0x00,0x00,0x00,0x01,0x00,0xF0};
unsigned char stop_adaptive_compensation[12] = {0x55,0xAA,0x07,0x01,0x00,0x07,0x00,0x00,0x00,0x00,0x01,0xF0};
unsigned char shutter_timer[12]  = {0x55,0xAA,0x07,0x01,0x00,0x01,0x00,0x00,0x00,0x02,0x05,0xF0};
unsigned char save_settings[12] = {0x55,0xAA,0x07,0x01,0x00,0x04,0x00,0x00,0x00,0x01,0x03,0xF0};

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
        usleep(1000000);
        ret = guide_usb_get_devcount();
        std::clog << "No camera found, trying to reconnect..." << std::endl;
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

    ret = guide_usb_sendcommand(1, adaptive_compensation, 12);
    if (ret < 0){
        std::clog << "Start adaptive compensation failed" << std::endl;
    } else {
        std::clog << "Start adaptive compensation successed" << std::endl;
    }
    ret = guide_usb_sendcommand(1, save_settings, 12);
    if (ret < 0){
        std::clog << "Saving settings failed" << std::endl;
    } else {
        std::clog << "Saving settings successed" << std::endl;
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
            cv::Mat yuv422(512, 640, CV_8UC2, pVideoData->frame_yuv_data);
            cv::cvtColor(yuv422, frame, cv::COLOR_YUV2BGR_UYVY);
            frame_in_buffer = true;
        }
        break;
     default:
        break;
    }
    return 0;
}