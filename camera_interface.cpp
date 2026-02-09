#include "camera_interface.hpp"

int serailCallBack(int id,guide_usb_serial_data_t *pSerialData);
int connectStatusCallBack(int id,guide_usb_device_status_e deviceStatus);
int frameCallBack(int id,guide_usb_frame_data_t *pVideoData);

    int FPS1 = 0;
    int FPS2 = 0;
    double startTime1,startTime2;

    // unsigned char ironred[12]  = { 0x55,0xAA,0x07,0x02,0x00,0x04,0x00,0x00,0x00,0x02,0x03,0xF0};
    // unsigned char whitehot[12] = { 0x55,0xAA,0x07,0x02,0x00,0x04,0x00,0x00,0x00,0x00,0x01,0xF0};
    // unsigned char hotiron[12]  = { 0x55,0xAA,0x07,0x02,0x00,0x04,0x00,0x00,0x00,0x03,0x02,0xF0};
    // unsigned char medical[12]  = { 0x55,0xAA,0x07,0x02,0x00,0x04,0x00,0x00,0x00,0x04,0x05,0xF0};
    // unsigned char arctic[12]   = { 0x55,0xAA,0x07,0x02,0x00,0x04,0x00,0x00,0x00,0x05,0x04,0xF0};
    // unsigned char rainbow1[12] = { 0x55,0xAA,0x07,0x02,0x00,0x04,0x00,0x00,0x00,0x06,0x07,0xF0};
    // unsigned char rainbow2[12] = { 0x55,0xAA,0x07,0x02,0x00,0x04,0x00,0x00,0x00,0x07,0x06,0xF0};
    // unsigned char tnit[12]     = { 0x55,0xAA,0x07,0x02,0x00,0x04,0x00,0x00,0x00,0x08,0x09,0xF0};
    // unsigned char shutter[12]  = { 0x55,0xAA,0x07,0x02,0x01,0x08,0x00,0x00,0x00,0x01,0x0d,0xF0};

double tick(void)
{
    struct timeval t;
    gettimeofday(&t, 0);
    return t.tv_sec + 1E-6 * t.tv_usec;
}

int CameraInterface::base(void)
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
        ret = guide_usb_opencommandcontrol(1,(OnSerialDataReceivedCB)serailCallBack);//Endpoint communication is enabled on device 1
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
    startTime1 = tick();

    int count = 6000000;
    while (count--)
    {
        usleep(10);
    }

    ret = guide_usb_closestream(1);
    printf("close 1 return:%d\n",ret);

    ret = guide_usb_closecommandcontrol(1);
    printf("closecommandcontrol 1 return:%d\n",ret);

    ret = guide_usb_exit(1);
    printf("exit 1 return:%d\n",ret);

    return ret;
}

int serailCallBack(int id,guide_usb_serial_data_t *pSerialData)
{
    switch (id)
    {
      case 1:
        //printf("ID:%d---->data length:%d \n",id,pSerialData->serial_recv_data_length);
        break;
      case 2:
        //printf("ID:%d---->data length:%d \n",id,pSerialData->serial_recv_data_length);
        break;
      break;
    }

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
      case 1: //设备1
        if(pVideoData->frame_src_data != NULL)
        {
            printf("pVideoData->frame_src_data[0]:%d\n",pVideoData->frame_src_data[0]);
            printf("pVideoData->frame_src_data[1]:%d\n",pVideoData->frame_src_data[1]);
            // std::cout << std::endl << "pVideoData->frame_yuv_data_length: " << pVideoData->frame_yuv_data_length << std::endl << std::endl;
            // std::cout << std::endl << "pVideoData->frame_yuv_data[0]: " << pVideoData->frame_yuv_data[0] << std::endl << std::endl;
            // cv::Mat yuv16bit(512, 640, CV_16UC1, pVideoData->frame_yuv_data);
            // cv::imshow("frame", yuv16bit);
            // cv::waitKey(10);
        }
        if(pVideoData->paramLine != NULL)
        {
            printf("pVideoData->paramLine[0]:%d\n",pVideoData->paramLine[0]);
            printf("pVideoData->paramLine[1]:%d\n",pVideoData->paramLine[1]);
            // std::cout << std::endl << "pVideoData->frame_yuv_data_length: " << pVideoData->frame_yuv_data_length << std::endl << std::endl;
            // std::cout << std::endl << "pVideoData->frame_yuv_data[0]: " << pVideoData->frame_yuv_data[0] << std::endl << std::endl;
            // cv::Mat yuv16bit(512, 640, CV_16UC1, pVideoData->frame_yuv_data);
            // cv::imshow("frame", yuv16bit);
            // cv::waitKey(10);
        }

        if(pVideoData->frame_yuv_data != NULL)
        {
            printf("pVideoData->frame_yuv_data[0]:%x\n",pVideoData->frame_yuv_data[0]);
            printf("pVideoData->frame_yuv_data[1]:%x\n",pVideoData->frame_yuv_data[1]);
            std::cout << std::endl << "pVideoData->frame_yuv_data_length: " << pVideoData->frame_yuv_data_length << std::endl << std::endl;
            std::cout << std::endl << "pVideoData->frame_yuv_data[0]: " << pVideoData->frame_yuv_data[0] << std::endl << std::endl;
            cv::Mat yuv16bit(512, 640, CV_16UC1, pVideoData->frame_yuv_data);
            cv::imshow("frame", yuv16bit);
            cv::waitKey(10);
        }

        FPS1++;
        if((tick()-startTime1)>1)
        {
            startTime1 = tick();
            printf("FPS1-------------------------%d\n",FPS1);
            FPS1 = 0;
        }
        break;
      case 2:
        FPS2++;
        if((tick()-startTime2)>1)
        {
            startTime2 = tick();
            printf("FPS2-------------------------%d\n",FPS2);
            FPS2 = 0;
        }
        break;
      case 3:
      break;
     default:
        break;
    }
    return 0;
}
