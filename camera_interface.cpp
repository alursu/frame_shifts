#include "camera_interface.hpp"

unsigned char close_shutter[12] = {0x55,0xAA,0x07,0xA0,0x02,0x08,0x00,0x00,0x00,0x00,0xAD,0xF0};
unsigned char open_shutter[12] = {0x55,0xAA,0x07,0xA0,0x02,0x08,0x00,0x00,0x00,0x01,0xAC,0xF0};
unsigned char adaptive_compensation[12] = {0x55,0xAA,0x07,0x01,0x00,0x07,0x00,0x00,0x00,0x01,0x00,0xF0};
unsigned char stop_adaptive_compensation[12] = {0x55,0xAA,0x07,0x01,0x00,0x07,0x00,0x00,0x00,0x00,0x01,0xF0};
unsigned char shutter_timer[12]  = {0x55,0xAA,0x07,0x01,0x00,0x01,0x00,0x00,0x00,0x02,0x05,0xF0};
unsigned char save_settings[12] = {0x55,0xAA,0x07,0x01,0x00,0x04,0x00,0x00,0x00,0x01,0x03,0xF0};
unsigned char raw_format_cmos[12] = {0x55, 0xAA, 0x07, 0x02, 0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0x05, 0xF0};
unsigned char check_video_mode[12] = {0x55, 0xAA, 0x07, 0x02, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x84, 0xF0};

int serialCallBack(int id,guide_usb_serial_data_t *pSerialData);
int connectStatusCallBack(int id,guide_usb_device_status_e deviceStatus);
int frameCallBack(int id,guide_usb_frame_data_t *pVideoData);

bool frame_in_buffer = false;
cv::Mat frame;

bool sendCommandAndGetResponse(int serial_fd, const std::vector<uint8_t>& cmd, std::vector<uint8_t>& response) {
    // 1. Очищаем буфер перед отправкой
    // tcflush(serial_fd, TCIFLUSH);
    
    // 2. Отправляем команду
    write(serial_fd, cmd.data(), cmd.size());
    
    // 3. Ждем ответа (устройству нужно время на обработку)
    usleep(200000); // 200 мс
    
    // 4. Читаем ответ (простейший вариант, в реальной жизни нужно читать по символу)
    uint8_t buffer[256];
    int n = read(serial_fd, buffer, sizeof(buffer));
    
    if (n > 0) {
        response.assign(buffer, buffer + n);
        return true;
    }
    return false;
}

// Функция для декодирования ответа на запрос состояния видео (55 AA 07 02 01 80 ...)
void parseVideoStatus(const std::vector<uint8_t>& response) {
    // Ожидаем ответ формата: 55 AA 13 02 01 ...
    if (response.size() < 23 || response[0] != 0x55 || response[1] != 0xAA) {
        std::cerr << "Неверный формат ответа" << std::endl;
        return;
    }
    
    // Ключевые байты согласно документации (смещения от начала пакета)
    uint8_t interfaceType = response[6];   // Byte 6: Тип интерфейса (USB, CMOS, LVDS...)
    uint8_t dataContent  = response[7];   // Byte 7: Формат данных (самое важное!)
    uint8_t byteOrder     = response[8];   // Byte 8: Порядок байт (MSB/LSB для RAW)
    
    // Расшифровка типа интерфейса
    std::cout << "=== Статус видео ===" << std::endl;
    std::cout << "Тип интерфейса: ";
    switch(interfaceType) {
        case 0x00: std::cout << "Выключен"; break;
        case 0x01: std::cout << "USB 2.0"; break;
        case 0x02: std::cout << "CMOS (DVP)"; break;
        case 0x04: std::cout << "BT.656"; break;
        case 0x07: std::cout << "LVDS"; break;
        default:   std::cout << "Неизвестный (0x" << std::hex << (int)interfaceType << ")"; break;
    }
    std::cout << std::endl;
    
    // Расшифровка формата данных (Самое важное!)
    std::cout << "Формат данных: ";
    switch(dataContent) {
        case 0x00: std::cout << "YUV422 (цветное видео)" << std::endl; break;
        case 0x01: std::cout << "YUV422 + параметры" << std::endl; break;
        case 0x02: std::cout << "RAW16 (Y16) - СЫРЫЕ ДАННЫЕ" << std::endl; break;
        case 0x03: std::cout << "RAW16 + параметры" << std::endl; break;
        case 0x04: std::cout << "YUV422 + RAW16" << std::endl; break;
        case 0x08: std::cout << "Температурная матрица" << std::endl; break;
        default:   std::cout << "Неизвестный (0x" << std::hex << (int)dataContent << ")" << std::endl; break;
    }
    
    // Расшифровка порядка байт (актуально для RAW)
    if (dataContent == 0x02 || dataContent == 0x03) {
        std::cout << "Порядок байт: ";
        switch(byteOrder) {
            case 0x00: std::cout << "MSB First (старший байт первый)"; break;
            case 0x01: std::cout << "LSB First (младший байт первый)"; break;
            default:   std::cout << "Неизвестный (0x" << std::hex << (int)byteOrder << ")"; break;
        }
        std::cout << std::endl;
    }
    std::cout << "==========================" << std::endl;
}

void CameraInterface::open()
{
    // 3. Команда запроса статуса видео (55 AA 07 02 01 80 00 00 00 84 F0)
    std::vector<uint8_t> request = {0x55, 0xAA, 0x07, 0x02, 0x01, 0x80, 0x00, 0x00, 0x00, 0x84, 0xF0};
    
    // 4. Отправляем запрос и получаем ответ
    std::vector<uint8_t> response;
    // sendCommandAndGetResponse(1, request, response);
    // parseVideoStatus(response);
    guide_usb_setloglevel(LOG_LEVEL_ERROR);//Setting a Log Level

    int ret = guide_usb_get_devcount();//Quantity of equipment acquired  Device ID No.: 1,2,3,4,...,count

    while (ret < 1) {
        usleep(1500000);
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

    // // Start adaptive compensation
    // ret = guide_usb_sendcommand(1, adaptive_compensation, 12);
    // if (ret < 0){
    //     std::clog << "Start adaptive compensation failed" << std::endl;
    // } else {
    //     std::clog << "Start adaptive compensation successed" << std::endl;
    // }

    // Stop adaptive compensation
    ret = guide_usb_sendcommand(1, stop_adaptive_compensation, 12);
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