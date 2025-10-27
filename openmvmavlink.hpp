// Supporting type definitions and API wrappers for OpenMV C++ environment

#ifndef OPENMV_CPP_API_H
#define OPENMV_CPP_API_H

#include <cstdint>
#include <cstring>
#include <cstdio>

// LED class wrapper
enum LEDColor {
    LED_RED,
    LED_GREEN,
    LED_BLUE
};

class LED {
private:
    LEDColor color;
    
public:
    LED(LEDColor c) : color(c) {
        // Initialize LED hardware
        led_init(color);
    }
    
    void on() {
        led_set_state(color, true);
    }
    
    void off() {
        led_set_state(color, false);
    }
    
private:
    void led_init(LEDColor c);
    void led_set_state(LEDColor c, bool state);
};

// UART class wrapper
class UART {
private:
    int port;
    int baudrate;
    int timeout;
    
public:
    UART(int p, int baud, int timeout_ms) : port(p), baudrate(baud), timeout(timeout_ms) {
        // Initialize UART hardware
        uart_init(port, baudrate, timeout);
    }
    
    void write(const uint8_t* data, size_t len) {
        uart_write_bytes(port, data, len);
    }
    
private:
    void uart_init(int p, int baud, int timeout_ms);
    void uart_write_bytes(int p, const uint8_t* data, size_t len);
};

// Displacement result structure
struct Displacement {
private:
    float x_trans;
    float y_trans;
    float resp;
    
public:
    Displacement() : x_trans(0.0f), y_trans(0.0f), resp(0.0f) {}
    
    Displacement(float x, float y, float r) : x_trans(x), y_trans(y), resp(r) {}
    
    float x_translation() const {
        return x_trans;
    }
    
    float y_translation() const {
        return y_trans;
    }
    
    float response() const {
        return resp;
    }
};

// FrameBuffer class wrapper
class FrameBuffer {
private:
    uint8_t* buffer;
    int width;
    int height;
    int pixformat;
    
public:
    FrameBuffer(int w, int h, int fmt) : width(w), height(h), pixformat(fmt) {
        size_t size = w * h * 2;  // RGB565 is 2 bytes per pixel
        buffer = new uint8_t[size];
    }
    
    ~FrameBuffer() {
        delete[] buffer;
    }
    
    void replace(FrameBuffer* other) {
        size_t size = width * height * 2;
        memcpy(buffer, other->buffer, size);
    }
    
    Displacement find_displacement(FrameBuffer* other) {
        // Call OpenMV's displacement calculation function
        float x_trans, y_trans, resp;
        imlib_find_displacement(buffer, other->buffer, width, height, &x_trans, &y_trans, &resp);
        return Displacement(x_trans, y_trans, resp);
    }
    
private:
    void imlib_find_displacement(uint8_t* fb1, uint8_t* fb2, int w, int h, 
                                  float* x_trans, float* y_trans, float* response);
};

// Clock class for FPS tracking
class Clock {
private:
    uint32_t last_tick;
    uint32_t frame_count;
    uint32_t fps_start_time;
    float current_fps;
    
public:
    Clock() : last_tick(0), frame_count(0), fps_start_time(0), current_fps(0.0f) {
        last_tick = millis();
        fps_start_time = last_tick;
    }
    
    void tick() {
        last_tick = millis();
        frame_count++;
        
        uint32_t elapsed = last_tick - fps_start_time;
        if (elapsed >= 1000) {
            current_fps = (frame_count * 1000.0f) / elapsed;
            frame_count = 0;
            fps_start_time = last_tick;
        }
    }
    
    float fps() const {
        return current_fps;
    }
    
private:
    uint32_t millis();
};

// Sensor API wrapper functions
enum PixFormat {
    PIXFORMAT_RGB565,
    PIXFORMAT_GRAYSCALE
};

enum FrameSize {
    FRAMESIZE_B64X64,
    FRAMESIZE_B128X128
};

void sensor_reset();
void sensor_set_pixformat(PixFormat fmt);
void sensor_set_framesize(FrameSize size);
void sensor_skip_frames(int time_ms);
FrameBuffer* sensor_snapshot();
FrameBuffer* sensor_alloc_extra_fb(int width, int height, PixFormat fmt);
int sensor_get_width();
int sensor_get_height();

#endif // OPENMV_CPP_API_H