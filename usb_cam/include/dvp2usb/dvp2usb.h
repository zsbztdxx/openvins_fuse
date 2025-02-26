#ifndef _VP2USB_H_
#define _VP2USB_H_

#include <libusb-1.0/libusb.h>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
extern "C"
{
#include <linux/videodev2.h>
// #include "libavcodec/avcodec.h"
// #include "libswscale/swscale.h"
// #include "libavutil/mem.h"
// #include "libavutil/frame.h"
// #include "libavutil/log.h"
}

class DVP2USB_CAMERA
{
public:
    DVP2USB_CAMERA(uint16_t vendorId, uint16_t productId, int image_width, int image_height,
                   std::string input_image_format, std::string publish_image_format);
    ~DVP2USB_CAMERA();
    int Init();
    void print_devs();
    int start();
    int stop();
    int save_to_file(unsigned char *data);
    int grab_image(sensor_msgs::Image *image);
    int stop_capturing(void);
    int start_capturing(void);
    bool is_capturing();

private:
    libusb_device **device_list = nullptr;
    libusb_device_handle *dev_handle = nullptr;
    uint16_t m_VendorId = 0;
    uint16_t m_ProductId = 0;
    int num_devices = 0;
    int image_width_ = 0, image_height_ = 0, channel_ = 1;
    uint8_t *imgbuf = nullptr;
    uint32_t bufLen = 0;
    int saveimg_idx = 0;
    bool is_capturing_ = false;
    std::string publish_image_format_ = "";
    unsigned int input_image_v4l_format_ = 0;
};

#endif
