#include "dvp2usb/dvp2usb.h"

#define EP_INTR (LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_IN)
#define EP_DATA (LIBUSB_RECIPIENT_ENDPOINT | LIBUSB_ENDPOINT_IN)
#define CTRL_IN (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define CTRL_OUT (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)

DVP2USB_CAMERA::DVP2USB_CAMERA(uint16_t vendorId, uint16_t productId, int image_width, int image_height,
                               std::string input_image_format, std::string publish_image_format)
{
    m_VendorId = vendorId;
    m_ProductId = productId;
    image_width_ = image_width;
    image_height_ = image_height;

    bufLen = image_width * image_height;
    imgbuf = new uint8_t[bufLen];

    publish_image_format_ = publish_image_format;
    input_image_v4l_format_ = v4l2_fourcc(input_image_format[0],
                                          input_image_format[1],
                                          input_image_format[2],
                                          input_image_format[3]);
    channel_ = sensor_msgs::image_encodings::numChannels(publish_image_format_);
}

DVP2USB_CAMERA::~DVP2USB_CAMERA()
{
    if (is_capturing_)
    {
        stop();
        is_capturing_ = false;
    }
    if (imgbuf)
    {
        delete[] imgbuf;
    }
    if (dev_handle)
    {
        libusb_release_interface(dev_handle, 0);
        libusb_close(dev_handle);
        dev_handle = nullptr;
    }
    if (device_list)
    {
        libusb_free_device_list(device_list, 1);
        device_list = nullptr;
    }
    libusb_exit(nullptr);
}

int DVP2USB_CAMERA::grab_image(sensor_msgs::Image *image)
{
    if (imgbuf == nullptr)
        return -1;

#if 1 //同步确保图像流不会错位, 但频率从27hz减到13hz
    if (stop() < 0)
    {
        printf("grab_image stop error \n");
        return -1;
    }
    if (start() < 0)
    {
        printf("grab_image start error \n");
        return -1;
    }
#endif
    image->header.stamp = ros::Time::now();
    int transferred = 0;
    int ret = -1;
    if ((ret = libusb_bulk_transfer(dev_handle, EP_DATA, imgbuf, bufLen, &transferred, 0)) < 0)
    {
        printf(" libusb_bulk_transfer err  %d, %d\r\n", ret, transferred);
        return -1;
    }
    if (input_image_v4l_format_ == V4L2_PIX_FMT_GREY)
    {
        fillImage(*image, publish_image_format_, image_height_, image_width_, channel_ * image_width_, imgbuf);
    }
    return 0;
}

int DVP2USB_CAMERA::stop_capturing(void)
{
    if (stop() >= 0)
    {
        is_capturing_ = false;
        return 0;
    }
    return -1;
}

int DVP2USB_CAMERA::start_capturing(void)
{
    if (start() >= 0)
    {
        is_capturing_ = true;
        return 0;
    }
    return -1;
}

bool DVP2USB_CAMERA::is_capturing()
{
    return is_capturing_;
}

int DVP2USB_CAMERA::save_to_file(unsigned char *data)
{
    if (saveimg_idx < 20)
    {
        FILE *f;
        char filename[64];
        snprintf(filename, sizeof(filename), "finger%d.pgm", saveimg_idx++);
        f = fopen(filename, "w");
        if (!f)
            return -1;
        char temp[64];
        sprintf(temp, "P5 %d %d 255 ", image_width_, image_height_);
        fputs(temp, f);
        (void)fwrite(imgbuf, 1, image_width_ * image_height_, f);
        fclose(f);
        printf("saved image to %s\n", filename);
    }
    return 0;
}

int DVP2USB_CAMERA::Init()
{
    if (libusb_init(nullptr) < 0)
    {
        return -1;
    }
    print_devs();

    dev_handle = libusb_open_device_with_vid_pid(nullptr, m_VendorId, m_ProductId);
    if (dev_handle == nullptr)
    {
        printf("can not open device: %04X:%04X\n", m_VendorId, m_ProductId);
        return -1;
    }

    if (libusb_set_auto_detach_kernel_driver(dev_handle, 1) < 0)
    {
        printf("auto detach error \n");
        return -1;
    }
    if (libusb_kernel_driver_active(dev_handle, 0) < 0)
    {
        printf("driver active error \n");
        return -1;
    }
    if (libusb_claim_interface(dev_handle, 0) < 0)
    {
        printf("claim interface error \n");
        return -1;
    }
    return 0;
}

int DVP2USB_CAMERA::start()
{
    uint8_t cmd;
    if (dev_handle && libusb_control_transfer(dev_handle, CTRL_OUT, 0xa8, 0x00, 0, &cmd, 0, 0) < 0)
    {
        printf("0xa8 error \n");
        return -1;
    }
    return 0;
}
int DVP2USB_CAMERA::stop()
{
    uint8_t cmd;
    if (dev_handle && libusb_control_transfer(dev_handle, CTRL_OUT, 0xa9, 0x00, 0, &cmd, 0, 0) < 0)
    {
        printf("0xa9 error \n");
        return -1;
    }
    return 0;
}
void DVP2USB_CAMERA::print_devs()
{
    libusb_device *dev;
    int i = 0, j = 0;
    uint8_t path[8];
    std::string speed;
    unsigned char s[256];
    num_devices = libusb_get_device_list(NULL, &device_list);
    while ((dev = device_list[i++]) != nullptr)
    {
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(dev, &desc);
        if (r < 0)
        {
            fprintf(stderr, "failed to get device descriptor");
            return;
        }
        switch (libusb_get_device_speed(dev))
        {
        case LIBUSB_SPEED_LOW:
            speed = "1.5M";
            break;
        case LIBUSB_SPEED_FULL:
            speed = "12M";
            break;
        case LIBUSB_SPEED_HIGH:
            speed = "480M";
            break;
        case LIBUSB_SPEED_SUPER:
            speed = "5G";
            break;
        default:
            speed = "Unknown";
        }

        printf("%04x:%04x (bus %d, device %d) speed: %s",
               desc.idVendor, desc.idProduct,
               libusb_get_bus_number(dev), libusb_get_device_address(dev), speed.c_str());

        r = libusb_get_port_numbers(dev, path, sizeof(path));
        if (r > 0)
        {
            printf(" path: %d", path[0]);
            for (j = 1; j < r; j++)
                printf(".%d", path[j]);
        }

        if (!dev_handle)
        {
            libusb_open(dev, &dev_handle);
        }

        if (dev_handle)
        {
            if (desc.iManufacturer)
            {
                r = libusb_get_string_descriptor_ascii(dev_handle, desc.iManufacturer, s, sizeof(s));
                if (r > 0)
                    printf("  Manufacturer: %s", (char *)s);
            }

            if (desc.iProduct)
            {
                r = libusb_get_string_descriptor_ascii(dev_handle, desc.iProduct, s, sizeof(s));
                if (r > 0)
                    printf("  Product: %s", (char *)s);
            }

            if (desc.iSerialNumber)
            {
                r = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, s, sizeof(s));
                if (r > 0)
                    printf("  Serial Number: %s", (char *)s);
            }
            libusb_close(dev_handle);
            dev_handle = nullptr;
        }
        printf("\n");
    }
}
