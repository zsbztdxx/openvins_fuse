/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef USB_CAM_USB_CAM_H
#define USB_CAM_USB_CAM_H

#include <asm/types.h> /* for videodev2.h */
#include <deque>
extern "C"
{
#include <linux/videodev2.h>
#include "libavcodec/avcodec.h"
#include "libswscale/swscale.h"
#include "libavutil/mem.h"
#include "libavutil/frame.h"
#include "libavutil/log.h"
}

// legacy reasons
#include <libavcodec/version.h>
#if LIBAVCODEC_VERSION_MAJOR < 55
#define AV_CODEC_ID_MJPEG CODEC_ID_MJPEG
#endif

#include <string>
#include <sstream>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "isp_dwe/isp_dwe_api.h"
#include "usb_cam/NormalCameraConfig.h"
#include <csjw_msgs/mjpeg.h>

namespace usb_cam
{

  class UsbCam
  {
  public:
    typedef enum
    {
      IO_METHOD_READ,
      IO_METHOD_MMAP,
      IO_METHOD_USERPTR,
      IO_METHOD_OVERLAY,
      IO_METHOD_DMABUF,
      IO_METHOD_UNKNOWN,
    } io_method;

    UsbCam();
    ~UsbCam();

    // start camera
    int start(const std::string &dev, io_method io, std::string input_image_format,
              int image_width, int image_height, int framerate, double framerate_tb, double time_delay_per_byte,
              bool process_image_in_other_thread, std::string publish_image_format,
              bool enable_isp, bool enable_dwe, const std::string IspFileName = "", const std::string DweFileName = "");
    // shutdown camera
    void shutdown(void);

    // grabs a new image from the camera
    int grab_image(sensor_msgs::Image *image);

    // enables/disable auto focus
    void set_auto_focus(int value);

    // Set video device parameters
    void set_v4l_parameter(const std::string &param, int value, const std::string dev = "");
    void set_v4l_parameter(const std::string &param, const std::string &value, const std::string dev = "");
    int get_v4l_parameter_list(const std::string dev = "", const std::string camera_name = "");
    int get_v4l_format_list(std::set<std::string> &formatList, const std::string dev = "");
    int get_v4l_fields(std::set<std::string> &fieldList, const std::string dev = "");
    int get_v4l_framesizes(std::set<std::string> &framesizeList, const std::string format, const std::string dev = "");
    int get_v4l_frameintervals(std::set<int> &frameintervalList, const std::string format,
                               int width, int height, const std::string dev = "");

    static io_method io_method_from_string(const std::string &str);
    static AVPixelFormat v4l_format_to_avpixel_format(const unsigned int input_image_v4l_format);
    static AVPixelFormat ros_image_format_to_avpixel_format(std::string ros_image_format);
    static v4l2_field v4l2_field_from_string(const std::string &str);

    void stop_capturing(void);
    void start_capturing(void);
    bool is_capturing();

    void CfgCB_NormalCamera(NormalCameraConfig &config, uint32_t level);

    typedef struct
    {
      uint8_t data[6220800] = {0}; // TODO:size map be outsize
      int dataLen = 0;
      ros::Time stamp;
    } org_data;

    // private:
    typedef struct
    {
      int width = 0;
      int height = 0;
      int bytes_per_pixel = 0;
      int image_size = 0;
      ros::Time stamp;
      char *image = nullptr;
      // int is_new = 0;
    } camera_image_t;

    struct buffer
    {
      void *start = nullptr;
      size_t length = 0;
      size_t offset = 0;
    };

    typedef struct frameSize_
    {
      int width;
      int height;
    } frameSize;

    typedef struct _v4lCtrlPara
    {
      int value_int = 0;
      int default_int = 0;
      int min_int = 0;
      int max_int = 0;
      int step_int = 0;
    } v4lCtrlPara;

    int init_mjpeg_decoder(int image_width, int image_height, int channel);
    void mjpeg2rgb(char *MJPEG, int len, char *RGB, int NumPixels);
    void mjpeg2mono8(char *MJPEG, int len, char *MONO8, int NumPixels);
    int process_image(const void *src, int len, camera_image_t *dest);
    int read_frame();
    void uninit_device(void);
    int init_isp_dwe(void);
    void init_read(unsigned int buffer_size);
    void init_mmap(void);
    void init_userp(unsigned int buffer_size);
    void init_device(int image_width, int image_height, int framerate);
    void close_device(void);
    void open_device(void);
    int grab_image();
    bool is_capturing_ = false;

    std::string camera_dev_ = "";
    io_method io_ = IO_METHOD_UNKNOWN;
    int fd_video = -1;
    ISP_API *app_isp = nullptr;
    DWE_API *app_dwe = nullptr;
    buffer *buffers_ = nullptr;
    unsigned int n_buffers_ = 0;
    AVFrame *avframe_camera_ = nullptr;
    AVFrame *avframe_output_ = nullptr;
    AVCodec *avcodec_ = nullptr;
    AVDictionary *avoptions_ = nullptr;
    AVCodecContext *avcodec_context_ = nullptr;
    int avframe_input_image_size_ = 0;
    int avframe_output_image_size_ = 0;
    struct SwsContext *video_sws_ = nullptr;
    camera_image_t *image_ = nullptr;
    std::deque<org_data> datas;
    int orgDataNum = 0;
    double time_delay_per_byte_ = 0.0;
    bool process_image_in_other_thread_ = false;
    uint32_t image_width_ = 0;
    uint32_t image_height_ = 0;
    uint32_t framerate_ = 0;
    double framerate_tb_ = 0.0;
    bool enable_isp_ = false;
    bool enable_dwe_ = false;
    std::string ispFileName_ = "";
    std::string dweFileName_ = "";
    std::string publish_image_format_ = "";
    unsigned int input_image_v4l_format_ = 0;
    AVPixelFormat input_image_avpixel_format_ = AV_PIX_FMT_NONE;
    AVPixelFormat output_image_avpixel_format_ = AV_PIX_FMT_NONE;
    int m_v4l2_buf_type = 0;
    std::map<std::string, v4lCtrlPara> ctrlParaList;
  };

} // namespace usb_cam

#endif
