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
#define __STDC_CONSTANT_MACROS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/fill_image.h>

#include <usb_cam/usb_cam.h>

#if (defined __arm__) || (defined __aarch64__)
#include <arm_neon.h>
#endif

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace usb_cam
{

  void monotonicToRealTime(const timespec &monotonic_time, timespec &real_time, int time_delay)
  {
    struct timespec real_sample1, real_sample2, monotonic_sample;

    // TODO(lucasw) Disable interrupts here?
    // otherwise what if there is a delay/interruption between sampling the times?
    clock_gettime(CLOCK_REALTIME, &real_sample1);      // 1970.1.1到目前的时间, 更改系统时间会更改获取的值
    clock_gettime(CLOCK_MONOTONIC, &monotonic_sample); // 获取的时间为系统重启到现在的时间, 更改系统时
    clock_gettime(CLOCK_REALTIME, &real_sample2);

    timespec time_diff;
    time_diff.tv_sec = real_sample2.tv_sec - monotonic_sample.tv_sec;
    time_diff.tv_nsec = real_sample2.tv_nsec - monotonic_sample.tv_nsec;

    // This isn't available outside of the kernel
    // real_time = timespec_add(monotonic_time, time_diff);

    const long NSEC_PER_SEC = 1000000000;
    real_time.tv_sec = monotonic_time.tv_sec + time_diff.tv_sec;
    real_time.tv_nsec = monotonic_time.tv_nsec + time_diff.tv_nsec - time_delay;
    if (real_time.tv_nsec >= NSEC_PER_SEC)
    {
      ++real_time.tv_sec;
      real_time.tv_nsec -= NSEC_PER_SEC;
    }
    else if (real_time.tv_nsec < 0)
    {
      --real_time.tv_sec;
      real_time.tv_nsec += NSEC_PER_SEC;
    }
  }

  static void errno_exit(const char *s)
  {
    ROS_ERROR("%s error %d, %s", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }

  static int xioctl(int fd, int request, void *arg)
  {
    int r;

    do
      r = ioctl(fd, request, arg);
    while (-1 == r && EINTR == errno);

    return r;
  }

  const unsigned char uchar_clipping_table[] = {
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, // -128 - -121
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, // -120 - -113
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, // -112 - -105
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, // -104 -  -97
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //  -96 -  -89
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //  -88 -  -81
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //  -80 -  -73
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //  -72 -  -65
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //  -64 -  -57
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //  -56 -  -49
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //  -48 -  -41
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //  -40 -  -33
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //  -32 -  -25
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //  -24 -  -17
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //  -16 -   -9
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0, //   -8 -   -1
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
      31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
      60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88,
      89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113,
      114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136,
      137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
      160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182,
      183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205,
      206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228,
      229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251,
      252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, // 256-263
      255, 255, 255, 255, 255, 255, 255, 255,                     // 264-271
      255, 255, 255, 255, 255, 255, 255, 255,                     // 272-279
      255, 255, 255, 255, 255, 255, 255, 255,                     // 280-287
      255, 255, 255, 255, 255, 255, 255, 255,                     // 288-295
      255, 255, 255, 255, 255, 255, 255, 255,                     // 296-303
      255, 255, 255, 255, 255, 255, 255, 255,                     // 304-311
      255, 255, 255, 255, 255, 255, 255, 255,                     // 312-319
      255, 255, 255, 255, 255, 255, 255, 255,                     // 320-327
      255, 255, 255, 255, 255, 255, 255, 255,                     // 328-335
      255, 255, 255, 255, 255, 255, 255, 255,                     // 336-343
      255, 255, 255, 255, 255, 255, 255, 255,                     // 344-351
      255, 255, 255, 255, 255, 255, 255, 255,                     // 352-359
      255, 255, 255, 255, 255, 255, 255, 255,                     // 360-367
      255, 255, 255, 255, 255, 255, 255, 255,                     // 368-375
      255, 255, 255, 255, 255, 255, 255, 255,                     // 376-383
  };
  const int clipping_table_offset = 128;

  /** Clip a value to the range 0<val<255. For speed this is done using an
   * array, so can only cope with numbers in the range -128<val<383.
   */
  static unsigned char CLIPVALUE(int val)
  {
    // Old method (if)
    /*   val = val < 0 ? 0 : val; */
    /*   return val > 255 ? 255 : val; */

    // New method (array)
    return uchar_clipping_table[val + clipping_table_offset];
  }

  /**
   * Conversion from YUV to RGB.
   * The normal conversion matrix is due to Julien (surname unknown):
   *
   * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
   * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
   * [ B ]   [  1.0   1.770   0.0   ] [ V ]
   *
   * and the firewire one is similar:
   *
   * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
   * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
   * [ B ]   [  1.0   1.015   0.0   ] [ V ]
   *
   * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
   *                   do not get you back to the same RGB!)
   * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
   * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
   * [ B ]   [  1.0   2.041   0.002 ] [ V ]
   *
   */
  static unsigned char RGB2Gray(const unsigned char R, const unsigned char G, const unsigned char B)
  {
    return (R * 38 + G * 75 + B * 15) >> 7;
  }

  static void YUV2RGB(const unsigned char y, const unsigned char u, const unsigned char v, unsigned char *r,
                      unsigned char *g, unsigned char *b)
  {
    const int y2 = (int)y;
    const int u2 = (int)u - 128;
    const int v2 = (int)v - 128;
    // std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;

    // This is the normal YUV conversion, but
    // appears to be incorrect for the firewire cameras
    //   int r2 = y2 + ( (v2*91947) >> 16);
    //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
    //   int b2 = y2 + ( (u2*115999) >> 16);
    // This is an adjusted version (UV spread out a bit)
    int r2 = y2 + ((v2 * 37221) >> 15);
    int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
    int b2 = y2 + ((u2 * 66883) >> 15);
    // std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;

    // Cap the values.
    *r = CLIPVALUE(r2);
    *g = CLIPVALUE(g2);
    *b = CLIPVALUE(b2);
  }

  void uyvy2rgb(char *YUV, char *RGB, int NumPixels)
  {
    int i, j;
    unsigned char y0, y1, u, v;
    unsigned char r, g, b;
    for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
      u = (unsigned char)YUV[i + 0];
      y0 = (unsigned char)YUV[i + 1];
      v = (unsigned char)YUV[i + 2];
      y1 = (unsigned char)YUV[i + 3];
      YUV2RGB(y0, u, v, &r, &g, &b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB(y1, u, v, &r, &g, &b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
    }
  }

  static void yuyv2mono8(uint8_t *YUYV, uint8_t *MONO8, int NumPixels)
  {
    //// TODO(lucasw) Disable interrupts here?
    //// otherwise what if there is a delay/interruption between sampling the times?
#if (!defined __arm__) && (!defined __aarch64__) // native_c
    for (; NumPixels > 0; NumPixels -= 1, YUYV += 2, MONO8 += 1)
    {
      // first byte is low byte, second byte is high byte; smash together and convert to 8-bit
      *MONO8 = *YUYV;
    }
#else // neon_intrinsics
    for (; NumPixels > 0; NumPixels -= 8, YUYV += 16, MONO8 += 8)
    {
      uint8x8x2_t y = vld2_u8(YUYV);
      vst1_u8(MONO8, y.val[0]);
    }
#endif
  }

  static void yuyv2rgb(char *YUV, char *RGB, int NumPixels)
  {
    int i, j;
    unsigned char y0, y1, u, v;
    unsigned char r, g, b;

    for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
      y0 = (unsigned char)YUV[i + 0];
      u = (unsigned char)YUV[i + 1];
      y1 = (unsigned char)YUV[i + 2];
      v = (unsigned char)YUV[i + 3];
      YUV2RGB(y0, u, v, &r, &g, &b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB(y1, u, v, &r, &g, &b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
    }
  }

  static void rgb242mono8(char *RGB, char *MONO, int NumPixels)
  {
    for (int i = 0, j = 0; i < NumPixels; i++, j += 3)
    {
      MONO[i] = RGB2Gray(RGB[j], RGB[j + 1], RGB[j + 2]);
    }
  }
  static void bgr242mono8(char *RGB, char *MONO, int NumPixels)
  {
    for (int i = 0, j = 0; i < NumPixels; i++, j += 3)
    {
      MONO[i] = RGB2Gray(RGB[j + 2], RGB[j + 1], RGB[j]);
    }
  }

  void rgb242rgb(char *YUV, char *RGB, int NumPixels)
  {
    memcpy(RGB, YUV, NumPixels * 3);
  }

  UsbCam::UsbCam()
  {
  }
  UsbCam::~UsbCam()
  {
    shutdown();
  }

  int UsbCam::init_mjpeg_decoder(int image_width, int image_height, int channel)
  {
    avcodec_register_all();

    avcodec_ = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    if (!avcodec_)
    {
      ROS_ERROR("Could not find MJPEG decoder");
      return 0;
    }

    avcodec_context_ = avcodec_alloc_context3(avcodec_);
#if LIBAVCODEC_VERSION_MAJOR < 55
    avframe_camera_ = avcodec_alloc_frame();
    avframe_output_ = avcodec_alloc_frame();
#else
    avframe_camera_ = av_frame_alloc();
    avframe_output_ = av_frame_alloc();
#endif
    if (channel < 2)
    {
      // TODO:灰度格式申请空间有问题，导致后面程序出错，这里强行改为其他格式申请更大空间
      // TODO:本函数ffmpeg弃用(is deprecated),修改可能修复上述问题
      avpicture_alloc((AVPicture *)avframe_output_, AV_PIX_FMT_YUV420P, image_width, image_height);
    }
    else
    {
      avpicture_alloc((AVPicture *)avframe_output_, output_image_avpixel_format_, image_width, image_height);
    }
    avcodec_context_->codec_id = AV_CODEC_ID_MJPEG;
    avcodec_context_->width = image_width;
    avcodec_context_->height = image_height;

#if LIBAVCODEC_VERSION_MAJOR > 52
    avcodec_context_->pix_fmt = input_image_avpixel_format_;
    avcodec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
#endif
    avframe_input_image_size_ = avpicture_get_size(input_image_avpixel_format_, image_width, image_height);
    avframe_output_image_size_ = avpicture_get_size(output_image_avpixel_format_, image_width, image_height);

    /* open it */
    if (avcodec_open2(avcodec_context_, avcodec_, &avoptions_) < 0)
    {
      ROS_ERROR("Could not open MJPEG Decoder");
      return 0;
    }
    return 1;
  }

  void UsbCam::mjpeg2mono8(char *MJPEG, int len, char *MONO8, int NumPixels)
  {
    int got_picture;

    memset(MONO8, 0, avframe_output_image_size_);

#if LIBAVCODEC_VERSION_MAJOR > 52
    int decoded_len;
    AVPacket avpkt;
    av_init_packet(&avpkt);

    avpkt.size = len;
    avpkt.data = (unsigned char *)MJPEG;
    decoded_len = avcodec_decode_video2(avcodec_context_, avframe_camera_, &got_picture, &avpkt);

    if (decoded_len < 0)
    {
      ROS_ERROR("Error while decoding frame.");
      return;
    }
#else
    avcodec_decode_video(avcodec_context_, avframe_camera_, &got_picture, (uint8_t *)MJPEG, len);
#endif

    if (!got_picture)
    {
      ROS_ERROR("Webcam: expected picture but didn't get it...");
      return;
    }

    int xsize = avcodec_context_->width;
    int ysize = avcodec_context_->height;
    int pic_size = avpicture_get_size(avcodec_context_->pix_fmt, xsize, ysize);
    if (pic_size != avframe_input_image_size_)
    {
      ROS_ERROR("outbuf size mismatch.  pic_size: %d bufsize: %d", pic_size, avframe_input_image_size_);
      return;
    }

    video_sws_ = sws_getContext(xsize, ysize, avcodec_context_->pix_fmt, xsize, ysize, AV_PIX_FMT_GRAY8, SWS_BILINEAR, nullptr,
                                nullptr, nullptr);
    sws_scale(video_sws_, avframe_camera_->data, avframe_camera_->linesize, 0, ysize, avframe_output_->data,
              avframe_output_->linesize);
    sws_freeContext(video_sws_);

    int size = avpicture_layout((AVPicture *)avframe_output_, AV_PIX_FMT_GRAY8, xsize, ysize, (uint8_t *)MONO8, avframe_output_image_size_);
    if (size != avframe_output_image_size_)
    {
      ROS_ERROR("webcam: avpicture_layout error: %d", size);
      return;
    }
  }

  void UsbCam::mjpeg2rgb(char *MJPEG, int len, char *RGB, int NumPixels)
  {
    int got_picture;

    memset(RGB, 0, avframe_output_image_size_);

#if LIBAVCODEC_VERSION_MAJOR > 52
    int decoded_len;
    AVPacket avpkt;
    av_init_packet(&avpkt);

    avpkt.size = len;
    avpkt.data = (unsigned char *)MJPEG;
    decoded_len = avcodec_decode_video2(avcodec_context_, avframe_camera_, &got_picture, &avpkt);

    if (decoded_len < 0)
    {
      ROS_ERROR("Error while decoding frame.");
      return;
    }
#else
    avcodec_decode_video(avcodec_context_, avframe_camera_, &got_picture, (uint8_t *)MJPEG, len);
#endif

    if (!got_picture)
    {
      ROS_ERROR("Webcam: expected picture but didn't get it...");
      return;
    }

    int xsize = avcodec_context_->width;
    int ysize = avcodec_context_->height;
    int pic_size = avpicture_get_size(avcodec_context_->pix_fmt, xsize, ysize);
    if (pic_size != avframe_input_image_size_)
    {
      ROS_ERROR("outbuf size mismatch.  pic_size: %d bufsize: %d", pic_size, avframe_input_image_size_);
      return;
    }

    video_sws_ = sws_getContext(xsize, ysize, avcodec_context_->pix_fmt, xsize, ysize, AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr,
                                nullptr, nullptr);
    sws_scale(video_sws_, avframe_camera_->data, avframe_camera_->linesize, 0, ysize, avframe_output_->data,
              avframe_output_->linesize);
    sws_freeContext(video_sws_);

    int size = avpicture_layout((AVPicture *)avframe_output_, AV_PIX_FMT_RGB24, xsize, ysize, (uint8_t *)RGB, avframe_output_image_size_);
    if (size != avframe_output_image_size_)
    {
      ROS_ERROR("webcam: avpicture_layout error: %d", size);
      return;
    }
  }

  int UsbCam::process_image(const void *src, int len, camera_image_t *dest)
  {
    static int lastWidth = 0, lastHeight = 0;
    static int lastSize = 0;
    if (dest->width != lastWidth || dest->height != lastHeight || lastSize == 0)
    {
      lastWidth = dest->width;
      lastHeight = dest->height;
      lastSize = dest->width * dest->height * dest->bytes_per_pixel;
    }
#ifdef test_time
    struct timespec start, end;
    clock_gettime(CLOCK_REALTIME, &start);
#endif
    if (input_image_v4l_format_ == V4L2_PIX_FMT_YUYV)
    {
      if (publish_image_format_.compare(sensor_msgs::image_encodings::MONO8) == 0)
      {
        yuyv2mono8((uint8_t *)src, (uint8_t *)dest->image, lastSize);
        // ROS_WARN("1");
      }
      else if (publish_image_format_.compare(sensor_msgs::image_encodings::RGB8) == 0)
      {
        yuyv2rgb((char *)src, dest->image, lastSize);
      }
      else
      {
        return -1;
      }
    }
    else if (input_image_v4l_format_ == V4L2_PIX_FMT_UYVY)
    {
      if (publish_image_format_.compare(sensor_msgs::image_encodings::RGB8) == 0)
      {
        uyvy2rgb((char *)src, dest->image, lastSize);
      }
      else
      {
        return -1;
      }
    }
    else if (input_image_v4l_format_ == V4L2_PIX_FMT_MJPEG)
    {
      if (publish_image_format_.compare(sensor_msgs::image_encodings::MONO8) == 0)
      {
        mjpeg2mono8((char *)src, len, dest->image, lastSize);
      }
      else if (publish_image_format_.compare(sensor_msgs::image_encodings::RGB8) == 0)
      {
        mjpeg2rgb((char *)src, len, dest->image, lastSize);
      }
      else
      {
        return -1;
      }
    }
    else if (input_image_v4l_format_ == V4L2_PIX_FMT_RGB24)
    {
      if (publish_image_format_.compare(sensor_msgs::image_encodings::RGB8) == 0)
      {
        rgb242rgb((char *)src, dest->image, lastSize);
      }
      else if (publish_image_format_.compare(sensor_msgs::image_encodings::MONO8) == 0)
      {
        rgb242mono8((char *)src, dest->image, lastSize);
      }
      else
      {
        return -1;
      }
    }
    else if (input_image_v4l_format_ == V4L2_PIX_FMT_BGR24)
    {
      if (publish_image_format_.compare(sensor_msgs::image_encodings::MONO8) == 0)
      {
        bgr242mono8((char *)src, dest->image, lastSize);
      }
      else
      {
        return -1;
      }
    }
    else if (input_image_v4l_format_ == V4L2_PIX_FMT_GREY)
    {
      if (publish_image_format_.compare(sensor_msgs::image_encodings::MONO8) == 0)
      {
        memcpy(dest->image, (char *)src, lastSize);
      }
      else
      {
        return -1;
      }
    }
    else
    {
      return -1;
    }

#ifdef test_time
    clock_gettime(CLOCK_REALTIME, &end);

    static timespec sum_time_diff = {0};
    static int sum = 0;
    timespec time_diff = {0};
    time_diff.tv_sec = end.tv_sec - start.tv_sec;
    if (end.tv_nsec >= start.tv_nsec)
    {
      time_diff.tv_nsec = end.tv_nsec - start.tv_nsec;
    }
    else
    {
      time_diff.tv_sec -= 1;
      time_diff.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
    }
    sum_time_diff.tv_sec += time_diff.tv_sec;
    sum_time_diff.tv_nsec += time_diff.tv_nsec;
    while (sum_time_diff.tv_nsec >= 1000000000)
    {
      sum_time_diff.tv_nsec -= 1000000000;
      sum_time_diff.tv_sec += 1;
    }
    sum++;

    printf("cur:%ld.%09ld sum=%ld.%09ld num=%d avg=%ld.%09ld\r\n",
           time_diff.tv_sec, time_diff.tv_nsec,
           sum_time_diff.tv_sec, sum_time_diff.tv_nsec, sum,
           sum_time_diff.tv_sec / sum, (sum_time_diff.tv_sec % sum * 1000000000 + sum_time_diff.tv_nsec) / sum);
#endif
    return 0;
  }

  int UsbCam::read_frame()
  {
    struct v4l2_buffer buf;
    struct v4l2_plane planes = {0};
    unsigned int i;
    int len;
    ros::Time stamp;
    timespec buf_time;
    timespec real_time;
    // static double lastStamp = -1;
    int ret = -1;
    switch (io_)
    {
    default:
    case IO_METHOD_UNKNOWN:
      break;
    case IO_METHOD_READ:
      len = read(fd_video, buffers_[0].start, buffers_[0].length);
      if (len == -1)
      {
        switch (errno)
        {
        case EAGAIN:
          return 0;

        case EIO:
          /* Could ignore EIO, see spec. */

          /* fall through */

        default:
          errno_exit("read");
        }
      }
      stamp = ros::Time::now();
      // if (stamp.toSec() < lastStamp + framerate_tb_)
      // {
      //   assert(false);
      //   // if (-1 == xioctl(fd_video, VIDIOC_QBUF, &buf))
      //   //   errno_exit("VIDIOC_QBUF");
      //   return 0;
      // }
      if (process_image_in_other_thread_)
      {
        org_data od;
        memcpy(od.data, (char *)buffers_[0].start, len);
        od.dataLen = len;
        od.stamp = stamp;
        datas.push_back(od);
      }
      else
      {
        ret = process_image(buffers_[0].start, len, image_);
        if (ret >= 0)
          ret = 1;
      }
      image_->stamp = stamp;
      // lastStamp = stamp.toSec();
      // TODO(lucasw) how to get timestamp with this method?

      break;

    case IO_METHOD_MMAP:
      CLEAR(buf);
      if (V4L2_TYPE_IS_MULTIPLANAR(m_v4l2_buf_type))
      {
        CLEAR(planes);
        buf.length = 1;
        buf.m.planes = &planes;
      }

      buf.type = m_v4l2_buf_type;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fd_video, VIDIOC_DQBUF, &buf))
      {
        switch (errno)
        {
        case EAGAIN:
          return 0;

        case EIO:
          /* Could ignore EIO, see spec. */

          /* fall through */

        default:
          errno_exit("VIDIOC_DQBUF");
        }
      }

      // need to get buf time here otherwise process_image will zero it
      TIMEVAL_TO_TIMESPEC(&buf.timestamp, &buf_time);
      monotonicToRealTime(buf_time, real_time, time_delay_per_byte_ * buf.bytesused);
      stamp = ros::Time(real_time.tv_sec, real_time.tv_nsec);
      // if (stamp.toSec() < lastStamp + framerate_tb_)
      // {
      //   if (-1 == xioctl(fd_video, VIDIOC_QBUF, &buf))
      //     errno_exit("VIDIOC_QBUF");
      //   return 0;
      // }
      assert(buf.index < n_buffers_);
      len = buf.bytesused;
      if (process_image_in_other_thread_)
      {
        org_data od;
        memcpy(od.data, (char *)buffers_[buf.index].start, len);
        od.dataLen = len;
        od.stamp = stamp;
        datas.push_back(od);
      }
      else
      {
        ret = process_image(buffers_[buf.index].start, len, image_);
        if (ret >= 0)
          ret = 1;
      }

      if (-1 == xioctl(fd_video, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");

      image_->stamp = stamp;
      // lastStamp = stamp.toSec();

      break;

    case IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = m_v4l2_buf_type;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fd_video, VIDIOC_DQBUF, &buf))
      {
        switch (errno)
        {
        case EAGAIN:
          return 0;

        case EIO:
          /* Could ignore EIO, see spec. */

          /* fall through */

        default:
          errno_exit("VIDIOC_DQBUF");
        }
      }
      TIMEVAL_TO_TIMESPEC(&buf.timestamp, &buf_time);
      monotonicToRealTime(buf_time, real_time, time_delay_per_byte_ * buf.bytesused);
      stamp = ros::Time(real_time.tv_sec, real_time.tv_nsec);

      // if (stamp.toSec() < lastStamp + framerate_tb_)
      // {
      //   assert(false);
      //   // if (-1 == xioctl(fd_video, VIDIOC_QBUF, &buf))
      //   //   errno_exit("VIDIOC_QBUF");
      //   return 0;
      // }
      for (i = 0; i < n_buffers_; ++i)
        if (buf.m.userptr == (unsigned long)buffers_[i].start && buf.length == buffers_[i].length)
          break;

      assert(i < n_buffers_);
      len = buf.bytesused;
      if (process_image_in_other_thread_)
      {
        org_data od;
        memcpy(od.data, (char *)buf.m.userptr, len);
        od.dataLen = len;
        od.stamp = stamp;
        datas.push_back(od);
      }
      else
      {
        ret = process_image((void *)buf.m.userptr, len, image_);
        if (ret >= 0)
          ret = 1;
      }

      if (-1 == xioctl(fd_video, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");

      image_->stamp = stamp;
      // lastStamp = stamp.toSec();
      break;
    }

    return ret;
  }

  bool UsbCam::is_capturing()
  {
    return is_capturing_;
  }

  void UsbCam::stop_capturing(void)
  {
    if (!is_capturing_)
      return;

    is_capturing_ = false;
    enum v4l2_buf_type type;

    switch (io_)
    {
    default:
    case IO_METHOD_UNKNOWN:
      break;
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = (v4l2_buf_type)m_v4l2_buf_type;

      if (-1 == xioctl(fd_video, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");

      break;
    }
  }

  void UsbCam::start_capturing(void)
  {

    if (is_capturing_)
      return;

    unsigned int i;
    enum v4l2_buf_type type;

    switch (io_)
    {
    default:
    case IO_METHOD_UNKNOWN:
      break;
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i)
      {
        struct v4l2_buffer buf;
        CLEAR(buf);
        struct v4l2_plane planes = {0};
        if (V4L2_TYPE_IS_MULTIPLANAR(m_v4l2_buf_type))
        {
          CLEAR(planes);
          buf.length = 1;
          buf.m.planes = &planes;
          buf.m.planes->length = buffers_[i].length;
          buf.m.planes->m.mem_offset = buffers_[i].offset;
          // buf.m.planes->m.mem_offset = (unsigned long)buffers_[i].start;
        }

        buf.type = m_v4l2_buf_type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd_video, VIDIOC_QBUF, &buf))
          errno_exit("VIDIOC_QBUF");
      }

      type = (v4l2_buf_type)m_v4l2_buf_type;

      if (-1 == xioctl(fd_video, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");

      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i)
      {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = m_v4l2_buf_type;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = (unsigned long)buffers_[i].start;
        buf.length = buffers_[i].length;

        if (-1 == xioctl(fd_video, VIDIOC_QBUF, &buf))
          errno_exit("VIDIOC_QBUF");
      }

      type = (v4l2_buf_type)m_v4l2_buf_type;

      if (-1 == xioctl(fd_video, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");

      break;
    }
    is_capturing_ = true;
  }

  void UsbCam::uninit_device(void)
  {
    unsigned int i;

    switch (io_)
    {
    default:
    case IO_METHOD_UNKNOWN:
      break;
    case IO_METHOD_READ:
      free(buffers_[0].start);
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i)
        if (-1 == munmap(buffers_[i].start, buffers_[i].length))
          errno_exit("munmap");
      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i)
        free(buffers_[i].start);
      break;
    }

    free(buffers_);
  }

  void UsbCam::init_read(unsigned int buffer_size)
  {
    buffers_ = (buffer *)calloc(1, sizeof(*buffers_));

    if (!buffers_)
    {
      ROS_ERROR("Out of memory");
      exit(EXIT_FAILURE);
    }

    buffers_[0].length = buffer_size;
    buffers_[0].start = malloc(buffer_size);

    if (!buffers_[0].start)
    {
      ROS_ERROR("Out of memory");
      exit(EXIT_FAILURE);
    }
  }

  void UsbCam::init_mmap(void)
  {
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 2;
    req.type = m_v4l2_buf_type;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd_video, VIDIOC_REQBUFS, &req))
    {
      if (EINVAL == errno)
      {
        ROS_ERROR_STREAM(camera_dev_ << " does not support memory mapping");
        exit(EXIT_FAILURE);
      }
      else
      {
        errno_exit("VIDIOC_REQBUFS");
      }
    }

    if (req.count < 2)
    {
      ROS_ERROR_STREAM("Insufficient buffer memory on " << camera_dev_);
      exit(EXIT_FAILURE);
    }

    buffers_ = (buffer *)calloc(req.count, sizeof(*buffers_));

    if (!buffers_)
    {
      ROS_ERROR("Out of memory");
      exit(EXIT_FAILURE);
    }

    for (n_buffers_ = 0; n_buffers_ < req.count; ++n_buffers_)
    {
      struct v4l2_buffer buf;
      CLEAR(buf);
      struct v4l2_plane planes = {0};
      if (V4L2_TYPE_IS_MULTIPLANAR(m_v4l2_buf_type))
      {
        CLEAR(planes);
        buf.length = 1;
        buf.m.planes = &planes;
      }

      buf.type = m_v4l2_buf_type;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = n_buffers_;

      if (-1 == xioctl(fd_video, VIDIOC_QUERYBUF, &buf))
        errno_exit("VIDIOC_QUERYBUF");

      if (V4L2_TYPE_IS_MULTIPLANAR(m_v4l2_buf_type))
      {
        buffers_[n_buffers_].length = buf.m.planes->length;
        buffers_[n_buffers_].offset = buf.m.planes->m.mem_offset;
      }
      else
      {
        buffers_[n_buffers_].length = buf.length;
        buffers_[n_buffers_].offset = buf.m.offset;
      }
      buffers_[n_buffers_].start = mmap(nullptr /* start anywhere */,
                                        buffers_[n_buffers_].length,
                                        PROT_READ | PROT_WRITE /* required */,
                                        MAP_SHARED /* recommended */,
                                        fd_video,
                                        buffers_[n_buffers_].offset);
      if (MAP_FAILED == buffers_[n_buffers_].start)
        errno_exit("mmap");
      memset(buffers_[n_buffers_].start, 0xFF, buffers_[n_buffers_].length);
    }
  }

  void UsbCam::init_userp(unsigned int buffer_size)
  {
    struct v4l2_requestbuffers req;
    unsigned int page_size;

    page_size = getpagesize();
    buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

    CLEAR(req);

    req.count = 4;
    req.type = m_v4l2_buf_type;
    req.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(fd_video, VIDIOC_REQBUFS, &req))
    {
      if (EINVAL == errno)
      {
        ROS_ERROR_STREAM(camera_dev_ << " does not support "
                                        "user pointer i/o");
        exit(EXIT_FAILURE);
      }
      else
      {
        errno_exit("VIDIOC_REQBUFS");
      }
    }

    buffers_ = (buffer *)calloc(4, sizeof(*buffers_));

    if (!buffers_)
    {
      ROS_ERROR("Out of memory");
      exit(EXIT_FAILURE);
    }

    for (n_buffers_ = 0; n_buffers_ < 4; ++n_buffers_)
    {
      buffers_[n_buffers_].length = buffer_size;
      buffers_[n_buffers_].start = memalign(/* boundary */ page_size, buffer_size);

      if (!buffers_[n_buffers_].start)
      {
        ROS_ERROR("Out of memory");
        exit(EXIT_FAILURE);
      }
    }
  }

  void UsbCam::init_device(int image_width, int image_height, int framerate)
  {
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;

    if (-1 == xioctl(fd_video, VIDIOC_QUERYCAP, &cap))
    {
      if (EINVAL == errno)
      {
        ROS_ERROR_STREAM(camera_dev_ << " is no V4L2 device");
        exit(EXIT_FAILURE);
      }
      else
      {
        errno_exit("VIDIOC_QUERYCAP");
      }
    }

    ROS_INFO("cap.capabilities=%X\r\n", cap.capabilities);

    if (!(cap.capabilities & V4L2_CAP_DEVICE_CAPS))
    {
      ROS_ERROR_STREAM(camera_dev_ << " is no video capture device");
      exit(EXIT_FAILURE);
    }

    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ROS_INFO("capabilities: V4L2_CAP_VIDEO_CAPTURE");
    }
    else if (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
      ROS_INFO("capabilities: V4L2_CAP_VIDEO_OUTPUT");
    }
    else if (cap.capabilities & V4L2_CAP_VIDEO_OVERLAY)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_VIDEO_OVERLAY;
      ROS_INFO("capabilities: V4L2_CAP_VIDEO_OVERLAY");
    }
    else if (cap.capabilities & V4L2_CAP_VBI_CAPTURE)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_VBI_CAPTURE;
      ROS_INFO("capabilities: V4L2_CAP_VBI_CAPTURE");
    }
    else if (cap.capabilities & V4L2_CAP_VBI_OUTPUT)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_VBI_OUTPUT;
      ROS_INFO("capabilities: V4L2_CAP_VBI_OUTPUT");
    }
    else if (cap.capabilities & V4L2_CAP_SLICED_VBI_CAPTURE)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_SLICED_VBI_CAPTURE;
      ROS_INFO("capabilities: V4L2_CAP_SLICED_VBI_CAPTURE");
    }
    else if (cap.capabilities & V4L2_CAP_SLICED_VBI_OUTPUT)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_SLICED_VBI_OUTPUT;
      ROS_INFO("capabilities: V4L2_CAP_SLICED_VBI_OUTPUT");
    }
    else if (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT_OVERLAY)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY;
      ROS_INFO("capabilities: V4L2_CAP_VIDEO_OUTPUT_OVERLAY");
    }
    else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      ROS_INFO("capabilities: V4L2_CAP_VIDEO_CAPTURE_MPLANE");
    }
    else if (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
      ROS_INFO("capabilities: V4L2_CAP_VIDEO_OUTPUT_MPLANE");
    }
    else if (cap.capabilities & V4L2_CAP_SDR_CAPTURE)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_SDR_CAPTURE;
      ROS_INFO("capabilities: V4L2_CAP_SDR_CAPTURE");
    }
    else if (cap.capabilities & V4L2_CAP_SDR_OUTPUT)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_SDR_OUTPUT;
      ROS_INFO("capabilities: V4L2_CAP_SDR_OUTPUT");
    }
    else if (cap.capabilities & V4L2_CAP_META_CAPTURE)
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_META_CAPTURE;
      ROS_INFO("capabilities: V4L2_CAP_META_CAPTURE");
    }
    else
    {
      m_v4l2_buf_type = V4L2_BUF_TYPE_PRIVATE;
      ROS_ERROR_STREAM(camera_dev_ << " is  unknown V4L2_BUF_TYPE");
      exit(EXIT_FAILURE);
    }

    switch (io_)
    {
    default:
    case IO_METHOD_UNKNOWN:
      break;
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE))
      {
        ROS_ERROR_STREAM(camera_dev_ << " does not support read i/o");
        exit(EXIT_FAILURE);
      }

      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING))
      {
        ROS_ERROR_STREAM(camera_dev_ << " does not support streaming i/o");
        exit(EXIT_FAILURE);
      }
      break;
    }

    /* Select video input, video standard and tune here. */

    CLEAR(cropcap);

    cropcap.type = m_v4l2_buf_type;

    if (0 == xioctl(fd_video, VIDIOC_CROPCAP, &cropcap))
    {
      crop.type = m_v4l2_buf_type;
      crop.c = cropcap.defrect; /* reset to default */

      if (-1 == xioctl(fd_video, VIDIOC_S_CROP, &crop))
      {
        switch (errno)
        {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
        }
      }
    }
    else
    {
      /* Errors ignored. */
    }

    uint32_t sizeImage = image_width * image_height * 4; // TODO: 获取通道数，其他格式maybe通道数为1or2or3
    if (!V4L2_TYPE_IS_OUTPUT(m_v4l2_buf_type))
    {
      CLEAR(fmt);
      fmt.type = m_v4l2_buf_type;
      if (xioctl(fd_video, VIDIOC_G_FMT, &fmt) < 0)
        errno_exit("Couldn't query v4l fmt!");

      if (fmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
      {
        fmt.fmt.pix.width = image_width;
        fmt.fmt.pix.height = image_height;
        fmt.fmt.pix.pixelformat = input_image_v4l_format_;
        // fmt.fmt.pix.field = m_v4l2_field;
        if (-1 == xioctl(fd_video, VIDIOC_S_FMT, &fmt))
          errno_exit("VIDIOC_S_FMT");
      }
      else if (fmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
      {
        fmt.fmt.pix_mp.width = image_width;
        fmt.fmt.pix_mp.height = image_height;
        fmt.fmt.pix_mp.pixelformat = input_image_v4l_format_;
        // fmt.fmt.pix_mp.field = m_v4l2_field;
        if (-1 == xioctl(fd_video, VIDIOC_S_FMT, &fmt))
          errno_exit("VIDIOC_S_FMT");
      }
      else if (fmt.type == V4L2_BUF_TYPE_VIDEO_OVERLAY)
      {
      }
      else if (fmt.type == V4L2_BUF_TYPE_VBI_CAPTURE)
      {
      }
      else if (fmt.type == V4L2_BUF_TYPE_SLICED_VBI_CAPTURE)
      {
      }
      else if (fmt.type == V4L2_BUF_TYPE_SDR_CAPTURE)
      {
      }
      else if (fmt.type == V4L2_BUF_TYPE_META_CAPTURE)
      {
      }
      if (xioctl(fd_video, VIDIOC_G_FMT, &fmt) < 0)
        errno_exit("Couldn't query v4l fmt!");
      if (fmt.fmt.pix.sizeimage > 0)
      {
        sizeImage = fmt.fmt.pix.sizeimage;
      }
      else if (fmt.fmt.pix.bytesperline > 0)
      {
        sizeImage = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
      }
      else
      {
        sizeImage = fmt.fmt.pix.width * fmt.fmt.pix.height * 4; // TODO: 获取通道数，其他格式maybe通道数为1or2or3
      }
    }

    struct v4l2_streamparm stream_params;
    memset(&stream_params, 0, sizeof(stream_params));
    stream_params.type = m_v4l2_buf_type;
    if (xioctl(fd_video, VIDIOC_G_PARM, &stream_params) < 0)
      errno_exit("Couldn't query v4l fps!");

    if (V4L2_TYPE_IS_OUTPUT(m_v4l2_buf_type))
    {
    }
    else
    {
      stream_params.parm.capture.timeperframe.numerator = 1;
      stream_params.parm.capture.timeperframe.denominator = framerate;
      if (xioctl(fd_video, VIDIOC_S_PARM, &stream_params) < 0)
        ROS_WARN("Couldn't set camera framerate");
      else
        ROS_DEBUG("Set framerate to be %i", framerate);
    }

    switch (io_)
    {
    default:
    case IO_METHOD_UNKNOWN:
      break;
    case IO_METHOD_READ:
      init_read(sizeImage);
      break;

    case IO_METHOD_MMAP:
      init_mmap();
      break;

    case IO_METHOD_USERPTR:
      init_userp(sizeImage);
      break;
    }
  }

  void UsbCam::close_device(void)
  {
    if (-1 == close(fd_video))
      errno_exit("close");

    fd_video = -1;

    if (app_dwe)
    {
      delete app_dwe;
      app_dwe = nullptr;
    }
    if (app_isp)
    {
      delete app_isp;
      app_isp = nullptr;
    }
  }

  void UsbCam::open_device(void)
  {
    struct stat st;

    if (-1 == stat(camera_dev_.c_str(), &st))
    {
      ROS_ERROR_STREAM("Cannot identify '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
      exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode))
    {
      ROS_ERROR_STREAM(camera_dev_ << " is no device");
      exit(EXIT_FAILURE);
    }

    fd_video = open(camera_dev_.c_str(), O_RDWR /* required */, 0);

    if (-1 == fd_video)
    {
      ROS_ERROR_STREAM("Cannot open '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
      exit(EXIT_FAILURE);
    }
  }

  int UsbCam::start(const std::string &dev, io_method io_method,
                    std::string input_image_format, int image_width, int image_height,
                    int framerate, double framerate_tb, double time_delay_per_byte,
                    bool process_image_in_other_thread, std::string publish_image_format,
                    bool enable_isp, bool enable_dwe, const std::string IspFileName, const std::string DweFileName)
  {
    camera_dev_ = dev;
    time_delay_per_byte_ = time_delay_per_byte;
    process_image_in_other_thread_ = process_image_in_other_thread;
    io_ = io_method;
    publish_image_format_ = publish_image_format;
    image_width_ = image_width;
    image_height_ = image_height;
    framerate_ = framerate;
    framerate_tb_ = framerate_tb;
    enable_isp_ = enable_isp;
    enable_dwe_ = enable_dwe;
    ispFileName_ = IspFileName;
    dweFileName_ = DweFileName;
    input_image_v4l_format_ = v4l2_fourcc(input_image_format[0],
                                          input_image_format[1],
                                          input_image_format[2],
                                          input_image_format[3]);
    int channel = sensor_msgs::image_encodings::numChannels(publish_image_format_);
    if (input_image_v4l_format_ == V4L2_PIX_FMT_MJPEG)
    {
      input_image_avpixel_format_ = v4l_format_to_avpixel_format(input_image_v4l_format_);
      if (input_image_avpixel_format_ == AV_PIX_FMT_NONE)
      {
        return -1;
      }
      output_image_avpixel_format_ = ros_image_format_to_avpixel_format(publish_image_format);
      if (output_image_avpixel_format_ == AV_PIX_FMT_NONE)
      {
        return -1;
      }
      init_mjpeg_decoder(image_width, image_height, channel);
    }

    init_isp_dwe();

    open_device();
    init_device(image_width, image_height, framerate);
    if (!V4L2_TYPE_IS_OUTPUT(m_v4l2_buf_type))
    {
      start_capturing();
    }

    image_ = (camera_image_t *)calloc(1, sizeof(camera_image_t));

    image_->width = image_width;
    image_->height = image_height;
    image_->bytes_per_pixel = channel; // corrected 11/10/15 (BYTES not BITS per pixel)
    image_->image_size = image_->width * image_->height * image_->bytes_per_pixel;
    // image_->is_new = 0;
    image_->image = (char *)calloc(image_->image_size, sizeof(char));
    memset(image_->image, 0, image_->image_size * sizeof(char));

    // orgDataNum = framerate;
    // datas = new org_data[orgDataNum];
    return 0;
  }

  void UsbCam::shutdown(void)
  {
    stop_capturing();
    uninit_device();
    close_device();

    if (avcodec_context_)
    {
      avcodec_close(avcodec_context_);
      av_free(avcodec_context_);
      avcodec_context_ = nullptr;
    }
    if (avframe_camera_)
      av_free(avframe_camera_);
    avframe_camera_ = nullptr;
    if (avframe_output_)
      av_free(avframe_output_);
    avframe_output_ = nullptr;
    if (image_)
      free(image_);
    image_ = nullptr;
    // if(datas)
    // {
    //   for(int i=0; i<orgDataNum; i++)
    //   {
    //     if(datas[i].data)
    //     {
    //       delete datas[i].data;
    //       datas[i].data = nullptr;
    //     }
    //   }
    //   delete []datas;
    // }
    // datas = nullptr;
  }

  int UsbCam::grab_image(sensor_msgs::Image *msg)
  {
    // grab the image
    int ret = grab_image();
    if (ret > 0 && !process_image_in_other_thread_)
    {
      // stamp the image
      msg->header.stamp = image_->stamp;
      // fill the info
      int chan = sensor_msgs::image_encodings::numChannels(publish_image_format_);
      if (chan > 0)
      {
        fillImage(*msg, publish_image_format_, image_->height, image_->width, chan * image_->width,
                  image_->image);
      }
    }
    return ret;
  }

  int UsbCam::grab_image()
  {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(fd_video, &fds);

    /* Timeout. */
    tv.tv_sec = 5;
    tv.tv_usec = 0;

    r = select(fd_video + 1, &fds, nullptr, nullptr, &tv);
    // if the v4l2_buffer timestamp isn't available use this time, though
    // it may be 10s of milliseconds after the frame acquisition.
    image_->stamp = ros::Time::now();

    if (-1 == r)
    {
      if (EINTR == errno)
        return -1;

      errno_exit("select");
    }

    if (0 == r)
    {
      ROS_ERROR("select timeout");
      exit(EXIT_FAILURE);
    }

    int ret = read_frame();
    // if (ret > 0)
    //   image_->is_new = 1;

    return ret;
  }

  // enables/disables auto focus
  void UsbCam::set_auto_focus(int value)
  {
    struct v4l2_queryctrl queryctrl;
    struct v4l2_ext_control control;

    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CID_FOCUS_AUTO;

    if (-1 == xioctl(fd_video, VIDIOC_QUERYCTRL, &queryctrl))
    {
      if (errno != EINVAL)
      {
        perror("VIDIOC_QUERYCTRL");
        return;
      }
      else
      {
        ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
        return;
      }
    }
    else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    {
      ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
      return;
    }
    else
    {
      memset(&control, 0, sizeof(control));
      control.id = V4L2_CID_FOCUS_AUTO;
      control.value = value;

      if (-1 == xioctl(fd_video, VIDIOC_S_CTRL, &control))
      {
        perror("VIDIOC_S_CTRL");
        return;
      }
    }
  }

  /**
   * Set video device parameter via call to v4l-utils.
   *
   * @param param The name of the parameter to set
   * @param param The value to assign
   */
  void UsbCam::set_v4l_parameter(const std::string &param, int value, const std::string dev)
  {
    set_v4l_parameter(param, boost::lexical_cast<std::string>(value), dev);
  }
  /**
   * Set video device parameter via call to v4l-utils.
   *
   * @param param The name of the parameter to set
   * @param param The value to assign
   */
  void UsbCam::set_v4l_parameter(const std::string &param, const std::string &value, const std::string dev)
  {
    // build the command
    std::stringstream ss;
    if (dev.size() > 0)
    {
      ss << "v4l2-ctl --device=" << dev << " -c " << param << "=" << value << " 2>&1";
    }
    else
    {
      ss << "v4l2-ctl --device=" << camera_dev_ << " -c " << param << "=" << value << " 2>&1";
    }
    std::string cmd = ss.str();

    // capture the output
    std::string output;
    int buffer_size = 256;
    char buffer[buffer_size];
    FILE *stream = popen(cmd.c_str(), "r");
    if (stream)
    {
      while (!feof(stream))
        if (fgets(buffer, buffer_size, stream) != nullptr)
          output.append(buffer);
      pclose(stream);
      // any output should be an error
      if (output.length() > 0)
        ROS_WARN("%s", output.c_str());
    }
    else
      ROS_WARN("usb_cam_node could not run '%s'", cmd.c_str());
  }

  int UsbCam::get_v4l_parameter_list(const std::string dev, const std::string camera_name)
  {
    char line[256];
    char cmd[256];
    if (dev.size() > 0)
    {
      sprintf(cmd, "v4l2-ctl --device=%s -L", dev.c_str());
    }
    else
    {
      sprintf(cmd, "v4l2-ctl --device=%s -L", camera_dev_.c_str());
    }

    FILE *fp = popen(cmd, "r");
    if (fp != nullptr)
    {
      ctrlParaList.clear();
      while (fgets(line, sizeof(line), fp) != nullptr)
      {
        char sName[64] = {0};
        char sValue[5][64] = {0};
        bool bFind = false;
        v4lCtrlPara curV4lCtrlPara;
        sscanf(line, "%s%*[^:]:%s%s%s%s%s", sName, sValue[0], sValue[1], sValue[2], sValue[3], sValue[4]);
        // 获取数值
        for (int i = 0; i < 5; i++)
        {
          if (strlen(sValue[i]) > 0)
          {
            char valueName[64] = {0};
            int val = 0;
            sscanf(sValue[i], "%[^=]=%d", valueName, &val);
            if (strlen(valueName) > 0)
            {
              if (!bFind)
              {
                bFind = true;
                ROS_INFO("%s:", sName);
              }
              ROS_INFO("\t%s = %d", valueName, val);
              if (strstr(valueName, "value") != nullptr)
              {
                curV4lCtrlPara.value_int = val;
              }
              else if (strstr(valueName, "default") != nullptr)
              {
                curV4lCtrlPara.default_int = val;
              }
              else if (strstr(valueName, "min") != nullptr)
              {
                curV4lCtrlPara.min_int = val;
              }
              else if (strstr(valueName, "max") != nullptr)
              {
                curV4lCtrlPara.max_int = val;
              }
              else if (strstr(valueName, "step") != nullptr)
              {
                curV4lCtrlPara.step_int = val;
              }
            }
          }
        }
        ctrlParaList.insert(std::pair<std::string, v4lCtrlPara>(sName, curV4lCtrlPara));
      }
      pclose(fp);
      return 0;
    }
    return -1;
    // cmd return on pc:       zoom_absolute 0x009a090d (int)    : min=0 max=3 step=1 default=0 value=0
    // cmd return on imx8mq:   zoom_absolute 0x009a090d (int)    : min=0 max=3 step=1 default=0 value=0
  }

  int UsbCam::get_v4l_format_list(std::set<std::string> &formatList, const std::string dev)
  {
    char line[512];
    char cmd[256];
    if (dev.size() > 0)
    {
      sprintf(cmd, "v4l2-ctl --device=%s --list-formats", dev.c_str());
    }
    else
    {
      sprintf(cmd, "v4l2-ctl --device=%s --list-formats", camera_dev_.c_str());
    }
    FILE *fp = popen(cmd, "r");
    if (fp != nullptr)
    {
      formatList.clear();
      ROS_INFO("Pixel Format:");
      while (fgets(line, sizeof(line), fp) != nullptr)
      {
        if (strstr(line, "]") != nullptr ||
            strstr(line, "Pixel Format") != nullptr)
        {
          char t[32] = {0};
          sscanf(line, "%*[^']'%[^']", t);
          std::string s = t;
          formatList.insert(s);
          ROS_INFO("\t%s", s.c_str());
        }
      }
      pclose(fp);
      return formatList.size();
    }
    else
    {
      return -1;
    }

    // cmd return on pc:
    // Index       : 0
    // Type        : Video Capture
    // Pixel Format: 'MJPG' (compressed)
    // Name        : Motion-JPEG

    // cmd return on imx8mp:
    // [0]: 'MJPG' (Motion-JPEG, compressed)
  }

  int UsbCam::get_v4l_fields(std::set<std::string> &fieldList, const std::string dev)
  {
    char line[512];
    char cmd[256];
    if (dev.size() > 0)
    {
      sprintf(cmd, "v4l2-ctl --device=%s --list-fields", dev.c_str());
    }
    else
    {
      sprintf(cmd, "v4l2-ctl --device=%s --list-fields", camera_dev_.c_str());
    }
    FILE *fp = popen(cmd, "r");
    if (fp != nullptr)
    {
      fieldList.clear();
      ROS_INFO("Video Fields:");
      int i = 0;
      while (fgets(line, sizeof(line), fp) != nullptr)
      {
        if (i > 0)
        {
          char t[64] = {0};
          sscanf(line, "%s", t);
          std::string s = t;
          fieldList.insert(s);
          ROS_INFO("\t%s", s.c_str());
        }
        i++;
      }
      pclose(fp);
      return fieldList.size();
    }
    else
    {
      return -1;
    }
  }

  int UsbCam::get_v4l_framesizes(std::set<std::string> &framesizeList, const std::string format, const std::string dev)
  {
    char line[512];
    char cmd[256];
    if (dev.size() > 0)
    {
      sprintf(cmd, "v4l2-ctl --device=%s --list-framesizes %s", dev.c_str(), format.c_str());
    }
    else
    {
      sprintf(cmd, "v4l2-ctl --device=%s --list-framesizes %s", camera_dev_.c_str(), format.c_str());
    }
    FILE *fp = popen(cmd, "r");
    if (fp != nullptr)
    {
      framesizeList.clear();
      ROS_INFO("%s enum framesizes:", format.c_str());
      int i = 0;
      while (fgets(line, sizeof(line), fp) != nullptr)
      {
        if (i > 0)
        {
          char t1[64] = {0};
          char t2[64] = {0};
          char t3[64] = {0};
          sscanf(line, "%s%s%s", t1, t2, t3);
          std::string s = t3;
          framesizeList.insert(s);
          ROS_INFO("\t%s", t3);
        }
        i++;
      }
      pclose(fp);
      return framesizeList.size();
    }
    else
    {
      return -1;
    }
  }

  int UsbCam::get_v4l_frameintervals(std::set<int> &frameintervalList, const std::string format, int width, int height, const std::string dev)
  {
    char line[512];
    char cmd[256];
    if (dev.size() > 0)
    {
      sprintf(cmd, "v4l2-ctl --device=%s --list-frameintervals width=%d,height=%d,pixelformat=%s",
              dev.c_str(), width, height, format.c_str());
    }
    else
    {
      sprintf(cmd, "v4l2-ctl --device=%s --list-frameintervals width=%d,height=%d,pixelformat=%s",
              dev.c_str(), width, height, format.c_str());
    }
    FILE *fp = popen(cmd, "r");
    if (fp != nullptr)
    {
      frameintervalList.clear();
      ROS_INFO("%s(%dx%d) frame interval:", format.c_str(), width, height);
      int i = 0;
      while (fgets(line, sizeof(line), fp) != nullptr)
      {
        if (i > 0)
        {
          int t;
          sscanf(line, "%*[^(](%d", &t);
          frameintervalList.insert(t);
          ROS_INFO("\t%d", t);
        }
        i++;
      }
      pclose(fp);
      return frameintervalList.size();
    }
    else
    {
      return -1;
    }
  }

  UsbCam::io_method UsbCam::io_method_from_string(const std::string &str)
  {
    if (str == "mmap")
      return IO_METHOD_MMAP;
    else if (str == "read")
      return IO_METHOD_READ;
    else if (str == "userptr")
      return IO_METHOD_USERPTR;
    else if (str == "overlay")
      return IO_METHOD_OVERLAY;
    else if (str == "dma")
      return IO_METHOD_DMABUF;
    else
      return IO_METHOD_UNKNOWN;
  }

  v4l2_field UsbCam::v4l2_field_from_string(const std::string &str)
  {
    std::string strUpper = "";
    for (auto c : str)
    {
      strUpper += std::toupper(c);
    }
    if (strUpper == "NONE")
      return V4L2_FIELD_NONE;
    else if (strUpper == "TOP")
      return V4L2_FIELD_TOP;
    else if (strUpper == "BOTTOM")
      return V4L2_FIELD_BOTTOM;
    else if (strUpper == "oveINTERLACEDrlay")
      return V4L2_FIELD_INTERLACED;
    else if (strUpper == "SEQ_TB")
      return V4L2_FIELD_SEQ_TB;
    else if (strUpper == "SEQ_BT")
      return V4L2_FIELD_SEQ_BT;
    else if (strUpper == "ALTERNATE")
      return V4L2_FIELD_ALTERNATE;
    else if (strUpper == "INTERLACED_TB")
      return V4L2_FIELD_INTERLACED_TB;
    else if (strUpper == "INTERLACED_BT")
      return V4L2_FIELD_INTERLACED_BT;
    else
      return V4L2_FIELD_ANY;
  }

  AVPixelFormat UsbCam::ros_image_format_to_avpixel_format(std::string ros_image_format)
  {
    if (ros_image_format.compare(sensor_msgs::image_encodings::MONO8) == 0)
    {
      return AV_PIX_FMT_GRAY8;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::MONO16) == 0)
    {
      return AV_PIX_FMT_GRAY16;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::YUV422) == 0)
    {
      return AV_PIX_FMT_YUYV422;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::RGB8) == 0)
    {
      return AV_PIX_FMT_RGB24;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::RGBA8) == 0)
    {
      return AV_PIX_FMT_RGBA;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::RGB16) == 0)
    {
      return AV_PIX_FMT_RGB48;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::RGBA16) == 0)
    {
      return AV_PIX_FMT_RGBA64;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BGR8) == 0)
    {
      return AV_PIX_FMT_BGR24;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BGRA8) == 0)
    {
      return AV_PIX_FMT_BGRA;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BGR16) == 0)
    {
      return AV_PIX_FMT_BGR48;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BGRA16) == 0)
    {
      return AV_PIX_FMT_BGRA64;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BAYER_RGGB8) == 0)
    {
      return AV_PIX_FMT_BAYER_RGGB8;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BAYER_BGGR8) == 0)
    {
      return AV_PIX_FMT_BAYER_BGGR8;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BAYER_GBRG8) == 0)
    {
      return AV_PIX_FMT_BAYER_GBRG8;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BAYER_GRBG8) == 0)
    {
      return AV_PIX_FMT_BAYER_GRBG8;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BAYER_RGGB16) == 0)
    {
      return AV_PIX_FMT_BAYER_RGGB16;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BAYER_BGGR16) == 0)
    {
      return AV_PIX_FMT_BAYER_BGGR16;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BAYER_GBRG16) == 0)
    {
      return AV_PIX_FMT_BAYER_GBRG16;
    }
    else if (ros_image_format.compare(sensor_msgs::image_encodings::BAYER_GRBG16) == 0)
    {
      return AV_PIX_FMT_BAYER_GRBG16;
    }
    else
    {
      // TYPE_8UC1等opencv格式类型没有对应的
    }

    return AV_PIX_FMT_NONE;
  }

  AVPixelFormat UsbCam::v4l_format_to_avpixel_format(const unsigned int input_image_v4l_format)
  {
    switch (input_image_v4l_format)
    {
    /* RGB formats */
    case V4L2_PIX_FMT_RGB332:
      return AV_PIX_FMT_RGB8;
    case V4L2_PIX_FMT_RGB444:
      return AV_PIX_FMT_RGB444;
    case V4L2_PIX_FMT_ARGB444:
      break; // v4l2_fourcc('A', 'R', '1', '2') /* 16  aaaarrrr ggggbbbb */
    case V4L2_PIX_FMT_XRGB444:
      break; // v4l2_fourcc('X', 'R', '1', '2') /* 16  xxxxrrrr ggggbbbb */
    case V4L2_PIX_FMT_RGB555:
      return AV_PIX_FMT_RGB555;
    case V4L2_PIX_FMT_ARGB555:
      break; // v4l2_fourcc('A', 'R', '1', '5') /* 16  ARGB-1-5-5-5  */
    case V4L2_PIX_FMT_XRGB555:
      break; // v4l2_fourcc('X', 'R', '1', '5') /* 16  XRGB-1-5-5-5  */
    case V4L2_PIX_FMT_RGB565:
      return AV_PIX_FMT_RGB565;
    case V4L2_PIX_FMT_RGB555X:
      break; // v4l2_fourcc('R', 'G', 'B', 'Q') /* 16  RGB-5-5-5 BE  */
    case V4L2_PIX_FMT_ARGB555X:
      break; // v4l2_fourcc_be('A', 'R', '1', '5') /* 16  ARGB-5-5-5 BE */
    case V4L2_PIX_FMT_XRGB555X:
      break; // v4l2_fourcc_be('X', 'R', '1', '5') /* 16  XRGB-5-5-5 BE */
    case V4L2_PIX_FMT_RGB565X:
      break; // v4l2_fourcc('R', 'G', 'B', 'R') /* 16  RGB-5-6-5 BE  */
    case V4L2_PIX_FMT_BGR666:
      break; // v4l2_fourcc('B', 'G', 'R', 'H') /* 18  BGR-6-6-6	  */
    case V4L2_PIX_FMT_BGR24:
      return AV_PIX_FMT_BGR24;
    case V4L2_PIX_FMT_RGB24:
      return AV_PIX_FMT_RGB24;
    case V4L2_PIX_FMT_BGR32:
      return AV_PIX_FMT_BGR32;
    case V4L2_PIX_FMT_ABGR32:
      return AV_PIX_FMT_ABGR;
    case V4L2_PIX_FMT_XBGR32:
      return AV_PIX_FMT_0BGR;
    case V4L2_PIX_FMT_RGB32:
      return AV_PIX_FMT_RGB32;
    case V4L2_PIX_FMT_ARGB32:
      return AV_PIX_FMT_ARGB;
    case V4L2_PIX_FMT_XRGB32:
      return AV_PIX_FMT_0RGB;

    /* Grey formats */
    case V4L2_PIX_FMT_GREY:
      return AV_PIX_FMT_GRAY8;
    case V4L2_PIX_FMT_Y4:
      break; // v4l2_fourcc('Y', '0', '4', ' ') /*  4  Greyscale     */
    case V4L2_PIX_FMT_Y6:
      break; // v4l2_fourcc('Y', '0', '6', ' ') /*  6  Greyscale     */
    case V4L2_PIX_FMT_Y10:
      return AV_PIX_FMT_GRAY10;
    case V4L2_PIX_FMT_Y12:
      return AV_PIX_FMT_GRAY12;
    case V4L2_PIX_FMT_Y16:
      return AV_PIX_FMT_GRAY16;
    case V4L2_PIX_FMT_Y16_BE:
      return AV_PIX_FMT_GRAY16BE;

    /* Grey bit-packed formats */
    case V4L2_PIX_FMT_Y10BPACK:
      break; // v4l2_fourcc('Y', '1', '0', 'B') /* 10  Greyscale bit-packed */

    /* Palette formats */
    case V4L2_PIX_FMT_PAL8:
      return AV_PIX_FMT_PAL8;

    /* Chrominance formats */
    case V4L2_PIX_FMT_UV8:
      break; // v4l2_fourcc('U', 'V', '8', ' ') /*  8  UV 4:4 */

    /* Luminance+Chrominance formats */
    case V4L2_PIX_FMT_YUYV:
      return AV_PIX_FMT_YUYV422;
    case V4L2_PIX_FMT_YYUV:
      break; // v4l2_fourcc('Y', 'Y', 'U', 'V') /* 16  YUV 4:2:2     */
    case V4L2_PIX_FMT_YVYU:
      return AV_PIX_FMT_YVYU422;
    case V4L2_PIX_FMT_UYVY:
      return AV_PIX_FMT_UYVY422;
    case V4L2_PIX_FMT_VYUY:
      break; // v4l2_fourcc('V', 'Y', 'U', 'Y') /* 16  YUV 4:2:2     */
    case V4L2_PIX_FMT_Y41P:
      break; // v4l2_fourcc('Y', '4', '1', 'P') /* 12  YUV 4:1:1     */
    case V4L2_PIX_FMT_YUV444:
      break; // v4l2_fourcc('Y', '4', '4', '4') /* 16  xxxxyyyy uuuuvvvv */
    case V4L2_PIX_FMT_YUV555:
      break; // v4l2_fourcc('Y', 'U', 'V', 'O') /* 16  YUV-5-5-5     */
    case V4L2_PIX_FMT_YUV565:
      break; // v4l2_fourcc('Y', 'U', 'V', 'P') /* 16  YUV-5-6-5     */
    case V4L2_PIX_FMT_YUV32:
      break; // v4l2_fourcc('Y', 'U', 'V', '4') /* 32  YUV-8-8-8-8   */
    case V4L2_PIX_FMT_HI240:
      break; // v4l2_fourcc('H', 'I', '2', '4') /*  8  8-bit color   */
    case V4L2_PIX_FMT_HM12:
      break; // v4l2_fourcc('H', 'M', '1', '2') /*  8  YUV 4:2:0 16x16 macroblocks */
    case V4L2_PIX_FMT_M420:
      break; // v4l2_fourcc('M', '4', '2', '0') /* 12  YUV 4:2:0 2 lines y, 1 line uv interleaved */

    /* two planes -- one Y, one Cr + Cb interleaved  */
    case V4L2_PIX_FMT_NV12:
      break; // v4l2_fourcc('N', 'V', '1', '2') /* 12  Y/CbCr 4:2:0  */
    case V4L2_PIX_FMT_NV21:
      break; // v4l2_fourcc('N', 'V', '2', '1') /* 12  Y/CrCb 4:2:0  */
    case V4L2_PIX_FMT_NV16:
      break; // v4l2_fourcc('N', 'V', '1', '6') /* 16  Y/CbCr 4:2:2  */
    case V4L2_PIX_FMT_NV61:
      break; // v4l2_fourcc('N', 'V', '6', '1') /* 16  Y/CrCb 4:2:2  */
    case V4L2_PIX_FMT_NV24:
      break; // v4l2_fourcc('N', 'V', '2', '4') /* 24  Y/CbCr 4:4:4  */
    case V4L2_PIX_FMT_NV42:
      break; // v4l2_fourcc('N', 'V', '4', '2') /* 24  Y/CrCb 4:4:4  */

    /* two non contiguous planes - one Y, one Cr + Cb interleaved  */
    case V4L2_PIX_FMT_NV12M:
      break; // v4l2_fourcc('N', 'M', '1', '2') /* 12  Y/CbCr 4:2:0  */
    case V4L2_PIX_FMT_NV21M:
      break; // v4l2_fourcc('N', 'M', '2', '1') /* 21  Y/CrCb 4:2:0  */
    case V4L2_PIX_FMT_NV16M:
      break; // v4l2_fourcc('N', 'M', '1', '6') /* 16  Y/CbCr 4:2:2  */
    case V4L2_PIX_FMT_NV61M:
      break; // v4l2_fourcc('N', 'M', '6', '1') /* 16  Y/CrCb 4:2:2  */
    case V4L2_PIX_FMT_NV12MT:
      break; // v4l2_fourcc('T', 'M', '1', '2') /* 12  Y/CbCr 4:2:0 64x32 macroblocks */
    case V4L2_PIX_FMT_NV12MT_16X16:
      break; // v4l2_fourcc('V', 'M', '1', '2') /* 12  Y/CbCr 4:2:0 16x16 macroblocks */

    /* three planes - Y Cb, Cr */
    case V4L2_PIX_FMT_YUV410:
      break; // v4l2_fourcc('Y', 'U', 'V', '9') /*  9  YUV 4:1:0     */
    case V4L2_PIX_FMT_YVU410:
      break; // v4l2_fourcc('Y', 'V', 'U', '9') /*  9  YVU 4:1:0     */
    case V4L2_PIX_FMT_YUV411P:
      return AV_PIX_FMT_YUV411P;
    case V4L2_PIX_FMT_YUV420:
      break; // v4l2_fourcc('Y', 'U', '1', '2') /* 12  YUV 4:2:0     */
    case V4L2_PIX_FMT_YVU420:
      break; // v4l2_fourcc('Y', 'V', '1', '2') /* 12  YVU 4:2:0     */
    case V4L2_PIX_FMT_YUV422P:
      return AV_PIX_FMT_YUV422P;

    /* three non contiguous planes - Y, Cb, Cr */
    case V4L2_PIX_FMT_YUV420M:
      break; // v4l2_fourcc('Y', 'M', '1', '2') /* 12  YUV420 planar */
    case V4L2_PIX_FMT_YVU420M:
      break; // v4l2_fourcc('Y', 'M', '2', '1') /* 12  YVU420 planar */
    case V4L2_PIX_FMT_YUV422M:
      break; // v4l2_fourcc('Y', 'M', '1', '6') /* 16  YUV422 planar */
    case V4L2_PIX_FMT_YVU422M:
      break; // v4l2_fourcc('Y', 'M', '6', '1') /* 16  YVU422 planar */
    case V4L2_PIX_FMT_YUV444M:
      break; // v4l2_fourcc('Y', 'M', '2', '4') /* 24  YUV444 planar */
    case V4L2_PIX_FMT_YVU444M:
      break; // v4l2_fourcc('Y', 'M', '4', '2') /* 24  YVU444 planar */

    /* Bayer formats - see http://www.siliconimaging.com/RGB%20Bayer.htm */
    case V4L2_PIX_FMT_SBGGR8:
      break; // v4l2_fourcc('B', 'A', '8', '1') /*  8  BGBG.. GRGR.. */
    case V4L2_PIX_FMT_SGBRG8:
      break; // v4l2_fourcc('G', 'B', 'R', 'G') /*  8  GBGB.. RGRG.. */
    case V4L2_PIX_FMT_SGRBG8:
      break; // v4l2_fourcc('G', 'R', 'B', 'G') /*  8  GRGR.. BGBG.. */
    case V4L2_PIX_FMT_SRGGB8:
      break; // v4l2_fourcc('R', 'G', 'G', 'B') /*  8  RGRG.. GBGB.. */
    case V4L2_PIX_FMT_SBGGR10:
      break; // v4l2_fourcc('B', 'G', '1', '0') /* 10  BGBG.. GRGR.. */
    case V4L2_PIX_FMT_SGBRG10:
      break; // v4l2_fourcc('G', 'B', '1', '0') /* 10  GBGB.. RGRG.. */
    case V4L2_PIX_FMT_SGRBG10:
      break; // v4l2_fourcc('B', 'A', '1', '0') /* 10  GRGR.. BGBG.. */
    case V4L2_PIX_FMT_SRGGB10:
      break; // v4l2_fourcc('R', 'G', '1', '0') /* 10  RGRG.. GBGB.. */
      /* 10bit raw bayer packed, 5 bytes for every 4 pixels */
    case V4L2_PIX_FMT_SBGGR10P:
      break; // v4l2_fourcc('p', 'B', 'A', 'A')
    case V4L2_PIX_FMT_SGBRG10P:
      break; // v4l2_fourcc('p', 'G', 'A', 'A')
    case V4L2_PIX_FMT_SGRBG10P:
      break; // v4l2_fourcc('p', 'g', 'A', 'A')
    case V4L2_PIX_FMT_SRGGB10P:
      break; // v4l2_fourcc('p', 'R', 'A', 'A')
      /* 10bit raw bayer a-law compressed to 8 bits */
    case V4L2_PIX_FMT_SBGGR10ALAW8:
      break; // v4l2_fourcc('a', 'B', 'A', '8')
    case V4L2_PIX_FMT_SGBRG10ALAW8:
      break; // v4l2_fourcc('a', 'G', 'A', '8')
    case V4L2_PIX_FMT_SGRBG10ALAW8:
      break; // v4l2_fourcc('a', 'g', 'A', '8')
    case V4L2_PIX_FMT_SRGGB10ALAW8:
      break; // v4l2_fourcc('a', 'R', 'A', '8')
      /* 10bit raw bayer DPCM compressed to 8 bits */
    case V4L2_PIX_FMT_SBGGR10DPCM8:
      break; // v4l2_fourcc('b', 'B', 'A', '8')
    case V4L2_PIX_FMT_SGBRG10DPCM8:
      break; // v4l2_fourcc('b', 'G', 'A', '8')
    case V4L2_PIX_FMT_SGRBG10DPCM8:
      break; // v4l2_fourcc('B', 'D', '1', '0')
    case V4L2_PIX_FMT_SRGGB10DPCM8:
      break; // v4l2_fourcc('b', 'R', 'A', '8')
    case V4L2_PIX_FMT_SBGGR12:
      break; // v4l2_fourcc('B', 'G', '1', '2') /* 12  BGBG.. GRGR.. */
    case V4L2_PIX_FMT_SGBRG12:
      break; // v4l2_fourcc('G', 'B', '1', '2') /* 12  GBGB.. RGRG.. */
    case V4L2_PIX_FMT_SGRBG12:
      break; // v4l2_fourcc('B', 'A', '1', '2') /* 12  GRGR.. BGBG.. */
    case V4L2_PIX_FMT_SRGGB12:
      break; // v4l2_fourcc('R', 'G', '1', '2') /* 12  RGRG.. GBGB.. */
      /* 12bit raw bayer packed, 6 bytes for every 4 pixels */
    case V4L2_PIX_FMT_SBGGR12P:
      break; // v4l2_fourcc('p', 'B', 'C', 'C')
    case V4L2_PIX_FMT_SGBRG12P:
      break; // v4l2_fourcc('p', 'G', 'C', 'C')
    case V4L2_PIX_FMT_SGRBG12P:
      break; // v4l2_fourcc('p', 'g', 'C', 'C')
    case V4L2_PIX_FMT_SRGGB12P:
      break; // v4l2_fourcc('p', 'R', 'C', 'C')
    case V4L2_PIX_FMT_SBGGR16:
      break; // v4l2_fourcc('B', 'Y', 'R', '2') /* 16  BGBG.. GRGR.. */
    case V4L2_PIX_FMT_SGBRG16:
      break; // v4l2_fourcc('G', 'B', '1', '6') /* 16  GBGB.. RGRG.. */
    case V4L2_PIX_FMT_SGRBG16:
      break; // v4l2_fourcc('G', 'R', '1', '6') /* 16  GRGR.. BGBG.. */
    case V4L2_PIX_FMT_SRGGB16:
      break; // v4l2_fourcc('R', 'G', '1', '6') /* 16  RGRG.. GBGB.. */

    /* HSV formats */
    case V4L2_PIX_FMT_HSV24:
      break; // v4l2_fourcc('H', 'S', 'V', '3')
    case V4L2_PIX_FMT_HSV32:
      break; // v4l2_fourcc('H', 'S', 'V', '4')

    /* compressed formats */
    case V4L2_PIX_FMT_MJPEG:
      return AV_PIX_FMT_YUV422P; //
    case V4L2_PIX_FMT_JPEG:
      break; // v4l2_fourcc('J', 'P', 'E', 'G') /* JFIF JPEG     */
    case V4L2_PIX_FMT_DV:
      break; // v4l2_fourcc('d', 'v', 's', 'd') /* 1394          */
    case V4L2_PIX_FMT_MPEG:
      break; // v4l2_fourcc('M', 'P', 'E', 'G') /* MPEG-1/2/4 Multiplexed */
    case V4L2_PIX_FMT_H264_NO_SC:
      break; // v4l2_fourcc('A', 'V', 'C', '1') /* H264 without start codes */
    case V4L2_PIX_FMT_H264_MVC:
      break; // v4l2_fourcc('M', '2', '6', '4') /* H264 MVC */
    case V4L2_PIX_FMT_H263:
      break; // v4l2_fourcc('H', '2', '6', '3') /* H263          */
#if FF_API_VDPAU
    case V4L2_PIX_FMT_H264:
      return AV_PIX_FMT_VDPAU_H264;
    case V4L2_PIX_FMT_MPEG1:
      return AV_PIX_FMT_VDPAU_MPEG1;
    case V4L2_PIX_FMT_MPEG2:
      return AV_PIX_FMT_VDPAU_MPEG2;
    case V4L2_PIX_FMT_MPEG4:
      return AV_PIX_FMT_VDPAU_MPEG4;
#endif
    case V4L2_PIX_FMT_XVID:
      break; // v4l2_fourcc('X', 'V', 'I', 'D') /* Xvid           */
    case V4L2_PIX_FMT_VC1_ANNEX_G:
      break; // v4l2_fourcc('V', 'C', '1', 'G') /* SMPTE 421M Annex G compliant stream */
    case V4L2_PIX_FMT_VC1_ANNEX_L:
      break; // v4l2_fourcc('V', 'C', '1', 'L') /* SMPTE 421M Annex L compliant stream */
    case V4L2_PIX_FMT_VP8:
      break; // v4l2_fourcc('V', 'P', '8', '0') /* VP8 */
    case V4L2_PIX_FMT_VP9:
      break; // v4l2_fourcc('V', 'P', '9', '0') /* VP9 */
    case V4L2_PIX_FMT_HEVC:
      break; // v4l2_fourcc('H', 'E', 'V', 'C') /* HEVC aka H.265 */

    /*  Vendor-specific formats   */
    case V4L2_PIX_FMT_CPIA1:
      break; // v4l2_fourcc('C', 'P', 'I', 'A') /* cpia1 YUV */
    case V4L2_PIX_FMT_WNVA:
      break; // v4l2_fourcc('W', 'N', 'V', 'A') /* Winnov hw compress */
    case V4L2_PIX_FMT_SN9C10X:
      break; // v4l2_fourcc('S', '9', '1', '0') /* SN9C10x compression */
    case V4L2_PIX_FMT_SN9C20X_I420:
      break; // v4l2_fourcc('S', '9', '2', '0') /* SN9C20x YUV 4:2:0 */
    case V4L2_PIX_FMT_PWC1:
      break; // v4l2_fourcc('P', 'W', 'C', '1') /* pwc older webcam */
    case V4L2_PIX_FMT_PWC2:
      break; // v4l2_fourcc('P', 'W', 'C', '2') /* pwc newer webcam */
    case V4L2_PIX_FMT_ET61X251:
      break; // v4l2_fourcc('E', '6', '2', '5') /* ET61X251 compression */
    case V4L2_PIX_FMT_SPCA501:
      break; // v4l2_fourcc('S', '5', '0', '1') /* YUYV per line */
    case V4L2_PIX_FMT_SPCA505:
      break; // v4l2_fourcc('S', '5', '0', '5') /* YYUV per line */
    case V4L2_PIX_FMT_SPCA508:
      break; // v4l2_fourcc('S', '5', '0', '8') /* YUVY per line */
    case V4L2_PIX_FMT_SPCA561:
      break; // v4l2_fourcc('S', '5', '6', '1') /* compressed GBRG bayer */
    case V4L2_PIX_FMT_PAC207:
      break; // v4l2_fourcc('P', '2', '0', '7') /* compressed BGGR bayer */
    case V4L2_PIX_FMT_MR97310A:
      break; // v4l2_fourcc('M', '3', '1', '0') /* compressed BGGR bayer */
    case V4L2_PIX_FMT_JL2005BCD:
      break; // v4l2_fourcc('J', 'L', '2', '0') /* compressed RGGB bayer */
    case V4L2_PIX_FMT_SN9C2028:
      break; // v4l2_fourcc('S', 'O', 'N', 'X') /* compressed GBRG bayer */
    case V4L2_PIX_FMT_SQ905C:
      break; // v4l2_fourcc('9', '0', '5', 'C') /* compressed RGGB bayer */
    case V4L2_PIX_FMT_PJPG:
      break; // v4l2_fourcc('P', 'J', 'P', 'G') /* Pixart 73xx JPEG */
    case V4L2_PIX_FMT_OV511:
      break; // v4l2_fourcc('O', '5', '1', '1') /* ov511 JPEG */
    case V4L2_PIX_FMT_OV518:
      break; // v4l2_fourcc('O', '5', '1', '8') /* ov518 JPEG */
    case V4L2_PIX_FMT_STV0680:
      break; // v4l2_fourcc('S', '6', '8', '0') /* stv0680 bayer */
    case V4L2_PIX_FMT_TM6000:
      break; // v4l2_fourcc('T', 'M', '6', '0') /* tm5600/tm60x0 */
    case V4L2_PIX_FMT_CIT_YYVYUY:
      break; // v4l2_fourcc('C', 'I', 'T', 'V') /* one line of Y then 1 line of VYUY */
    case V4L2_PIX_FMT_KONICA420:
      break; // v4l2_fourcc('K', 'O', 'N', 'I') /* YUV420 planar in blocks of 256 pixels */
    case V4L2_PIX_FMT_JPGL:
      break; // v4l2_fourcc('J', 'P', 'G', 'L') /* JPEG-Lite */
    case V4L2_PIX_FMT_SE401:
      break; // v4l2_fourcc('S', '4', '0', '1') /* se401 janggu compressed rgb */
    case V4L2_PIX_FMT_S5C_UYVY_JPG:
      break; // v4l2_fourcc('S', '5', 'C', 'I') /* S5C73M3 interleaved UYVY/JPEG */
    case V4L2_PIX_FMT_Y8I:
      break; // v4l2_fourcc('Y', '8', 'I', ' ') /* Greyscale 8-bit L/R interleaved */
    case V4L2_PIX_FMT_Y12I:
      break; // v4l2_fourcc('Y', '1', '2', 'I') /* Greyscale 12-bit L/R interleaved */
    case V4L2_PIX_FMT_Z16:
      break; // v4l2_fourcc('Z', '1', '6', ' ') /* Depth data 16-bit */
    case V4L2_PIX_FMT_MT21C:
      break; // v4l2_fourcc('M', 'T', '2', '1') /* Mediatek compressed block mode  */
    case V4L2_PIX_FMT_INZI:
      break; // v4l2_fourcc('I', 'N', 'Z', 'I') /* Intel Planar Greyscale 10-bit and Depth 16-bit */

    /* SDR formats - used only for Software Defined Radio devices */
    case V4L2_SDR_FMT_CU8:
      break; // v4l2_fourcc('C', 'U', '0', '8') /* IQ u8 */
    case V4L2_SDR_FMT_CU16LE:
      break; // v4l2_fourcc('C', 'U', '1', '6') /* IQ u16le */
    case V4L2_SDR_FMT_CS8:
      break; // v4l2_fourcc('C', 'S', '0', '8') /* complex s8 */
    case V4L2_SDR_FMT_CS14LE:
      break; // v4l2_fourcc('C', 'S', '1', '4') /* complex s14le */
    case V4L2_SDR_FMT_RU12LE:
      break; // v4l2_fourcc('R', 'U', '1', '2') /* real u12le */
    case V4L2_SDR_FMT_PCU16BE:
      break; // v4l2_fourcc('P', 'C', '1', '6') /* planar complex u16be */
    case V4L2_SDR_FMT_PCU18BE:
      break; // v4l2_fourcc('P', 'C', '1', '8') /* planar complex u18be */
    case V4L2_SDR_FMT_PCU20BE:
      break; // v4l2_fourcc('P', 'C', '2', '0') /* planar complex u20be */

    /* Touch formats - used for Touch devices */
    case V4L2_TCH_FMT_DELTA_TD16:
      break; // v4l2_fourcc('T', 'D', '1', '6') /* 16-bit signed deltas */
    case V4L2_TCH_FMT_DELTA_TD08:
      break; // v4l2_fourcc('T', 'D', '0', '8') /* 8-bit signed deltas */
    case V4L2_TCH_FMT_TU16:
      break; // v4l2_fourcc('T', 'U', '1', '6') /* 16-bit unsigned touch data */
    case V4L2_TCH_FMT_TU08:
      break; // v4l2_fourcc('T', 'U', '0', '8') /* 8-bit unsigned touch data */

    /* Meta-data formats */
    case V4L2_META_FMT_VSP1_HGO:
      break; // v4l2_fourcc('V', 'S', 'P', 'H') /* R-Car VSP1 1-D Histogram */
    case V4L2_META_FMT_VSP1_HGT:
      break; // v4l2_fourcc('V', 'S', 'P', 'T') /* R-Car VSP1 2-D Histogram */
    }
    return AV_PIX_FMT_NONE;
  }

  void UsbCam::CfgCB_NormalCamera(NormalCameraConfig &config, uint32_t level)
  {
    // set camera parameters
    if (config.brightness >= 0 && ctrlParaList.find("brightness") != ctrlParaList.end())
    {
      set_v4l_parameter("brightness", config.brightness, camera_dev_);
    }

    if (config.contrast >= 0 && ctrlParaList.find("contrast") != ctrlParaList.end())
    {
      set_v4l_parameter("contrast", config.contrast, camera_dev_);
    }

    if (config.saturation >= 0 && ctrlParaList.find("saturation") != ctrlParaList.end())
    {
      set_v4l_parameter("saturation", config.saturation, camera_dev_);
    }

    if (config.sharpness >= 0 && ctrlParaList.find("sharpness") != ctrlParaList.end())
    {
      set_v4l_parameter("sharpness", config.sharpness, camera_dev_);
    }

    if (config.gain >= 0 && ctrlParaList.find("gain") != ctrlParaList.end())
    {
      set_v4l_parameter("gain", config.gain, camera_dev_);
    }

    if (config.gain >= 0 && ctrlParaList.find("gamma") != ctrlParaList.end())
    {
      set_v4l_parameter("gamma", config.gamma, camera_dev_);
    }

    if (config.gain >= 0 && ctrlParaList.find("hue") != ctrlParaList.end())
    {
      set_v4l_parameter("hue", config.hue, camera_dev_);
    }

    if (config.gain >= 0 && ctrlParaList.find("backlight_compensation") != ctrlParaList.end())
    {
      set_v4l_parameter("backlight_compensation", config.backlight_compensation, camera_dev_);
    }

    if (config.gain >= 0 && ctrlParaList.find("pan_absolute") != ctrlParaList.end())
    {
      set_v4l_parameter("pan_absolute", config.pan_absolute, camera_dev_);
    }

    if (config.gain >= 0 && ctrlParaList.find("tilt_absolute") != ctrlParaList.end())
    {
      set_v4l_parameter("tilt_absolute", config.tilt_absolute, camera_dev_);
    }

    if (config.gain >= 0 && ctrlParaList.find("zoom_absolute") != ctrlParaList.end())
    {
      set_v4l_parameter("zoom_absolute", config.zoom_absolute, camera_dev_);
    }

    // check auto white balance
    if (config.white_balance_temperature_auto)
    {
      if (ctrlParaList.find("white_balance_temperature_auto") != ctrlParaList.end())
      {
        set_v4l_parameter("white_balance_temperature_auto", 1, camera_dev_);
      }
      else if (ctrlParaList.find("white_balance_automatic") != ctrlParaList.end())
      {
        set_v4l_parameter("white_balance_automatic", 1, camera_dev_);
      }
    }
    else
    {
      if (ctrlParaList.find("white_balance_temperature_auto") != ctrlParaList.end())
      {
        set_v4l_parameter("white_balance_temperature_auto", 0, camera_dev_);
      }
      else if (ctrlParaList.find("white_balance_automatic") != ctrlParaList.end())
      {
        set_v4l_parameter("white_balance_automatic", 0, camera_dev_);
      }
      if (ctrlParaList.find("white_balance_temperature") != ctrlParaList.end())
      {
        set_v4l_parameter("white_balance_temperature", config.white_balance_temperature, camera_dev_);
      }
    }

    // check auto exposure
    if (config.exposure_auto_priority)
    {
      if (ctrlParaList.find("exposure_auto_priority") != ctrlParaList.end())
      {
        set_v4l_parameter("exposure_auto_priority", 1, camera_dev_);
      }
      else if (ctrlParaList.find("exposure_auto") != ctrlParaList.end())
      {
        set_v4l_parameter("exposure_auto", 3, camera_dev_);
      }
      else if (ctrlParaList.find("auto_exposure") != ctrlParaList.end())
      {
        set_v4l_parameter("auto_exposure", 3, camera_dev_);
      }
    }
    else
    {
      if (ctrlParaList.find("exposure_auto_priority") != ctrlParaList.end())
      {
        set_v4l_parameter("exposure_auto_priority", 0, camera_dev_);
      }
      else if (ctrlParaList.find("exposure_auto") != ctrlParaList.end())
      {
        set_v4l_parameter("exposure_auto", 1, camera_dev_);
      }
      else if (ctrlParaList.find("auto_exposure") != ctrlParaList.end())
      {
        set_v4l_parameter("auto_exposure", 1, camera_dev_);
      }
      if (ctrlParaList.find("exposure_absolute") != ctrlParaList.end())
      {
        set_v4l_parameter("exposure_absolute", config.exposure_absolute, camera_dev_);
      }
      else if (ctrlParaList.find("exposure_time_absolute") != ctrlParaList.end())
      {
        set_v4l_parameter("exposure_time_absolute", config.exposure_absolute, camera_dev_);
      }
    }

    // check auto focus
    if (config.focus_auto)
    {
      set_auto_focus(1);
      if (ctrlParaList.find("focus_auto") != ctrlParaList.end())
      {
        set_v4l_parameter("focus_auto", 1, camera_dev_);
      }
    }
    else
    {
      if (ctrlParaList.find("focus_auto") != ctrlParaList.end())
      {
        set_v4l_parameter("focus_auto", 0, camera_dev_);
      }
      if (ctrlParaList.find("focus_absolute") != ctrlParaList.end())
      {
        set_v4l_parameter("focus_absolute", config.focus_absolute, camera_dev_);
      }
    }
  }

  int UsbCam::init_isp_dwe()
  {
    if (app_dwe != nullptr)
    {
      delete app_dwe;
      app_dwe = nullptr;
    }
    if (app_isp != nullptr)
    {
      delete app_isp;
      app_isp = nullptr;
    }
    if ((enable_isp_ && ispFileName_.empty()) ||
        (enable_dwe_ && dweFileName_.empty()))
    {
      char cmd[256] = "ls /dev/v4l-subdev*";
      char line[512];
      std::set<int> indexs;
      FILE *fp = popen(cmd, "r");
      if (fp)
      {
        while (fgets(line, sizeof(line), fp) != nullptr)
        {
          int index;
          sscanf(line, "/dev/v4l-subdev%d", &index);
          indexs.insert(index);
        }
        pclose(fp);
      }
      for (auto i : indexs)
      {
        sprintf(cmd, "cat /sys/class/video4linux/v4l-subdev%d/name", i);
        fp = popen(cmd, "r");
        if (fp)
        {
          while (fgets(line, sizeof(line), fp) != nullptr)
          {
            if (strstr(line, "isp") != nullptr)
            {
              if (enable_isp_ && ispFileName_.empty())
              {
                ispFileName_ = "/dev/v4l-subdev" + std::to_string(i);
              }
            }
            else if (strstr(line, "dwe") != nullptr)
            {
              if (enable_dwe_ && dweFileName_.empty())
              {
                dweFileName_ = "/dev/v4l-subdev" + std::to_string(i);
              }
            }
          }
          pclose(fp);
        }
      }
    }
    if (enable_isp_ && !ispFileName_.empty())
    {
      app_isp = new ISP_API(ispFileName_);
      if (app_isp)
      {
        int ret = -1;
        // 复位模块
        if ((ret = app_isp->Reset()) < 0)
        {
          ROS_ERROR("ISP Reset %d", ret);
          return ret;
        }
        // 获取模块功能
        if ((ret = app_isp->IOC_Get_Feature(&app_isp->isp_feature)) < 0)
        {
          ROS_ERROR("ISP IOC_Get_Feature %d", ret);
          return ret;
        }

        if (app_isp->isp_feature & ISP_EE_SUPPORT)
        {
          struct isp_ee_context ee_ctx;
          memset(&ee_ctx, 0, sizeof(struct isp_ee_context));
          ee_ctx.enable = false;
          // ee_ctx.src_strength = 0;
          // ee_ctx.strength = 0;
          // ee_ctx.input_sel = 0;
          // ee_ctx.y_gain = 0;
          // ee_ctx.uv_gain = 0;
          // ee_ctx.edge_gain = 0;
          if ((ret = app_isp->Set_EE(&ee_ctx)) < 0)
          {
            ROS_ERROR("ISP Set_EE %d", ret);
            return ret;
          }
        }
        if (app_isp->isp_feature & ISP_2DNR_SUPPORT)
        {
        }
        if (app_isp->isp_feature & ISP_3DNR_SUPPORT)
        {
        }
        if (app_isp->isp_feature & ISP_WDR3_SUPPORT)
        {
        }
        if (app_isp->isp_feature & ISP_MIV2_SUPPORT)
        {
        }
        if (app_isp->isp_feature & ISP_AEV2_SUPPORT)
        {
        }
        if (app_isp->isp_feature & ISP_COMPAND_SUPPORT)
        {
        }
        if (app_isp->isp_feature & ISP_HDR_STITCH_SUPPORT)
        {
        }

        struct isp_context isp_ctx;
        memset(&isp_ctx, 0, sizeof(struct isp_context));
        isp_ctx.mode = MRV_ISP_ISP_MODE_RAW;
        // isp_ctx.sample_edge;
        // isp_ctx.hSyncLowPolarity;
        // isp_ctx.vSyncLowPolarity;
        isp_ctx.bayer_pattern = MRV_ISP_BAYER_PAT_RG;
        isp_ctx.sub_sampling = MRV_ISP_CONV_422_NONCO;
        isp_ctx.seq_ccir = MRV_ISP_CCIR_SEQ_YCBYCR;
        isp_ctx.field_selection = MRV_ISP_FIELD_SELECTION_BOTH; // MRV_ISP_FIELD_SELECTION_EVEN
        isp_ctx.input_selection = MRV_ISP_INPUT_SELECTION_12EXT;
        isp_ctx.latency_fifo = MRV_ISP_LATENCY_FIFO_SELECTION_INPUT_FORMATTER;
        // isp_ctx.acqWindow.x;
        // isp_ctx.acqWindow.y;
        isp_ctx.acqWindow.width = image_width_;
        isp_ctx.acqWindow.height = image_height_;
        // isp_ctx.ofWindow.x;
        // isp_ctx.ofWindow.y;
        isp_ctx.ofWindow.width = image_width_;
        isp_ctx.ofWindow.height = image_height_;
        // isp_ctx.isWindow.x;
        // isp_ctx.isWindow.y;
        isp_ctx.isWindow.width = image_width_;
        isp_ctx.isWindow.height = image_height_;
        // isp_ctx.stitching_mode;
        if ((ret = app_isp->Set_Input(&isp_ctx)) < 0)
        {
          ROS_ERROR("ISP Set_Input %d", ret);
          return ret;
        }
        isp_ctx.bypass_mode = MRV_ISP_DEMOSAIC_MODE_STD;
        isp_ctx.demosaic_threshold = MRV_ISP_DEMOSAIC_TH_MAX_TEXTURE_DETECTION;
        if ((ret = app_isp->Set_Demosaic(&isp_ctx)) < 0)
        {
          ROS_ERROR("ISP Set_Demosaic %d", ret);
          return ret;
        }
        // ret = app_isp->Set_Buffer(struct isp_buffer_context * context);
        // ret = app_isp->Set_Bp_Buffer(struct isp_bp_buffer_context * context);

        // struct isp_mcm_context mcm_ctx;
        // memset(&mcm_ctx, 0, sizeof(struct isp_mcm_context));
        // mcm_ctx.bypass_enable = true;
        // mcm_ctx.vsync_blank;
        // mcm_ctx.vsync_duration = image_width_;
        // mcm_ctx.hsync_blank;
        // mcm_ctx.hsync_preample;
        // if ((ret = Set_MCM(&mcm_ctx)) < 0)
        // {
        //   ROS_ERROR("ISP Set_MCM %d", ret);
        //   return ret;
        // }

        // 初始化输出格式(放在image stablilizer 初始化后)
        struct isp_mi_context mi_ctx;
        memset(&mi_ctx, 0, sizeof(struct isp_mi_context));
        mi_ctx.burst_len = MRV_MI_BURST_LEN_CHROM_4;
        mi_ctx.path[IC_MI_PATH_MAIN].enable = true;
        mi_ctx.path[IC_MI_PATH_MAIN].out_mode = IC_MI_DATAMODE_YUV422;
        mi_ctx.path[IC_MI_PATH_MAIN].in_mode = IC_MI_DATAMODE_RAW8;
        mi_ctx.path[IC_MI_PATH_MAIN].data_layout = IC_MI_DATASTORAGE_INTERLEAVED;
        mi_ctx.path[IC_MI_PATH_MAIN].data_alignMode = ISP_MI_DATA_UNALIGN_MODE;
        mi_ctx.path[IC_MI_PATH_MAIN].in_width = image_width_;
        mi_ctx.path[IC_MI_PATH_MAIN].in_height = image_height_;
        mi_ctx.path[IC_MI_PATH_MAIN].out_width = image_width_;
        mi_ctx.path[IC_MI_PATH_MAIN].out_height = image_height_;
        mi_ctx.path[IC_MI_PATH_MAIN].hscale = 0;
        mi_ctx.path[IC_MI_PATH_MAIN].vscale = 0;
        mi_ctx.path[IC_MI_PATH_MAIN].pixelformat = input_image_v4l_format_;
        if ((ret = app_isp->MI_Start(&mi_ctx)) < 0)
        {
          ROS_ERROR("ISP MI_Start %d", ret);
          return ret;
        }

        if ((ret = app_isp->Enable()) < 0)
        {
          ROS_ERROR("ISP Enable %d", ret);
          return ret;
        }
        bool enable = false;
        if ((ret = app_isp->Is_Enable(&enable)) < 0 || !enable)
        {
          ROS_ERROR("ISP Is_Enable %d, %d", ret, enable);
          return ret;
        }
        if ((ret = app_isp->Start_Stream(&framerate_)) < 0)
        {
          ROS_ERROR("ISP Start_Stream %d", ret);
          return ret;
        }
      }
    }
    if (enable_dwe_ && !dweFileName_.empty())
    {
      app_dwe = new DWE_API(dweFileName_);
      if (app_dwe)
      {
      }
    }
    return 0;
  }

} // namespace usb_cam
