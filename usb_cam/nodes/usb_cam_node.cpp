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

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <sensor_msgs/fill_image.h>
#include "dynamic_reconfigure/server.h"
#include "dvp2usb/dvp2usb.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace usb_cam
{
    class UsbCamNode
    {
    public:
        // private ROS node handle
        ros::NodeHandle node_;

        // shared image message
        sensor_msgs::Image img_;
        image_transport::CameraPublisher image_pub_;
        // image_transport::CameraPublisher image_pub_left_,image_pub_right_;
        ros::Publisher image_pub_left_, image_pub_right_;
        ros::Publisher jpeg_pub_;
        ros::Subscriber sub_imu_;

        // parameters
        std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_, publish_image_format_;
        // std::string start_service_name_, start_service_name_;
        bool streaming_status_, bSubscriberTopic_, bStereo_;
        int image_width_, image_height_, framerate_;
        double time_delay_per_byte_, image_valid_time_, framerate_tb_;
        bool process_image_in_other_thread_ = false;
        bool enable_imx_isp_ = false, enable_imx_dewarp_ = false;
        boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
        std::string MJPEG_Topic_;
        int bPUB_MJPEG_;

        dynamic_reconfigure::Server<NormalCameraConfig> *pDynamicConfigServer_NormalCamera = nullptr;
        dynamic_reconfigure::Server<NormalCameraConfig>::CallbackType cbDynamicConfig_NormalCamera;
        NormalCameraConfig CameraDefaultConfig, CameraMinConfig, CameraMaxConfig;

        bool isDvpUSBCam = false;
        UsbCam cam_;
        DVP2USB_CAMERA *DvpUSB_Cam = nullptr;
        ros::ServiceServer service_start_, service_stop_;

        void mjpeg_callback(const csjw_msgs::mjpegPtr &jpeg)
        {
            cam_.mjpeg2mono8((char *)&jpeg->data[0], jpeg->size, cam_.image_->image, jpeg->height * jpeg->width);
            // stamp the image
            img_.header.stamp = jpeg->header.stamp;
            // fill the info
            int chan = sensor_msgs::image_encodings::numChannels(publish_image_format_);
            if (chan > 0)
            {
                fillImage(img_, publish_image_format_, jpeg->height, jpeg->width, chan * jpeg->width,
                          cam_.image_->image);
            }

            if (bStereo_)
            {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_, publish_image_format_);
                cv::Mat image = cv_ptr->image;

                cv::Mat leftImg = image(cv::Rect(0, 0, img_.width >> 1, img_.height)).clone();
                cv::Mat rightImg = image(cv::Rect(img_.width >> 1, 0, img_.width >> 1, img_.height)).clone();

                sensor_msgs::ImagePtr img_left_ptr = cv_bridge::CvImage(std_msgs::Header(), publish_image_format_, leftImg).toImageMsg();
                sensor_msgs::ImagePtr img_right_ptr = cv_bridge::CvImage(std_msgs::Header(), publish_image_format_, rightImg).toImageMsg();
                img_left_ptr->header.stamp = img_.header.stamp;
                img_right_ptr->header.stamp = img_.header.stamp;

                image_pub_left_.publish(*img_left_ptr);
                image_pub_right_.publish(*img_right_ptr);
            }
            else
            {
                // grab the camera info
                sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
                ci->header.frame_id = jpeg->header.frame_id;
                ci->header.stamp = jpeg->header.stamp;

                // publish the image
                image_pub_.publish(img_, *ci);
            }
        }

        bool service_start_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
        {
            if (isDvpUSBCam)
            {
                if (DvpUSB_Cam && DvpUSB_Cam->start_capturing() >= 0)
                {
                    return true;
                }
            }
            else
            {
                cam_.start_capturing();
                return true;
            }
            return false;
        }

        bool service_stop_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
        {
            if (isDvpUSBCam)
            {
                if (DvpUSB_Cam && DvpUSB_Cam->stop_capturing() >= 0)
                {
                    return true;
                }
            }
            else
            {
                cam_.stop_capturing();
                return true;
            }
            return false;
        }

        UsbCamNode() : node_("~")
        {
            // advertise the main image topic
            image_transport::ImageTransport it(node_);

#if (defined __arm__) || (defined __aarch64__)
            node_.param("enable_imx_isp", enable_imx_isp_, false);
            node_.param("enable_imx_dewarp", enable_imx_dewarp_, false);
#endif
            if (!enable_imx_isp_)
            {
                if (pDynamicConfigServer_NormalCamera == nullptr)
                {
                    pDynamicConfigServer_NormalCamera = new dynamic_reconfigure::Server<NormalCameraConfig>(node_);
                    pDynamicConfigServer_NormalCamera->getConfigDefault(CameraDefaultConfig);
                    pDynamicConfigServer_NormalCamera->getConfigMin(CameraMinConfig);
                    pDynamicConfigServer_NormalCamera->getConfigMax(CameraMaxConfig);
                }
            }
            // grab the parameters
            node_.param("video_device", video_device_name_, std::string("/dev/video0"));
            node_.param("bStereo", bStereo_, false);

            node_.param("brightness", CameraDefaultConfig.brightness, 128);
            node_.param("contrast", CameraDefaultConfig.contrast, 32);
            node_.param("saturation", CameraDefaultConfig.saturation, 0);
            node_.param("sharpness", CameraDefaultConfig.sharpness, 0);
            // possible values: mmap, read, userptr
            node_.param("io_method", io_method_name_, std::string("mmap"));
            node_.param("image_width", image_width_, 640);
            node_.param("image_height", image_height_, 480);
            node_.param("framerate", framerate_, 60);
            node_.param("framerate_tb", framerate_tb_, 0.0);
            node_.param("time_delay_per_byte", time_delay_per_byte_, 53.865640777582844);
            // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
            node_.param("v4l_pixel_format", pixel_format_name_, std::string("MJPG"));
            node_.param("publish_image_format", publish_image_format_, std::string("mono8"));
            // enable/disable autofocus
            node_.param("focus_auto", CameraDefaultConfig.focus_auto, false);
            node_.param("focus_absolute", CameraDefaultConfig.focus_absolute, 68);
            // enable/disable autoexposure
            if (!node_.param("exposure_auto_priority", CameraDefaultConfig.exposure_auto_priority, true))
            {
                int temp_exposure_auto;
                if (!node_.param("exposure_auto", temp_exposure_auto, 3))
                {
                    node_.param("auto_exposure", temp_exposure_auto, 3);
                }
                CameraDefaultConfig.exposure_auto_priority = (temp_exposure_auto == 3);
            }
            if (!node_.param("exposure_absolute", CameraDefaultConfig.exposure_absolute, 10000))
            {
                node_.param("exposure_time_absolute", CameraDefaultConfig.exposure_absolute, 10000);
            }
            // enable/disable auto white balance temperature
            if (!node_.param("white_balance_temperature_auto", CameraDefaultConfig.white_balance_temperature_auto, true))
            {
                node_.param("white_balance_automatic", CameraDefaultConfig.white_balance_temperature_auto, true);
            }
            node_.param("white_balance_temperature", CameraDefaultConfig.white_balance_temperature, 4600);
            node_.param("gain", CameraDefaultConfig.gain, 120);
            node_.param("gamma", CameraDefaultConfig.gamma, 120);
            node_.param("hue", CameraDefaultConfig.hue, 0);
            node_.param("backlight_compensation", CameraDefaultConfig.backlight_compensation, 1);
            node_.param("pan_absolute", CameraDefaultConfig.pan_absolute, 0);
            node_.param("tilt_absolute", CameraDefaultConfig.tilt_absolute, 0);
            node_.param("zoom_absolute", CameraDefaultConfig.zoom_absolute, 0);
            node_.param("process_image_in_other_thread", process_image_in_other_thread_, true);
            node_.param("image_valid_time", image_valid_time_, 0.1);

            // load the camera info
            node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
            node_.param("camera_name", camera_name_, std::string("head_camera"));
            node_.param("camera_info_url", camera_info_url_, std::string(""));

            node_.param("bSubscriberTopic", bSubscriberTopic_, false);
            node_.param("MJPEG_Topic", MJPEG_Topic_, std::string("/usb_cam/image_mjpeg"));
            node_.param("bPUB_MJPEG", bPUB_MJPEG_, 0);
            if (bPUB_MJPEG_ > 0 && !bSubscriberTopic_)
            {
                jpeg_pub_ = node_.advertise<csjw_msgs::mjpeg>(MJPEG_Topic_, 120);
                process_image_in_other_thread_ = true;
            }
            if (bPUB_MJPEG_ != 1 || bSubscriberTopic_)
            {
                if (bStereo_)
                {
                    image_pub_left_ = node_.advertise<sensor_msgs::Image>("image_raw_left", 120);
                    image_pub_right_ = node_.advertise<sensor_msgs::Image>("image_raw_right", 120);
                }
                else
                {
                    image_pub_ = it.advertiseCamera("image_raw", 1);
                }
            }

            cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));
            // check for default camera info
            if (!cinfo_->isCalibrated())
            {
                cinfo_->setCameraName(video_device_name_);
                sensor_msgs::CameraInfo camera_info;
                camera_info.header.frame_id = img_.header.frame_id;
                camera_info.width = image_width_;
                camera_info.height = image_height_;
                cinfo_->setCameraInfo(camera_info);
            }
            // create Services
            if (bSubscriberTopic_)
            {
                std::string input_image_format = "MJPG";
                cam_.input_image_v4l_format_ = v4l2_fourcc(input_image_format[0],
                                                           input_image_format[1],
                                                           input_image_format[2],
                                                           input_image_format[3]);
                cam_.input_image_avpixel_format_ = cam_.v4l_format_to_avpixel_format(cam_.input_image_v4l_format_);
                if (cam_.input_image_avpixel_format_ == AV_PIX_FMT_NONE)
                {
                    ROS_ASSERT("input_image_avpixel_format_");
                }
                cam_.output_image_avpixel_format_ = cam_.ros_image_format_to_avpixel_format("mono8");
                if (cam_.output_image_avpixel_format_ == AV_PIX_FMT_NONE)
                {
                    ROS_ASSERT("output_image_avpixel_format_");
                }
                if (cam_.init_mjpeg_decoder(image_width_, image_height_, 1) <= 0)
                {
                    ROS_ASSERT("init_mjpeg_decoder");
                }
                cam_.image_ = (usb_cam::UsbCam::camera_image_t *)calloc(1, sizeof(usb_cam::UsbCam::camera_image_t));
                cam_.image_->width = image_width_;
                cam_.image_->height = image_height_;
                cam_.image_->bytes_per_pixel = 1;
                cam_.image_->image_size = image_width_ * image_height_;
                cam_.image_->image = (char *)calloc(4096 * 3072, sizeof(char));
                sub_imu_ = node_.subscribe(MJPEG_Topic_, 240, &UsbCamNode::mjpeg_callback, this);
            }
            else
            {
                service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
                service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

                if (pixel_format_name_.size() != 4)
                {
                    ROS_FATAL("v4l_pixel_format is error, string length must 4");
                    node_.shutdown();
                    return;
                }

                // set the IO method
                UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
                if (io_method == UsbCam::IO_METHOD_UNKNOWN)
                {
                    ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
                    node_.shutdown();
                    return;
                }

                if (access(video_device_name_.c_str(), F_OK) < 0)
                {
                    if (video_device_name_.find(":") != std::string::npos)
                    {
                        int vID, pID;
                        sscanf(video_device_name_.c_str(), "%x:%x", &vID, &pID);
                        DvpUSB_Cam = new DVP2USB_CAMERA(vID, pID, image_width_, image_height_, pixel_format_name_, publish_image_format_);
                        if (DvpUSB_Cam)
                        {
                            isDvpUSBCam = true;
                            DvpUSB_Cam->Init();
                            DvpUSB_Cam->start_capturing();
                        }
                        else
                        {
                            node_.shutdown();
                            return;
                        }
                    }
                    else
                    {
                        node_.shutdown();
                        return;
                    }
                }

                if (!isDvpUSBCam)
                {
                    // 获取摄像头支持的参数配置表
                    cam_.get_v4l_parameter_list(video_device_name_, camera_name_);

                    // 获取摄像头支持的数据格式
                    std::set<std::string> formatList;
                    if (cam_.get_v4l_format_list(formatList, video_device_name_) < 0 ||
                        formatList.find(pixel_format_name_) == formatList.end())
                    {
                        ROS_FATAL("This camera do not support pix format '%s'", pixel_format_name_.c_str());
                        node_.shutdown();
                        return;
                    }

                    if (!enable_imx_isp_)
                    {
                        std::set<std::string> framesizeList;
                        std::string frameSize_ = std::to_string(image_width_) + "x" + std::to_string(image_height_);
                        if (cam_.get_v4l_framesizes(framesizeList, pixel_format_name_, video_device_name_) < 0 ||
                            framesizeList.find(frameSize_) == framesizeList.end())
                        {
                            ROS_FATAL("This camera do not support framesize '%s'", frameSize_.c_str());
                            node_.shutdown();
                            return;
                        }
                        std::set<int> frameintervalList;
                        if (cam_.get_v4l_frameintervals(frameintervalList, pixel_format_name_,
                                                        image_width_, image_height_, video_device_name_) < 0 ||
                            frameintervalList.find(framerate_) == frameintervalList.end())
                        {
                            ROS_FATAL("This camera do not support frameinterval '%d'", framerate_);
                            node_.shutdown();
                            return;
                        }
                    }

                    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
                             image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);
                    // start the camera
                    if (cam_.start(video_device_name_.c_str(), io_method, pixel_format_name_, image_width_,
                                   image_height_, framerate_, framerate_tb_, time_delay_per_byte_, process_image_in_other_thread_,
                                   publish_image_format_, enable_imx_isp_, enable_imx_dewarp_) < 0)
                    {
                        ROS_FATAL("camera start init error ");
                        node_.shutdown();
                        return;
                    }

                    if (!enable_imx_isp_)
                    {
                        if (pDynamicConfigServer_NormalCamera != nullptr)
                        {
                            if (cam_.ctrlParaList.find("brightness") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.brightness = cam_.ctrlParaList["brightness"].min_int;
                                CameraMaxConfig.brightness = cam_.ctrlParaList["brightness"].max_int;
                            }
                            if (cam_.ctrlParaList.find("contrast") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.contrast = cam_.ctrlParaList["contrast"].min_int;
                                CameraMaxConfig.contrast = cam_.ctrlParaList["contrast"].max_int;
                            }
                            if (cam_.ctrlParaList.find("saturation") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.saturation = cam_.ctrlParaList["saturation"].min_int;
                                CameraMaxConfig.saturation = cam_.ctrlParaList["saturation"].max_int;
                            }
                            if (cam_.ctrlParaList.find("sharpness") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.sharpness = cam_.ctrlParaList["sharpness"].min_int;
                                CameraMaxConfig.sharpness = cam_.ctrlParaList["sharpness"].max_int;
                            }
                            if (cam_.ctrlParaList.find("gain") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.gain = cam_.ctrlParaList["gain"].min_int;
                                CameraMaxConfig.gain = cam_.ctrlParaList["gain"].max_int;
                            }
                            if (cam_.ctrlParaList.find("gamma") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.gamma = cam_.ctrlParaList["gamma"].min_int;
                                CameraMaxConfig.gamma = cam_.ctrlParaList["gamma"].max_int;
                            }
                            if (cam_.ctrlParaList.find("white_balance_temperature") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.white_balance_temperature = cam_.ctrlParaList["white_balance_temperature"].min_int;
                                CameraMaxConfig.white_balance_temperature = cam_.ctrlParaList["white_balance_temperature"].max_int;
                            }
                            if (cam_.ctrlParaList.find("exposure_absolute") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.exposure_absolute = cam_.ctrlParaList["exposure_absolute"].min_int;
                                CameraMaxConfig.exposure_absolute = cam_.ctrlParaList["exposure_absolute"].max_int;
                            }
                            else if (cam_.ctrlParaList.find("exposure_time_absolute") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.exposure_absolute = cam_.ctrlParaList["exposure_time_absolute"].min_int;
                                CameraMaxConfig.exposure_absolute = cam_.ctrlParaList["exposure_time_absolute"].max_int;
                            }
                            if (cam_.ctrlParaList.find("focus_absolute") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.focus_absolute = cam_.ctrlParaList["focus_absolute"].min_int;
                                CameraMaxConfig.focus_absolute = cam_.ctrlParaList["focus_absolute"].max_int;
                            }
                            if (cam_.ctrlParaList.find("hue") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.hue = cam_.ctrlParaList["hue"].min_int;
                                CameraMaxConfig.hue = cam_.ctrlParaList["hue"].max_int;
                            }
                            if (cam_.ctrlParaList.find("backlight_compensation") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.backlight_compensation = cam_.ctrlParaList["backlight_compensation"].min_int;
                                CameraMaxConfig.backlight_compensation = cam_.ctrlParaList["backlight_compensation"].max_int;
                            }
                            if (cam_.ctrlParaList.find("pan_absolute") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.pan_absolute = cam_.ctrlParaList["pan_absolute"].min_int;
                                CameraMaxConfig.pan_absolute = cam_.ctrlParaList["pan_absolute"].max_int;
                            }
                            if (cam_.ctrlParaList.find("tilt_absolute") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.tilt_absolute = cam_.ctrlParaList["tilt_absolute"].min_int;
                                CameraMaxConfig.tilt_absolute = cam_.ctrlParaList["tilt_absolute"].max_int;
                            }
                            if (cam_.ctrlParaList.find("zoom_absolute") != cam_.ctrlParaList.end())
                            {
                                CameraMinConfig.zoom_absolute = cam_.ctrlParaList["zoom_absolute"].min_int;
                                CameraMaxConfig.zoom_absolute = cam_.ctrlParaList["zoom_absolute"].max_int;
                            }
                            pDynamicConfigServer_NormalCamera->setConfigDefault(CameraDefaultConfig);
                            pDynamicConfigServer_NormalCamera->setConfigMin(CameraMinConfig);
                            pDynamicConfigServer_NormalCamera->setConfigMax(CameraMaxConfig);
                            cbDynamicConfig_NormalCamera = boost::bind(&usb_cam::UsbCam::CfgCB_NormalCamera, &cam_, _1, _2);
                            pDynamicConfigServer_NormalCamera->setCallback(cbDynamicConfig_NormalCamera);
                        }
                    }
                }

                if (process_image_in_other_thread_)
                {
                    boost::thread t(boost::bind(&UsbCamNode::Thread_ProcessImage, this));
                }
            }
        }

        virtual ~UsbCamNode()
        {
            if (pDynamicConfigServer_NormalCamera)
            {
                delete pDynamicConfigServer_NormalCamera;
                pDynamicConfigServer_NormalCamera = nullptr;
            }
            cam_.shutdown();
            if (DvpUSB_Cam)
            {
                delete DvpUSB_Cam;
                DvpUSB_Cam = nullptr;
            }
        }
        void Thread_ProcessImage()
        {
            // TODO：占用内存资源，后期需要把这部分代码优化，及移植到算法代码程序里减少内存使用
            while (!bSubscriberTopic_)
            {
                usleep(10);
                if (cam_.image_ != nullptr && cam_.datas.size() > 0) // for ov9281
                {
                    ros::Time curTime = ros::Time::now();
                    // std::string MJPEG_Topic;
                    // int bPUB_MJPEG;
                    if (curTime.toSec() - cam_.datas.front().stamp.toSec() < image_valid_time_)
                    {
                        if (bPUB_MJPEG_ > 0)
                        {
                            csjw_msgs::mjpeg jpegMsg;
                            jpegMsg.header.stamp = cam_.datas.front().stamp;
                            jpegMsg.header.frame_id = img_.header.frame_id;
                            jpegMsg.height = cam_.image_->height;
                            jpegMsg.width = cam_.image_->width;
                            jpegMsg.size = cam_.datas.front().dataLen;
                            jpegMsg.data.resize(jpegMsg.size);
                            memcpy(&jpegMsg.data[0], cam_.datas.front().data, jpegMsg.size);
                            jpeg_pub_.publish(jpegMsg);
                        }
                        if (bPUB_MJPEG_ != 1 && cam_.process_image(cam_.datas.front().data,
                                                                   cam_.datas.front().dataLen,
                                                                   cam_.image_) >= 0)
                        {
                            // stamp the image
                            img_.header.stamp = cam_.datas.front().stamp;
                            // fill the info
                            int chan = sensor_msgs::image_encodings::numChannels(cam_.publish_image_format_);
                            if (chan > 0)
                            {
                                fillImage(img_, cam_.publish_image_format_, cam_.image_->height, cam_.image_->width, chan * cam_.image_->width,
                                          cam_.image_->image);
                            }
                            cam_.datas.pop_front();

                            // publish the image
                            if (bStereo_)
                            {
                                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_, publish_image_format_);
                                cv::Mat image = cv_ptr->image;

                                cv::Mat leftImg = image(cv::Rect(0, 0, img_.width >> 1, img_.height)).clone();
                                cv::Mat rightImg = image(cv::Rect(img_.width >> 1, 0, img_.width >> 1, img_.height)).clone();

                                sensor_msgs::ImagePtr img_left_ptr = cv_bridge::CvImage(std_msgs::Header(), publish_image_format_, leftImg).toImageMsg();
                                sensor_msgs::ImagePtr img_right_ptr = cv_bridge::CvImage(std_msgs::Header(), publish_image_format_, rightImg).toImageMsg();
                                img_left_ptr->header.stamp = img_.header.stamp;
                                img_right_ptr->header.stamp = img_.header.stamp;

                                image_pub_left_.publish(*img_left_ptr);
                                image_pub_right_.publish(*img_right_ptr);
                            }
                            else
                            {
                                // grab the camera info
                                sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
                                ci->header.frame_id = img_.header.frame_id;
                                ci->header.stamp = img_.header.stamp;
                                image_pub_.publish(img_, *ci);
                            }
                        }
                        else
                        {
                            cam_.datas.pop_front();
                        }
                    }
                    else
                    {
                        cam_.datas.pop_front();
                    }
                }
            }
        }
        bool take_and_send_image()
        {
            // grab the image
            int ret = cam_.grab_image(&img_);
            if (ret < 0)
                return false;
            if (ret > 0 && !process_image_in_other_thread_)
            {
                if (bStereo_)
                {
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_, publish_image_format_);
                    cv::Mat image = cv_ptr->image;

                    cv::Mat leftImg = image(cv::Rect(0, 0, img_.width >> 1, img_.height)).clone();
                    cv::Mat rightImg = image(cv::Rect(img_.width >> 1, 0, img_.width >> 1, img_.height)).clone();

                    sensor_msgs::ImagePtr img_left_ptr = cv_bridge::CvImage(std_msgs::Header(), publish_image_format_, leftImg).toImageMsg();
                    sensor_msgs::ImagePtr img_right_ptr = cv_bridge::CvImage(std_msgs::Header(), publish_image_format_, rightImg).toImageMsg();
                    img_left_ptr->header.stamp = img_.header.stamp;
                    img_right_ptr->header.stamp = img_.header.stamp;

                    image_pub_left_.publish(*img_left_ptr);
                    image_pub_right_.publish(*img_right_ptr);
                }
                else
                {
                    // grab the camera info
                    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
                    ci->header.frame_id = img_.header.frame_id;
                    ci->header.stamp = img_.header.stamp;

                    // publish the image
                    image_pub_.publish(img_, *ci);
                }
            }

            return true;
        }

        bool spin()
        {
            // ros::Rate loop_rate(2000);
            ros::Rate loop_rate((framerate_tb_ < 1e-6) ? 600 : (1 / framerate_tb_));
            while (node_.ok())
            {
                if (!bSubscriberTopic_)
                {
                    if (isDvpUSBCam)
                    {
                        if (DvpUSB_Cam && DvpUSB_Cam->is_capturing() && DvpUSB_Cam->grab_image(&img_) >= 0)
                        {
                            if (bStereo_)
                            {
                                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_, publish_image_format_);
                                cv::Mat image = cv_ptr->image;

                                cv::Mat leftImg = image(cv::Rect(0, 0, img_.width >> 1, img_.height)).clone();
                                cv::Mat rightImg = image(cv::Rect(img_.width >> 1, 0, img_.width >> 1, img_.height)).clone();

                                sensor_msgs::ImagePtr img_left_ptr = cv_bridge::CvImage(std_msgs::Header(), publish_image_format_, leftImg).toImageMsg();
                                sensor_msgs::ImagePtr img_right_ptr = cv_bridge::CvImage(std_msgs::Header(), publish_image_format_, rightImg).toImageMsg();
                                img_left_ptr->header.stamp = img_.header.stamp;
                                img_right_ptr->header.stamp = img_.header.stamp;

                                image_pub_left_.publish(*img_left_ptr);
                                image_pub_right_.publish(*img_right_ptr);
                            }
                            else
                            {
                                // grab the camera info
                                sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
                                ci->header.frame_id = img_.header.frame_id;
                                ci->header.stamp = img_.header.stamp;
                                // publish the image
                                image_pub_.publish(img_, *ci);
                            }
                        }
                    }
                    else
                    {
                        if (cam_.is_capturing())
                        {
                            if (!take_and_send_image())
                                ROS_WARN("USB camera did not respond in time.");
                        }
                    }
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
            return true;
        }
    };

} // namespace usb_cam

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usb_cam");
    usb_cam::UsbCamNode a;
    a.spin();
    return EXIT_SUCCESS;
}
