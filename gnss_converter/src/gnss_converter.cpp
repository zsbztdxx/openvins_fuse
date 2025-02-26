#include "gnss_converter.h"
void signalHandler(int signum)
{
    g_bQuitProcess = true;
    // 自身退出
    exit(signum);
}

void Thread_Serial(ros::NodeHandle *node, const int i_serial)
{
#define MaxRxBufLen 2048
    if (node != nullptr && i_serial >= 0)
    {
        char line[300] = {0};
        uint8_t serialRxBuf[MaxRxBufLen * 2] = {0};
        int serialRxBufLen = 0;
        int fileUartLog = -1;
        // 保存日志
        if (!g_Config_param.uart_log_dir.empty())
        {
            struct timeval tv;
            struct tm tm;
            GetSysDateTime(&tv, &tm);
            sprintf(line, "mkdir -p %s/%04d%02d%02d%02d%02d%02d/",
                    g_Config_param.uart_log_dir.c_str(),
                    tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                    tm.tm_hour, tm.tm_min, tm.tm_sec);
            system(line);
            sprintf(line, "touch %s/%04d%02d%02d%02d%02d%02d/DM711.log",
                    g_Config_param.uart_log_dir.c_str(),
                    tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                    tm.tm_hour, tm.tm_min, tm.tm_sec);
            system(line);
            sprintf(line, "%s/%04d%02d%02d%02d%02d%02d/DM711.log",
                    g_Config_param.uart_log_dir.c_str(),
                    tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                    tm.tm_hour, tm.tm_min, tm.tm_sec);
            fileUartLog = open(line, O_WRONLY | O_CREAT);
        }
        // sprintf(line, "CSHG ins imuangle 180 180\r\n");
        // write(i_serial, line, strlen(line));
        // printf("set imu angel 180 180\r\n");
        // usleep(9999);
        // sprintf(line, "CSHG saveconfig\r\n");
        // write(i_serial, line, strlen(line));
        // printf("saveconfig\r\n");
        // usleep(9999);
        // 避免没有打开log
        if (CSHG_IMUDATAA_IsNeed())
        {
            sprintf(line, "CSHG OPEN IMUDATAA ontime 0.01\r\n");
            write(i_serial, line, strlen(line));
            printf("Open imudataA\r\n");
            usleep(9999);
        }
        if (CSHG_IMUDATAB_IsNeed())
        {
            sprintf(line, "CSHG OPEN IMUDATAB ontime 0.01\r\n");
            write(i_serial, line, strlen(line));
            printf("Open imudataB\r\n");
            usleep(9999);
        }
        if (CSHG_SATNAVA_IsNeed())
        {
            sprintf(line, "CSHG OPEN SATNAVA ontime 0.1\r\n");
            write(i_serial, line, strlen(line));
            printf("Open satnavA\r\n");
            usleep(9999);
        }
        if (CSHG_SATNAVB_IsNeed())
        {
            sprintf(line, "CSHG OPEN SATNAVB ontime 0.1\r\n");
            write(i_serial, line, strlen(line));
            printf("Open satnavB\r\n");
            usleep(9999);
        }
        if (CSHG_INSNAVA_IsNeed())
        {
            sprintf(line, "CSHG OPEN INSNAVA ontime 0.01\r\n");
            write(i_serial, line, strlen(line));
            printf("Open insnavA\r\n");
            usleep(9999);
        }
        if (CSHG_INSNAVB_IsNeed())
        {
            sprintf(line, "CSHG OPEN INSNAVB ontime 0.01\r\n");
            write(i_serial, line, strlen(line));
            printf("Open insnavB\r\n");
            usleep(9999);
        }
        if (CSHG_INSPVAA_IsNeed())
        {
            sprintf(line, "CSHG OPEN INSPVAA ontime 0.01\r\n");
            write(i_serial, line, strlen(line));
            printf("Open inspvaA\r\n");
            usleep(9999);
        }
        if (CSHG_INSPVAB_IsNeed())
        {
            sprintf(line, "CSHG OPEN INSPVAB ontime 0.01\r\n");
            write(i_serial, line, strlen(line));
            printf("Open inspvaB\r\n");
            usleep(9999);
        }
        if (CSHG_GGA_IsNeed())
        {
            sprintf(line, "CSHG OPEN GGA ontime 0.1\r\n");
            write(i_serial, line, strlen(line));
            printf("Open GGA\r\n");
            usleep(9999);
        }
        if (CSHG_GGP_IsNeed())
        {
            sprintf(line, "CSHG OPEN GGP ontime 0.2\r\n");
            write(i_serial, line, strlen(line));
            printf("Open GGP\r\n");
            usleep(9999);
        }
        // 处理协议
        while (!g_bQuitProcess && ros::ok())
        {
            // 阻塞方式读串口
            int curRxLen = read(i_serial, serialRxBuf + serialRxBufLen, 1);
            if (curRxLen > 0)
            {
                if (!g_Config_param.uart_log_dir.empty() && fileUartLog >= 0)
                {
                    write(fileUartLog, serialRxBuf + serialRxBufLen, curRxLen);
                }
                serialRxBufLen += curRxLen;
                int curProcLen;
                int procLen = 0;
                do
                {
                    uint8_t ptcl;
                    uint32_t frameStartIndex = 0;
                    curProcLen = GNSS_CheckFrameValid(MsgSource_UART_GNSS, serialRxBuf + procLen, serialRxBufLen - procLen, &ptcl, &frameStartIndex);
                    if (curProcLen > 0)
                    {
                        bool needTrans = false;
                        GNSS_Process(MsgSource_UART_GNSS,
                                     serialRxBuf + procLen + frameStartIndex,
                                     serialRxBufLen - procLen - frameStartIndex,
                                     ptcl, &needTrans);
                        procLen += curProcLen;
                    }
                } while (curProcLen > 0);
                if (procLen > 0)
                {
                    memmove(serialRxBuf, serialRxBuf + procLen, sizeof(serialRxBuf) - (size_t)procLen);
                    serialRxBufLen -= (uint16_t)procLen;
                }
                else if (serialRxBufLen >= sizeof(serialRxBuf) - MaxRxBufLen)
                {
                    // 缓存区快慢还没收到一帧有效报文
                    memmove(serialRxBuf, serialRxBuf + sizeof(serialRxBuf) - MaxRxBufLen, MaxRxBufLen);
                    serialRxBufLen -= MaxRxBufLen;
                }
            }
        }
        if (fileUartLog >= 0)
        {
            close(fileUartLog);
            fileUartLog = -1;
        }
    }
}

// 对时精度正负正负0.5ms
void Thread_ReadPPS(void)
{
    if (g_Config_param.enable_set_time == 2)
    {
        // int fileLed1 = open("/sys/class/leds/led-1/brightness", O_WRONLY);
        // int fileLed2 = open("/sys/class/leds/led-2/brightness", O_WRONLY);
        // int fileLed3 = open("/sys/class/leds/led-3/brightness", O_WRONLY);
        // int fileLed4 = open("/sys/class/leds/led-4/brightness", O_WRONLY);
        // bool openLed = false;
        int filePPS = open(g_Config_param.pps_input.c_str(), O_RDONLY);
        if (filePPS >= 0)
        {
            struct input_event buf;
            while (!g_bQuitProcess && ros::ok())
            {
                int ret = read(filePPS, (char *)&buf, sizeof(struct input_event)); // delay 600~640us
                if (buf.code == 240 && buf.value == 1)
                {
                    if (buf.time.tv_usec > 500000)
                    {
                        buf.time.tv_sec += 1;
                    }
                    else
                    {
                        buf.time.tv_sec += 0;
                    }
                    buf.time.tv_usec = g_Config_param.delay_pps_settime_us; // 添加延时(中断延时+对时延时),depend on the test of Thread_TestTime
                    settimeofday(&buf.time, nullptr);
                }
            }
            close(filePPS);
        }
        else
        {
            printf("gnss driver can not open pps: %s\r\n", g_Config_param.pps_input.c_str());
        }

        // write(fileLed1, "255", sizeof("255"));   //"led write control" delay about 10~12.8us
        // close(fileLed1);
        // close(fileLed2);
        // close(fileLed3);
        // close(fileLed4);
    }
}

void Thread_TestTime()
{
    // int fileLed3 = open("/sys/class/leds/led-3/brightness", O_WRONLY);
    int fileLed4 = open("/sys/class/leds/led-4/brightness", O_WRONLY);
    if (fileLed4 >= 0)
    {
        bool bOpenLed = false;
        struct timeval cur_tv;
        while (!g_bQuitProcess && ros::ok())
        {
            usleep(1);
            if (gettimeofday(&cur_tv, nullptr) >= 0) //"gettimeofday" delay about 12us
            {
                if (cur_tv.tv_usec >= 0 && cur_tv.tv_usec < 10000)
                {
                    if (!bOpenLed)
                    {
                        write(fileLed4, "255", sizeof("255"));
                        bOpenLed = true;
                    }
                }
                else
                {
                    if (bOpenLed)
                    {
                        write(fileLed4, "0", sizeof("0"));
                        bOpenLed = false;
                    }
                }
            }
        }
        // close(fileLed3);
        close(fileLed4);
    }
}

void Thread_Ntrip_Transfer(ros::NodeHandle *node, const int i_serial, const int i_serial2)
{
    if (node != nullptr)
    {
        //./str2str -in ntrip://qxuzky002:f29fbf1@rtk.ntrip.qxwz.com:8002/AUTO -out test.log -n 5000 -p 23 116 13
        //./str2str -in ntrip://qxuzky002:f29fbf1@rtk.ntrip.qxwz.com:8002/AUTO -out serial://ttyUSB1:1115200 -n 5000 -p 23 116 13
        if (g_Config_param.enable_rtk_ntrip_transfer &&
            !g_Config_param.ntrip_addr.empty())
        {
            std_msgs::UInt8 ntripInfo;
            ros::Publisher pub_ntrip_ = node->advertise<std_msgs::UInt8>("ntrip_state", 10);
            uint8_t buff[NTRIP_MAXRSP];
            uint32_t tick = 0, ticknmea = 0, tickpub = 0;
            ntrip_t *pNtrip = nullptr;
            ntripInfo.data = 0;
            int iii = 0;
            std::list<uint32_t> recvNtrip;
            tick = ticknmea = tickpub = tickget();
            while (!g_bQuitProcess && ros::ok())
            {
                tick = tickget();
                if (pNtrip == nullptr)
                {
                    pNtrip = openntrip();
                }
                else
                {
                    memset(buff, 0, sizeof(buff));
                    int ret = readntrip(pNtrip, buff, sizeof(buff)); // 读Ntrip数据
                    if (ret > 0 && i_serial >= 0)
                    {
                        write(i_serial, buff, ret); // 发送到RTK板子
                        // 更新Ntrip接收状态
                        recvNtrip.push_back(tick);
                        ntripInfo.data |= 1;
                    }
                    if (ret > 0 && i_serial2 >= 0)
                    {
                        write(i_serial2, buff, ret); // 发送到RTK外部板子
                    }
                    // 1s发送一次gga信息给Ntrip
                    if (pNtrip->state == 2 && tick - ticknmea >= 1000) // 1s
                    {
                        if (PackNMEA_GGA((char *)buff, sizeof(buff)) >= 0)
                        {
                            writentrip(pNtrip, buff, strlen((char *)buff));
                            ticknmea = tick;
                        }
                    }
                }
                // 1s更新一次ntrip状态
                if (tick - tickpub >= 1000)
                {
                    // 更新Ntrip接收状态
                    while (!recvNtrip.empty() && tick - recvNtrip.front() > 10000) // 10s
                    {
                        recvNtrip.pop_front();
                    }
                    ntripInfo.data = (recvNtrip.size() << 1) + (ntripInfo.data & 1);
                    if (pNtrip == nullptr || pNtrip->state != 2)
                    {
                        ntripInfo.data &= 0xFFFFFFFE;
                    }
                    // ros发送Ntrip状态
                    pub_ntrip_.publish(ntripInfo);
                    tickpub = tick;
                }
                usleep(500000 - (tickget() - tick) * 1000); // 500ms
            }
            // 更新Ntrip接收状态
            while (!recvNtrip.empty() && tick - recvNtrip.front() > 10000) // 10s
            {
                recvNtrip.pop_front();
            }
            ntripInfo.data = recvNtrip.size() << 1;
            ntripInfo.data &= 0xFFFFFFFE;
            // ros发送Ntrip状态
            pub_ntrip_.publish(ntripInfo);
            // 关闭ntrip
            if (pNtrip)
            {
                closentrip(pNtrip);
            }
        }
    }
}

int main(int argc, char **argv)
{
    // 获取ros_ws路径
    if (Get_executable_path(g_rosws_path, (char *)"/install") < 0)
    {
        if (Get_executable_path(g_rosws_path, (char *)"/install_isolated") < 0)
        {
            if (Get_executable_path(g_rosws_path, (char *)"/devel") < 0)
            {
                std::cout << "Get_executable_path path error" << std::endl;
            }
        }
    }
    char line[300] = {0};
    sprintf(line, "mkdir -p %s/install/share/gnss_converter/log/", g_rosws_path.c_str());
    system(line);
    // glog初始化
    char tempPath[256] = {0};
    sprintf(tempPath, "%s/install/share/gnss_converter/log/", g_rosws_path.c_str());
    FLAGS_log_dir = tempPath;               // 保存路径
    FLAGS_logtostderr = false;              // TRUE:标准输出,FALSE:文件输出
    FLAGS_alsologtostderr = true;           // 除了日志文件之外是否需要标准输出
    FLAGS_colorlogtostderr = false;         // 标准输出带颜色
    FLAGS_logbufsecs = 0;                   // 设置可以缓冲日志的最大秒数，0指实时输出
    FLAGS_max_log_size = 100;               // 日志文件大小(单位：MB)
    FLAGS_stop_logging_if_full_disk = true; // 磁盘满时是否记录到磁盘
    google::InitGoogleLogging("gnss_converter");
    LOG(INFO) << "start gnss_converter";
    LOG(WARNING) << "start gnss_converter";
    LOG(ERROR) << "start gnss_converter";

    ros::init(argc, argv, "gnss_converter");
    ros::NodeHandle nh("~");
    // imu
    nh.param<std::string>("serial_port", g_Config_param.serial_port, "");
    nh.param<int>("serial_port_baud", g_Config_param.serial_port_baud, 460800);
    nh.param<std::string>("serial_port2", g_Config_param.serial_port2, "");
    nh.param<int>("serial_port2_baud", g_Config_param.serial_port2_baud, 460800);

    nh.param<std::string>("uart_log_dir", g_Config_param.uart_log_dir, "");

    nh.param<std::string>("pps_input", g_Config_param.pps_input, "");
    nh.param<int>("enable_set_time", g_Config_param.enable_set_time, 2);
    nh.param<int>("delay_pps_settime_us", g_Config_param.delay_pps_settime_us, 120);

    nh.param<int>("enable_read_gga", g_Config_param.enable_read_gga, -1);
    nh.param<int>("enable_read_ggp", g_Config_param.enable_read_ggp, -1);
    nh.param<int>("enable_read_rmc", g_Config_param.enable_read_rmc, -1);
    nh.param<int>("enable_read_gsv", g_Config_param.enable_read_gsv, -1);
    nh.param<int>("enable_read_inspva", g_Config_param.enable_read_inspva, -1);
    nh.param<int>("enable_read_imudata", g_Config_param.enable_read_imudata, -1);
    nh.param<int>("enable_read_satnav", g_Config_param.enable_read_satnav, 1);
    nh.param<int>("enable_read_insnav", g_Config_param.enable_read_insnav, 1);

    nh.param<int>("enable_nav_sat_fix", g_Config_param.enable_nav_sat_fix, 0);
    nh.param<int>("enable_gps_fix", g_Config_param.enable_gps_fix, 0);
    nh.param<int>("enable_master_sat_num", g_Config_param.enable_master_sat_num, 0);
    nh.param<int>("enable_slaver_sat_num", g_Config_param.enable_slaver_sat_num, 0);
    nh.param<int>("enable_gnss_vel", g_Config_param.enable_gnss_vel, 0);
    nh.param<int>("enable_ins_vel", g_Config_param.enable_ins_vel, 0);
    nh.param<int>("enable_imu", g_Config_param.enable_imu, 0);

    // 转发Ntrip定位参数
    nh.param<bool>("enable_rtk_ntrip_transfer", g_Config_param.enable_rtk_ntrip_transfer, true);
    nh.param<std::string>("ntrip_addr", g_Config_param.ntrip_addr, "");
    nh.param<int>("ntrip_port", g_Config_param.ntrip_port, 0);
    nh.param<std::string>("ntrip_user", g_Config_param.ntrip_user, "");
    nh.param<std::string>("ntrip_pwd", g_Config_param.ntrip_pwd, "");
    nh.param<std::string>("ntrip_mnt", g_Config_param.ntrip_mnt, "");
    nh.param<double>("rtk_ntrip_latitude", g_last_latitude, 23.0);
    nh.param<double>("rtk_ntrip_longitude", g_last_longitude, 113.0);

    if (g_Config_param.enable_nav_sat_fix >= 0)
    {
        g_pub_nav_sat_fix = nh.advertise<sensor_msgs::NavSatFix>("NavSatFix", 10);
    }
    // TODO:
    // if (g_Config_param.enable_gps_fix >= 0)
    // {
    //     g_pub_gps_fix = nh.advertise<gps_common::GPSFix>("GpsFix", 10);
    // }
    if (g_Config_param.enable_master_sat_num >= 0)
    {
        g_pub_MasterSatNum = nh.advertise<std_msgs::UInt8>("MasterSatNum", 10);
    }
    if (g_Config_param.enable_slaver_sat_num >= 0)
    {
        g_pub_SlaverSatNum = nh.advertise<std_msgs::UInt8>("SlaverSatNum", 10);
    }
    if (g_Config_param.enable_gnss_vel >= 0)
    {
        g_pub_gnss_vel = nh.advertise<geometry_msgs::TwistStamped>("GnssVel", 10);
    }
    if (g_Config_param.enable_ins_vel >= 0)
    {
        g_pub_ins_vel = nh.advertise<geometry_msgs::TwistStamped>("InsVel", 10);
    }
    if (g_Config_param.enable_imu >= 0)
    {
        g_pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 10);
    }

    if (g_Config_param.enable_read_gga > 0)
    {
        g_pub_gga = nh.advertise<csjw_msgs::gga>("gga", 2);
    }

    if (g_Config_param.enable_read_ggp > 0)
    {
        g_pub_ggp = nh.advertise<csjw_msgs::ggp>("ggp", 2);
    }

    // TODO:
    // if (g_Config_param.enable_read_rmc > 0)
    // {
    //     g_pub_rmc = nh.advertise<nmea_msgs::Gprmc>("rmc", 10);
    // }
    // TODO:
    // if (g_Config_param.enable_read_gsv > 0)
    // {
    //     g_pub_bdgsv = nh.advertise<nmea_msgs::Gpgsv>("BDGsv", 10);
    //     g_pub_gagsv = nh.advertise<nmea_msgs::Gpgsv>("GAGsv", 10);
    //     g_pub_glgsv = nh.advertise<nmea_msgs::Gpgsv>("GLGsv", 10);
    //     g_pub_gpgsv = nh.advertise<nmea_msgs::Gpgsv>("GPGsv", 10);
    // }

    if (g_Config_param.enable_read_inspva > 0)
    {
        g_pub_inspva = nh.advertise<csjw_msgs::inspva>("inspva", 10);
    }
    if (g_Config_param.enable_read_imudata > 0)
    {
        g_pub_imudata = nh.advertise<csjw_msgs::imudata>("imudata", 10);
    }
    if (g_Config_param.enable_read_satnav > 0)
    {
        g_pub_satnav = nh.advertise<csjw_msgs::satnav>("satnav", 10);
    }
    if (g_Config_param.enable_read_insnav > 0)
    {
        g_pub_insnav = nh.advertise<csjw_msgs::insnav>("insnav", 10);
    }

    int serial;
    serial = open(g_Config_param.serial_port.c_str(), O_RDWR);
    if (serial < 0 || SetComPara(serial, g_Config_param.serial_port_baud, 8, 1, 'N') < 0)
    {
        LOG(ERROR) << "gnss driver can not open serial: " << g_Config_param.serial_port;
    }

    int serial2;
    serial2 = open(g_Config_param.serial_port2.c_str(), O_RDWR);
    if (serial2 >= 0)
    {
        SetComPara(serial2, g_Config_param.serial_port2_baud, 8, 1, 'N');
    }

    boost::thread thread_serial(&Thread_Serial, &nh, serial);
    boost::thread thread_pps(&Thread_ReadPPS);
    boost::thread thread_ntrip(&Thread_Ntrip_Transfer, &nh, serial, serial2);

    // 异常中断
    signal(SIGABRT, signalHandler);

    ros::spin();

    thread_serial.join();
    thread_pps.join();
    thread_ntrip.join();

    close(serial);
    return 0;
}