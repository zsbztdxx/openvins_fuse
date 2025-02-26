#pragma once

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include <string>
#include <string.h>
#include <vector>
#include <list>
#include <map>
#include <mutex>
#include "assert.h"
#include "sys/time.h"
#include <sys/types.h>
#include <signal.h>
#include <sys/stat.h>
#include <algorithm>
#include <sys/statfs.h>
#include <dirent.h>
#include <stdarg.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/sem.h>
#include <sys/wait.h>
#include <sys/timeb.h>
#include <linux/rtc.h>
#include <sys/ioctl.h>

#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <pthread.h>
#include <linux/mman.h>
#include <linux/watchdog.h>
#include <linux/input.h>

#include <fstream>
#include <iostream>
#include "glog/logging.h"

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include <gps_common/GPSFix.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gprmc.h>
#include <nmea_msgs/Gpgsv.h>
#include <novatel_gps_msgs/Inspva.h>
#include <csjw_msgs/imudata.h>
#include <csjw_msgs/satnav.h>
#include <csjw_msgs/insnav.h>
#include <csjw_msgs/inspva.h>
#include <csjw_msgs/gga.h>
#include <csjw_msgs/ggp.h>

// 参数配置
typedef struct _Config_param
{
    std::string serial_port = "";
    int serial_port_baud = 115200;
    std::string serial_port2 = "";
    int serial_port2_baud = 115200;
    std::string pps_input = "";
    int delay_pps_settime_us = 0;
    int32_t enable_set_time = 0;

    std::string uart_log_dir = "";

    // <0:不读, =0:只读不发送, >0:读并发送
    int enable_read_gga = -1;     // <0:不读, =0:只读不发送, >0:读并发送
    int enable_read_ggp = -1;     // <0:不读, =0:只读不发送, >0:读并发送
    int enable_read_rmc = -1;     // <0:不读, =0:只读不发送, >0:读并发送
    int enable_read_gsv = -1;     // <0:不读, =0:只读不发送, >0:读并发送
    int enable_read_inspva = -1;  // <0:不读, =0:只读不发送, 1=B，2=A
    int enable_read_imudata = -1; // <0:不读, =0:只读不发送, 1=B，2=A
    int enable_read_satnav = 1;   // <0:不读, =0:只读不发送, 1=B，2=A
    int enable_read_insnav = 1;   // <0:不读, =0:只读不发送, 1=B，2=A

    // 输出sensor_msgs::NavSatFix信息, 信息来源: -1:不输出, 0:SATNAVB, 1:SATNAVA, 2:GGA, 3~N:待定
    int enable_nav_sat_fix = -1;
    // 输出gps_common::GPSFix信息, 信息来源: -1:不输出, 0:SATNAVB, 1:SATNAVA, 2:GGA, 3~N:待定
    int enable_gps_fix = -1;
    // 输出卫星星数std_msgs::UInt8信息, 信息来源: -1:不输出, 0:SATNAVB, 1:SATNAVA, 2:GGA, 3~N:待定
    int enable_master_sat_num = -1;
    // 输出卫星星数std_msgs::UInt8信息, 信息来源: -1:不输出, 0:SATNAVB, 1:SATNAVA, 2:GGS, 3~N:待定
    int enable_slaver_sat_num = -1;
    // 输出卫导速度geometry_msgs::TwistStamped信息, -信息来源: 1:不输出, 0:SATNAVB, 1:SATNAVA, 2~N:待定
    int enable_gnss_vel = -1;
    // 输出卫惯速度geometry_msgs::TwistStamped信息, 信息来源: -1:不输出, 0:INSNAVB, 1:INSNAVA, 2~N:待定
    int enable_ins_vel = -1;
    // 输出sensor_msgs::Imu信息, 信息来源: -1:不输出, 0:IMUDATAB, 1:IMUDATAA 2~N:待定
    int enable_imu = 0;

    bool enable_rtk_ntrip_transfer = true;
    std::string ntrip_addr = "rtk.ntrip.qxwz.com";
    int ntrip_port = 8002;
    std::string ntrip_user = "qxuzky002";
    std::string ntrip_pwd = "f29fbf1";
    std::string ntrip_mnt = "AUTO";
} Config_param;

typedef enum
{
    MsgSource_UART1,
    MsgSource_UART2,
    MsgSource_UART_4G,
    MsgSource_UART_WIFI_BT,
    MsgSource_UART_GNSS,
    MsgSource_NUM,

    MsgSource_UNKNOWN = -1, // 127
} MsgSource;

typedef struct
{
    uint32_t seconds;
    uint32_t nanoseconds;
} tTimeTickNanoSec;

typedef struct
{                /* time struct */
    time_t time; /* time (s) expressed by standard time_t */
    double sec;  /* fraction of second under 1 s */
} gtime_t;

#define MSGSRC_NUM 16

#define PROTOCOL_UNKNOWN 0                  // 未知协议
#define PROTOCOL_UDS 1                      // UDS
#define PROTOCOL_ODB 2                      // ODB
#define PROTOCOL_J1939 3                    // J1939
#define PROTOCOL_Trans 4                    // 透传
#define PROTOCOL_REG_CAN 5                  // CAN寄存器协议
#define PROTOCOL_CSHG_GNSS_BIN 21           // 本公司自定义gnss数据协议，如INSPVAB
#define PROTOCOL_RTCM3 22                   // RTCM3协议，用于接收基站信息
#define PROTOCOL_NMEA0183 23                // NMEA0183协议，如GGA
#define PROTOCOL_CSHG_GNSS_ASCII 24         // 本公司自定义gnss数据协议，如INSPVAA POSDATAA ODMDATAA
#define PROTOCOL_CSHG_OTA 25                // 本公司统一OTA协议
#define PROTOCOL_CSHG_CMD 26                // 本公司模块配置命令
#define PROTOCOL_CSHG_CMD_RET 27            // 本公司模块配置命令的回复
#define PROTOCOL_CSHG_CMD_Simplify 28       // 本公司模块配置简化命令
#define PROTOCOL_CSHG_GNSS_BIN2 29          // 本公司自定义gnss数据协议，如POSDATAB ODMDATAB
#define PROTOCOL_CSHG_GNSS_ASCII2 30        // 兼容诺瓦泰/北云公司自定义 ASCII 数据协议，如HEADINGA
#define PROTOCOL_CSHG_GNSS_BIN3 31          // 兼容诺瓦泰/北云公司自定义 二进制 标准 数据协议，如RANGEB, INSPVAXB
#define PROTOCOL_CSHG_GNSS_BIN4 32          // 兼容诺瓦泰/北云公司自定义 二进制 简化 数据协议，如RAWIMUSB
#define PROTOCOL_NTRIP_RET 33               // ntrip协议回复
#define PROTOCOL_RONGGAN_GNSS_BIN 50        // 融感串口协议(比亚迪在用)(与诺瓦泰兼容除 PROTOCOL_CSHG_GNSS_BIN3 PROTOCOL_CSHG_GNSS_BIN4 外指令)
#define PROTOCOL_LIDE_GNSS_BIN 51           // 立德串口协议(比亚迪在用)
#define PROTOCOL_CSHG_CMDLOG 52             // 本公司调试打印信息
#define PROTOCOL_CAN_REG_IN_CSJW 200        // 本公司CAN输入寄存器协议
#define PROTOCOL_CAN_REG_IN_BYD_TEST 201    // 比亚迪(自动驾驶)CAN输入寄存器协议
#define PROTOCOL_CAN_REG_IN_BYD_RONGGAN 202 // 比亚迪云巴(融感)CAN输入寄存器协议
#define PROTOCOL_CAN_REG_IN_BYD_LIDE 203    // 比亚迪云巴(立得)CAN输入寄存器协议

typedef enum
{
    // cshg open / close XXX 指令(除CANID外)
    CSHG_GNSS_LOG_INDEX_INSPVAA = 0,
    CSHG_GNSS_LOG_INDEX_INSPVAB,
    CSHG_GNSS_LOG_INDEX_INSPOSA,
    CSHG_GNSS_LOG_INDEX_INSIMUA,
    CSHG_GNSS_LOG_INDEX_INSSRCA,
    CSHG_GNSS_LOG_INDEX_POSDATAA,
    CSHG_GNSS_LOG_INDEX_POSDATAB,
    CSHG_GNSS_LOG_INDEX_IMUDATAA,
    CSHG_GNSS_LOG_INDEX_IMUDATAB,
    CSHG_GNSS_LOG_INDEX_GGA,
    CSHG_GNSS_LOG_INDEX_RMC, // = 10
    CSHG_GNSS_LOG_INDEX_ZDA,
    CSHG_GNSS_LOG_INDEX_CPD,
    CSHG_GNSS_LOG_INDEX_CDD,
    CSHG_GNSS_LOG_INDEX_PKG,
    CSHG_GNSS_LOG_INDEX_GSA, // = 15
    CSHG_GNSS_LOG_INDEX_GSV,
    CSHG_GNSS_LOG_INDEX_KSXT,
    CSHG_GNSS_LOG_INDEX_BD2EPHB,
    CSHG_GNSS_LOG_INDEX_BD3EPHB,
    CSHG_GNSS_LOG_INDEX_GPSEPHB, // = 20
    CSHG_GNSS_LOG_INDEX_GLOEPHB,
    CSHG_GNSS_LOG_INDEX_GALEPHB,
    CSHG_GNSS_LOG_INDEX_ODMPLUSA,
    CSHG_GNSS_LOG_INDEX_ODMPLUSB,
    CSHG_GNSS_LOG_INDEX_ODMDATAA,
    CSHG_GNSS_LOG_INDEX_ODMDATAB,
    CSHG_GNSS_LOG_INDEX_RTCM999,
    CSHG_GNSS_LOG_INDEX_RTCM1005, // 基准站信息(XYZ)
    CSHG_GNSS_LOG_INDEX_RTCM1006, // 基准站信息(XYZ, 含高程)
    CSHG_GNSS_LOG_INDEX_RTCM1013, // = 30 // 系统参数
    CSHG_GNSS_LOG_INDEX_RTCM1019, // GPS 星历
    CSHG_GNSS_LOG_INDEX_RTCM1020, // GLONASS 星历
    CSHG_GNSS_LOG_INDEX_RTCM1033, // Receiver and Antenna Description
    CSHG_GNSS_LOG_INDEX_RTCM1042, // BDS 星历
    CSHG_GNSS_LOG_INDEX_RTCM1044, // QZSS 星历
    CSHG_GNSS_LOG_INDEX_RTCM1045, // Galileo F/NAV 星历
    CSHG_GNSS_LOG_INDEX_RTCM1046, // Galileo I/NAV 星历
    CSHG_GNSS_LOG_INDEX_RTCM1074, // GPS MSM4
    CSHG_GNSS_LOG_INDEX_RTCM1084, // GLONASS MSM4
    CSHG_GNSS_LOG_INDEX_RTCM1094, // = 40 // GALILEO MSM4
    CSHG_GNSS_LOG_INDEX_RTCM1114, // QZSS MSM4
    CSHG_GNSS_LOG_INDEX_RTCM1124, // BDS MSM4
    CSHG_GNSS_LOG_INDEX_RTCM1075, // GPS MSM5
    CSHG_GNSS_LOG_INDEX_RTCM1085, // GLONASS MSM5
    CSHG_GNSS_LOG_INDEX_RTCM1095, // GALILEO MSM5
    CSHG_GNSS_LOG_INDEX_RTCM1115, // QZSS MSM5
    CSHG_GNSS_LOG_INDEX_RTCM1125, // BDS MSM5
    CSHG_GNSS_LOG_INDEX_RTCM1076, // GPS MSM6
    CSHG_GNSS_LOG_INDEX_RTCM1086, // GLONASS MSM6
    CSHG_GNSS_LOG_INDEX_RTCM1096, // = 50 // GALILEO MSM6
    CSHG_GNSS_LOG_INDEX_RTCM1116, // QZSS MSM6
    CSHG_GNSS_LOG_INDEX_RTCM1126, // BDS MSM6
    CSHG_GNSS_LOG_INDEX_RTCM1077, // GPS MSM7
    CSHG_GNSS_LOG_INDEX_RTCM1087, // GLONASS MSM7
    CSHG_GNSS_LOG_INDEX_RTCM1097, // GALILEO MSM7
    CSHG_GNSS_LOG_INDEX_RTCM1117, // QZSS MSM7
    CSHG_GNSS_LOG_INDEX_RTCM1127, // BDS MSM7
    CSHG_GNSS_LOG_INDEX_DTM,
    CSHG_GNSS_LOG_INDEX_GBS,
    CSHG_GNSS_LOG_INDEX_GLL, // = 60
    CSHG_GNSS_LOG_INDEX_GST,
    CSHG_GNSS_LOG_INDEX_HDT,
    CSHG_GNSS_LOG_INDEX_ORI,
    CSHG_GNSS_LOG_INDEX_NTR,
    CSHG_GNSS_LOG_INDEX_ROT,
    CSHG_GNSS_LOG_INDEX_TRA,
    CSHG_GNSS_LOG_INDEX_VTG,
    CSHG_GNSS_LOG_INDEX_PRANGEB,
    CSHG_GNSS_LOG_INDEX_PRANGE2B,
    CSHG_GNSS_LOG_INDEX_PRANGEALLB, // = 70
    CSHG_GNSS_LOG_INDEX_EGSVA,
    CSHG_GNSS_LOG_INDEX_EGSV2A,
    CSHG_GNSS_LOG_INDEX_EGSVB,
    CSHG_GNSS_LOG_INDEX_EGSV2B,
    CSHG_GNSS_LOG_INDEX_BESTPOSA,
    CSHG_GNSS_LOG_INDEX_BESTVELA,
    CSHG_GNSS_LOG_INDEX_BESTXYZA,
    CSHG_GNSS_LOG_INDEX_HEADINGA,
    CSHG_GNSS_LOG_INDEX_PSRDOPA,
    CSHG_GNSS_LOG_INDEX_PSRPOSA, // = 80
    CSHG_GNSS_LOG_INDEX_PSRVELA,
    CSHG_GNSS_LOG_INDEX_PTNLAVR,
    CSHG_GNSS_LOG_INDEX_PTNLPJK,
    CSHG_GNSS_LOG_INDEX_RANGEB,     // 报文太长,屏蔽不支持
    CSHG_GNSS_LOG_INDEX_RANGE2B,    // 报文太长,屏蔽不支持
    CSHG_GNSS_LOG_INDEX_RANGECMPB,  // 报文太长,屏蔽不支持
    CSHG_GNSS_LOG_INDEX_RANGECMP2B, // 报文太长,屏蔽不支持
    CSHG_GNSS_LOG_INDEX_TIMEA,
    CSHG_GNSS_LOG_INDEX_YBM, // TODO: 数据输出命令包含非字符类,导致校验出错,无法透传
    CSHG_GNSS_LOG_INDEX_GGS, // = 90
    CSHG_GNSS_LOG_INDEX_INSPVASA,
    CSHG_GNSS_LOG_INDEX_INSPVASB,
    CSHG_GNSS_LOG_INDEX_INSPVAXA,
    CSHG_GNSS_LOG_INDEX_INSPVAXB,
    CSHG_GNSS_LOG_INDEX_RAWIMUA,
    CSHG_GNSS_LOG_INDEX_RAWIMUB,
    CSHG_GNSS_LOG_INDEX_RAWIMUSA,
    CSHG_GNSS_LOG_INDEX_RAWIMUSB,
    CSHG_GNSS_LOG_INDEX_RAWIMUXA,
    CSHG_GNSS_LOG_INDEX_RAWIMUXB, // = 100
    CSHG_GNSS_LOG_INDEX_RAWIMUSXA,
    CSHG_GNSS_LOG_INDEX_RAWIMUSXB,
    CSHG_GNSS_LOG_INDEX_RONGGAN_GNSS,
    CSHG_GNSS_LOG_INDEX_RONGGAN_ODM,
    CSHG_GNSS_LOG_INDEX_RONGGAN_RAWIMUSB,
    CSHG_GNSS_LOG_INDEX_EVENTALLA,
    CSHG_GNSS_LOG_INDEX_EVENTALLB,
    CSHG_GNSS_LOG_INDEX_INSRMC,
    CSHG_GNSS_LOG_INDEX_GGI,
    CSHG_GNSS_LOG_INDEX_AGRICA, // = 110
    CSHG_GNSS_LOG_INDEX_AGRICB,
    CSHG_GNSS_LOG_INDEX_INSPTNLPJKSA,
    CSHG_GNSS_LOG_INDEX_BINSPVAA,
    CSHG_GNSS_LOG_INDEX_BINSPVAB,
    CSHG_GNSS_LOG_INDEX_BINSPOSA,
    CSHG_GNSS_LOG_INDEX_BINSPOSB,
    CSHG_GNSS_LOG_INDEX_SATNAVA,
    CSHG_GNSS_LOG_INDEX_SATNAVB,
    CSHG_GNSS_LOG_INDEX_INSNAVA,
    CSHG_GNSS_LOG_INDEX_INSNAVB, // = 120
    CSHG_GNSS_LOG_NUM,           // = 121
    CSHG_GNSS_LOG_UNKNOWN = -1,
} CSHG_GNSS_LOG;

#define __PI_ (3.1415926535897932384626433832795)          // PI
#define __PI_Mult2_ (6.283185307179586476925286766559)     // PI * 2
#define __PI_DIV2_ (1.5707963267948966192313216916398)     // PI / 2
#define __PI_DIV4_ (0.78539816339744830961566084581988)    // PI / 4
#define MultPI_Div180 (0.01745329251994329576923690768489) // 1.0 * PI / 180
#define Mult180_DivPI (57.2957795130823208767981548141)    // 1.0 * 180 / PI

extern Config_param g_Config_param;
extern std::string g_rosws_path;
extern bool g_bQuitProcess;
extern double g_last_latitude, g_last_longitude, g_last_altitude;
extern ros::Publisher g_pub_gga, g_pub_ggp, g_pub_rmc, g_pub_inspva, g_pub_imudata, g_pub_satnav, g_pub_insnav;
extern ros::Publisher g_pub_bdgsv, g_pub_gpgsv, g_pub_glgsv, g_pub_gagsv;
extern ros::Publisher g_pub_nav_sat_fix, g_pub_gps_fix, g_pub_gnss_vel, g_pub_ins_vel, g_pub_imu, g_pub_MasterSatNum, g_pub_SlaverSatNum;

extern int GetSysDateTime(struct timeval *out_tv, struct tm *out_tm);
extern uint32_t tickget();

extern int Get_executable_path(std::string &Mappath, char *select);

extern int SetComPara(int fd, int bps, int databits, int stopbits, int parity);

extern uint16_t crc16(const uint8_t *buf, const uint32_t len);
extern uint32_t rtk_crc24q(const uint32_t lastCRC, const uint8_t *buff, const uint32_t len);
extern uint32_t ota_crc32(const uint32_t lastCRC, const uint8_t *buff, const uint32_t len);
extern uint32_t CRC32_Cal(const uint32_t lastCRC, const uint8_t *buff, const uint32_t len);

extern double dmm2deg(double dmm);
extern double deg2dmm(double deg);

extern void gpst2time(const int week, const double sec, gtime_t *const o_data); // 周内秒+周数 转 GPS日期时间
extern void bdst2time(const int week, const double sec, gtime_t *const o_data); // 周内秒+周数 转 BD日期时间
extern void gpst2utc(gtime_t t, int8_t leapSec, gtime_t *const o_data);         // + GPS/BD闰秒 (GPS时间和UTC时间差)
extern void bdt2gpst(const gtime_t t, gtime_t *const o_data);
extern void gpst2bdt(const gtime_t t, gtime_t *const o_data);
extern void time2epoch(gtime_t t, double *ep); // GPS日期时间 转 UTC日期时间
extern void epoch2time(const double *ep, gtime_t *const o_data);
extern void timeadd(gtime_t t, const double sec, gtime_t *const o_data);

extern int8_t ConvMSBBitUInt1(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint8_t *const o_data);
extern int8_t ConvMSBBitUInt2_UInt9(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint16_t *const o_data);
extern int8_t ConvMSBBitUInt10_UInt17(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint32_t *const o_data);
extern int8_t ConvMSBBitUInt18_UInt25(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint32_t *const o_data);
extern int8_t ConvMSBBitUInt26_UInt33(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint64_t *const o_data);
extern int8_t ConvMSBBitUInt34_UInt41(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint64_t *const o_data);
extern int8_t ConvMSBBitUInt42_UInt49(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint64_t *const o_data);
extern int8_t ConvMSBBitUInt50_UInt57(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint64_t *const o_data);

extern int8_t ConvMSBBitInt2_Int9(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int16_t *const o_data);
extern int8_t ConvMSBBitInt10_Int17(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int32_t *const o_data);
extern int8_t ConvMSBBitInt18_Int25(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int32_t *const o_data);
extern int8_t ConvMSBBitInt26_Int33(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data);
extern int8_t ConvMSBBitInt34_Int41(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data);
extern int8_t ConvMSBBitInt42_Int49(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data);
extern int8_t ConvMSBBitInt50_Int57(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data);

extern int8_t ConvLSBBitUInt1(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint8_t *const o_data);
extern int8_t ConvLSBBitUInt2_UInt9(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint16_t *const o_data);
extern int8_t ConvLSBBitUInt10_UInt17(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint32_t *const o_data);
extern int8_t ConvLSBBitUInt18_UInt25(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint32_t *const o_data);
extern int8_t ConvLSBBitUInt26_UInt33(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint64_t *const o_data);
extern int8_t ConvLSBBitUInt34_UInt41(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint64_t *const o_data);
extern int8_t ConvLSBBitUInt42_UInt49(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint64_t *const o_data);
extern int8_t ConvLSBBitUInt50_UInt57(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint64_t *const o_data);

extern int8_t ConvLSBBitInt2_Int9(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int16_t *const o_data);
extern int8_t ConvLSBBitInt10_Int17(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int32_t *const o_data);
extern int8_t ConvLSBBitInt18_Int25(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int32_t *const o_data);
extern int8_t ConvLSBBitInt26_Int33(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data);
extern int8_t ConvLSBBitInt34_Int41(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data);
extern int8_t ConvLSBBitInt42_Int49(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data);
extern int8_t ConvLSBBitInt50_Int57(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data);
