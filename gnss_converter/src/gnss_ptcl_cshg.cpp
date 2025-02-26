#include "gnss_ptcl_cshg.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <limits.h>

typedef enum _CSHG_CMD
{
    // 需要数据处理的协议, >=0 需要处理, <0:只定义enum但不处理
    CSHG_CMD_INSPVAA = 1,
    CSHG_CMD_INSPVAB = 2,
    CSHG_CMD_INSPOS = -3,
    CSHG_CMD_INSIMU = -4,
    CSHG_CMD_POSDATAA = -5,
    CSHG_CMD_POSDATAB = -6,
    CSHG_CMD_IMUDATAA = 7,
    CSHG_CMD_IMUDATAB = 8,
    CSHG_CMD_INSPVAXA = -9,
    CSHG_CMD_INSPVAXB = -10,
    CSHG_CMD_SATNAVA = 11,
    CSHG_CMD_SATNAVB = 12,
    CSHG_CMD_INSNAVA = 13,
    CSHG_CMD_INSNAVB = 14,
    CSHG_CMD_UNKNOWN = -100,
} eCSHG_CMD;

// 本公司自定义 二进制 ID(BIN)
#define BD2_Ephemeris_CMD 1
#define BD3_Ephemeris_CMD 2
#define BD3_2_Ephemeris_CMD 3
#define BD3_3_Ephemeris_CMD 4
#define GPS_Ephemeris_CMD 11
#define GLO_Ephemeris_CMD 21
#define GAL_Ephemeris_CMD 32
#define PRANGEB_CMD 61
#define PRANGE2B_CMD 62
#define PRANGEALLB_CMD 63
#define EGSVB_CMD 72
#define EGSV2B_CMD 73
#define INSRAWB_CMD 75
#define INSPVAB_CMD 76
#define BD2_IONO_CMD 205
#define BD3_IONO_CMD 206
#define GPS_IONO_CMD 214

// 本公司自定义 二进制 ID(BIN2)
#define POSDATAB_CMD 0xF090
#define IMUDATAB_CMD 0xF091
#define ODMPLUSB_CMD 0xF092
#define ODMDATAB_CMD 0xF093
#define SATNAVB_CMD 0xF094
#define INSNAVB_CMD 0xF095

// 诺瓦泰 / 北云 二进制 ID(BIN3/BIN4)
#define GNSS_BIN_ID_GPSEPHEM 7
#define GNSS_BIN_ID_IONUTC 8    /* message id: cnav iono and utc data */
#define GNSS_BIN_ID_RAWEPHEM 41 /* message id: cnav raw ephemeris */
#define GNSS_BIN_ID_BESTPOS 42
#define GNSS_BIN_ID_RANGE 43  /* message id: cnav range measurement */
#define GNSS_BIN_ID_RANGE2 45 /* message id: cnav range measurement */
#define GNSS_BIN_ID_PSRPOS 47
#define GNSS_BIN_ID_ALMANAC 73 /* message id: cnav decoded almanac */
#define GNSS_BIN_ID_RAWALM 74  /* message id: cnav raw almanac */
#define GNSS_BIN_ID_TRACKSTAT 83
#define GNSS_BIN_ID_BESTVEL 99
#define GNSS_BIN_ID_PSRVEL 100
#define GNSS_BIN_ID_TIME 101
#define GNSS_BIN_ID_RANGECMP 140 /* message id: cnav range compressed */
#define GNSS_BIN_ID_PSRDOPA 174
#define GNSS_BIN_ID_MARKTIME 231
#define GNSS_BIN_ID_BESTXYZ 241
#define GNSS_BIN_ID_INSATTB 263
#define GNSS_BIN_ID_INSPOSB 265
#define GNSS_BIN_ID_INSSPDB 266
#define GNSS_BIN_ID_INSVELB 267
#define GNSS_BIN_ID_RAWIMUB 268
#define GNSS_BIN_ID_RAWWAASFRAME 287 /* message id: cnav raw waas frame */
#define GNSS_BIN_ID_EVENTALLB 308
#define GNSS_BIN_ID_EVENTMARKB 309
#define GNSS_BIN_ID_RAWIMUSB 325
#define GNSS_BIN_ID_INSPVAB 507
#define GNSS_BIN_ID_INSPVASB 508
#define GNSS_BIN_ID_MARK2TIME 616
#define GNSS_BIN_ID_GLOALMANAC 718   /* message id: cnav glonass decoded almanac */
#define GNSS_BIN_ID_GLOEPHEMERIS 723 /* message id: cnav glonass ephemeris */
#define GNSS_BIN_ID_BESTUTM 726
#define GNSS_BIN_ID_CORRIMUDATAB 812
#define GNSS_BIN_ID_CORRIMUDATASB 813
#define GNSS_BIN_ID_RONGGAN_ODM 911 // 融感(比亚迪)里程计输入
#define GNSS_BIN_ID_HEADING 971
#define GNSS_BIN_ID_RAWSBASFRAME 973  /* message id: oem6 raw sbas frame */
#define GNSS_BIN_ID_RAWCNAVFRAME 1066 /* message id: oem6 raw cnav frame data */
#define GNSS_BIN_ID_GALALMANAC 1120   /* message id: oem6 decoded galileo almanac */
#define GNSS_BIN_ID_GALCLOCK 1121     /* message id: oem6 galileo clockinformation */
#define GNSS_BIN_ID_GALEPHEMERIS 1122 /* message id: oem6 decoded galileo ephemeris */
#define GNSS_BIN_ID_GALIONO 1127      /* message id: oem6 decoded galileo iono corrections */
#define GNSS_BIN_ID_RANGECMP2 1273    /* message id: cnav range compressed */
#define GNSS_BIN_ID_GALINAVEPHEMERIS 1309
#define GNSS_BIN_ID_GALFNAVEPHEMERIS 1310
#define GNSS_BIN_ID_QZSSRAWEPHEM 1330    /* message id: oem6 qzss raw ephemeris */
#define GNSS_BIN_ID_QZSSRAWSUBFRAME 1331 /* message id: oem6 qzss raw subframe */
#define GNSS_BIN_ID_HEADING2 1335
#define GNSS_BIN_ID_QZSSIONUTC 1347     /* message id: oem6 qzss ion/utc parameters */
#define GNSS_BIN_ID_GALFNAVRAWPAGE 1413 /* message id: oem6 raw galileo f/nav paga data */
#define GNSS_BIN_ID_GALINAVRAWWORD 1414 /* message id: oem6 raw galileo i/nav word data */
#define GNSS_BIN_ID_BESTGNSSPOS 1429
#define GNSS_BIN_ID_BESTGNSSVEL 1430
#define GNSS_BIN_ID_RAWIMUXB 1461
#define GNSS_BIN_ID_RAWIMUSXB 1462
#define GNSS_BIN_ID_INSPVAXB 1465
#define GNSS_BIN_ID_BDSEPHEMERIS 1696 /* message id: oem6 decoded bds ephemeris */
#define GNSS_BIN_ID_INSCALSTATUSB 1961
#define GNSS_BIN_ID_INSSTDEVB 2051
#define GNSS_BIN_ID_RAWDMI 2262
#define GNSS_BIN_ID_SOLINFO 6666
#define GNSS_BIN_ID_INTEGRITYINFO 10090
#define GNSS_BIN_ID_AGRICB 11276

typedef enum _INSPVAA_INDEX
{
    INSPVAA_INDEX_CMD = 0,
    INSPVAA_INDEX_count,       // 1	count	秒内10ms计数值	Uint8
    INSPVAA_INDEX_gpstime,     // 2	gpstime	gps时间（惯导时）	FP64
    INSPVAA_INDEX_ins_status,  // 3	ins_status	组合解算状态	Uint8
    INSPVAA_INDEX_ins_Lat,     // 4	ins_Lat	组合纬度(度)	FP64
    INSPVAA_INDEX_ins_Lon,     // 5	ins_Lon	组合经度(度)	FP64
    INSPVAA_INDEX_ins_Hgt,     // 6	ins_Hgt	组合高度(m)	FP32
    INSPVAA_INDEX_ve,          // 7	ve	组合东向速度（m/s）	FP32
    INSPVAA_INDEX_vn,          // 8	vn	组合北向速度（m/s）	FP32
    INSPVAA_INDEX_vu,          // 9	vu	组合天向速度（m/s）	FP32
    INSPVAA_INDEX_pitch,       // 10	pitch	组合俯仰角(度)	FP32
    INSPVAA_INDEX_roll,        // 11	roll	组合横滚(度)	FP32
    INSPVAA_INDEX_head,        // 12	head	组合航向角(度)	FP32
    INSPVAA_INDEX_gnss_status, // 13	gnss_status	卫导定位状态	Uint8
    INSPVAA_INDEX_headstatus,  // 14	headstatus	卫导定向状态	Uint8
    INSPVAA_INDEX_Baseline,    // 15	Baseline	基线长度	FP32
    INSPVAA_INDEX_gnss_Lat,    // 16	gnss_Lat	卫导纬度(度)	FP64
    INSPVAA_INDEX_gnss_Lon,    // 17	gnss_Lon	卫导经度(度)	FP64
    INSPVAA_INDEX_gnss_Hgt,    // 18	gnss_Hgt	卫导高度(m)	FP32
    INSPVAA_INDEX_gnss_normv,  // 19	gnss_normv	卫导合速度（m/s）	FP32
    INSPVAA_INDEX_Hdop,        // 20	Hdop	卫导水平精度因子	FP32
    INSPVAA_INDEX_gnss_satM,   // 21	gnss_satM	主天线卫星数（个）	Uint8
    INSPVAA_INDEX_gnss_satS,   // 22	gnss_satS	从天线卫星数（个）	Uint8
    INSPVAA_INDEX_odov,        // 23	odov	里程计速度（m/s）	FP32
    INSPVAA_INDEX_odoflag,     // 24	odoflag	里程计标志（1-里程计速度有效 0-无里程计速度或速度无效）	Uint8
    INSPVAA_INDEX_gyrox,       // 25	gyrox	陀螺仪X轴(°/s)	FP32
    INSPVAA_INDEX_gyroy,       // 26	gyroy	陀螺仪Y轴(°/s)	FP32
    INSPVAA_INDEX_gyroz,       // 27	gyroz	陀螺仪Z轴(°/s)	FP32
    INSPVAA_INDEX_accex,       // 28	accex	加计X轴(m/s^2)	FP32
    INSPVAA_INDEX_accey,       // 29	accey	加计Y轴(m/s^2)	FP32
    INSPVAA_INDEX_accez,       // 30	accez	加计Z轴(m/s^2)	FP32
    INSPVAA_INDEX_accstdval,   // 31	accstdval	加计方差	FP32
    INSPVAA_INDEX_MAX_NUM,
} eINSPVAA_INDEX;

typedef enum _INSPOSA_INDEX
{
    INSPOSA_INDEX_CMD = 0,
    INSPOSA_INDEX_count,      // 1	count	秒内10ms计数值	Uint8
    INSPOSA_INDEX_gpstime,    // 2	gpstime	gps时间（惯导时）	FP64
    INSPOSA_INDEX_ins_status, // 3	ins_status	组合解算状态	Uint8
    INSPOSA_INDEX_ins_Lat,    // 4	ins_Lat	组合纬度(度)	FP64
    INSPOSA_INDEX_ins_Lon,    // 5	ins_Lon	组合经度(度)	FP64
    INSPOSA_INDEX_ins_Hgt,    // 6	ins_Hgt	组合高度(m)	FP32
    INSPOSA_INDEX_ve,         // 7	ve	组合东向速度（m/s）	FP32
    INSPOSA_INDEX_vn,         // 8	vn	组合北向速度（m/s）	FP32
    INSPOSA_INDEX_vu,         // 9	vu	组合天向速度（m/s）	FP32
    INSPOSA_INDEX_pitch,      // 10	pitch	组合俯仰角(度)	FP32
    INSPOSA_INDEX_roll,       // 11	roll	组合横滚(度)	FP32
    INSPOSA_INDEX_head,       // 12	head	组合航向角(度)	FP32
    INSPOSA_INDEX_MAX_NUM,
} eINSPOSA_INDEX;

typedef enum _INSIMUA_INDEX
{
    INSIMUA_INDEX_CMD = 0,
    INSIMUA_INDEX_count,     // 1	count	秒内10ms计数值	Uint8
    INSIMUA_INDEX_gpstime,   // 2	gpstime	gps时间（惯导时）	FP64
    INSIMUA_INDEX_gyrox,     // 3	gyrox	陀螺仪X轴(°/s)	FP32
    INSIMUA_INDEX_gyroy,     // 4	gyroy	陀螺仪Y轴(°/s)	FP32
    INSIMUA_INDEX_gyroz,     // 5	gyroz	陀螺仪Z轴(°/s)	FP32
    INSIMUA_INDEX_accex,     // 6	accex	加计X轴(m/s^2)	FP32
    INSIMUA_INDEX_accey,     // 7	accey	加计Y轴(m/s^2)	FP32
    INSIMUA_INDEX_accez,     // 8	accez	加计Z轴(m/s^2)	FP32
    INSIMUA_INDEX_accstdval, // 9	accstdval	加计方差	FP32
    INSIMUA_INDEX_MAX_NUM,
} eINSIMUA_INDEX;

typedef enum _POSDATAA_INDEX
{
    POSDATAA_INDEX_CMD = 0,
    POSDATAA_INDEX_timeSystem,
    POSDATAA_INDEX_weeks,
    POSDATAA_INDEX_weekSec,
    POSDATAA_INDEX_LeapSecond,
    POSDATAA_INDEX_SysStatus,
    POSDATAA_INDEX_posType,
    POSDATAA_INDEX_headType,
    POSDATAA_INDEX_lat,
    POSDATAA_INDEX_lon,
    POSDATAA_INDEX_hgt,
    POSDATAA_INDEX_ve,
    POSDATAA_INDEX_vn,
    POSDATAA_INDEX_vu,
    POSDATAA_INDEX_pitch,
    POSDATAA_INDEX_roll,
    POSDATAA_INDEX_head,
    POSDATAA_INDEX_sta_lat,
    POSDATAA_INDEX_sta_lon,
    POSDATAA_INDEX_sta_hgt,
    POSDATAA_INDEX_sta_ve,
    POSDATAA_INDEX_sta_vn,
    POSDATAA_INDEX_sta_vu,
    POSDATAA_INDEX_sta_pitch,
    POSDATAA_INDEX_sta_roll,
    POSDATAA_INDEX_sta_head,
    POSDATAA_INDEX_gnss_satM,
    POSDATAA_INDEX_gnss_satS,
    POSDATAA_INDEX_diff,
    POSDATAA_INDEX_odoflag,
    POSDATAA_INDEX_gear,
    POSDATAA_INDEX_flWheel,
    POSDATAA_INDEX_frWheel,
    POSDATAA_INDEX_blWheel,
    POSDATAA_INDEX_brWheel,
    POSDATAA_INDEX_MAX_NUM,
} ePOSDATAA_INDEX;

typedef enum _SATNAVA_INDEX
{
    SATNAVA_INDEX_CMD = 0,
    SATNAVA_INDEX_timeSystem,
    SATNAVA_INDEX_weeks,
    SATNAVA_INDEX_weekSec,
    SATNAVA_INDEX_LeapSecond,
    SATNAVA_INDEX_posState,
    SATNAVA_INDEX_aziState,
    SATNAVA_INDEX_lat,
    SATNAVA_INDEX_lon,
    SATNAVA_INDEX_hgt,
    SATNAVA_INDEX_ve,
    SATNAVA_INDEX_vn,
    SATNAVA_INDEX_vu,
    SATNAVA_INDEX_pitch,
    SATNAVA_INDEX_roll,
    SATNAVA_INDEX_head,
    SATNAVA_INDEX_sta_lat,
    SATNAVA_INDEX_sta_lon,
    SATNAVA_INDEX_sta_hgt,
    SATNAVA_INDEX_sta_ve,
    SATNAVA_INDEX_sta_vn,
    SATNAVA_INDEX_sta_vu,
    SATNAVA_INDEX_sta_pitch,
    SATNAVA_INDEX_sta_roll,
    SATNAVA_INDEX_sta_head,
    SATNAVA_INDEX_gnss_satM,
    SATNAVA_INDEX_gnss_satS,
    SATNAVA_INDEX_diff,
    SATNAVA_INDEX_MAX_NUM,
} eSATNAVA_INDEX;

typedef enum _INSNAVA_INDEX
{
    INSNAVA_INDEX_CMD = 0,
    INSNAVA_INDEX_timeSystem,
    INSNAVA_INDEX_weeks,
    INSNAVA_INDEX_weekSec,
    INSNAVA_INDEX_LeapSecond,
    INSNAVA_INDEX_insState,
    INSNAVA_INDEX_posState,
    INSNAVA_INDEX_lat,
    INSNAVA_INDEX_lon,
    INSNAVA_INDEX_hgt,
    INSNAVA_INDEX_ve,
    INSNAVA_INDEX_vn,
    INSNAVA_INDEX_vu,
    INSNAVA_INDEX_pitch,
    INSNAVA_INDEX_roll,
    INSNAVA_INDEX_head,
    INSNAVA_INDEX_sta_lat,
    INSNAVA_INDEX_sta_lon,
    INSNAVA_INDEX_sta_hgt,
    INSNAVA_INDEX_sta_ve,
    INSNAVA_INDEX_sta_vn,
    INSNAVA_INDEX_sta_vu,
    INSNAVA_INDEX_sta_pitch,
    INSNAVA_INDEX_sta_roll,
    INSNAVA_INDEX_sta_head,
    INSNAVA_INDEX_MAX_NUM,
} eINSNAVA_INDEX;

typedef enum _IMUDATAA_INDEX
{
    IMUDATAA_INDEX_CMD = 0,
    IMUDATAA_INDEX_timeSystem,
    IMUDATAA_INDEX_weeks,
    IMUDATAA_INDEX_weekSec,
    IMUDATAA_INDEX_LeapSecond,
    IMUDATAA_INDEX_accex,
    IMUDATAA_INDEX_accey,
    IMUDATAA_INDEX_accez,
    IMUDATAA_INDEX_gyrox,
    IMUDATAA_INDEX_gyroy,
    IMUDATAA_INDEX_gyroz,
    IMUDATAA_INDEX_magx,
    IMUDATAA_INDEX_magy,
    IMUDATAA_INDEX_magz,
    IMUDATAA_INDEX_MAX_NUM,
} eIMUDATAA_INDEX;

static void CSHG_POSDATA_Init(tPOSDATAData *io_Data);
static void CSHG_IMUDATA_Init(tIMUDATAData *io_Data);
static void CSHG_SATNAV_Init(tSATNAVData *io_Data);
static void CSHG_INSNAV_Init(tINSNAVData *io_Data);

static void CSHG_GetValue(const eCSHG_CMD i_eCmd, const uint32_t i_index, const char *i_pbInBuf, void *o_Data);
static void CSHG_INSPVA_GetValue(const uint32_t i_index, const char *i_pbInBuf, tINSPVAData *o_Data);
static void CSHG_IMUDATA_GetValue(const uint32_t i_index, const char *i_pbInBuf, tIMUDATAData *o_Data);
static void CSHG_SATNAV_GetValue(const uint32_t i_index, const char *i_pbInBuf, tSATNAVData *o_Data);
static void CSHG_INSNAV_GetValue(const uint32_t i_index, const char *i_pbInBuf, tINSNAVData *o_Data);

// POSDATAA IMUDATAA ODMDATAA INSPVAA INSIMUA INSPOSA CPP CPD PKG等命令 数据处理
int CSHG_GNSS_ASCII_Process(const MsgSource i_TpSource, uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans)
{
    eCSHG_CMD cmd = CSHG_CMD_UNKNOWN;
    i_pubInBuf[i_udwInBufLen - 9] = 0; // 截断字符串，方便后续字符串操作
    char *startPtr = (char *)i_pubInBuf + 1;
    char *curPtr = strchr(startPtr, ',');
    uint32_t no = 0;
    void *p = NULL;
    if (curPtr == NULL)
    {
        // 命令异常不透传不处理
        i_pubInBuf[i_udwInBufLen - 9] = '*'; // 恢复
        *o_pbNeedTrans = false;
        return (int)i_udwInBufLen;
    }
    CSHG_GNSS_LOG log = CSHG_GNSS_CMD_GET_LOG_FROM_NAME(startPtr);
    if (log < (CSHG_GNSS_LOG)0 ||
        log >= CSHG_GNSS_LOG_NUM)
    {
        // 不支持的命令不透传不处理
        i_pubInBuf[i_udwInBufLen - 9] = '*'; // 恢复
        *o_pbNeedTrans = false;
        return (int)i_udwInBufLen;
    }

    switch (log)
    {
    default:
        // 此部分命令不需处理数据, 减少计算量, 只透传
        cmd = CSHG_CMD_UNKNOWN;
        break;
    case CSHG_GNSS_LOG_INDEX_INSPVAA:
        if (CSHG_INSPVAA_IsNeed())
        {
            p = malloc(sizeof(tINSPVAData));
            cmd = CSHG_CMD_INSPVAA;
        }
        break;
    case CSHG_GNSS_LOG_INDEX_IMUDATAA:
        if (CSHG_IMUDATAA_IsNeed())
        {
            p = malloc(sizeof(tIMUDATAData));
            if (p != NULL)
            {
                CSHG_IMUDATA_Init((tIMUDATAData *)p);
                cmd = CSHG_CMD_IMUDATAA;
            }
        }
        break;
    case CSHG_GNSS_LOG_INDEX_SATNAVA:
        if (CSHG_SATNAVA_IsNeed())
        {
            p = malloc(sizeof(tSATNAVData));
            if (p != NULL)
            {
                CSHG_SATNAV_Init((tSATNAVData *)p);
                cmd = CSHG_CMD_SATNAVA;
            }
        }
        break;
    case CSHG_GNSS_LOG_INDEX_INSNAVA:
        if (CSHG_INSNAVA_IsNeed())
        {
            p = malloc(sizeof(tINSNAVData));
            if (p != NULL)
            {
                CSHG_INSNAV_Init((tINSNAVData *)p);
                cmd = CSHG_CMD_INSNAVA;
            }
        }
        break;
    }

    if (p != NULL) // 需要处理数据, 否则直接透传
    {
        no++;
        startPtr = curPtr + 1;
        curPtr = strchr(startPtr, ',');
        // 取数据
        while (curPtr != NULL)
        {
            *curPtr = 0; // 替换 ','并截断字符串
            if (strlen(startPtr) > 0)
            {
                CSHG_GetValue(cmd, no, startPtr, p);
            }
            no++;
            *curPtr = ','; // 恢复
            startPtr = curPtr + 1;
            curPtr = strchr(startPtr, ',');
        }
        if (strlen(startPtr) > 0)
        {
            CSHG_GetValue(cmd, no++, startPtr, p);
        }
        // 数据取完，最终赋值
        switch (cmd)
        {
        case CSHG_CMD_INSPVAA:
            break;
        case CSHG_CMD_IMUDATAA:
            break;
        case CSHG_CMD_SATNAVA:
            break;
        case CSHG_CMD_INSNAVA:
            break;
        default:
            break;
        }
    }
    if (p != NULL)
        free(p);
    i_pubInBuf[i_udwInBufLen - 9] = '*'; // 恢复

    // 透传
    *o_pbNeedTrans = true;
    return (int)i_udwInBufLen;
}

// HEADINGA等兼容诺瓦泰/北云 ASCII 命令 数据处理
int CSHG_GNSS_ASCII2_Process(const MsgSource i_TpSource, uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans)
{
    eCSHG_CMD cmd = CSHG_CMD_UNKNOWN;
    i_pubInBuf[i_udwInBufLen - 11] = 0; // 截断字符串，方便后续字符串操作
    char *startPtr = (char *)i_pubInBuf + 1;
    char *curPtr = strchr(startPtr, ',');
    uint32_t no = 0;
    void *p = NULL;
    if (curPtr == NULL)
    {
        // 命令异常不透传不处理
        i_pubInBuf[i_udwInBufLen - 11] = '*'; // 恢复
        *o_pbNeedTrans = false;
        return (int)i_udwInBufLen;
    }
    CSHG_GNSS_LOG log = CSHG_GNSS_CMD_GET_LOG_FROM_NAME(startPtr);
    if (log < (CSHG_GNSS_LOG)0 ||
        log >= CSHG_GNSS_LOG_NUM)
    {
        // 不支持的命令不透传不处理
        i_pubInBuf[i_udwInBufLen - 11] = '*'; // 恢复
        *o_pbNeedTrans = false;
        return (int)i_udwInBufLen;
    }
    i_pubInBuf[i_udwInBufLen - 11] = '*'; // 恢复
    // 透传
    *o_pbNeedTrans = true;
    return (int)i_udwInBufLen;
}

// INSPVAB 等命令 数据处理
int CSHG_GNSS_BIN_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans)
{
    if (i_TpSource == MsgSource_UART_GNSS) // 只处理从gnss模块接收的命令
    {
        // 取帧长
        uint16_t wLen = (uint16_t)i_pubInBuf[6] + (uint16_t)(i_pubInBuf[7] << 8);
        // 取命令类型
        uint16_t CMD = (uint16_t)i_pubInBuf[4] + (uint16_t)(i_pubInBuf[5] << 8);
        int offset = 8;
        // 命令处理
        switch (CMD)
        {
        default:
            // 未知协议或不支持配置的协议,不透传不处理
            *o_pbNeedTrans = false;
            return (int)i_udwInBufLen;
            break;
        case INSPVAB_CMD: // INSPVA
            break;
        }
        // 需要处理
        if (CMD == INSPVAB_CMD && CSHG_INSPVAB_IsNeed())
        {
            tINSPVAData tempINSPVAData;
            memset(&tempINSPVAData, 0, sizeof(tINSPVAData));
            // count10ms	10ms计数值	Uint8  1
            tempINSPVAData.count10ms = i_pubInBuf[offset];
            offset += 1;
            // gpstime	gps时间（惯导时）	FP64  8
            memcpy(&tempINSPVAData.gpstime, &i_pubInBuf[offset], 8);
            offset += 8;
            // ins_status	组合解算状态	Uint8  1
            tempINSPVAData.ins_status = i_pubInBuf[offset];
            offset += 1;
            // ins_Lat	组合纬度(度)	FP64  8
            memcpy(&tempINSPVAData.ins_Lat, &i_pubInBuf[offset], 8);
            offset += 8;
            // ins_Lon	组合经度(度)	FP64  8
            memcpy(&tempINSPVAData.ins_Lon, &i_pubInBuf[offset], 8);
            offset += 8;
            // ins_Hgt	组合高度(m)	FP32  4
            memcpy(&tempINSPVAData.ins_Hgt, &i_pubInBuf[offset], 4);
            offset += 4;
            // ve	组合东向速度（m/s）	FP32  4
            memcpy(&tempINSPVAData.ve, &i_pubInBuf[offset], 4);
            offset += 4;
            // vn	组合北向速度（m/s）	FP32  4
            memcpy(&tempINSPVAData.vn, &i_pubInBuf[offset], 4);
            offset += 4;
            // vu	组合天向速度（m/s）	FP32  4
            memcpy(&tempINSPVAData.vu, &i_pubInBuf[offset], 4);
            offset += 4;
            // pitch	组合俯仰角(度)	FP32  4
            memcpy(&tempINSPVAData.pitch, &i_pubInBuf[offset], 4);
            offset += 4;
            // roll	组合横滚(度)	FP32  4
            memcpy(&tempINSPVAData.roll, &i_pubInBuf[offset], 4);
            offset += 4;
            // head	组合航向角(度)	FP32  4
            memcpy(&tempINSPVAData.head, &i_pubInBuf[offset], 4);
            offset += 4;
            // gnss_status	卫导定位状态	Uint8  1
            tempINSPVAData.gnss_status = i_pubInBuf[offset];
            offset += 1;
            // headstatus	卫导定向状态	Uint8  1
            tempINSPVAData.headstatus = i_pubInBuf[offset];
            offset += 1;
            // Baseline	基线长度	FP32  4
            memcpy(&tempINSPVAData.Baseline, &i_pubInBuf[offset], 4);
            offset += 4;
            // gnss_Lat	卫导纬度(度)	FP64  8
            memcpy(&tempINSPVAData.gnss_Lat, &i_pubInBuf[offset], 8);
            offset += 8;
            // tempINSPVAData.gnss_Lat *= Mult180_DivPI; // 弧度转度
            // gnss_Lon	卫导经度(度)	FP64  8
            memcpy(&tempINSPVAData.gnss_Lon, &i_pubInBuf[offset], 8);
            offset += 8;
            // tempINSPVAData.gnss_Lon *= Mult180_DivPI; // 弧度转度
            // gnss_Hgt	卫导高度(m)	FP32  4
            memcpy(&tempINSPVAData.gnss_Hgt, &i_pubInBuf[offset], 4);
            offset += 4;
            // gnss_normv	卫导合速度（m/s）	FP32  4
            memcpy(&tempINSPVAData.gnss_normv, &i_pubInBuf[offset], 4);
            offset += 4;
            // Hdop	卫导水平精度因子	FP32  4
            memcpy(&tempINSPVAData.Hdop, &i_pubInBuf[offset], 4);
            offset += 4;
            // gnss_satM	主天线卫星数（个）	Uint8  1
            tempINSPVAData.gnss_satM = i_pubInBuf[offset];
            offset += 1;
            // gnss_satS	从天线卫星数（个）	Uint8  1
            tempINSPVAData.gnss_satS = i_pubInBuf[offset];
            offset += 1;
            // odov	里程计速度（m/s）	FP32  4
            memcpy(&tempINSPVAData.odov, &i_pubInBuf[offset], 4);
            offset += 4;
            // odoflag	里程计标志	Uint8  1
            tempINSPVAData.odoflag = i_pubInBuf[offset];
            offset += 1;
            // gyrox	陀螺仪X轴(°/s)	FP32  4
            memcpy(&tempINSPVAData.gyrox, &i_pubInBuf[offset], 4);
            offset += 4;
            // gyroy	陀螺仪Y轴(°/s)	FP32  4
            memcpy(&tempINSPVAData.gyroy, &i_pubInBuf[offset], 4);
            offset += 4;
            // gyroz	陀螺仪Z轴(°/s)	FP32  4
            memcpy(&tempINSPVAData.gyroz, &i_pubInBuf[offset], 4);
            offset += 4;
            // accex	加计X轴(m/s^2)	FP32  4
            memcpy(&tempINSPVAData.accex, &i_pubInBuf[offset], 4);
            offset += 4;
            // accey	加计Y轴(m/s^2)	FP32  4
            memcpy(&tempINSPVAData.accey, &i_pubInBuf[offset], 4);
            offset += 4;
            // accez	加计Z轴(m/s^2)	FP32  4
            memcpy(&tempINSPVAData.accez, &i_pubInBuf[offset], 4);
            offset += 4;
            // accstdval	加计方差	FP32  4
            memcpy(&tempINSPVAData.accstdval, &i_pubInBuf[offset], 4);
            offset += 4;

            if (g_Config_param.enable_read_inspva == 1)
            {
                csjw_msgs::inspva inspva_msg;
                inspva_msg.gpstime = tempINSPVAData.gpstime;         // 2	gpstime	gps时间（惯导时）	FP64
                inspva_msg.ins_status = tempINSPVAData.ins_status;   // 3	ins_status	组合解算状态	Uint8
                inspva_msg.ins_lat = tempINSPVAData.ins_Lat;         // 4	ins_Lat	组合纬度(度)	FP64
                inspva_msg.ins_lon = tempINSPVAData.ins_Lon;         // 5	ins_Lon	组合经度(度)	FP64
                inspva_msg.ins_hgt = tempINSPVAData.ins_Hgt;         // 6	ins_Hgt	组合高度(m)	FP32
                inspva_msg.vele = tempINSPVAData.ve;                 // 7	ve	组合东向速度（m/s）	FP32
                inspva_msg.veln = tempINSPVAData.vn;                 // 8	vn	组合北向速度（m/s）	FP32
                inspva_msg.velu = tempINSPVAData.vu;                 // 9	vu	组合天向速度（m/s）	FP32
                inspva_msg.roll = tempINSPVAData.pitch;              // 10	pitch	组合俯仰角(度)	FP32
                inspva_msg.pitch = tempINSPVAData.roll;              // 11	roll	组合横滚(度)	FP32
                inspva_msg.yaw = tempINSPVAData.head;                // 12	head	组合航向角(度)	FP32
                inspva_msg.gnss_status = tempINSPVAData.gnss_status; // 13	gnss_status	卫导定位状态	Uint8
                inspva_msg.head_status = tempINSPVAData.headstatus;  // 14	headstatus	卫导定向状态	Uint8
                inspva_msg.base_line = tempINSPVAData.Baseline;      // 15	Baseline	基线长度	FP32
                inspva_msg.gnss_lat = tempINSPVAData.gnss_Lat;       // 16	gnss_Lat	卫导纬度(度)	FP64
                inspva_msg.gnss_lon = tempINSPVAData.gnss_Lon;       // 17	gnss_Lon	卫导经度(度)	FP64
                inspva_msg.gnss_hgt = tempINSPVAData.gnss_Hgt;       // 18	gnss_Hgt	卫导高度(m)	FP32
                inspva_msg.gnss_normv = tempINSPVAData.gnss_normv;   // 19	gnss_normv	卫导合速度（m/s）	FP32
                inspva_msg.hdop = tempINSPVAData.Hdop;               // 20	Hdop	卫导水平精度因子	FP32
                inspva_msg.satM = tempINSPVAData.gnss_satM;          // 21	gnss_satM	主天线卫星数（个）	Uint8
                inspva_msg.satS = tempINSPVAData.gnss_satS;          // 22	gnss_satS	从天线卫星数（个）	Uint8
                inspva_msg.odomv = tempINSPVAData.odov;              // 23	odov	里程计速度（m/s）	FP32
                inspva_msg.odomflag = tempINSPVAData.odoflag;        // 24	odoflag	里程计标志（1-里程计速度有效 0-无里程计速度或速度无效）	Uint8
                inspva_msg.gyrox = tempINSPVAData.gyrox;             // 25	gyrox	陀螺仪X轴(°/s)	FP32
                inspva_msg.gyroy = tempINSPVAData.gyroy;             // 26	gyroy	陀螺仪Y轴(°/s)	FP32
                inspva_msg.gyroz = tempINSPVAData.gyroz;             // 27	gyroz	陀螺仪Z轴(°/s)	FP32
                inspva_msg.accx = tempINSPVAData.accex;              // 28	accex	加计X轴(m/s^2)	FP32
                inspva_msg.accy = tempINSPVAData.accey;              // 29	accey	加计Y轴(m/s^2)	FP32
                inspva_msg.accz = tempINSPVAData.accez;              // 30	accez	加计Z轴(m/s^2)	FP32
                inspva_msg.accstdval = tempINSPVAData.accstdval;     // 31	accstdval	加计方差	FP32

                g_pub_inspva.publish(inspva_msg);
            }
            if (g_Config_param.enable_imu == 2 && g_GnssWeekNum > 0 && g_GnssWeekNum != 0xFFFFFFFF)
            {
                sensor_msgs::Imu imu_msgs;
                imu_msgs.header.frame_id = "gnss";
                imu_msgs.header.stamp.sec = tempINSPVAData.gpstime - tempINSPVAData.count10ms * 0.01 + g_GnssWeekNum * 604800;
                imu_msgs.header.stamp.nsec = tempINSPVAData.count10ms * 1e7;
                imu_msgs.orientation_covariance.assign(0);
                imu_msgs.angular_velocity_covariance.assign(0);
                imu_msgs.linear_acceleration_covariance.assign(0);
                imu_msgs.orientation.x = 0;
                imu_msgs.orientation.y = 0;
                imu_msgs.orientation.z = 0;
                imu_msgs.orientation.w = 1;

                imu_msgs.angular_velocity.x = tempINSPVAData.gyrox * MultPI_Div180;
                imu_msgs.angular_velocity.y = tempINSPVAData.gyroy * MultPI_Div180;
                imu_msgs.angular_velocity.z = tempINSPVAData.gyroz * MultPI_Div180;

                imu_msgs.linear_acceleration.x = tempINSPVAData.accex;
                imu_msgs.linear_acceleration.y = tempINSPVAData.accey;
                imu_msgs.linear_acceleration.z = tempINSPVAData.accez;
                g_pub_imu.publish(imu_msgs);
            }
            if (g_Config_param.enable_master_sat_num == 3)
            {
                std_msgs::UInt8 master_sat_num_msg;
                master_sat_num_msg.data = tempINSPVAData.gnss_satM;
                g_pub_MasterSatNum.publish(master_sat_num_msg);
            }
            if (g_Config_param.enable_slaver_sat_num == 3)
            {
                std_msgs::UInt8 slaver_sat_num_msg;
                slaver_sat_num_msg.data = tempINSPVAData.gnss_satS;
                g_pub_SlaverSatNum.publish(slaver_sat_num_msg);
            }
        }

        *o_pbNeedTrans = true;
        return (int)i_udwInBufLen;
    }
    *o_pbNeedTrans = false;
    return (int)i_udwInBufLen;
}

// POSDATAB IMUDATAB等命令 数据处理
int CSHG_GNSS_BIN2_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans)
{
    // 取帧长
    uint16_t wLen = (uint16_t)i_pubInBuf[10] + (uint16_t)(i_pubInBuf[11] << 8);
    // 取命令类型
    uint16_t CMD = (uint16_t)i_pubInBuf[8] + (uint16_t)(i_pubInBuf[9] << 8);
    int offset = 12;
    uint8_t bitOffset = 0;
    uint8_t nextBitOffset;
    uint8_t tempU8;
    uint16_t tempU16;
    uint32_t tempU32;
    uint64_t tempU64;
    //            int8_t tempS8;
    int16_t tempS16;
    int32_t tempS32;
    int64_t tempS64;
    // 命令处理
    switch (CMD)
    {
    default:
        // 未知协议或不支持配置的协议,不透传不处理
        *o_pbNeedTrans = false;
        return (int)i_udwInBufLen;
        break;
    case IMUDATAB_CMD:
        break;
    case SATNAVB_CMD:
        break;
    case INSNAVB_CMD:
        break;
    }
    if (CMD == IMUDATAB_CMD && CSHG_IMUDATAB_IsNeed())
    {
        tIMUDATAData tempIMUDATAData;
        CSHG_IMUDATA_Init(&tempIMUDATAData);

        nextBitOffset = bitOffset + 3;
        if (ConvMSBBitUInt2_UInt9(&i_pubInBuf[offset], nextBitOffset, 3, &tempU16) == 0)
            tempIMUDATAData.m_iTimeSystem = (uint8_t)tempU16;
        offset += nextBitOffset >> 3;
        bitOffset = nextBitOffset & 0x7;

        nextBitOffset = bitOffset + 13;
        if (ConvMSBBitUInt10_UInt17(&i_pubInBuf[offset], nextBitOffset, 13, &tempU32) == 0)
        {
            tempIMUDATAData.m_iWeekNum = (uint16_t)tempU32;
            g_GnssWeekNum = tempU32;
        }
        offset += nextBitOffset >> 3;
        bitOffset = nextBitOffset & 0x7;

        nextBitOffset = bitOffset + 32;
        if (ConvMSBBitUInt26_UInt33(&i_pubInBuf[offset], nextBitOffset, 32, &tempU64) == 0)
            tempIMUDATAData.m_dSecInWeek = (double)tempU64 * (double)0.001;
        else
            tempIMUDATAData.m_dSecInWeek = nan("");
        offset += nextBitOffset >> 3;
        bitOffset = nextBitOffset & 0x7;

        if ((tempIMUDATAData.m_iTimeSystem == 1 || tempIMUDATAData.m_iTimeSystem == 2) &&
            tempIMUDATAData.m_iWeekNum > 0 && tempIMUDATAData.m_iWeekNum != 0xFFFF &&
            // *(uint32_t *)&(tempIMUDATAData.m_dSecInWeek) != 0xFFFFFFFF)
            !isnan(tempIMUDATAData.m_dSecInWeek))
        {
            // 时间有效再处理
            nextBitOffset = bitOffset + 8;
            if (ConvMSBBitInt2_Int9(&i_pubInBuf[offset], nextBitOffset, 8, &tempS16) == 0)
                tempIMUDATAData.m_iLeapSecond = (int8_t)tempS16;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;
            nextBitOffset = bitOffset + 17;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 17, &tempS32) == 0)
                tempIMUDATAData.m_fAccelX = (float)(tempS32 * 0.001);
            else
                tempIMUDATAData.m_fAccelX = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 17;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 17, &tempS32) == 0)
                tempIMUDATAData.m_fAccelY = (float)(tempS32 * 0.001);
            else
                tempIMUDATAData.m_fAccelY = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 17;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 17, &tempS32) == 0)
                tempIMUDATAData.m_fAccelZ = (float)(tempS32 * 0.001);
            else
                tempIMUDATAData.m_fAccelZ = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 20;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 20, &tempS32) == 0)
                tempIMUDATAData.m_fGroX = (float)(tempS32 * 0.001);
            else
                tempIMUDATAData.m_fGroX = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 20;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 20, &tempS32) == 0)
                tempIMUDATAData.m_fGroY = (float)(tempS32 * 0.001);
            else
                tempIMUDATAData.m_fGroY = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 20;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 20, &tempS32) == 0)
                tempIMUDATAData.m_fGroZ = (float)(tempS32 * 0.001);
            else
                tempIMUDATAData.m_fGroZ = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 19;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 19, &tempS32) == 0)
                tempIMUDATAData.m_fMagX = (float)(tempS32 * 0.00001);
            else
                tempIMUDATAData.m_fMagX = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 19;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 19, &tempS32) == 0)
                tempIMUDATAData.m_fMagY = (float)(tempS32 * 0.00001);
            else
                tempIMUDATAData.m_fMagY = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 19;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 19, &tempS32) == 0)
                tempIMUDATAData.m_fMagZ = (float)(tempS32 * 0.00001);
            else
                tempIMUDATAData.m_fMagZ = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            double utcTime[6] = {0};
            gtime_t time;
            if (tempIMUDATAData.m_iTimeSystem == 1)
            {
                // 北斗系统
                bdst2time((int)tempIMUDATAData.m_iWeekNum, tempIMUDATAData.m_dSecInWeek, &time); // 周内秒+周数 转 GPS日期时间
            }
            else if (tempIMUDATAData.m_iTimeSystem == 2)
            {
                // GPS系统
                gpst2time((int)tempIMUDATAData.m_iWeekNum, tempIMUDATAData.m_dSecInWeek, &time); // 周内秒+周数 转 GPS日期时间
            }
            gtime_t time_Leap;
            ros::Time utc_time;
            gpst2utc(time, tempIMUDATAData.m_iLeapSecond, &time_Leap); // + GPS闰秒 (GPS时间和UTC时间差)
            utc_time.sec = time_Leap.time;
            utc_time.nsec = time_Leap.sec * 1e9;

            if (g_Config_param.enable_read_imudata == 1)
            {
                csjw_msgs::imudata imudata_msg;
                imudata_msg.header.header.frame_id = "gnss";
                imudata_msg.header.header.stamp = utc_time;

                imudata_msg.header.time_system = tempIMUDATAData.m_iTimeSystem;
                imudata_msg.header.week_num = tempIMUDATAData.m_iWeekNum;
                imudata_msg.header.secs_in_week = tempIMUDATAData.m_dSecInWeek;
                imudata_msg.header.leap_sec = tempIMUDATAData.m_iLeapSecond;

                imudata_msg.accel[0] = tempIMUDATAData.m_fAccelX;
                imudata_msg.accel[1] = tempIMUDATAData.m_fAccelY;
                imudata_msg.accel[2] = tempIMUDATAData.m_fAccelZ;
                imudata_msg.gyro[0] = tempIMUDATAData.m_fGroX;
                imudata_msg.gyro[1] = tempIMUDATAData.m_fGroY;
                imudata_msg.gyro[2] = tempIMUDATAData.m_fGroZ;
                imudata_msg.mag[0] = tempIMUDATAData.m_fMagX;
                imudata_msg.mag[1] = tempIMUDATAData.m_fMagY;
                imudata_msg.mag[2] = tempIMUDATAData.m_fMagZ;
                g_pub_imudata.publish(imudata_msg);
            }
            if (g_Config_param.enable_imu == 0)
            {
                sensor_msgs::Imu imu_msgs;
                imu_msgs.header.frame_id = "gnss";
                imu_msgs.header.stamp = utc_time;
                imu_msgs.orientation_covariance.assign(0);
                imu_msgs.angular_velocity_covariance.assign(0);
                imu_msgs.linear_acceleration_covariance.assign(0);
                imu_msgs.orientation.x = 0;
                imu_msgs.orientation.y = 0;
                imu_msgs.orientation.z = 0;
                imu_msgs.orientation.w = 1;

                imu_msgs.angular_velocity.x = tempIMUDATAData.m_fGroX * MultPI_Div180;
                imu_msgs.angular_velocity.y = tempIMUDATAData.m_fGroY * MultPI_Div180;
                imu_msgs.angular_velocity.z = tempIMUDATAData.m_fGroZ * MultPI_Div180;

                imu_msgs.linear_acceleration.x = tempIMUDATAData.m_fAccelX;
                imu_msgs.linear_acceleration.y = tempIMUDATAData.m_fAccelY;
                imu_msgs.linear_acceleration.z = tempIMUDATAData.m_fAccelZ;
                g_pub_imu.publish(imu_msgs);
            }
            // 对时
            struct timeval cur_tv;
            if (g_Config_param.enable_set_time > 0 &&
                (gettimeofday(&cur_tv, nullptr) < 0 ||
                 (time_Leap.time >= cur_tv.tv_sec && time_Leap.time - cur_tv.tv_sec >= 2) ||
                 (cur_tv.tv_sec >= time_Leap.time && cur_tv.tv_sec - time_Leap.time >= 2)))
            {
                cur_tv.tv_sec = time_Leap.time;
                cur_tv.tv_usec = time_Leap.sec * 1e6;
                if (settimeofday(&cur_tv, nullptr) < 0)
                {
                    printf("gnss set time : no permission, please use sudo, or \"sudo su\"");
                }
            }
        }
    }
    if (CMD == SATNAVB_CMD && CSHG_SATNAVB_IsNeed())
    {
        tSATNAVData tempSATNAVData;
        CSHG_SATNAV_Init(&tempSATNAVData);

        nextBitOffset = bitOffset + 3;
        if (ConvMSBBitUInt2_UInt9(&i_pubInBuf[offset], nextBitOffset, 3, &tempU16) == 0)
            tempSATNAVData.m_iTimeSystem = (uint8_t)tempU16;
        offset += nextBitOffset >> 3;
        bitOffset = nextBitOffset & 0x7;

        nextBitOffset = bitOffset + 13;
        if (ConvMSBBitUInt10_UInt17(&i_pubInBuf[offset], nextBitOffset, 13, &tempU32) == 0)
        {
            tempSATNAVData.m_iWeekNum = (uint16_t)tempU32;
            g_GnssWeekNum = tempU32;
        }
        offset += nextBitOffset >> 3;
        bitOffset = nextBitOffset & 0x7;

        nextBitOffset = bitOffset + 32;
        if (ConvMSBBitUInt26_UInt33(&i_pubInBuf[offset], nextBitOffset, 32, &tempU64) == 0)
            tempSATNAVData.m_dSecInWeek = (double)tempU64 * (double)0.001;
        else
            tempSATNAVData.m_dSecInWeek = nan("");
        offset += nextBitOffset >> 3;
        bitOffset = nextBitOffset & 0x7;

        if ((tempSATNAVData.m_iTimeSystem == 1 || tempSATNAVData.m_iTimeSystem == 2) &&
            tempSATNAVData.m_iWeekNum > 0 && tempSATNAVData.m_iWeekNum != 0xFFFF &&
            // *(uint32_t *)&(tempSATNAVData.m_dSecInWeek) != 0xFFFFFFFF)
            !isnan(tempSATNAVData.m_dSecInWeek))
        {
            nextBitOffset = bitOffset + 8;
            if (ConvMSBBitInt2_Int9(&i_pubInBuf[offset], nextBitOffset, 8, &tempS16) == 0)
                tempSATNAVData.m_iLeapSecond = (int8_t)tempS16;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 6;
            if (ConvMSBBitUInt2_UInt9(&i_pubInBuf[offset], nextBitOffset, 6, &tempU16) == 0)
                tempSATNAVData.m_iPosState = (uint8_t)tempU16;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 6;
            if (ConvMSBBitUInt2_UInt9(&i_pubInBuf[offset], nextBitOffset, 6, &tempU16) == 0)
                tempSATNAVData.m_iAziState = (uint8_t)tempU16;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 36;
            if (ConvMSBBitInt34_Int41(&i_pubInBuf[offset], nextBitOffset, 36, &tempS64) == 0)
                tempSATNAVData.m_dLatitude = (double)tempS64 * (double)1e-8;
            else
                tempSATNAVData.m_dLatitude = nan("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 36;
            if (ConvMSBBitInt34_Int41(&i_pubInBuf[offset], nextBitOffset, 36, &tempS64) == 0)
                tempSATNAVData.m_dLongitude = (double)tempS64 * (double)1e-8;
            else
                tempSATNAVData.m_dLongitude = nan("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 30;
            if (ConvMSBBitInt26_Int33(&i_pubInBuf[offset], nextBitOffset, 30, &tempS64) == 0)
                tempSATNAVData.m_dAltitude = (double)tempS64 * (double)0.001;
            else
                tempSATNAVData.m_dAltitude = nan("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 16, &tempS32) == 0)
                tempSATNAVData.m_fVelE = (float)(tempS32 * 0.01);
            else
                tempSATNAVData.m_fVelE = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 16, &tempS32) == 0)
                tempSATNAVData.m_fVelN = (float)(tempS32 * 0.01);
            else
                tempSATNAVData.m_fVelN = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 16, &tempS32) == 0)
                tempSATNAVData.m_fVelU = (float)(tempS32 * 0.01);
            else
                tempSATNAVData.m_fVelU = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 16, &tempS32) == 0)
                tempSATNAVData.m_fPitch = (float)(tempS32 * 0.01);
            else
                tempSATNAVData.m_fPitch = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 16, &tempS32) == 0)
                tempSATNAVData.m_fRoll = (float)(tempS32 * 0.01);
            else
                tempSATNAVData.m_fRoll = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitUInt10_UInt17(&i_pubInBuf[offset], nextBitOffset, 16, &tempU32) == 0)
                tempSATNAVData.m_fAzimuth = (float)(tempU32 * 0.01);
            else
                tempSATNAVData.m_fAzimuth = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 18;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 18, &tempS32) == 0)
                tempSATNAVData.m_fStdLat = (float)(tempS32 * 0.001);
            else
                tempSATNAVData.m_fStdLat = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 18;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 18, &tempS32) == 0)
                tempSATNAVData.m_fStdLong = (float)(tempS32 * 0.001);
            else
                tempSATNAVData.m_fStdLong = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 18;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 18, &tempS32) == 0)
                tempSATNAVData.m_fStdAltitude = (float)(tempS32 * 0.001);
            else
                tempSATNAVData.m_fStdAltitude = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempSATNAVData.m_fStdVelE = (float)(tempS32 * 0.01);
            else
                tempSATNAVData.m_fStdVelE = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempSATNAVData.m_fStdVelN = (float)(tempS32 * 0.01);
            else
                tempSATNAVData.m_fStdVelN = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempSATNAVData.m_fStdVelU = (float)(tempS32 * 0.01);
            else
                tempSATNAVData.m_fStdVelU = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempSATNAVData.m_fStdPitch = (float)(tempS32 * 0.01);
            else
                tempSATNAVData.m_fStdPitch = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempSATNAVData.m_fStdRoll = (float)(tempS32 * 0.01);
            else
                tempSATNAVData.m_fStdRoll = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempSATNAVData.m_fStdAzimuth = (float)(tempS32 * 0.01);
            else
                tempSATNAVData.m_fStdAzimuth = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 8;
            if (ConvMSBBitUInt2_UInt9(&i_pubInBuf[offset], nextBitOffset, 8, &tempU16) == 0)
                tempSATNAVData.m_iGnssSatM = (uint8_t)tempU16;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 8;
            if (ConvMSBBitUInt2_UInt9(&i_pubInBuf[offset], nextBitOffset, 8, &tempU16) == 0)
                tempSATNAVData.m_iGnssSatS = (uint8_t)tempU16;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitUInt10_UInt17(&i_pubInBuf[offset], nextBitOffset, 16, &tempU32) == 0)
                tempSATNAVData.m_fDiffAge = (float)(tempU32 * 0.1);
            else
                tempSATNAVData.m_fDiffAge = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            // 保留字节
            nextBitOffset = bitOffset + 32;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            // 保留字节
            nextBitOffset = bitOffset + 32;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            if (isnormal(tempSATNAVData.m_dLatitude) &&
                isnormal(tempSATNAVData.m_dLongitude) &&
                isnormal(tempSATNAVData.m_dAltitude))
            {
                g_last_latitude = tempSATNAVData.m_dLatitude;
                g_last_longitude = tempSATNAVData.m_dLongitude;
                g_last_altitude = tempSATNAVData.m_dAltitude;
            }

            double utcTime[6] = {0};
            gtime_t time;
            if (tempSATNAVData.m_iTimeSystem == 1)
            {
                // 北斗系统
                bdst2time((int)tempSATNAVData.m_iWeekNum, tempSATNAVData.m_dSecInWeek, &time); // 周内秒+周数 转 GPS日期时间
            }
            else if (tempSATNAVData.m_iTimeSystem == 2)
            {
                // GPS系统
                gpst2time((int)tempSATNAVData.m_iWeekNum, tempSATNAVData.m_dSecInWeek, &time); // 周内秒+周数 转 GPS日期时间
            }
            gtime_t time_Leap;
            ros::Time utc_time;
            gpst2utc(time, tempSATNAVData.m_iLeapSecond, &time_Leap); // + GPS闰秒 (GPS时间和UTC时间差)
            utc_time.sec = time_Leap.time;
            utc_time.nsec = time_Leap.sec * 1e9;

            if (g_Config_param.enable_read_satnav > 0)
            {
                csjw_msgs::satnav satnav_msg;
                satnav_msg.header.header.frame_id = "gnss";
                satnav_msg.header.header.stamp = utc_time;
                satnav_msg.header.time_system = tempSATNAVData.m_iTimeSystem;
                satnav_msg.header.week_num = tempSATNAVData.m_iWeekNum;
                satnav_msg.header.secs_in_week = tempSATNAVData.m_dSecInWeek;
                satnav_msg.header.leap_sec = tempSATNAVData.m_iLeapSecond;

                satnav_msg.pos_status = tempSATNAVData.m_iPosState;
                satnav_msg.header_status = tempSATNAVData.m_iAziState;
                satnav_msg.latitude = tempSATNAVData.m_dLatitude;
                satnav_msg.longitude = tempSATNAVData.m_dLongitude;
                satnav_msg.altitude = tempSATNAVData.m_dAltitude;
                satnav_msg.vel_east = tempSATNAVData.m_fVelE;
                satnav_msg.vel_north = tempSATNAVData.m_fVelN;
                satnav_msg.vel_up = tempSATNAVData.m_fVelU;
                satnav_msg.pitch = tempSATNAVData.m_fPitch;
                satnav_msg.roll = tempSATNAVData.m_fRoll;
                satnav_msg.yaw = tempSATNAVData.m_fAzimuth;
                satnav_msg.std_latitude = tempSATNAVData.m_fStdLat;
                satnav_msg.std_longitude = tempSATNAVData.m_fStdLong;
                satnav_msg.std_altitude = tempSATNAVData.m_fStdAltitude;
                satnav_msg.std_vel_east = tempSATNAVData.m_fStdVelE;
                satnav_msg.std_vel_north = tempSATNAVData.m_fStdVelN;
                satnav_msg.std_vel_up = tempSATNAVData.m_fStdVelU;
                satnav_msg.std_pitch = tempSATNAVData.m_fStdPitch;
                satnav_msg.std_roll = tempSATNAVData.m_fStdRoll;
                satnav_msg.std_yaw = tempSATNAVData.m_fStdAzimuth;
                satnav_msg.sat_num_master = tempSATNAVData.m_iGnssSatM;
                satnav_msg.sat_num_slaver = tempSATNAVData.m_iGnssSatS;
                satnav_msg.diff_age = tempSATNAVData.m_fDiffAge;
                g_pub_satnav.publish(satnav_msg);
            }
            if (g_Config_param.enable_nav_sat_fix == 0)
            {
                sensor_msgs::NavSatFix NavSatFix_msg;
                NavSatFix_msg.header.frame_id = "gnss";
                NavSatFix_msg.header.stamp = utc_time;
                NavSatFix_msg.status.status = ((tempSATNAVData.m_iPosState == 4) ? sensor_msgs::NavSatStatus::STATUS_FIX : sensor_msgs::NavSatStatus::STATUS_NO_FIX);
                NavSatFix_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_COMPASS;
                NavSatFix_msg.latitude = tempSATNAVData.m_dLatitude;
                NavSatFix_msg.longitude = tempSATNAVData.m_dLongitude;
                NavSatFix_msg.altitude = tempSATNAVData.m_dAltitude;
                NavSatFix_msg.position_covariance[0] = std::pow(tempSATNAVData.m_fStdLat, 2);
                NavSatFix_msg.position_covariance[4] = std::pow(tempSATNAVData.m_fStdLong, 2);
                NavSatFix_msg.position_covariance[8] = std::pow(tempSATNAVData.m_fStdAltitude, 2);
                NavSatFix_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
                g_pub_nav_sat_fix.publish(NavSatFix_msg);
            }
            // if (g_Config_param.enable_gps_fix == 0)
            // {
            //     gps_common::GPSFix GPSFix_msg;
            // }
            if (g_Config_param.enable_master_sat_num == 0)
            {
                std_msgs::UInt8 master_sat_num_msg;
                master_sat_num_msg.data = tempSATNAVData.m_iGnssSatM;
                g_pub_MasterSatNum.publish(master_sat_num_msg);
            }
            if (g_Config_param.enable_slaver_sat_num == 0)
            {
                std_msgs::UInt8 slaver_sat_num_msg;
                slaver_sat_num_msg.data = tempSATNAVData.m_iGnssSatS;
                g_pub_SlaverSatNum.publish(slaver_sat_num_msg);
            }
            if (g_Config_param.enable_gnss_vel == 0)
            {
                geometry_msgs::TwistStamped gnss_vel_msg;
                gnss_vel_msg.header.frame_id = "gnss";
                gnss_vel_msg.header.stamp = utc_time;
                gnss_vel_msg.twist.linear.x = tempSATNAVData.m_fVelE; // 航速:节/s转m/s
                gnss_vel_msg.twist.linear.y = tempSATNAVData.m_fVelN;
                gnss_vel_msg.twist.linear.z = tempSATNAVData.m_fVelU;
                gnss_vel_msg.twist.angular.x = 0;
                gnss_vel_msg.twist.angular.y = 0;
                gnss_vel_msg.twist.angular.z = 0;
                g_pub_gnss_vel.publish(gnss_vel_msg);
            }
            // 对时
            struct timeval cur_tv;
            if (g_Config_param.enable_set_time > 0 &&
                (gettimeofday(&cur_tv, nullptr) < 0 ||
                 (time_Leap.time >= cur_tv.tv_sec && time_Leap.time - cur_tv.tv_sec >= 2) ||
                 (cur_tv.tv_sec >= time_Leap.time && cur_tv.tv_sec - time_Leap.time >= 2)))
            {
                cur_tv.tv_sec = time_Leap.time;
                cur_tv.tv_usec = time_Leap.sec * 1e6;
                if (settimeofday(&cur_tv, nullptr) < 0)
                {
                    printf("gnss set time : no permission, please use sudo, or \"sudo su\"");
                }
            }
        }
    }
    if (CMD == INSNAVB_CMD && CSHG_INSNAVB_IsNeed())
    {
        tINSNAVData tempINSNAVData;
        CSHG_INSNAV_Init(&tempINSNAVData);

        nextBitOffset = bitOffset + 3;
        if (ConvMSBBitUInt2_UInt9(&i_pubInBuf[offset], nextBitOffset, 3, &tempU16) == 0)
            tempINSNAVData.m_iTimeSystem = (uint8_t)tempU16;
        offset += nextBitOffset >> 3;
        bitOffset = nextBitOffset & 0x7;

        nextBitOffset = bitOffset + 13;
        if (ConvMSBBitUInt10_UInt17(&i_pubInBuf[offset], nextBitOffset, 13, &tempU32) == 0)
        {
            tempINSNAVData.m_iWeekNum = (uint16_t)tempU32;
            g_GnssWeekNum = tempU32;
        }
        offset += nextBitOffset >> 3;
        bitOffset = nextBitOffset & 0x7;

        nextBitOffset = bitOffset + 32;
        if (ConvMSBBitUInt26_UInt33(&i_pubInBuf[offset], nextBitOffset, 32, &tempU64) == 0)
            tempINSNAVData.m_dSecInWeek = (double)tempU64 * (double)0.001;
        else
            tempINSNAVData.m_dSecInWeek = nan("");
        offset += nextBitOffset >> 3;
        bitOffset = nextBitOffset & 0x7;

        if ((tempINSNAVData.m_iTimeSystem == 1 || tempINSNAVData.m_iTimeSystem == 2) &&
            tempINSNAVData.m_iWeekNum > 0 && tempINSNAVData.m_iWeekNum != 0xFFFF &&
            // *(uint32_t *)&(tempINSNAVData.m_dSecInWeek) != 0xFFFFFFFF)
            !isnan(tempINSNAVData.m_dSecInWeek))
        {
            nextBitOffset = bitOffset + 8;
            if (ConvMSBBitInt2_Int9(&i_pubInBuf[offset], nextBitOffset, 8, &tempS16) == 0)
                tempINSNAVData.m_iLeapSecond = (int8_t)tempS16;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 4;
            if (ConvMSBBitUInt2_UInt9(&i_pubInBuf[offset], nextBitOffset, 4, &tempU16) == 0)
                tempINSNAVData.m_iInsState = (uint8_t)tempU16;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 8;
            if (ConvMSBBitUInt2_UInt9(&i_pubInBuf[offset], nextBitOffset, 8, &tempU16) == 0)
                tempINSNAVData.m_iPosState = (uint8_t)tempU16;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 36;
            if (ConvMSBBitInt34_Int41(&i_pubInBuf[offset], nextBitOffset, 36, &tempS64) == 0)
                tempINSNAVData.m_dLatitude = (double)tempS64 * (double)1e-8;
            else
                tempINSNAVData.m_dLatitude = nan("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 36;
            if (ConvMSBBitInt34_Int41(&i_pubInBuf[offset], nextBitOffset, 36, &tempS64) == 0)
                tempINSNAVData.m_dLongitude = (double)tempS64 * (double)1e-8;
            else
                tempINSNAVData.m_dLongitude = nan("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 30;
            if (ConvMSBBitInt26_Int33(&i_pubInBuf[offset], nextBitOffset, 30, &tempS64) == 0)
                tempINSNAVData.m_dAltitude = (double)tempS64 * (double)0.001;
            else
                tempINSNAVData.m_dAltitude = nan("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 16, &tempS32) == 0)
                tempINSNAVData.m_fVelE = (float)(tempS32 * 0.01);
            else
                tempINSNAVData.m_fVelE = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 16, &tempS32) == 0)
                tempINSNAVData.m_fVelN = (float)(tempS32 * 0.01);
            else
                tempINSNAVData.m_fVelN = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 16, &tempS32) == 0)
                tempINSNAVData.m_fVelU = (float)(tempS32 * 0.01);
            else
                tempINSNAVData.m_fVelU = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 16, &tempS32) == 0)
                tempINSNAVData.m_fPitch = (float)(tempS32 * 0.01);
            else
                tempINSNAVData.m_fPitch = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 16, &tempS32) == 0)
                tempINSNAVData.m_fRoll = (float)(tempS32 * 0.01);
            else
                tempINSNAVData.m_fRoll = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 16;
            if (ConvMSBBitUInt10_UInt17(&i_pubInBuf[offset], nextBitOffset, 16, &tempU32) == 0)
                tempINSNAVData.m_fAzimuth = (float)(tempU32 * 0.01);
            else
                tempINSNAVData.m_fAzimuth = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 18;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 18, &tempS32) == 0)
                tempINSNAVData.m_fStdLat = (float)(tempS32 * 0.001);
            else
                tempINSNAVData.m_fStdLat = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 18;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 18, &tempS32) == 0)
                tempINSNAVData.m_fStdLong = (float)(tempS32 * 0.001);
            else
                tempINSNAVData.m_fStdLong = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 18;
            if (ConvMSBBitInt18_Int25(&i_pubInBuf[offset], nextBitOffset, 18, &tempS32) == 0)
                tempINSNAVData.m_fStdAltitude = (float)(tempS32 * 0.001);
            else
                tempINSNAVData.m_fStdAltitude = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempINSNAVData.m_fStdVelE = (float)(tempS32 * 0.01);
            else
                tempINSNAVData.m_fStdVelE = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempINSNAVData.m_fStdVelN = (float)(tempS32 * 0.01);
            else
                tempINSNAVData.m_fStdVelN = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempINSNAVData.m_fStdVelU = (float)(tempS32 * 0.01);
            else
                tempINSNAVData.m_fStdVelU = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempINSNAVData.m_fStdPitch = (float)(tempS32 * 0.01);
            else
                tempINSNAVData.m_fStdPitch = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempINSNAVData.m_fStdRoll = (float)(tempS32 * 0.01);
            else
                tempINSNAVData.m_fStdRoll = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            nextBitOffset = bitOffset + 12;
            if (ConvMSBBitInt10_Int17(&i_pubInBuf[offset], nextBitOffset, 12, &tempS32) == 0)
                tempINSNAVData.m_fStdAzimuth = (float)(tempS32 * 0.01);
            else
                tempINSNAVData.m_fStdAzimuth = nanf("");
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            // 保留字节
            nextBitOffset = bitOffset + 32;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            // 保留字节
            nextBitOffset = bitOffset + 32;
            offset += nextBitOffset >> 3;
            bitOffset = nextBitOffset & 0x7;

            if (isnormal(tempINSNAVData.m_dLatitude) &&
                isnormal(tempINSNAVData.m_dLongitude) &&
                isnormal(tempINSNAVData.m_dAltitude))
            {
                g_last_latitude = tempINSNAVData.m_dLatitude;
                g_last_longitude = tempINSNAVData.m_dLongitude;
                g_last_altitude = tempINSNAVData.m_dAltitude;
            }

            double utcTime[6] = {0};
            gtime_t time;
            if (tempINSNAVData.m_iTimeSystem == 1)
            {
                // 北斗系统
                bdst2time((int)tempINSNAVData.m_iWeekNum, tempINSNAVData.m_dSecInWeek, &time); // 周内秒+周数 转 GPS日期时间
            }
            else if (tempINSNAVData.m_iTimeSystem == 2)
            {
                // GPS系统
                gpst2time((int)tempINSNAVData.m_iWeekNum, tempINSNAVData.m_dSecInWeek, &time); // 周内秒+周数 转 GPS日期时间
            }
            gtime_t time_Leap;
            ros::Time utc_time;
            gpst2utc(time, tempINSNAVData.m_iLeapSecond, &time_Leap); // + GPS闰秒 (GPS时间和UTC时间差)
            utc_time.sec = time_Leap.time;
            utc_time.nsec = time_Leap.sec * 1e9;

            if (g_Config_param.enable_read_insnav > 0)
            {
                csjw_msgs::insnav insav_msg;
                insav_msg.header.header.frame_id = "gnss";
                insav_msg.header.header.stamp = utc_time;
                insav_msg.header.time_system = tempINSNAVData.m_iTimeSystem;
                insav_msg.header.week_num = tempINSNAVData.m_iWeekNum;
                insav_msg.header.secs_in_week = tempINSNAVData.m_dSecInWeek;
                insav_msg.header.leap_sec = tempINSNAVData.m_iLeapSecond;

                insav_msg.nav_status = tempINSNAVData.m_iInsState;
                insav_msg.pos_status = tempINSNAVData.m_iPosState;
                insav_msg.latitude = tempINSNAVData.m_dLatitude;
                insav_msg.longitude = tempINSNAVData.m_dLongitude;
                insav_msg.altitude = tempINSNAVData.m_dAltitude;
                insav_msg.vel_east = tempINSNAVData.m_fVelE;
                insav_msg.vel_north = tempINSNAVData.m_fVelN;
                insav_msg.vel_up = tempINSNAVData.m_fVelU;
                insav_msg.pitch = tempINSNAVData.m_fPitch;
                insav_msg.roll = tempINSNAVData.m_fRoll;
                insav_msg.yaw = tempINSNAVData.m_fAzimuth;
                insav_msg.std_latitude = tempINSNAVData.m_fStdLat;
                insav_msg.std_longitude = tempINSNAVData.m_fStdLong;
                insav_msg.std_altitude = tempINSNAVData.m_fStdAltitude;
                insav_msg.std_vel_east = tempINSNAVData.m_fStdVelE;
                insav_msg.std_vel_north = tempINSNAVData.m_fStdVelN;
                insav_msg.std_vel_up = tempINSNAVData.m_fStdVelU;
                insav_msg.std_pitch = tempINSNAVData.m_fStdPitch;
                insav_msg.std_roll = tempINSNAVData.m_fStdRoll;
                insav_msg.std_yaw = tempINSNAVData.m_fStdAzimuth;
                g_pub_insnav.publish(insav_msg);
            }
            if (g_Config_param.enable_ins_vel == 0)
            {
                geometry_msgs::TwistStamped ins_vel_msg;
                ins_vel_msg.header.frame_id = "gnss";
                ins_vel_msg.header.stamp = utc_time;
                ins_vel_msg.twist.linear.x = tempINSNAVData.m_fVelE; // 航速:节/s转m/s
                ins_vel_msg.twist.linear.y = tempINSNAVData.m_fVelN;
                ins_vel_msg.twist.linear.z = tempINSNAVData.m_fVelU;
                ins_vel_msg.twist.angular.x = 0;
                ins_vel_msg.twist.angular.y = 0;
                ins_vel_msg.twist.angular.z = 0;
                g_pub_ins_vel.publish(ins_vel_msg);
            }

            // 对时
            struct timeval cur_tv;
            if (g_Config_param.enable_set_time > 0 &&
                (gettimeofday(&cur_tv, nullptr) < 0 ||
                 (time_Leap.time >= cur_tv.tv_sec && time_Leap.time - cur_tv.tv_sec >= 2) ||
                 (cur_tv.tv_sec >= time_Leap.time && cur_tv.tv_sec - time_Leap.time >= 2)))
            {
                cur_tv.tv_sec = time_Leap.time;
                cur_tv.tv_usec = time_Leap.sec * 1e6;
                if (settimeofday(&cur_tv, nullptr) < 0)
                {
                    printf("gnss set time : no permission, please use sudo, or \"sudo su\"");
                }
            }
        }
    }

    *o_pbNeedTrans = true;
    return wLen + 15;
}

// RANGEB INSPVAXB等兼容诺瓦泰/北云 标准二进制 命令 数据处理
int CSHG_GNSS_BIN3_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans)
{
    if (i_TpSource == MsgSource_UART_GNSS) // 只处理从gnss模块接收的命令
    {
        // 取帧长
        uint8_t headerLen = i_pubInBuf[3];
        uint16_t wLen = (uint16_t)i_pubInBuf[8] + (uint16_t)(i_pubInBuf[9] << 8);
        // 取命令类型
        uint16_t CMD = (uint16_t)i_pubInBuf[4] + (uint16_t)(i_pubInBuf[5] << 8);
        uint16_t weekNum = (uint16_t)i_pubInBuf[14] + (uint16_t)(i_pubInBuf[15] << 8);
        uint32_t mSecInWeek = (uint32_t)i_pubInBuf[16] + (uint32_t)(i_pubInBuf[17] << 8) + (uint32_t)(i_pubInBuf[18] << 16) + (uint32_t)(i_pubInBuf[19] << 24);
        int offset = headerLen;
        // 命令处理
        switch (CMD)
        {
        default:
            // 未知协议或不支持配置的协议,不透传不处理
            *o_pbNeedTrans = false;
            return (int)i_udwInBufLen;
            break;
        }

        *o_pbNeedTrans = true;
        return (int)i_udwInBufLen;
    }
    *o_pbNeedTrans = false;
    return (int)i_udwInBufLen;
}

// RAWIMUSB等兼容诺瓦泰/北云 简化二进制 命令 数据处理
int CSHG_GNSS_BIN4_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans)
{
    if (i_TpSource == MsgSource_UART_GNSS) // 只处理从gnss模块接收的命令
    {
        // 取帧长
        uint8_t wLen = i_pubInBuf[3];
        // 取命令类型
        uint16_t CMD = (uint16_t)i_pubInBuf[4] + (uint16_t)(i_pubInBuf[5] << 8);
        uint16_t weekNum = (uint16_t)i_pubInBuf[6] + (uint16_t)(i_pubInBuf[7] << 8);
        uint32_t mSecInWeek = (uint32_t)i_pubInBuf[8] + (uint32_t)(i_pubInBuf[9] << 8) + (uint32_t)(i_pubInBuf[10] << 16) + (uint32_t)(i_pubInBuf[11] << 24);
        // int offset = 12;
        uint8_t logIndex = 0xFF;
        uint8_t logIndex2 = 0xFF;
        int ret;
        // 命令处理
        switch (CMD)
        {
        default:
            // 未知协议或不支持配置的协议,不透传不处理
            *o_pbNeedTrans = false;
            return (int)i_udwInBufLen;
        }
        *o_pbNeedTrans = true;
        return (int)i_udwInBufLen;
    }
    else // 接收处理
    {
        // 取帧长
        // uint8_t wLen = i_pubInBuf[3];
        // 取命令类型
        uint16_t CMD = (uint16_t)i_pubInBuf[4] + (uint16_t)(i_pubInBuf[5] << 8);
        // uint16_t weekNum = (uint16_t)i_pubInBuf[6] + (uint16_t)(i_pubInBuf[7] << 8);
        // uint32_t mSecInWeek = (uint32_t)i_pubInBuf[8] + (uint32_t)(i_pubInBuf[9] << 8) + (uint32_t)(i_pubInBuf[10] << 16) + (uint32_t)(i_pubInBuf[11] << 24);
        int offset = 12;
        // 命令处理
        switch (CMD)
        {
        default:
            // 未知协议或不支持配置的协议,不透传不处理
            *o_pbNeedTrans = false;
            return (int)i_udwInBufLen;
        }
    }
    *o_pbNeedTrans = false;
    return (int)i_udwInBufLen;
}

// POSDATAA IMUDATAA ODMDATAA INSPVAA INSIMUA INSPOSA CPP CPD PKG等命令 数据处理
int CSHG_GNSS_ASCII_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen)
{
    if (i_udwInBufLen > 10 &&
        i_pubInBuf[0] == '$')
    {
        // 找到 '$' 起始符
        for (uint32_t end = 1; end + 9 <= i_udwInBufLen; end++)
        {
            if (i_pubInBuf[end] < 0x20 || i_pubInBuf[end] > 0x7E)
            {
                break;
            }
            if (i_pubInBuf[end] == '*' &&
                i_pubInBuf[end + 7] == 0x0D &&
                i_pubInBuf[end + 8] == 0x0A)
            {
                // 找到 '*' 以及 '\r\n' 结束符
                uint32_t calCRC;
                // 计算校验码
                calCRC = rtk_crc24q(0, &i_pubInBuf[1], end - 1);
                char hexCRC[9];
                sprintf(hexCRC, "%06X", calCRC);
                if ((hexCRC[0] == i_pubInBuf[end + 1] || hexCRC[0] + 32 == i_pubInBuf[end + 1]) && // 32 == 'a' - 'A'
                    (hexCRC[1] == i_pubInBuf[end + 2] || hexCRC[1] + 32 == i_pubInBuf[end + 2]) &&
                    (hexCRC[2] == i_pubInBuf[end + 3] || hexCRC[2] + 32 == i_pubInBuf[end + 3]) &&
                    (hexCRC[3] == i_pubInBuf[end + 4] || hexCRC[3] + 32 == i_pubInBuf[end + 4]) &&
                    (hexCRC[4] == i_pubInBuf[end + 5] || hexCRC[4] + 32 == i_pubInBuf[end + 5]) &&
                    (hexCRC[5] == i_pubInBuf[end + 6] || hexCRC[5] + 32 == i_pubInBuf[end + 6]))
                {
                    return (int)(end + 9);
                }
            }
        }
    }
    return -1;
}

// HEADINGA等兼容诺瓦泰/北云 ASCII 命令 有效性判断
int CSHG_GNSS_ASCII2_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen)
{
    if (i_udwInBufLen > 12 &&
        i_pubInBuf[0] == '#')
    {
        // 找到 '$' 起始符
        for (uint32_t end = 1; end + 11 <= i_udwInBufLen; end++)
        {
            if (i_pubInBuf[end] < 0x20 || i_pubInBuf[end] > 0x7E)
            {
                break;
            }
            if (i_pubInBuf[end] == '*' &&
                i_pubInBuf[end + 9] == 0x0D &&
                i_pubInBuf[end + 10] == 0x0A)
            {
                // 找到 '*' 以及 '\r\n' 结束符
                uint32_t calCRC;
                // 计算校验码
                calCRC = CRC32_Cal(0, &i_pubInBuf[1], end - 1);
                char hexCRC[9];
                sprintf(hexCRC, "%08X", calCRC);
                if ((hexCRC[0] == i_pubInBuf[end + 1] || hexCRC[0] + 32 == i_pubInBuf[end + 1]) && // 32 == 'a' - 'A'
                    (hexCRC[1] == i_pubInBuf[end + 2] || hexCRC[1] + 32 == i_pubInBuf[end + 2]) &&
                    (hexCRC[2] == i_pubInBuf[end + 3] || hexCRC[2] + 32 == i_pubInBuf[end + 3]) &&
                    (hexCRC[3] == i_pubInBuf[end + 4] || hexCRC[3] + 32 == i_pubInBuf[end + 4]) &&
                    (hexCRC[4] == i_pubInBuf[end + 5] || hexCRC[4] + 32 == i_pubInBuf[end + 5]) &&
                    (hexCRC[5] == i_pubInBuf[end + 6] || hexCRC[5] + 32 == i_pubInBuf[end + 6]) &&
                    (hexCRC[6] == i_pubInBuf[end + 7] || hexCRC[6] + 32 == i_pubInBuf[end + 7]) &&
                    (hexCRC[7] == i_pubInBuf[end + 8] || hexCRC[7] + 32 == i_pubInBuf[end + 8]))
                {
                    return (int)(end + 11);
                }
            }
        }
    }
    return -1;
}

// INSPVAB 等命令 有效性判断
int CSHG_GNSS_BIN_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen)
{
    if (i_udwInBufLen >= 11 &&  // 命令最小长度判断
        i_pubInBuf[0] == 'C' && // 命令头判断
        i_pubInBuf[1] == 'S' && // 命令头判断
        i_pubInBuf[2] == 'H' && // 命令头判断
        i_pubInBuf[3] == 'G')   // 命令头判断
    {
        // 取帧长
        uint16_t wLen = (uint16_t)i_pubInBuf[6] + (uint16_t)(i_pubInBuf[7] << 8);
        if ((uint32_t)wLen + 11 <= i_udwInBufLen) // 命令长度判断
        {
            uint32_t inCRC, calCRC;
            // 取校验码
            inCRC = ((uint32_t)i_pubInBuf[8 + wLen] << 16) +
                    ((uint32_t)i_pubInBuf[8 + wLen + 1] << 8) +
                    (uint32_t)i_pubInBuf[8 + wLen + 2];
            // 计算校验码
            calCRC = rtk_crc24q(0, &i_pubInBuf[0], wLen + 8);
            if (inCRC == calCRC) // 校验码判断
            {
                return wLen + 11;
            }
        }
    }
    return -1;
}
// POSDATAB IMUDATAB ODMDATAB 等命令 有效性判断
int CSHG_GNSS_BIN2_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen)
{
    if (i_udwInBufLen >= 15 &&  // 命令最小长度判断
        i_pubInBuf[0] == '$' && // 命令头判断
        i_pubInBuf[1] == 'H' && // 命令头判断
        i_pubInBuf[2] == 'E' && // 命令头判断
        i_pubInBuf[3] == 'X' && // 命令头判断
        i_pubInBuf[4] == 'D' && // 命令头判断
        i_pubInBuf[5] == 'A' && // 命令头判断
        i_pubInBuf[6] == 'T' && // 命令头判断
        i_pubInBuf[7] == 'A')   // 命令头判断
    {
        // 取帧长
        uint16_t wLen = (uint16_t)i_pubInBuf[10] + (uint16_t)(i_pubInBuf[11] << 8);
        if ((uint32_t)wLen + 15 <= i_udwInBufLen) // 命令长度判断
        {
            uint32_t inCRC, calCRC;
            // 取校验码
            inCRC = ((uint32_t)i_pubInBuf[12 + wLen] << 16) +
                    ((uint32_t)i_pubInBuf[12 + wLen + 1] << 8) +
                    (uint32_t)i_pubInBuf[12 + wLen + 2];
            // 计算校验码
            calCRC = rtk_crc24q(0, &i_pubInBuf[0], wLen + 12);
            if (inCRC == calCRC) // 校验码判断
            {
                return wLen + 15;
            }
        }
    }
    return -1;
}

// RANGEB INSPVAXB等兼容诺瓦泰/北云 标准二进制 命令 有效性判断
int CSHG_GNSS_BIN3_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen)
{
    if (i_udwInBufLen >= 32 &&   // 命令最小长度判断
        i_pubInBuf[0] == 0xAA && // 命令头判断
        i_pubInBuf[1] == 0x44 && // 命令头判断
        i_pubInBuf[2] == 0x12 && // 命令头判断
        i_pubInBuf[6] == 0x00)   // 二进制标志位
    {
        // 取帧长
        uint8_t headerLen = i_pubInBuf[3];
        uint16_t dataLen = (uint16_t)i_pubInBuf[8] + (uint16_t)(i_pubInBuf[9] << 8);
        if ((uint32_t)(headerLen + dataLen + 4) <= i_udwInBufLen) // 命令长度判断
        {
            uint32_t inCRC, calCRC;
            // 取校验码
            memcpy(&inCRC, i_pubInBuf + headerLen + dataLen, 4);
            // 计算校验码
            calCRC = CRC32_Cal(0, &i_pubInBuf[0], headerLen + dataLen);
            if (inCRC == calCRC) // 校验码判断
            {
                return headerLen + dataLen + 4;
            }
        }
    }
    return -1;
}

// RAWIMUSB等兼容诺瓦泰/北云 简化二进制 命令 有效性判断
int CSHG_GNSS_BIN4_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen)
{
    if (i_udwInBufLen >= 16 &&   // 命令最小长度判断
        i_pubInBuf[0] == 0xAA && // 命令头判断
        i_pubInBuf[1] == 0x44 && // 命令头判断
        i_pubInBuf[2] == 0x13)
    {
        // 取帧长
        uint8_t dataLen = i_pubInBuf[3];
        if ((uint32_t)(12 + dataLen + 4) <= i_udwInBufLen) // 命令长度判断
        {
            uint32_t inCRC, calCRC;
            // 取校验码
            memcpy(&inCRC, i_pubInBuf + 12 + dataLen, 4);
            // 计算校验码
            calCRC = CRC32_Cal(0, &i_pubInBuf[0], 12 + dataLen);
            if (inCRC == calCRC) // 校验码判断
            {
                return 12 + dataLen + 4;
            }
        }
    }
    return -1;
}

uint8_t CSHG_GNSS_ConvNovatelPosStateToGGAPosState(const uint8_t i_Novatel, uint8_t *o_GGA)
{
    uint8_t ret;
    switch (i_Novatel)
    {
    default:
        ret = 0;
        break;
    case 0: // 未解算
        ret = 0;
        break;
    case 1: // 位置由命令输入固定
        ret = 7;
        break;
    case 16: // 单点
    case 53: // INS单点解
        ret = 1;
        break;
    case 17: // 伪距差分
    case 54: // INS伪距差分
        ret = 2;
        break;
    case 34: // 窄带浮点
    case 55: // INS浮点
        ret = 5;
        break;
    case 50: // 窄带固定解
    case 56: // INS固定
        ret = 4;
        break;
    case 69: // PPP
        ret = 8;
        break;
    }
    if (o_GGA != NULL)
    {
        *o_GGA = ret;
    }
    return ret;
}

uint8_t CSHG_GNSS_ConvGGAPosStateToNovatelPosState(const uint8_t i_GGA, uint8_t *o_Novatel)
{
    uint8_t ret;
    switch (i_GGA)
    {
    default:
        ret = 0;
        break;
    case 0: // 未解算
        ret = 0;
        break;
    case 1: // 单点
        ret = 16;
        break;
    case 2: // 伪距差分
        ret = 17;
        break;
    case 4: // 窄带固定解
        ret = 50;
        break;
    case 5: // 窄带浮点
        ret = 34;
        break;
    case 7: // 位置由命令输入固定
        ret = 1;
        break;
    case 8: // PPP
        ret = 69;
        break;
    }
    if (o_Novatel != NULL)
    {
        *o_Novatel = ret;
    }
    return ret;
}

static void CSHG_GetValue(const eCSHG_CMD i_eCmd, const uint32_t i_index, const char *i_pbInBuf, void *o_Data)
{
    switch (i_eCmd)
    {
    default:
        // 此部分命令不需处理数据, 减少计算量, 只透传
        break;
    case CSHG_CMD_INSPVAA:
        CSHG_INSPVA_GetValue(i_index, i_pbInBuf, (tINSPVAData *)o_Data);
        break;
    case CSHG_CMD_IMUDATAA:
        CSHG_IMUDATA_GetValue(i_index, i_pbInBuf, (tIMUDATAData *)o_Data);
        break;
    case CSHG_CMD_SATNAVA:
        CSHG_SATNAV_GetValue(i_index, i_pbInBuf, (tSATNAVData *)o_Data);
        break;
    case CSHG_CMD_INSNAVA:
        CSHG_INSNAV_GetValue(i_index, i_pbInBuf, (tINSNAVData *)o_Data);
        break;
    }
}

static void CSHG_INSPVA_GetValue(const uint32_t i_index, const char *i_pbInBuf, tINSPVAData *o_Data)
{
    if (o_Data != NULL && i_pbInBuf != NULL)
    {
        switch ((eINSPVAA_INDEX)i_index)
        {
        default:
            break;
        case INSPVAA_INDEX_count: // 1	count	秒内10ms计数值	Uint8
            o_Data->count10ms = (uint8_t)atoi(i_pbInBuf);
            break;
        case INSPVAA_INDEX_gpstime: // 2	gpstime	gps时间（惯导时）	FP64
            o_Data->gpstime = atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_ins_status: // 3	ins_status	组合解算状态	Uint8
            o_Data->ins_status = (uint8_t)atoi(i_pbInBuf);
            break;
        case INSPVAA_INDEX_ins_Lat: // 4	ins_Lat	组合纬度(度)	FP64
            o_Data->ins_Lat = atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_ins_Lon: // 5	ins_Lon	组合经度(度)	FP64
            o_Data->ins_Lon = atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_ins_Hgt: // 6	ins_Hgt	组合高度(m)	FP32
            o_Data->ins_Hgt = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_ve: // 7	ve	组合东向速度（m/s）	FP32
            o_Data->ve = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_vn: // 8	vn	组合北向速度（m/s）	FP32
            o_Data->vn = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_vu: // 9	vu	组合天向速度（m/s）	FP32
            o_Data->vu = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_pitch: // 10	pitch	组合俯仰角(度)	FP32
            o_Data->pitch = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_roll: // 11	roll	组合横滚(度)	FP32
            o_Data->roll = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_head: // 12	head	组合航向角(度)	FP32
            o_Data->head = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_gnss_status: // 13	gnss_status	卫导定位状态	Uint8
            o_Data->gnss_status = (uint8_t)atoi(i_pbInBuf);
            break;
        case INSPVAA_INDEX_headstatus: // 14	headstatus	卫导定向状态	Uint8
            o_Data->headstatus = (uint8_t)atoi(i_pbInBuf);
            break;
        case INSPVAA_INDEX_Baseline: // 15	Baseline	基线长度	FP32
            o_Data->Baseline = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_gnss_Lat: // 16	gnss_Lat	卫导纬度(度)	FP64
            o_Data->gnss_Lat = atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_gnss_Lon: // 17	gnss_Lon	卫导经度(度)	FP64
            o_Data->gnss_Lon = atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_gnss_Hgt: // 18	gnss_Hgt	卫导高度(m)	FP32
            o_Data->gnss_Hgt = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_gnss_normv: // 19	gnss_normv	卫导合速度（m/s）	FP32
            o_Data->gnss_normv = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_Hdop: // 20	Hdop	卫导水平精度因子	FP32
            o_Data->Hdop = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_gnss_satM: // 21	gnss_satM	主天线卫星数（个）	Uint8
            o_Data->gnss_satM = (uint8_t)atoi(i_pbInBuf);
            break;
        case INSPVAA_INDEX_gnss_satS: // 22	gnss_satS	从天线卫星数（个）	Uint8
            o_Data->gnss_satS = (uint8_t)atoi(i_pbInBuf);
            break;
        case INSPVAA_INDEX_odov: // 23	odov	里程计速度（m/s）	FP32
            o_Data->odov = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_odoflag: // 24	odoflag	里程计标志（1-里程计速度有效 0-无里程计速度或速度无效）	Uint8
            o_Data->odoflag = (uint8_t)atoi(i_pbInBuf);
            break;
        case INSPVAA_INDEX_gyrox: // 25	gyrox	陀螺仪X轴(°/s)	FP32
            o_Data->gyrox = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_gyroy: // 26	gyroy	陀螺仪Y轴(°/s)	FP32
            o_Data->gyroy = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_gyroz: // 27	gyroz	陀螺仪Z轴(°/s)	FP32
            o_Data->gyroz = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_accex: // 28	accex	加计X轴(m/s^2)	FP32
            o_Data->accex = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_accey: // 29	accey	加计Y轴(m/s^2)	FP32
            o_Data->accey = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_accez: // 30	accez	加计Z轴(m/s^2)	FP32
            o_Data->accez = (float)atof(i_pbInBuf);
            break;
        case INSPVAA_INDEX_accstdval: // 31	accstdval	加计方差	FP32
            o_Data->accstdval = (float)atof(i_pbInBuf);
            break;
        }
    }
}

static void CSHG_IMUDATA_GetValue(const uint32_t i_index, const char *i_pbInBuf, tIMUDATAData *o_Data)
{
    if (o_Data != NULL && i_pbInBuf != NULL)
    {
        switch ((eIMUDATAA_INDEX)i_index)
        {
        default:
            break;
        case IMUDATAA_INDEX_timeSystem:
            o_Data->m_iTimeSystem = (uint8_t)atoi(i_pbInBuf);
            break;
        case IMUDATAA_INDEX_weeks:
            o_Data->m_iWeekNum = (uint16_t)atoi(i_pbInBuf);
            g_GnssWeekNum = o_Data->m_iWeekNum;
            break;
        case IMUDATAA_INDEX_weekSec:
            o_Data->m_dSecInWeek = atof(i_pbInBuf);
            break;
        case IMUDATAA_INDEX_LeapSecond:
            o_Data->m_iLeapSecond = (int8_t)atoi(i_pbInBuf);
            break;
        case IMUDATAA_INDEX_accex:
            o_Data->m_fAccelX = (float)atof(i_pbInBuf);
            break;
        case IMUDATAA_INDEX_accey:
            o_Data->m_fAccelY = (float)atof(i_pbInBuf);
            break;
        case IMUDATAA_INDEX_accez:
            o_Data->m_fAccelZ = (float)atof(i_pbInBuf);
            break;
        case IMUDATAA_INDEX_gyrox:
            o_Data->m_fGroX = (float)atof(i_pbInBuf);
            break;
        case IMUDATAA_INDEX_gyroy:
            o_Data->m_fGroY = (float)atof(i_pbInBuf);
            break;
        case IMUDATAA_INDEX_gyroz:
            o_Data->m_fGroZ = (float)atof(i_pbInBuf);
            break;
        case IMUDATAA_INDEX_magx:
            o_Data->m_fMagX = (float)atof(i_pbInBuf);
            break;
        case IMUDATAA_INDEX_magy:
            o_Data->m_fMagY = (float)atof(i_pbInBuf);
            break;
        case IMUDATAA_INDEX_magz:
            o_Data->m_fMagZ = (float)atof(i_pbInBuf);
            break;
        }
    }
}

static void CSHG_SATNAV_GetValue(const uint32_t i_index, const char *i_pbInBuf, tSATNAVData *o_Data)
{
    if (o_Data != NULL && i_pbInBuf != NULL)
    {
        switch ((eSATNAVA_INDEX)i_index)
        {
        default:
            break;
        case SATNAVA_INDEX_timeSystem:
            o_Data->m_iTimeSystem = (uint8_t)atoi(i_pbInBuf);
            break;
        case SATNAVA_INDEX_weeks:
            o_Data->m_iWeekNum = (uint16_t)atoi(i_pbInBuf);
            g_GnssWeekNum = o_Data->m_iWeekNum;
            break;
        case SATNAVA_INDEX_weekSec:
            o_Data->m_dSecInWeek = atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_LeapSecond:
            o_Data->m_iLeapSecond = (int8_t)atoi(i_pbInBuf);
            break;
        case SATNAVA_INDEX_posState:
            o_Data->m_iPosState = (uint8_t)atoi(i_pbInBuf);
        case SATNAVA_INDEX_aziState:
            o_Data->m_iAziState = (uint8_t)atoi(i_pbInBuf);
            break;
        case SATNAVA_INDEX_lat:
            o_Data->m_dLatitude = atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_lon:
            o_Data->m_dLongitude = atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_hgt:
            o_Data->m_dAltitude = atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_ve:
            o_Data->m_fVelE = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_vn:
            o_Data->m_fVelN = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_vu:
            o_Data->m_fVelU = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_pitch:
            o_Data->m_fPitch = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_roll:
            o_Data->m_fRoll = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_head:
            o_Data->m_fAzimuth = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_sta_lat:
            o_Data->m_fStdLat = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_sta_lon:
            o_Data->m_fStdLong = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_sta_hgt:
            o_Data->m_fStdAltitude = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_sta_ve:
            o_Data->m_fStdVelE = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_sta_vn:
            o_Data->m_fStdVelN = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_sta_vu:
            o_Data->m_fStdVelU = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_sta_pitch:
            o_Data->m_fStdPitch = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_sta_roll:
            o_Data->m_fStdRoll = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_sta_head:
            o_Data->m_fStdAzimuth = (float)atof(i_pbInBuf);
            break;
        case SATNAVA_INDEX_gnss_satM:
            o_Data->m_iGnssSatM = (uint8_t)atoi(i_pbInBuf);
            break;
        case SATNAVA_INDEX_gnss_satS:
            o_Data->m_iGnssSatS = (uint8_t)atoi(i_pbInBuf);
            break;
        case SATNAVA_INDEX_diff:
            o_Data->m_fDiffAge = (float)atof(i_pbInBuf);
            break;
        }
    }
}

static void CSHG_INSNAV_GetValue(const uint32_t i_index, const char *i_pbInBuf, tINSNAVData *o_Data)
{
    if (o_Data != NULL && i_pbInBuf != NULL)
    {
        switch ((eINSNAVA_INDEX)i_index)
        {
        default:
            break;
        case INSNAVA_INDEX_timeSystem:
            o_Data->m_iTimeSystem = (uint8_t)atoi(i_pbInBuf);
            break;
        case INSNAVA_INDEX_weeks:
            o_Data->m_iWeekNum = (uint16_t)atoi(i_pbInBuf);
            g_GnssWeekNum = o_Data->m_iWeekNum;
            break;
        case INSNAVA_INDEX_weekSec:
            o_Data->m_dSecInWeek = atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_LeapSecond:
            o_Data->m_iLeapSecond = (int8_t)atoi(i_pbInBuf);
            break;
        case INSNAVA_INDEX_insState:
            o_Data->m_iInsState = (uint8_t)atoi(i_pbInBuf);
            break;
        case INSNAVA_INDEX_posState:
            o_Data->m_iPosState = (uint8_t)atoi(i_pbInBuf);
            break;
        case INSNAVA_INDEX_lat:
            o_Data->m_dLatitude = atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_lon:
            o_Data->m_dLongitude = atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_hgt:
            o_Data->m_dAltitude = atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_ve:
            o_Data->m_fVelE = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_vn:
            o_Data->m_fVelN = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_vu:
            o_Data->m_fVelU = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_pitch:
            o_Data->m_fPitch = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_roll:
            o_Data->m_fRoll = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_head:
            o_Data->m_fAzimuth = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_sta_lat:
            o_Data->m_fStdLat = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_sta_lon:
            o_Data->m_fStdLong = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_sta_hgt:
            o_Data->m_fStdAltitude = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_sta_ve:
            o_Data->m_fStdVelE = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_sta_vn:
            o_Data->m_fStdVelN = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_sta_vu:
            o_Data->m_fStdVelU = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_sta_pitch:
            o_Data->m_fStdPitch = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_sta_roll:
            o_Data->m_fStdRoll = (float)atof(i_pbInBuf);
            break;
        case INSNAVA_INDEX_sta_head:
            o_Data->m_fStdAzimuth = (float)atof(i_pbInBuf);
            break;
        }
    }
}
static void CSHG_POSDATA_Init(tPOSDATAData *io_Data)
{
    if (io_Data != NULL)
    {
        memset(io_Data, 0xFF, sizeof(tPOSDATAData));
        io_Data->m_iLeapSecond = (int8_t)0x80; // 闰秒, 周内秒+周数+闰秒=UTC

        io_Data->m_dSecInWeek = nan(""); // 3 自本周日0:00:00至当前的秒数
        io_Data->m_dLatitude = nan("");  // 7 北纬为正，南纬为负，范围[-90,+90]，8位小数
        io_Data->m_dLongitude = nan(""); // 8 东经为正，西经为负，范围[-180,+180]，8位小数
        io_Data->m_dAltitude = nan("");  // 9 范围[-20000,+20000],3位小数

        io_Data->m_fVelE = nanf(""); // 10 ENU下东向速度
        io_Data->m_fVelN = nanf(""); // 11 ENU下北向速度
        io_Data->m_fVelU = nanf(""); // 12 ENU下对天速度

        io_Data->m_fPitch = nanf("");   // 13 俯仰角
        io_Data->m_fRoll = nanf("");    // 14 横滚角
        io_Data->m_fAzimuth = nanf(""); // 15 航向角

        io_Data->m_fStdLat = nanf("");      // 16 纬度标准差
        io_Data->m_fStdLong = nanf("");     // 17 经度标准差
        io_Data->m_fStdAltitude = nanf(""); // 18 高程标准差

        io_Data->m_fStdVelE = nanf(""); // 19 东向速度标准差
        io_Data->m_fStdVelN = nanf(""); // 20 北向速度标准差
        io_Data->m_fStdVelU = nanf(""); // 21 天向速度标准差

        io_Data->m_fStdPitch = nanf("");   // 22 俯仰角标准差
        io_Data->m_fStdRoll = nanf("");    // 23 横滚角标准差
        io_Data->m_fStdAzimuth = nanf(""); // 24 航向角标准差
        io_Data->m_fDiffAge = nanf("");    // 27 差分龄期

        io_Data->m_fFlWheelSpeed = nanf(""); // 30 前左轮速
        io_Data->m_fFrWheelSpeed = nanf(""); // 31 前右轮速
        io_Data->m_fRlWheelSpeed = nanf(""); // 32 后左轮速
        io_Data->m_fRrWheelSpeed = nanf(""); // 33 后右轮速
    }
}
static void CSHG_IMUDATA_Init(tIMUDATAData *io_Data)
{
    if (io_Data != NULL)
    {
        memset(io_Data, 0xFF, sizeof(tIMUDATAData));
        io_Data->m_iLeapSecond = (int8_t)0x80; // 闰秒, 周内秒+周数+闰秒=UTC

        io_Data->m_fAccelX = nanf(""); // 4 加计X轴
        io_Data->m_fAccelY = nanf(""); // 5 加计Y轴
        io_Data->m_fAccelZ = nanf(""); // 6 加计Z轴

        io_Data->m_fGroX = nanf(""); // 7 陀螺仪X轴
        io_Data->m_fGroY = nanf(""); // 8 陀螺仪Y轴
        io_Data->m_fGroZ = nanf(""); // 9 陀螺仪Z轴

        io_Data->m_fMagX = nanf(""); // 10 磁力计X
        io_Data->m_fMagY = nanf(""); // 11 磁力计Y
        io_Data->m_fMagZ = nanf(""); // 12 磁力计Z
    }
}
static void CSHG_SATNAV_Init(tSATNAVData *io_Data)
{
    if (io_Data != NULL)
    {
        memset(io_Data, 0xFF, sizeof(tSATNAVData));
        io_Data->m_iLeapSecond = (int8_t)0x80; // 闰秒, 周内秒+周数+闰秒=UTC

        io_Data->m_dSecInWeek = nan(""); // 3 自本周日0:00:00至当前的秒数
        io_Data->m_dLatitude = nan("");  // 7 北纬为正，南纬为负，范围[-90,+90]，8位小数
        io_Data->m_dLongitude = nan(""); // 8 东经为正，西经为负，范围[-180,+180]，8位小数
        io_Data->m_dAltitude = nan("");  // 9 范围[-20000,+20000],3位小数

        io_Data->m_fVelE = nanf(""); // 10 ENU下东向速度
        io_Data->m_fVelN = nanf(""); // 11 ENU下北向速度
        io_Data->m_fVelU = nanf(""); // 12 ENU下对天速度

        io_Data->m_fPitch = nanf("");   // 13 俯仰角
        io_Data->m_fRoll = nanf("");    // 14 横滚角
        io_Data->m_fAzimuth = nanf(""); // 15 航向角

        io_Data->m_fStdLat = nanf("");      // 16 纬度标准差
        io_Data->m_fStdLong = nanf("");     // 17 经度标准差
        io_Data->m_fStdAltitude = nanf(""); // 18 高程标准差

        io_Data->m_fStdVelE = nanf(""); // 19 东向速度标准差
        io_Data->m_fStdVelN = nanf(""); // 20 北向速度标准差
        io_Data->m_fStdVelU = nanf(""); // 21 天向速度标准差

        io_Data->m_fStdPitch = nanf("");   // 22 俯仰角标准差
        io_Data->m_fStdRoll = nanf("");    // 23 横滚角标准差
        io_Data->m_fStdAzimuth = nanf(""); // 24 航向角标准差
        io_Data->m_fDiffAge = nanf("");    // 27 差分龄期
    }
}
static void CSHG_INSNAV_Init(tINSNAVData *io_Data)
{
    if (io_Data != NULL)
    {
        memset(io_Data, 0xFF, sizeof(tINSNAVData));
        io_Data->m_iLeapSecond = (int8_t)0x80; // 闰秒, 周内秒+周数+闰秒=UTC

        io_Data->m_dSecInWeek = nan(""); // 3 自本周日0:00:00至当前的秒数
        io_Data->m_dLatitude = nan("");  // 7 北纬为正，南纬为负，范围[-90,+90]，8位小数
        io_Data->m_dLongitude = nan(""); // 8 东经为正，西经为负，范围[-180,+180]，8位小数
        io_Data->m_dAltitude = nan("");  // 9 范围[-20000,+20000],3位小数

        io_Data->m_fVelE = nanf(""); // 10 ENU下东向速度
        io_Data->m_fVelN = nanf(""); // 11 ENU下北向速度
        io_Data->m_fVelU = nanf(""); // 12 ENU下对天速度

        io_Data->m_fPitch = nanf("");   // 13 俯仰角
        io_Data->m_fRoll = nanf("");    // 14 横滚角
        io_Data->m_fAzimuth = nanf(""); // 15 航向角

        io_Data->m_fStdLat = nanf("");      // 16 纬度标准差
        io_Data->m_fStdLong = nanf("");     // 17 经度标准差
        io_Data->m_fStdAltitude = nanf(""); // 18 高程标准差

        io_Data->m_fStdVelE = nanf(""); // 19 东向速度标准差
        io_Data->m_fStdVelN = nanf(""); // 20 北向速度标准差
        io_Data->m_fStdVelU = nanf(""); // 21 天向速度标准差

        io_Data->m_fStdPitch = nanf("");   // 22 俯仰角标准差
        io_Data->m_fStdRoll = nanf("");    // 23 横滚角标准差
        io_Data->m_fStdAzimuth = nanf(""); // 24 航向角标准差
    }
}

bool CSHG_INSPVAA_IsNeed(void)
{
    if (g_Config_param.enable_read_inspva == 2 ||
        g_Config_param.enable_imu == 3 ||
        g_Config_param.enable_master_sat_num == 4 ||
        g_Config_param.enable_slaver_sat_num == 4)
    {
        return true;
    }
    return false;
}

bool CSHG_INSPVAB_IsNeed(void)
{
    if (g_Config_param.enable_read_inspva == 1 ||
        g_Config_param.enable_imu == 2 ||
        g_Config_param.enable_master_sat_num == 3 ||
        g_Config_param.enable_slaver_sat_num == 3)
    {
        return true;
    }
    return false;
}

bool CSHG_IMUDATAA_IsNeed(void)
{
    if (g_Config_param.enable_read_imudata == 2 ||
        g_Config_param.enable_imu == 1)
    {
        return true;
    }
    return false;
}

bool CSHG_IMUDATAB_IsNeed(void)
{
    if (g_Config_param.enable_read_imudata == 1 ||
        g_Config_param.enable_imu == 0)
    {
        return true;
    }
    return false;
}
bool CSHG_SATNAVA_IsNeed(void)
{
    if (g_Config_param.enable_read_satnav == 2 ||
        g_Config_param.enable_nav_sat_fix == 1 ||
        // g_Config_param.enable_gps_fix == 1 ||
        g_Config_param.enable_master_sat_num == 1 ||
        g_Config_param.enable_slaver_sat_num == 1 ||
        g_Config_param.enable_gnss_vel == 1)
    {
        return true;
    }
    return false;
}
bool CSHG_SATNAVB_IsNeed(void)
{
    if (g_Config_param.enable_read_satnav == 1 ||
        g_Config_param.enable_nav_sat_fix == 0 ||
        // g_Config_param.enable_gps_fix == 0 ||
        g_Config_param.enable_master_sat_num == 0 ||
        g_Config_param.enable_slaver_sat_num == 0 ||
        g_Config_param.enable_gnss_vel == 0)
    {
        return true;
    }
    return false;
}
bool CSHG_INSNAVA_IsNeed(void)
{
    if (g_Config_param.enable_read_insnav == 2 ||
        g_Config_param.enable_ins_vel == 1)
    {
        return true;
    }
    return false;
}
bool CSHG_INSNAVB_IsNeed(void)
{
    if (g_Config_param.enable_read_insnav == 1 ||
        g_Config_param.enable_ins_vel == 0)
    {
        return true;
    }
    return false;
}

CSHG_GNSS_LOG CSHG_GNSS_CMD_GET_LOG_FROM_NAME(const char *i_name)
{
    CSHG_GNSS_LOG ret = CSHG_GNSS_LOG_UNKNOWN;
    if (i_name != NULL)
    {
        if (strstr(i_name, "INSPVAA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSPVAA;
        }
        else if (strstr(i_name, "INSPVAB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSPVAB;
        }
        else if (strstr(i_name, "SATNAVA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_SATNAVA;
        }
        else if (strstr(i_name, "SATNAVB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_SATNAVB;
        }
        else if (strstr(i_name, "INSNAVA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSNAVA;
        }
        else if (strstr(i_name, "INSNAVB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSNAVB;
        }
        else if (strstr(i_name, "INSPOSA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSPOSA;
        }
        else if (strstr(i_name, "INSIMUA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSIMUA;
        }
        else if (strstr(i_name, "INSSRCA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSSRCA;
        }
        else if (strstr(i_name, "POSDATAB") != NULL) // 因ASCII没有尾数A,先判断B
        {
            ret = CSHG_GNSS_LOG_INDEX_POSDATAB;
        }
        else if (strstr(i_name, "POSDATAA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_POSDATAA;
        }
        else if (strstr(i_name, "IMUDATAB") != NULL) // 因ASCII没有尾数A,先判断B
        {
            ret = CSHG_GNSS_LOG_INDEX_IMUDATAB;
        }
        else if (strstr(i_name, "IMUDATAA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_IMUDATAA;
        }
        else if (strstr(i_name, "ODMDATAB") != NULL) // 因ASCII没有尾数A,先判断B
        {
            ret = CSHG_GNSS_LOG_INDEX_ODMDATAB;
        }
        else if (strstr(i_name, "ODMDATAA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_ODMDATAA;
        }
        else if (strstr(i_name, "ODMPLUSB") != NULL) // 因ASCII没有尾数A,先判断B
        {
            ret = CSHG_GNSS_LOG_INDEX_ODMPLUSB;
        }
        else if (strstr(i_name, "ODMPLUSA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_ODMPLUSA;
        }
        else if (strstr(i_name, "GGA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_GGA;
        }
        else if (strstr(i_name, "RMC") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RMC;
        }
        else if (strstr(i_name, "ZDA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_ZDA;
        }
        else if (strstr(i_name, "CPD") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_CPD;
        }
        else if (strstr(i_name, "CDD") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_CDD;
        }
        else if (strstr(i_name, "PKG") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_PKG;
        }
        else if (strstr(i_name, "GSA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_GSA;
        }
        else if (strstr(i_name, "EGSVA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_EGSVA;
        }
        else if (strstr(i_name, "EGSV2A") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_EGSV2A;
        }
        else if (strstr(i_name, "EGSVB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_EGSVB;
        }
        else if (strstr(i_name, "EGSV2B") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_EGSV2B;
        }
        else if (strstr(i_name, "GSV") != NULL) // 因EGSVA EGSVB包含GSV, 要后判断
        {
            ret = CSHG_GNSS_LOG_INDEX_GSV;
        }
        else if (strstr(i_name, "KSXT") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_KSXT;
        }
        else if (strstr(i_name, "BD2EPHB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_BD2EPHB;
        }
        else if (strstr(i_name, "BD3EPHB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_BD3EPHB;
        }
        else if (strstr(i_name, "GPSEPHB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_GPSEPHB;
        }
        else if (strstr(i_name, "GLOEPHB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_GLOEPHB;
        }
        else if (strstr(i_name, "GALEPHB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_GALEPHB;
        }
        else if (strstr(i_name, "DTM") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_DTM;
        }
        else if (strstr(i_name, "GBS") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_GBS;
        }
        else if (strstr(i_name, "GLL") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_GLL;
        }
        else if (strstr(i_name, "GST") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_GST;
        }
        else if (strstr(i_name, "HDT") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_HDT;
        }
        else if (strstr(i_name, "ORI") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_ORI;
        }
        else if (strstr(i_name, "NTR") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_NTR;
        }
        else if (strstr(i_name, "ROT") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_ROT;
        }
        else if (strstr(i_name, "TRA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_TRA;
        }
        else if (strstr(i_name, "VTG") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_VTG;
        }
        else if (strstr(i_name, "PRANGEB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_PRANGEB;
        }
        else if (strstr(i_name, "PRANGE2B") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_PRANGE2B;
        }
        else if (strstr(i_name, "BESTPOSA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_BESTPOSA;
        }
        else if (strstr(i_name, "BESTVELA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_BESTVELA;
        }
        else if (strstr(i_name, "BESTXYZA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_BESTXYZA;
        }
        else if (strstr(i_name, "HEADINGA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_HEADINGA;
        }
        else if (strstr(i_name, "PSRDOPA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_PSRDOPA;
        }
        else if (strstr(i_name, "PSRPOSA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_PSRPOSA;
        }
        else if (strstr(i_name, "PSRVELA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_PSRVELA;
        }
        else if (strstr(i_name, "PTNL,AVR") != NULL) // 指令回复形式
        {
            ret = CSHG_GNSS_LOG_INDEX_PTNLAVR;
        }
        else if (strstr(i_name, "PTNLAVR") != NULL) // open/close命令形式
        {
            ret = CSHG_GNSS_LOG_INDEX_PTNLAVR;
        }
        else if (strstr(i_name, "PTNL,PJK") != NULL) // 指令回复形式
        {
            ret = CSHG_GNSS_LOG_INDEX_PTNLPJK;
        }
        else if (strstr(i_name, "PTNLPJK") != NULL) // open/close命令形式
        {
            ret = CSHG_GNSS_LOG_INDEX_PTNLPJK;
        }
        else if (strstr(i_name, "RANGEB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RANGEB;
        }
        else if (strstr(i_name, "RANGE2B") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RANGE2B;
        }
        else if (strstr(i_name, "RANGECMPB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RANGECMPB;
        }
        else if (strstr(i_name, "RANGECMP2B") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RANGECMP2B;
        }
        else if (strstr(i_name, "TIMEA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_TIMEA;
        }
        else if (strstr(i_name, "YBM") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_YBM;
        }
        else if (strstr(i_name, "GGS") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_GGS;
        }
        else if (strstr(i_name, "RTCM999") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM999;
        }
        else if (strstr(i_name, "RTCM1005") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1005;
        }
        else if (strstr(i_name, "RTCM1006") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1006;
        }
        else if (strstr(i_name, "RTCM1013") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1013;
        }
        else if (strstr(i_name, "RTCM1019") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1019;
        }
        else if (strstr(i_name, "RTCM1020") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1020;
        }
        else if (strstr(i_name, "RTCM1033") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1033;
        }
        else if (strstr(i_name, "RTCM1042") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1042;
        }
        else if (strstr(i_name, "RTCM1044") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1044;
        }
        else if (strstr(i_name, "RTCM1045") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1045;
        }
        else if (strstr(i_name, "RTCM1046") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1046;
        }
        else if (strstr(i_name, "RTCM1074") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1074;
        }
        else if (strstr(i_name, "RTCM1084") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1084;
        }
        else if (strstr(i_name, "RTCM1094") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1094;
        }
        else if (strstr(i_name, "RTCM1114") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1114;
        }
        else if (strstr(i_name, "RTCM1124") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1124;
        }
        else if (strstr(i_name, "RTCM1075") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1075;
        }
        else if (strstr(i_name, "RTCM1085") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1085;
        }
        else if (strstr(i_name, "RTCM1095") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1095;
        }
        else if (strstr(i_name, "RTCM1115") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1115;
        }
        else if (strstr(i_name, "RTCM1125") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1125;
        }
        else if (strstr(i_name, "RTCM1076") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1076;
        }
        else if (strstr(i_name, "RTCM1086") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1086;
        }
        else if (strstr(i_name, "RTCM1096") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1096;
        }
        else if (strstr(i_name, "RTCM1116") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1116;
        }
        else if (strstr(i_name, "RTCM1126") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1126;
        }
        else if (strstr(i_name, "RTCM1077") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1077;
        }
        else if (strstr(i_name, "RTCM1087") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1087;
        }
        else if (strstr(i_name, "RTCM1097") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1097;
        }
        else if (strstr(i_name, "RTCM1117") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1117;
        }
        else if (strstr(i_name, "RTCM1127") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RTCM1127;
        }
        else if (strstr(i_name, "RGGNSSB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RONGGAN_GNSS;
        }
        else if (strstr(i_name, "RGODMB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RONGGAN_ODM;
        }
        else if (strstr(i_name, "RGRAWIMUSB") != NULL) // 要比RAWIMUSB前判断,否则判断错误
        {
            ret = CSHG_GNSS_LOG_INDEX_RONGGAN_RAWIMUSB;
        }
        else if (strstr(i_name, "INSPVASA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSPVASA;
        }
        else if (strstr(i_name, "INSPVASB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSPVASB;
        }
        else if (strstr(i_name, "INSPVAXA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSPVAXA;
        }
        else if (strstr(i_name, "INSPVAXB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSPVAXB;
        }
        else if (strstr(i_name, "RAWIMUA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RAWIMUA;
        }
        else if (strstr(i_name, "RAWIMUB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RAWIMUB;
        }
        else if (strstr(i_name, "RAWIMUSA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RAWIMUSA;
        }
        else if (strstr(i_name, "RAWIMUSB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RAWIMUSB;
        }
        else if (strstr(i_name, "RAWIMUXA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RAWIMUXA;
        }
        else if (strstr(i_name, "RAWIMUXB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RAWIMUXB;
        }
        else if (strstr(i_name, "RAWIMUSXA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RAWIMUSXA;
        }
        else if (strstr(i_name, "RAWIMUSXB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_RAWIMUSXB;
        }
        else if (strstr(i_name, "EVENTALLA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_EVENTALLA;
        }
        else if (strstr(i_name, "EVENTALLB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_EVENTALLB;
        }
        else if (strstr(i_name, "INSRMC") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSRMC;
        }
        else if (strstr(i_name, "GGI") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_GGI;
        }
        else if (strstr(i_name, "AGRICA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_AGRICA;
        }
        else if (strstr(i_name, "AGRICB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_AGRICB;
        }
        else if (strstr(i_name, "INSPTNLPJKSA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_INSPTNLPJKSA;
        }
        else if (strstr(i_name, "BINSPVAA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_BINSPVAA;
        }
        else if (strstr(i_name, "BINSPVAB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_BINSPVAB;
        }
        else if (strstr(i_name, "BINSPOSA") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_BINSPOSA;
        }
        else if (strstr(i_name, "BINSPOSB") != NULL)
        {
            ret = CSHG_GNSS_LOG_INDEX_BINSPOSB;
        }
    }
    return ret;
}
