#pragma once
#include "global.h"

typedef struct
{
    double utc = 0.0;         // 1	UTC time(MSB,先保存时)	定位的UTC时间（时/分/秒）	hhmmss.ss	005806.00
    double lat = 0.0;              // 2	lat	纬度（ddmm.mm）	llll.lllllll	2814.1123380 （N-北纬为正 S-南纬为负）
    /* uint8_t lat_dir; */         // 3	lat dir	纬度方向（N-北纬 S-南纬）	a	N
    double lon = 0.0;              // 4	lon	经度（dddmm.mm）	yyyyy.yyyyyyy	11252.1603250（E-东经为正 W-西经为负）
    /* uint8_t lon_dir; */         // 5	lon dir	经度方向（E-东经 W-西经）	a	E
    uint8_t qual = 0;              // 6	qual	解算状态指示：	x	1
                                   // 		0 = 无效解 1 = 单点解 2 = 伪距差分
                                   //         3 = PPS 4 = 固定解 5 = 浮点解
                                   //         6 = 航位推算 7 = 用户输入 8 = PPP解
    uint8_t sats = 0;              // 7	#sats	参与解算卫星数	xx	31
    float HDOP = 0.0;              // 8	0.8	HDOP值，水平精度因子	x.x	0.8
    float height = 0.0;            // 9	119.3071	海拔高	x.x	119.3071
    /* uint8_t height_unit; */     // 10	M	海拔高单位，米	M	M
    float height_err = 0.0;        // 11	-16.151	高程异常值	x.x	-16.151
    /* uint8_t height_err_unit; */ // 12	M	高程异常单位，米	M	M
    float diff = 0.0;              // 13	0.0	差分数据龄期	xxxx	0.0
    uint16_t diff_ID = 0;          // 14	0000	差分站台ID号，取值范围0000~1023	xxxx
} tGGAData;

typedef struct
{
    double utc = 0.0;         // 1	UTC time(MSB,先保存时)	定位的UTC时间（时/分/秒）	hhmmss.ss	005806.00
    double lat = 0.0;              // 2	lat	纬度（ddmm.mm）	llll.lllllll	2814.1123380 （N-北纬为正 S-南纬为负）
    /* uint8_t lat_dir; */         // 3	lat dir	纬度方向（N-北纬 S-南纬）	a	N
    double lon = 0.0;              // 4	lon	经度（dddmm.mm）	yyyyy.yyyyyyy	11252.1603250（E-东经为正 W-西经为负）
    /* uint8_t lon_dir; */         // 5	lon dir	经度方向（E-东经 W-西经）	a	E
    uint8_t qual = 0;              // 6	qual	解算状态指示：	x	1
                                   // 		0 = 无效解 1 = 单点解 2 = 伪距差分
                                   //         3 = PPS 4 = 固定解 5 = 浮点解
                                   //         6 = 航位推算 7 = 用户输入 8 = PPP解
    uint8_t sats = 0;              // 7	#sats	参与解算卫星数	xx	31
    float HDOP = 0.0;              // 8	0.8	HDOP值，水平精度因子	x.x	0.8
    float height = 0.0;            // 9	119.3071	海拔高	x.x	119.3071
    /* uint8_t height_unit; */     // 10	M	海拔高单位，米	M	M
    float height_err = 0.0;        // 11	-16.151	高程异常值	x.x	-16.151
    /* uint8_t height_err_unit; */ // 12	M	高程异常单位，米	M	M
    float diff = 0.0;              // 13	0.0	差分数据龄期	xxxx	0.0
    uint16_t diff_ID = 0;          // 14	0000	差分站台ID号，取值范围0000~1023	xxxx	0000
    uint8_t rover_fix_level = 0;   // 15
    uint8_t base_fix_level = 0;    // 16
} tGGPData;

#define NMEA0183_CMD_INDEX 0

typedef enum _NMEA_GGA_INDEX
{
    NMEA_GGA_INDEX_CMD = 0,
    NMEA_GGA_INDEX_UTC = 1,
    NMEA_GGA_INDEX_Latitude = 2,
    NMEA_GGA_INDEX_Latitude_Direction = 3, // N-北纬 S-南纬
    NMEA_GGA_INDEX_Longitude = 4,
    NMEA_GGA_INDEX_Longitude_Direction = 5, // （E-东经 W-西经）
    NMEA_GGA_INDEX_Solution_Type = 6,
    NMEA_GGA_INDEX_Satellite_Num = 7,
    NMEA_GGA_INDEX_HDOP = 8,
    NMEA_GGA_INDEX_Altitude = 9,
    NMEA_GGA_INDEX_Altitude_Unit = 10,
    NMEA_GGA_INDEX_ErrZ = 11,
    NMEA_GGA_INDEX_ErrZ_Unit = 12,
    NMEA_GGA_INDEX_Differential_Age = 13,
    NMEA_GGA_INDEX_Differential_ID = 14,
    NMEA_GGA_INDEX_MAX_NUM,
} NMEA_GGA_INDEX;

typedef enum _NMEA_GGP_INDEX
{
    NMEA_GGP_INDEX_CMD = 0,
    NMEA_GGP_INDEX_UTC = 1,
    NMEA_GGP_INDEX_Latitude = 2,
    NMEA_GGP_INDEX_Latitude_Direction = 3, // N-北纬 S-南纬
    NMEA_GGP_INDEX_Longitude = 4,
    NMEA_GGP_INDEX_Longitude_Direction = 5, // E-东经 W-西经）
    NMEA_GGP_INDEX_Solution_Type = 6,
    NMEA_GGP_INDEX_Satellite_Num = 7,
    NMEA_GGP_INDEX_HDOP = 8,
    NMEA_GGP_INDEX_Altitude = 9,
    NMEA_GGP_INDEX_Altitude_Unit = 10,
    NMEA_GGP_INDEX_ErrZ = 11,
    NMEA_GGP_INDEX_ErrZ_Unit = 12,
    NMEA_GGP_INDEX_Differential_Age = 13,
    NMEA_GGP_INDEX_Differential_ID = 14,
    NMEA_GGP_INDEX_ROVER_FIX_LEVEL = 15,
    NMEA_GGP_INDEX_BASE_FIX_LEVEL = 16,
    NMEA_GGP_INDEX_MAX_NUM,
} NMEA_GGP_INDEX;

typedef enum _NMEA_RMC_INDEX
{
    NMEA_RMC_INDEX_CMD = 0,
    NMEA_RMC_INDEX_UTC = 1,
    NMEA_RMC_INDEX_DataStatus = 2,
    NMEA_RMC_INDEX_Latitude = 3,
    NMEA_RMC_INDEX_Latitude_Direction = 4, // N-北纬 S-南纬
    NMEA_RMC_INDEX_Longitude = 5,
    NMEA_RMC_INDEX_Longitude_Direction = 6, // （E-东经 W-西经）
    NMEA_RMC_INDEX_Speed = 7,               // 单位:节，1节=0.51444444444444 m/s=1.852km/h
    NMEA_RMC_INDEX_Direction = 8,           // 航向
    NMEA_RMC_INDEX_Date = 9,                // ddmmyy
    NMEA_RMC_INDEX_MagneticEclination = 10,
    NMEA_RMC_INDEX_MagneticEclination_Direction = 11,
    NMEA_RMC_INDEX_LocateStatus = 12,
    NMEA_RMC_INDEX_NavigationStatus = 13,
    NMEA_RMC_INDEX_MAX_NUM,
} NMEA_RMC_INDEX;

typedef enum _CSHG_INSRAW_INDEX
{
    CSHG_INSRAW_INDEX_CMD = 0,
    CSHG_INSRAW_INDEX_UTC = 1,
    CSHG_INSRAW_INDEX_INS_TIME = 2,
    CSHG_INSRAW_INDEX_GyroX = 3,
    CSHG_INSRAW_INDEX_GyroY = 4,
    CSHG_INSRAW_INDEX_GyroZ = 5,
    CSHG_INSRAW_INDEX_AccelX = 6,
    CSHG_INSRAW_INDEX_AccelY = 7,
    CSHG_INSRAW_INDEX_AccelZ = 8,
    CSHG_INSRAW_INDEX_OdomVel = 9,
    CSHG_INSRAW_INDEX_OdomFlag = 10,
    CSHG_INSRAW_INDEX_Temp = 11,
    CSHG_INSRAW_INDEX_MAX_NUM,
} CSHG_INSRAW_INDEX;

typedef enum _NMEA_GSV_INDEX
{
    NMEA_GSV_INDEX_CMD = 0,
    NMEA_GSV_INDEX_CMD_NUM = 1,
    NMEA_GSV_INDEX_CMD_INDEX = 2,
    NMEA_GSV_INDEX_NUM = 3,
    NMEA_GSV_INDEX_PRN1 = 4,
    NMEA_GSV_INDEX_ELEVATION1 = 5,
    NMEA_GSV_INDEX_AZIMUTH1 = 6,
    NMEA_GSV_INDEX_NOISE1 = 7,
    NMEA_GSV_INDEX_PRN2 = 8,
    NMEA_GSV_INDEX_ELEVATION2 = 9,
    NMEA_GSV_INDEX_AZIMUTH2 = 10,
    NMEA_GSV_INDEX_NOISE2 = 11,
    NMEA_GSV_INDEX_PRN3 = 12,
    NMEA_GSV_INDEX_ELEVATION3 = 13,
    NMEA_GSV_INDEX_AZIMUTH3 = 14,
    NMEA_GSV_INDEX_NOISE3 = 15,
    NMEA_GSV_INDEX_PRN4 = 16,
    NMEA_GSV_INDEX_ELEVATION4 = 17,
    NMEA_GSV_INDEX_AZIMUTH4 = 18,
    NMEA_GSV_INDEX_NOISE4 = 19,
    NMEA_GSV_INDEX_MAX_NUM,
} NMEA_GSV_INDEX;

extern bool CSHG_GGA_IsNeed(void);
extern bool CSHG_GGP_IsNeed(void);
extern int NMEA0183_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans);
extern int NMEA0183_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen);

extern int PackNMEA_GGA(char *buf, int MaxBuffLen);