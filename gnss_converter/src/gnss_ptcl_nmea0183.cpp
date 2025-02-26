#include "gnss_ptcl_nmea0183.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

bool CSHG_GGA_IsNeed(void)
{
    if (g_Config_param.enable_read_gga >= 0)
    {
        return true;
    }
    return false;
}

bool CSHG_GGP_IsNeed(void)
{
    if (g_Config_param.enable_read_ggp >= 0)
    {
        return true;
    }
    return false;
}

int NMEA0183_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans)
{
    if (i_TpSource == MsgSource_UART_GNSS) // 只处理从gnss模块接收的命令
    {
        char tempString[256] = {0};
        memcpy(tempString, i_pubInBuf, i_udwInBufLen);
        std::map<int, std::string> mapString;
        char *startPtr = tempString + 1;
        char *endPtr = strchr(startPtr, '\r');
        *endPtr = 0;
        char *curPtr = strchr(startPtr, ',');
        int no = 0;
        while (curPtr != nullptr)
        {
            *curPtr = 0;
            mapString.insert(std::pair<int, std::string>(no++, startPtr));
            startPtr = curPtr + 1;
            curPtr = strchr(startPtr, ',');
        }
        mapString.insert(std::pair<int, std::string>(no++, startPtr));

        if (strstr(mapString[NMEA0183_CMD_INDEX].c_str(), "GGA") != NULL &&
            mapString.size() == NMEA_GGA_INDEX_MAX_NUM)
        {
            if (mapString[NMEA_GGA_INDEX_UTC].size() > 0 &&
                mapString[NMEA_GGA_INDEX_Latitude].size() > 0 &&
                mapString[NMEA_GGA_INDEX_Longitude].size() > 0)
            {
                tGGAData ggaData;
                ggaData.utc = std::stod(mapString[NMEA_GGA_INDEX_UTC]);
                ggaData.lat = dmm2deg(std::stod(mapString[NMEA_GGA_INDEX_Latitude]));
                if (strchr(mapString[NMEA_GGA_INDEX_Latitude_Direction].c_str(), 'S') != nullptr)
                {
                    ggaData.lat = -ggaData.lat;
                }
                ggaData.lon = dmm2deg(std::stod(mapString[NMEA_GGA_INDEX_Longitude]));
                if (strchr(mapString[NMEA_GGA_INDEX_Longitude_Direction].c_str(), 'W') != nullptr)
                {
                    ggaData.lon = -ggaData.lon;
                }
                if (mapString[NMEA_GGA_INDEX_Solution_Type].size() > 0)
                {
                    ggaData.qual = std::stoi(mapString[NMEA_GGA_INDEX_Solution_Type]);
                }
                if (mapString[NMEA_GGA_INDEX_Satellite_Num].size() > 0)
                {
                    ggaData.sats = std::stoi(mapString[NMEA_GGA_INDEX_Satellite_Num]);
                }
                if (mapString[NMEA_GGA_INDEX_HDOP].size() > 0)
                {
                    ggaData.HDOP = std::stod(mapString[NMEA_GGA_INDEX_HDOP]);
                }
                if (mapString[NMEA_GGA_INDEX_Altitude].size() > 0)
                {
                    ggaData.height = std::stod(mapString[NMEA_GGA_INDEX_Altitude]);
                }
                if (mapString[NMEA_GGA_INDEX_ErrZ].size() > 0)
                {
                    ggaData.height_err = std::stod(mapString[NMEA_GGA_INDEX_ErrZ]);
                }
                if (mapString[NMEA_GGA_INDEX_Differential_Age].size() > 0)
                {
                    ggaData.diff = std::stod(mapString[NMEA_GGA_INDEX_Differential_Age]);
                }
                if (mapString[NMEA_GGA_INDEX_Differential_ID].size() > 0)
                {
                    ggaData.diff_ID = std::stod(mapString[NMEA_GGA_INDEX_Differential_ID]);
                }
                if (g_Config_param.enable_read_gga > 0)
                {
                    csjw_msgs::gga gga_msg;
                    gga_msg.utc_seconds = ggaData.utc;

                    gga_msg.lat = ggaData.lat;
                    gga_msg.lon = ggaData.lon;

                    gga_msg.gps_qual = ggaData.qual;

                    gga_msg.num_sats = ggaData.sats;
                    gga_msg.hdop = ggaData.HDOP;
                    gga_msg.alt = ggaData.height;
                    gga_msg.undulation = ggaData.height_err;
                    gga_msg.diff_age = ggaData.diff;
                    gga_msg.station_id = ggaData.diff_ID;

                    g_pub_gga.publish(gga_msg);
                }
            }

            *o_pbNeedTrans = true;
            return (int)i_udwInBufLen;
        }
        else if (strstr(mapString[NMEA0183_CMD_INDEX].c_str(), "GGP") != NULL &&
                 mapString.size() == NMEA_GGP_INDEX_MAX_NUM)
        {
            if (mapString[NMEA_GGP_INDEX_UTC].size() > 0 &&
                mapString[NMEA_GGP_INDEX_Latitude].size() > 0 &&
                mapString[NMEA_GGP_INDEX_Longitude].size() > 0)
            {
                tGGPData ggpData;
                ggpData.utc = std::stod(mapString[NMEA_GGP_INDEX_UTC]);
                ggpData.lat = dmm2deg(std::stod(mapString[NMEA_GGP_INDEX_Latitude]));
                if (strchr(mapString[NMEA_GGP_INDEX_Latitude_Direction].c_str(), 'S') != nullptr)
                {
                    ggpData.lat = -ggpData.lat;
                }
                ggpData.lon = dmm2deg(std::stod(mapString[NMEA_GGP_INDEX_Longitude]));
                if (strchr(mapString[NMEA_GGP_INDEX_Longitude_Direction].c_str(), 'W') != nullptr)
                {
                    ggpData.lon = -ggpData.lon;
                }
                if (mapString[NMEA_GGP_INDEX_Solution_Type].size() > 0)
                {
                    ggpData.qual = std::stoi(mapString[NMEA_GGP_INDEX_Solution_Type]);
                }
                if (mapString[NMEA_GGP_INDEX_Satellite_Num].size() > 0)
                {
                    ggpData.sats = std::stoi(mapString[NMEA_GGP_INDEX_Satellite_Num]);
                }
                if (mapString[NMEA_GGP_INDEX_HDOP].size() > 0)
                {
                    ggpData.HDOP = std::stod(mapString[NMEA_GGP_INDEX_HDOP]);
                }
                if (mapString[NMEA_GGP_INDEX_Altitude].size() > 0)
                {
                    ggpData.height = std::stod(mapString[NMEA_GGP_INDEX_Altitude]);
                }
                if (mapString[NMEA_GGP_INDEX_ErrZ].size() > 0)
                {
                    ggpData.height_err = std::stod(mapString[NMEA_GGP_INDEX_ErrZ]);
                }
                if (mapString[NMEA_GGP_INDEX_Differential_Age].size() > 0)
                {
                    ggpData.diff = std::stod(mapString[NMEA_GGP_INDEX_Differential_Age]);
                }
                if (mapString[NMEA_GGP_INDEX_Differential_ID].size() > 0)
                {
                    ggpData.diff_ID = std::stod(mapString[NMEA_GGP_INDEX_Differential_ID]);
                }
                if (mapString[NMEA_GGP_INDEX_ROVER_FIX_LEVEL].size() > 0)
                {
                    ggpData.rover_fix_level = std::stod(mapString[NMEA_GGP_INDEX_ROVER_FIX_LEVEL]);
                }
                if (mapString[NMEA_GGP_INDEX_BASE_FIX_LEVEL].size() > 0)
                {
                    ggpData.base_fix_level = std::stod(mapString[NMEA_GGP_INDEX_BASE_FIX_LEVEL]);
                }
                if (g_Config_param.enable_read_ggp > 0)
                {
                    csjw_msgs::ggp ggp_msg;
                    ggp_msg.utc_seconds = ggpData.utc;

                    ggp_msg.lat = ggpData.lat;
                    ggp_msg.lon = ggpData.lon;

                    ggp_msg.gps_qual = ggpData.qual;

                    ggp_msg.num_sats = ggpData.sats;
                    ggp_msg.hdop = ggpData.HDOP;
                    ggp_msg.alt = ggpData.height;
                    ggp_msg.undulation = ggpData.height_err;
                    ggp_msg.diff_age = ggpData.diff;
                    ggp_msg.station_id = ggpData.diff_ID;
                    ggp_msg.rover_fix_level = ggpData.rover_fix_level;
                    ggp_msg.base_fix_level = ggpData.base_fix_level;

                    g_pub_ggp.publish(ggp_msg);
                }
            }
            *o_pbNeedTrans = true;
            return (int)i_udwInBufLen;
        }
        else if (strstr(mapString[NMEA0183_CMD_INDEX].c_str(), "RMC") != NULL &&
                 mapString.size() == NMEA_RMC_INDEX_MAX_NUM)
        {
            *o_pbNeedTrans = true;
            return (int)i_udwInBufLen;
        }
        else if (strstr(mapString[NMEA0183_CMD_INDEX].c_str(), "GSV") != NULL &&
                 mapString.size() == NMEA_GSV_INDEX_MAX_NUM)
        {
            *o_pbNeedTrans = true;
            return (int)i_udwInBufLen;
        }
    }
    // 不是来自模块的协议不透传
    *o_pbNeedTrans = false;
    return (int)i_udwInBufLen;
}

int NMEA0183_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen)
{
    if (i_udwInBufLen >= 6 &&
        i_pubInBuf[0] == '$')
    {
        // 找到 '$' 起始符
        uint8_t cs = 0;
        for (uint32_t end = 1; end + 5 <= i_udwInBufLen; end++)
        {
            if (i_pubInBuf[end] < 0x20 || i_pubInBuf[end] > 0x7E)
            {
                break;
            }
            if (i_pubInBuf[end] != '*')
            {
                cs ^= i_pubInBuf[end];
            }
            else if (i_pubInBuf[end + 3] == 0x0D &&
                     i_pubInBuf[end + 4] == 0x0A)
            {
                // 找到 '*' 以及 '\r\n' 结束符
                char hexCS[3];
                sprintf(hexCS, "%02X", cs);
                if ((hexCS[0] == i_pubInBuf[end + 1] || hexCS[0] + 32 == i_pubInBuf[end + 1]) && // 32 == 'a' - 'A'
                    (hexCS[1] == i_pubInBuf[end + 2] || hexCS[1] + 32 == i_pubInBuf[end + 2]))
                {
                    // 校验码通过
                    return (int)end + 5;
                }
            }
        }
    }
    return -1;
}

int PackNMEA_GGA(char *buf, int MaxBuffLen)
{
    if (buf != nullptr)
    {
        struct timeval out_tv;
        struct tm out_tm;
        char *q, sum = 0, temp[8];
        GetSysDateTime(&out_tv, &out_tm);

        sprintf(buf, "$GPGGA,%02d%02d%02d.%02d,%010.7lf,%s,%010.7lf,%s,%d,0,0,%.3lf,M,0.0,M,0,0",
                out_tm.tm_hour, out_tm.tm_min, out_tm.tm_sec, (int)(out_tv.tv_usec / 10000),
                deg2dmm(std::fabs(g_last_latitude)), g_last_latitude >= 0.0 ? "N" : "S",
                deg2dmm(std::fabs(g_last_longitude)), g_last_longitude >= 0.0 ? "E" : "W",
                1, g_last_altitude);
        for (q = buf + 1, sum = 0; *q; q++)
            sum ^= *q; /* check-sum */
        sprintf(temp, "*%02X\r\n", sum);
        strcat(buf, temp);
        return strlen(buf);
    }
    return -1;
}
