
#include "gnss.h"
 uint32_t g_GnssWeekNum = 0;
// 解码接收命令
int GNSS_Process(const MsgSource i_TpSource,
                 const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen,
                 const uint8_t i_cPtcl, bool *o_pbNeedTrans)
{
    if (i_pubInBuf != NULL)
    {
        switch (i_cPtcl)
        {
        case PROTOCOL_RTCM3:
            return RTCM3_Process(i_TpSource, (uint8_t *)i_pubInBuf, i_udwInBufLen, o_pbNeedTrans);
        case PROTOCOL_CSHG_GNSS_BIN:
            return CSHG_GNSS_BIN_Process(i_TpSource, i_pubInBuf, i_udwInBufLen, o_pbNeedTrans);
        case PROTOCOL_CSHG_GNSS_BIN2:
            return CSHG_GNSS_BIN2_Process(i_TpSource, i_pubInBuf, i_udwInBufLen, o_pbNeedTrans);
        case PROTOCOL_CSHG_GNSS_BIN3:
            return CSHG_GNSS_BIN3_Process(i_TpSource, i_pubInBuf, i_udwInBufLen, o_pbNeedTrans);
        case PROTOCOL_CSHG_GNSS_BIN4:
            return CSHG_GNSS_BIN4_Process(i_TpSource, i_pubInBuf, i_udwInBufLen, o_pbNeedTrans);
        case PROTOCOL_CSHG_GNSS_ASCII:
            return CSHG_GNSS_ASCII_Process(i_TpSource, (uint8_t *)i_pubInBuf, i_udwInBufLen, o_pbNeedTrans);
        case PROTOCOL_CSHG_GNSS_ASCII2:
            return CSHG_GNSS_ASCII2_Process(i_TpSource, (uint8_t *)i_pubInBuf, i_udwInBufLen, o_pbNeedTrans);
        case PROTOCOL_NMEA0183:
            return NMEA0183_Process(i_TpSource, (uint8_t *)i_pubInBuf, i_udwInBufLen, o_pbNeedTrans);
        default:
            break;
        }
    }
    return -1;
}

// 检查报文有效性
int GNSS_CheckFrameValid(const MsgSource i_xSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, uint8_t *o_pcPtcl, uint32_t *o_pdwStartIndex)
{
    if (i_pubInBuf != NULL && o_pcPtcl != NULL && o_pdwStartIndex != NULL && i_udwInBufLen > 0)
    {
        // uint8_t cs;
        int ret = -1;
        uint32_t udwInBufLen = i_udwInBufLen;
        uint8_t *pubInBuf_Start1 = NULL; // 发现 $ 最早位置 (CSHG_GNSS_ASCII 以及 NMEA 需要)
        uint8_t *pubInBuf_Start2 = NULL; // 发现 # 最早位置 (CSHG_GNSS_ASCII2 需要)
        // uint8_t *pubInBuf_Start3 = NULL; // 发现 "cshg" 最早位置 (CSHG_GNSS_CMD 需要)
        // uint8_t *pubInBuf_Start4 = NULL; // 发现 > 最早位置 (CSHG_GNSS_CMD_RET 需要)
        uint8_t *pubInBuf_End = NULL; // 发现 \r\n 最早位置 (各个ASCII协议 需要)
        *o_pcPtcl = PROTOCOL_UNKNOWN;
        *o_pdwStartIndex = 0; // 帧起始第一字节的offset
        for (uint8_t *pubInBuf = (uint8_t *)i_pubInBuf;
             pubInBuf <= (uint8_t *)i_pubInBuf + i_udwInBufLen - 2;
             pubInBuf++, udwInBufLen--, (*o_pdwStartIndex)++)
        {
            if ((ret = RTCM3_CheckFrameValid(pubInBuf, udwInBufLen)) >= 0)
            {
                *o_pcPtcl = PROTOCOL_RTCM3;
            }
            // INSPVAB
            else if ((ret = CSHG_GNSS_BIN_CheckFrameValid(pubInBuf, udwInBufLen)) >= 0)
            {
                if (i_xSource == MsgSource_UART_GNSS)
                    *o_pcPtcl = PROTOCOL_CSHG_GNSS_BIN;
            }
            // POSDATAB/IMUDATAB ODMDATAB
            else if ((ret = CSHG_GNSS_BIN2_CheckFrameValid(pubInBuf, udwInBufLen)) >= 0)
            {
                *o_pcPtcl = PROTOCOL_CSHG_GNSS_BIN2;
            }
            // RANGEB
            else if ((ret = CSHG_GNSS_BIN3_CheckFrameValid(pubInBuf, udwInBufLen)) >= 0)
            {
                if (i_xSource == MsgSource_UART_GNSS)
                    *o_pcPtcl = PROTOCOL_CSHG_GNSS_BIN3;
            }
            // RAWIMUSB
            else if ((ret = CSHG_GNSS_BIN4_CheckFrameValid(pubInBuf, udwInBufLen)) >= 0)
            {
                *o_pcPtcl = PROTOCOL_CSHG_GNSS_BIN4;
            }

            if (ret < 0)
            {
                if (pubInBuf_Start1 == NULL && pubInBuf[0] == '$')
                {
                    pubInBuf_Start1 = pubInBuf;
                    pubInBuf_End = NULL;
                }
                else if (pubInBuf_Start2 == NULL && pubInBuf[0] == '#')
                {
                    pubInBuf_Start2 = pubInBuf;
                    pubInBuf_End = NULL;
                }
                else if (pubInBuf_End == NULL && udwInBufLen >= 2 &&
                         pubInBuf[0] == '\r' && pubInBuf[1] == '\n')
                {
                    pubInBuf_End = pubInBuf;
                }

                if (ret < 0)
                {
                    // GGA优先级(触发几率)适中偏低(输出频率相对低)
                    if (pubInBuf_End != NULL && pubInBuf_Start1 != NULL &&
                        (ret = NMEA0183_CheckFrameValid(pubInBuf_Start1, pubInBuf_End - pubInBuf_Start1 + 2)) >= 0)
                    {
                        if (i_xSource == MsgSource_UART_GNSS)
                            *o_pcPtcl = PROTOCOL_NMEA0183;
                        *o_pdwStartIndex = (int)(pubInBuf_Start1 - i_pubInBuf);
                    }
                    // INSPVAA POSDATAA ODMDATAA
                    else if (pubInBuf_End != NULL && pubInBuf_Start1 != NULL &&
                             (ret = CSHG_GNSS_ASCII_CheckFrameValid(pubInBuf_Start1, pubInBuf_End - pubInBuf_Start1 + 2)) >= 0)
                    {
                        *o_pcPtcl = PROTOCOL_CSHG_GNSS_ASCII;
                        *o_pdwStartIndex = (int)(pubInBuf_Start1 - i_pubInBuf);
                    }
                    // HEADINGA优先级(触发几率)适中(输出频率高)
                    else if (pubInBuf_End != NULL && pubInBuf_Start2 != NULL &&
                             (ret = CSHG_GNSS_ASCII2_CheckFrameValid(pubInBuf_Start2, pubInBuf_End - pubInBuf_Start2 + 2)) >= 0)
                    {
                        if (i_xSource == MsgSource_UART_GNSS)
                            *o_pcPtcl = PROTOCOL_CSHG_GNSS_ASCII2;
                        *o_pdwStartIndex = (int)(pubInBuf_Start2 - i_pubInBuf);
                    }
                }
                if(pubInBuf_End != NULL)
                {
                    pubInBuf_End = NULL;
                    pubInBuf_Start1 = NULL;
                    pubInBuf_Start2 = NULL;
                }
            }
            if (ret >= 0)
            {
                return ret + (int)(*o_pdwStartIndex);
            }
        }
    }
    return -1;
}
