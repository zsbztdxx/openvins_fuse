#include "gnss_ptcl_rtcm3.h"

#include <stdio.h>
#include "global.h"

int RTCM3_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen)
{
    if (i_udwInBufLen >= 7 &&           // 命令最小长度判断
        i_pubInBuf[0] == 0xD3 &&        // 命令头判断
        (i_pubInBuf[1] & 0xFC) == 0x00) // 命令头判断
    {
        uint16_t wLen = i_pubInBuf[2] + ((i_pubInBuf[1] & 0x3) << 8);
        if ((uint32_t)wLen + 6 <= i_udwInBufLen) // 命令长度判断
        {
            uint32_t inCRC = 0, calCRC = 0;
            // 取校验码
            inCRC = ((uint32_t)i_pubInBuf[3 + wLen] << 16) +
                    ((uint32_t)i_pubInBuf[3 + wLen + 1] << 8) +
                    (uint32_t)i_pubInBuf[3 + wLen + 2];
            // 计算校验码
            calCRC = rtk_crc24q(0, &i_pubInBuf[0], wLen + 3);
            if (inCRC == calCRC) // 校验码判断
            {
                return wLen + 6;
            }
        }
    }
    return -1;
}
int RTCM3_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans)
{
    // 实际上非从gnss模块接收的RTCM 不进入此函数(因在CheckFrameValid后直接透传了)
    (void)i_pubInBuf;
    if (i_TpSource == MsgSource_UART_4G)
    {
        *o_pbNeedTrans = true;
        return (int)i_udwInBufLen;
    }
    *o_pbNeedTrans = false;
    return -1;
}