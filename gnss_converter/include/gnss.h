#pragma once

#include "gnss_ptcl_nmea0183.h"
#include "gnss_ptcl_rtcm3.h"
#include "gnss_ptcl_cshg.h"

extern uint32_t g_GnssWeekNum;

// GNSS主流程
extern uint8_t GNSS_MainFuntion(void);

// 解码接收命令
extern int GNSS_Process(const MsgSource i_TpSource,
                        const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen,
                        const uint8_t i_cPtcl, bool *o_pbNeedTrans);

// 检查报文有效性
extern int GNSS_CheckFrameValid(const MsgSource i_xSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, uint8_t *o_pcPtcl, uint32_t *o_pdwStartIndex);
