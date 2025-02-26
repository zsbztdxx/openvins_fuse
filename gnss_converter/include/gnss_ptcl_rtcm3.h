#pragma once

#include "global.h"

extern int RTCM3_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans);
extern int RTCM3_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen);
