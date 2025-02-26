#pragma once

#include "global.h"
#include "ntrip.h"
#include "gnss.h"

extern void Thread_Serial(ros::NodeHandle *node, const int i_serial);
extern void Thread_ReadPPS(void);
extern void Thread_Ntrip_Transfer(ros::NodeHandle *node, const int i_serial, const int i_serial2);
