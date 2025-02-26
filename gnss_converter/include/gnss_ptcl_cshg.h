#pragma once

#include "global.h"
#include "gnss.h"

typedef struct
{
    uint16_t weekNum; // GPS周：自1980-1-6至当前的星期数（格林尼治时间）
    double secInWeek; // 周内秒, 自本周日0:00:00至当前的秒数
    // 惯导状态
    // 0:INS_INACTIVE:对准未激活
    // 1:INS_ALIGNING:正在粗对准
    // 2:INS_HIGH_VARIANCE:协方差较大,位姿未收敛
    // 3:INS_SOLUTION_GOOD:对准结果较好
    // 6:INS_SOLUTION_FREE:对准结果较差
    // 7:INS_ALIGNMENT_COMPLETE:粗对准完成
    // 8:DETERMINING_ORIENTATION:正在IMU重力对齐
    // 9:WAITING_INITIALPOS:等待位置解
    // 10:WAITING_AZIMUTH:等待航向角
    // 11:INITIALIZING_BIASES:初始化偏差
    // 12:MOTION_DETECT:检测到运动
    uint32_t insState; // ins惯导状态
    // 0:NONE:未定位
    // 1:FIXEDPOS:参数输入固定位置
    // 16:SINGLE:单点解
    // 17:PSRDIFF:差分解
    // 34:NARROW_FLOAT:窄带浮点解
    // 50:NARROW_INT:窄带固定解
    // 53:INS_PSRSP:INS伪距单点解
    // 54:INS_PSRDIFF:INS差分解
    // 55:INS_RTKFLOAT:INS浮点解
    // 56:INS_RTKFIXED:INS固定解
    // 69:PPP:精密单点解
    uint32_t posStatus;       // pos解算状态
    double lat;               // 纬度(度)
    double lon;               // 经度(度)
    double hgt;               // 海拔高
    float hgtErr;             // 高程异常值
    double vn;                // 北向速度（m/s）
    double ve;                // 东向速度（m/s）
    double vu;                // 天向速度（m/s）
    double roll;              // 横滚(度)
    double pitch;             // 俯仰角(度)
    double yaw;               // 航向角(度)
    float std_lat;            // 纬度(度) 标准差
    float std_lon;            // 经度(度)标准差
    float std_hgt;            // 大地高 标准差
    float std_vn;             // 北向速度（m/s）标准差
    float std_ve;             // 东向速度（m/s）标准差
    float std_vu;             // 天向速度（m/s）标准差
    float std_roll;           // 横滚(度)标准差
    float std_pitch;          // 俯仰角(度)标准差
    float std_yaw;            // 航向角(度)标准差
    uint32_t statusEx;        // 扩展解算状态
    uint16_t timeSinceUpdate; // 距离上次位置更新时间
} tINSPVAXData;

typedef struct
{
    uint8_t count10ms;   // 1	count	秒内10ms计数值	Uint8
    double gpstime;      // 2	gpstime	gps时间（惯导时）	FP64
    uint8_t ins_status;  // 3	ins_status	组合解算状态	Uint8
    double ins_Lat;      // 4	ins_Lat	组合纬度(度)	FP64
    double ins_Lon;      // 5	ins_Lon	组合经度(度)	FP64
    float ins_Hgt;       // 6	ins_Hgt	组合高度(m)	FP32
    float ve;            // 7	ve	组合东向速度（m/s）	FP32
    float vn;            // 8	vn	组合北向速度（m/s）	FP32
    float vu;            // 9	vu	组合天向速度（m/s）	FP32
    float pitch;         // 10	pitch	组合俯仰角(度)	FP32
    float roll;          // 11	roll	组合横滚(度)	FP32
    float head;          // 12	head	组合航向角(度)	FP32
    uint8_t gnss_status; // 13	gnss_status	卫导定位状态	Uint8
    uint8_t headstatus;  // 14	headstatus	卫导定向状态	Uint8
    float Baseline;      // 15	Baseline	基线长度	FP32
    double gnss_Lat;     // 16	gnss_Lat	卫导纬度(度)	FP64
    double gnss_Lon;     // 17	gnss_Lon	卫导经度(度)	FP64
    float gnss_Hgt;      // 18	gnss_Hgt	卫导高度(m)	FP32
    float gnss_normv;    // 19	gnss_normv	卫导合速度（m/s）	FP32
    float Hdop;          // 20	Hdop	卫导水平精度因子	FP32
    uint8_t gnss_satM;   // 21	gnss_satM	主天线卫星数（个）	Uint8
    uint8_t gnss_satS;   // 22	gnss_satS	从天线卫星数（个）	Uint8
    float odov;          // 23	odov	里程计速度（m/s）	FP32
    uint8_t odoflag;     // 24	odoflag	里程计标志（1-里程计速度有效 0-无里程计速度或速度无效）	Uint8
    float gyrox;         // 25	gyrox	陀螺仪X轴(°/s)	FP32
    float gyroy;         // 26	gyroy	陀螺仪Y轴(°/s)	FP32
    float gyroz;         // 27	gyroz	陀螺仪Z轴(°/s)	FP32
    float accex;         // 28	accex	加计X轴(m/s^2)	FP32
    float accey;         // 29	accey	加计Y轴(m/s^2)	FP32
    float accez;         // 30	accez	加计Z轴(m/s^2)	FP32
    float accstdval;     // 31	accstdval	加计方差	FP32
} tINSPVAData;

typedef struct
{
    uint8_t count;      // 1	count	秒内10ms计数值	Uint8
    double gpstime;     // 2	gpstime	gps时间（惯导时）	FP64
    uint8_t ins_status; // 3	ins_status	组合解算状态	Uint8
    double ins_Lat;     // 4	ins_Lat	组合纬度(度)	FP64
    double ins_Lon;     // 5	ins_Lon	组合经度(度)	FP64
    float ins_Hgt;      // 6	ins_Hgt	组合高度(m)	FP32
    float ve;           // 7	ve	组合东向速度（m/s）	FP32
    float vn;           // 8	vn	组合北向速度（m/s）	FP32
    float vu;           // 9	vu	组合天向速度（m/s）	FP32
    float pitch;        // 10	pitch	组合俯仰角(度)	FP32
    float roll;         // 11	roll	组合横滚(度)	FP32
    float head;         // 12	head	组合航向角(度)	FP32
} tINSPOSData;

typedef struct
{
    uint8_t count;   // 1	count	秒内10ms计数值	Uint8
    double gpstime;  // 2	gpstime	gps时间（惯导时）	FP64
    float gyrox;     // 3	gyrox	陀螺仪X轴(°/s)	FP32
    float gyroy;     // 4	gyroy	陀螺仪Y轴(°/s)	FP32
    float gyroz;     // 5	gyroz	陀螺仪Z轴(°/s)	FP32
    float accex;     // 6	accex	加计X轴(m/s^2)	FP32
    float accey;     // 7	accey	加计Y轴(m/s^2)	FP32
    float accez;     // 8	accez	加计Z轴(m/s^2)	FP32
    float accstdval; // 9	accstdval	加计方差	FP32
} tINSIMUData;

typedef struct
{
    uint8_t m_iTimeSystem; // 1 0-无效系统，1-BDS, 2-GPS，3-GAL，4-GLO, 5~7-保留
    uint16_t m_iWeekNum;   // 2 BDS周：自2006-1-1至当前的星期数（北京时间）GPS周：自1980-1-6至当前的星期数（格林尼治时间）
    double m_dSecInWeek;   // 3 自本周日0:00:00至当前的秒数
    int8_t m_iLeapSecond;  // 闰秒, 周内秒+周数+闰秒=UTC

    uint8_t m_iSysStatus; // 4 0：未定位1：纯卫导2：组合导航3：纯惯导
    uint8_t m_iPosType;   // 5 0 = 无效解 1 = 单点解 2 = 伪距差分 3 = PPS 4 = 固定解 5 = 浮点解 6 = 航位推算 7 = 用户输入 8 = PPP
    uint8_t m_iAziType;   // 6 0 = 无效解 1 = 单点解 2 = 伪距差分 3 = PPS 4 = 固定解 5 = 浮点解 6 = 航位推算 7 = 用户输入 8 = PPP
    double m_dLatitude;   // 7 北纬为正，南纬为负，范围[-90,+90]，8位小数
    double m_dLongitude;  // 8 东经为正，西经为负，范围[-180,+180]，8位小数
    double m_dAltitude;   // 9 范围[-20000,+20000],3位小数

    float m_fVelE; // 10 ENU下东向速度
    float m_fVelN; // 11 ENU下北向速度
    float m_fVelU; // 12 ENU下对天速度

    float m_fPitch;   // 13 俯仰角
    float m_fRoll;    // 14 横滚角
    float m_fAzimuth; // 15 航向角

    float m_fStdLat;      // 16 纬度标准差
    float m_fStdLong;     // 17 经度标准差
    float m_fStdAltitude; // 18 高程标准差

    float m_fStdVelE; // 19 东向速度标准差
    float m_fStdVelN; // 20 北向速度标准差
    float m_fStdVelU; // 21 天向速度标准差

    float m_fStdPitch;   // 22 俯仰角标准差
    float m_fStdRoll;    // 23 横滚角标准差
    float m_fStdAzimuth; // 24 航向角标准差

    uint8_t m_iGnssSatM; // 25 主天线卫星数
    uint8_t m_iGnssSatS; // 26 从天线卫星数
    float m_fDiffAge;    // 27 差分龄期

    uint8_t m_iOdoFlag; // 28 1-里程计速度有效 0-无里程计速度，空：无里程计
    uint8_t m_iGear;    // 29 0-N; 1-D; 2-R; 3-P; 4~6-备用

    float m_fFlWheelSpeed; // 30 前左轮速
    float m_fFrWheelSpeed; // 31 前右轮速
    float m_fRlWheelSpeed; // 32 后左轮速
    float m_fRrWheelSpeed; // 33 后右轮速

    uint8_t res[8]; // 保留
} tPOSDATAData;

typedef struct
{
    uint8_t m_iTimeSystem; // 1 0-无效系统，1-BDS, 2-GPS，3-GAL，4-GLO, 5~7-保留
    uint16_t m_iWeekNum;   // 2 BDS周：自2006-1-1至当前的星期数（北京时间）GPS周：自1980-1-6至当前的星期数（格林尼治时间）
    double m_dSecInWeek;   // 3 自本周日0:00:00至当前的秒数
    int8_t m_iLeapSecond;  // 4 闰秒, 周内秒+周数+闰秒=UTC

    uint8_t m_iPosState; // 5 卫导定位状态: 0 = 无效解 1 = 单点解 2 = 伪距差分 3 = PPS 4 = 固定解 5 = 浮点解 6 = 航位推算 7 = 用户输入 8 = PPP
    uint8_t m_iAziState; // 5 卫导定向状态: 0 = 无效解 1 = 单点解 2 = 伪距差分 3 = PPS 4 = 固定解 5 = 浮点解 6 = 航位推算 7 = 用户输入 8 = PPP
    double m_dLatitude;  // 7 北纬为正，南纬为负，范围[-90,+90]，8位小数
    double m_dLongitude; // 8 东经为正，西经为负，范围[-180,+180]，8位小数
    double m_dAltitude;  // 9 范围[-20000,+20000],3位小数

    float m_fVelE; // 10 ENU下东向速度
    float m_fVelN; // 11 ENU下北向速度
    float m_fVelU; // 12 ENU下对天速度

    float m_fPitch;   // 13 俯仰角
    float m_fRoll;    // 14 横滚角
    float m_fAzimuth; // 15 航向角

    float m_fStdLat;      // 16 纬度标准差
    float m_fStdLong;     // 17 经度标准差
    float m_fStdAltitude; // 18 高程标准差

    float m_fStdVelE; // 19 东向速度标准差
    float m_fStdVelN; // 20 北向速度标准差
    float m_fStdVelU; // 21 天向速度标准差

    float m_fStdPitch;   // 22 俯仰角标准差
    float m_fStdRoll;    // 23 横滚角标准差
    float m_fStdAzimuth; // 24 航向角标准差

    uint8_t m_iGnssSatM; // 25 天线卫星数
    uint8_t m_iGnssSatS; // 25 天线卫星数
    float m_fDiffAge;    // 26 差分龄期
    uint8_t res[8];      // 保留
} tSATNAVData;

typedef struct
{
    uint8_t m_iTimeSystem; // 1 0-无效系统，1-BDS, 2-GPS，3-GAL，4-GLO, 5~7-保留
    uint16_t m_iWeekNum;   // 2 BDS周：自2006-1-1至当前的星期数（北京时间）GPS周：自1980-1-6至当前的星期数（格林尼治时间）
    double m_dSecInWeek;   // 3 自本周日0:00:00至当前的秒数
    int8_t m_iLeapSecond;  // 4 闰秒, 周内秒+周数+闰秒=UTC

    // 惯导状态
    // 0:INS_INACTIVE:对准未激活
    // 1:INS_ALIGNING:正在粗对准
    // 2:INS_HIGH_VARIANCE:协方差较大,位姿未收敛
    // 3:INS_SOLUTION_GOOD:对准结果较好
    // 6:INS_SOLUTION_FREE:对准结果较差
    // 7:INS_ALIGNMENT_COMPLETE:粗对准完成
    // 8:DETERMINING_ORIENTATION:正在IMU重力对齐
    // 9:WAITING_INITIALPOS:等待位置解
    // 10:WAITING_AZIMUTH:等待航向角
    // 11:INITIALIZING_BIASES:初始化偏差
    // 12:MOTION_DETECT:检测到运动
    uint8_t m_iInsState; // 5 惯导状态

    // 0:NONE:未定位
    // 1:FIXEDPOS:参数输入固定位置
    // 16:SINGLE:单点解
    // 17:PSRDIFF:差分解
    // 34:NARROW_FLOAT:窄带浮点解
    // 50:NARROW_INT:窄带固定解
    // 53:INS_PSRSP:INS伪距单点解
    // 54:INS_PSRDIFF:INS差分解
    // 55:INS_RTKFLOAT:INS浮点解
    // 56:INS_RTKFIXED:INS固定解
    // 69:PPP:精密单点解
    uint8_t m_iPosState; // 5 定位状态

    double m_dLatitude;  // 6 北纬为正，南纬为负，范围[-90,+90]，8位小数
    double m_dLongitude; // 7 东经为正，西经为负，范围[-180,+180]，8位小数
    double m_dAltitude;  // 8 范围[-20000,+20000],3位小数

    float m_fVelE; // 10 ENU下东向速度
    float m_fVelN; // 11 ENU下北向速度
    float m_fVelU; // 12 ENU下对天速度

    float m_fPitch;   // 13 俯仰角
    float m_fRoll;    // 14 横滚角
    float m_fAzimuth; // 15 航向角

    float m_fStdLat;      // 16 纬度标准差
    float m_fStdLong;     // 17 经度标准差
    float m_fStdAltitude; // 18 高程标准差

    float m_fStdVelE; // 19 东向速度标准差
    float m_fStdVelN; // 20 北向速度标准差
    float m_fStdVelU; // 21 天向速度标准差

    float m_fStdPitch;   // 22 俯仰角标准差
    float m_fStdRoll;    // 23 横滚角标准差
    float m_fStdAzimuth; // 24 航向角标准差
    uint8_t res[8];      // 保留
} tINSNAVData;

typedef struct
{
    uint8_t m_iTimeSystem; // 1 0-无效系统，1-BDS, 2-GPS，3-GAL，4-GLO, 5~7-保留
    uint16_t m_iWeekNum;   // 2 BDS周：自2006-1-1至当前的星期数（北京时间）GPS周：自1980-1-6至当前的星期数（格林尼治时间）
    double m_dSecInWeek;   // 3 周内秒, 自本周日0:00:00至当前的秒数
    int8_t m_iLeapSecond;  // 闰秒, 周内秒+周数+闰秒=UTC

    float m_fAccelX; // 4 加计X轴
    float m_fAccelY; // 5 加计Y轴
    float m_fAccelZ; // 6 加计Z轴

    float m_fGroX; // 7 陀螺仪X轴
    float m_fGroY; // 8 陀螺仪Y轴
    float m_fGroZ; // 9 陀螺仪Z轴

    float m_fMagX; // 10 磁力计X
    float m_fMagY; // 11 磁力计Y
    float m_fMagZ; // 12 磁力计Z

    uint8_t res[8]; // 保留
} tIMUDATAData;

typedef struct
{
    uint8_t m_Status;  // 数据接收状态(1:数据接收,0:未接收),高5位有效,低3位备用,从高到低, 左前轮脉冲数, 右前轮脉冲数, 左后轮脉冲数, 右后轮脉冲数, 频率(只为了整合两帧CAN)
    uint8_t m_HZ;      // 频率
    uint8_t m_Valid;   // 有效位,高4位有效,低4位备用,从高到低, 左前轮脉冲数, 右前轮脉冲数, 左后轮脉冲数, 右后轮脉冲数
    uint16_t m_FLPlus; // 左前轮脉冲数
    uint16_t m_FRPlus; // 右前轮脉冲数
    uint16_t m_BLPlus; // 左后轮脉冲数
    uint16_t m_BRPlus; // 右后轮脉冲数
    bool m_FLBack;     // 是否后退
    bool m_FRBack;     // 是否后退
    bool m_BLBack;     // 是否后退
    bool m_BRBack;     // 是否后退
    uint8_t res[8];    // 保留
} tODMPLUSData;

typedef struct
{
    uint8_t m_Status; // 数据接收状态(1:数据接收,0:未接收),高7位有效,低1位备用, 从高位到低位, 左前轮速, 右前轮速, 左后轮速, 右后轮速, 合速度, 转向, 档位
    uint8_t m_HZ;     // 频率
    uint8_t m_Valid;  // 有效位,高7位有效,低1位备用, 从高位到低位, 左前轮速, 右前轮速, 左后轮速, 右后轮速, 合速度, 转向, 档位
    float m_FLSpeed;  // 左前轮速, m/s
    float m_FRSpeed;  // 右前轮速, m/s
    float m_BLSpeed;  // 左后轮速, m/s
    float m_BRSpeed;  // 右后轮速, m/s
    float m_Speed;    // 合速度, m/s
    float m_Turn;     // 转向: -180.0~180.0度
    uint8_t m_Gear;   // 档位,  0:无效, 1:P档, 2:R档, 3:N档, 4:D档
    uint8_t res[8];   // 保留
} tODMDATAData;

extern bool CSHG_INSPVAA_IsNeed(void);
extern bool CSHG_INSPVAB_IsNeed(void);
extern bool CSHG_IMUDATAA_IsNeed(void);
extern bool CSHG_IMUDATAB_IsNeed(void);
extern bool CSHG_SATNAVA_IsNeed(void);
extern bool CSHG_SATNAVB_IsNeed(void);
extern bool CSHG_INSNAVA_IsNeed(void);
extern bool CSHG_INSNAVB_IsNeed(void);

// POSDATAA IMUDATAA ODMDATAA INSPVAA INSIMUA INSPOSA CPP CPD PKG等命令 数据处理
extern int CSHG_GNSS_ASCII_Process(const MsgSource i_TpSource, uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans);
// HEADINGA等兼容诺瓦泰/北云 ASCII 命令 数据处理
extern int CSHG_GNSS_ASCII2_Process(const MsgSource i_TpSource, uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans);
// INSPVAB  等命令 数据处理
extern int CSHG_GNSS_BIN_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans);
// POSDATAB IMUDATAB ODMDATAB等命令 数据处理
extern int CSHG_GNSS_BIN2_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans);
// RANGEB INSPVAXB等兼容诺瓦泰/北云 标准二进制 命令 数据处理
extern int CSHG_GNSS_BIN3_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans);
// RAWIMUSB等兼容诺瓦泰/北云 简化二进制 命令 数据处理
extern int CSHG_GNSS_BIN4_Process(const MsgSource i_TpSource, const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen, bool *o_pbNeedTrans);

// POSDATAA IMUDATAA ODMDATAA INSPVAA INSIMUA INSPOSA CPP CPD PKG等命令 有效性判断
extern int CSHG_GNSS_ASCII_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen);
// HEADINGA等兼容诺瓦泰/北云 ASCII 命令 有效性判断
extern int CSHG_GNSS_ASCII2_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen);
// INSPVAB  等命令 有效性判断
extern int CSHG_GNSS_BIN_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen);
// POSDATAB IMUDATAB ODMDATAB等命令 有效性判断
extern int CSHG_GNSS_BIN2_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen);
// RANGEB INSPVAXB等兼容诺瓦泰/北云 标准二进制 命令 有效性判断
extern int CSHG_GNSS_BIN3_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen);
// RAWIMUSB等兼容诺瓦泰/北云 简化二进制 命令 有效性判断
extern int CSHG_GNSS_BIN4_CheckFrameValid(const uint8_t *i_pubInBuf, const uint32_t i_udwInBufLen);

extern uint8_t CSHG_GNSS_ConvNovatelPosStateToGGAPosState(const uint8_t i_Novatel, uint8_t *o_GGA);
extern uint8_t CSHG_GNSS_ConvGGAPosStateToNovatelPosState(const uint8_t i_GGA, uint8_t *o_Novatel);

extern CSHG_GNSS_LOG CSHG_GNSS_CMD_GET_LOG_FROM_NAME(const char *i_name);
