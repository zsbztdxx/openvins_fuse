#include "global.h"

Config_param g_Config_param;
std::string g_rosws_path;
double g_last_latitude = 23.0, g_last_longitude = 113.0, g_last_altitude = 0.0;
bool g_bQuitProcess = false;
ros::Publisher g_pub_gga, g_pub_ggp, g_pub_rmc, g_pub_inspva, g_pub_imudata, g_pub_satnav, g_pub_insnav;
ros::Publisher g_pub_bdgsv, g_pub_gpgsv, g_pub_glgsv, g_pub_gagsv;
ros::Publisher g_pub_nav_sat_fix, g_pub_gps_fix, g_pub_gnss_vel, g_pub_ins_vel, g_pub_imu, g_pub_MasterSatNum, g_pub_SlaverSatNum;

double dmm2deg(double dmm)
{
    return std::floor(dmm / 100.0) + std::fmod(dmm, 100.0) / 60.0;
}

double deg2dmm(double deg)
{
    double fraction, integer;
    fraction = std::modf(deg, &integer);
    return integer * 100 + fraction * 60;
}

int GetSysDateTime(struct timeval *out_tv, struct tm *out_tm)
{
    if (out_tv != nullptr)
    {
        if (gettimeofday(out_tv, nullptr) < 0)
        {
            return -1;
        }
        if (out_tm != nullptr)
        {
            struct tm *get_tm;
            get_tm = localtime(&out_tv->tv_sec);
            memcpy(out_tm, get_tm, sizeof(struct tm));
            // printf("%d-%d-%d %02d:%02d:%02d.%06d\r\n",
            //     get_tm->tm_year+1900, get_tm->tm_mon+1, get_tm->tm_mday,
            //     get_tm->tm_hour, get_tm->tm_min, get_tm->tm_sec,
            //     out_tv->tv_usec);
        }
        return 0;
    }
    return -1;
}

uint32_t tickget()
{
    struct timeval tv;
    GetSysDateTime(&tv, nullptr);
    return tv.tv_usec / 1000 + tv.tv_sec * 1000;
}

int Get_executable_path(std::string &Mappath, char *select)
{
    char *path_end;
    char processdir[256];
    if (readlink("/proc/self/exe", processdir, 256) <= 0)
    {
        std::cout << processdir << std::endl;
        return -1;
    }

    std::string path = processdir;
    int offindex = path.find(select, 0);
    if (offindex < 0)
    {
        // std::cout<<processdir<<'\t'<<select<<std::endl;
        return -1;
    }
    processdir[offindex] = '\0';
    Mappath = processdir;
    return 0;
}

int SetComPara(int fd, int bps, int databits, int stopbits, int parity)
{
    struct termios options;
    if (tcgetattr(fd, &options) != 0)
    {
        return -1;
    }
    tcgetattr(fd, &options);
    switch (bps)
    {
    case 300:
        cfsetispeed(&options, B300);
        cfsetospeed(&options, B300);
        break;
    case 600:
        cfsetispeed(&options, B600);
        cfsetospeed(&options, B600);
        break;
    case 1200:
        cfsetispeed(&options, B1200);
        cfsetospeed(&options, B1200);
        break;
    case 2400:
        cfsetispeed(&options, B2400);
        cfsetospeed(&options, B2400);
        break;
    case 4800:
        cfsetispeed(&options, B4800);
        cfsetospeed(&options, B4800);
        break;
    case 9600:
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        break;
    case 19200:
        cfsetispeed(&options, B19200);
        cfsetospeed(&options, B19200);
        break;
    case 38400:
        cfsetispeed(&options, B38400);
        cfsetospeed(&options, B38400);
        break;
    case 57600:
        cfsetispeed(&options, B57600);
        cfsetospeed(&options, B57600);
        break;
    case 115200:
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        break;
    case 230400:
        cfsetispeed(&options, B230400);
        cfsetospeed(&options, B230400);
        break;
    case 460800:
        cfsetispeed(&options, B460800);
        cfsetospeed(&options, B460800);
        break;
    case 500000:
        cfsetispeed(&options, B500000);
        cfsetospeed(&options, B500000);
        break;
    case 576000:
        cfsetispeed(&options, B576000);
        cfsetospeed(&options, B576000);
        break;
    case 921600:
        cfsetispeed(&options, B921600);
        cfsetospeed(&options, B921600);
        break;
    case 1000000:
        cfsetispeed(&options, B1000000);
        cfsetospeed(&options, B1000000);
        break;
    case 1152000:
        cfsetispeed(&options, B1152000);
        cfsetospeed(&options, B1152000);
        break;
    case 1500000:
        cfsetispeed(&options, B1500000);
        cfsetospeed(&options, B1500000);
        break;
    case 2000000:
        cfsetispeed(&options, B2000000);
        cfsetospeed(&options, B2000000);
        break;
    case 2500000:
        cfsetispeed(&options, B2500000);
        cfsetospeed(&options, B2500000);
        break;
    case 3000000:
        cfsetispeed(&options, B3000000);
        cfsetospeed(&options, B3000000);
        break;
    case 3500000:
        cfsetispeed(&options, B3500000);
        cfsetospeed(&options, B3500000);
        break;
    case 4000000:
        cfsetispeed(&options, B4000000);
        cfsetospeed(&options, B4000000);
        break;
    default:
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        break;
    }
    //	tcsetattr(fd,TCANOW,&Opt);

    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        return -1;
    }

    options.c_iflag &= ~ICRNL;
    options.c_iflag &= ~IXON;
    options.c_iflag &= ~IXOFF;

    switch (parity)
    {
    case 'n':
    case 'N':
        options.c_cflag &= ~PARENB; /* Clear parity enable */
        options.c_iflag &= ~INPCK;  /* Enable parity checking */
        break;
    case 'o':
    case 'O':
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK; /* Disnable parity checking */
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB; /* Enable parity */
        options.c_cflag &= ~PARODD;
        //			options.c_iflag |= INPCK;       /* Disnable parity checking */
        options.c_iflag |= IGNBRK; /* Disnable parity checking */
        break;
    case 'S':
    case 's': /*as no parity*/
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        return -1;
    }

    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        return -1;
    }

    /* Set input parity option */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input*/
    options.c_oflag &= ~OPOST;                          /*Output*/

    if (parity != 'n')
    {
        options.c_iflag |= INPCK;
    }
    tcflush(fd, TCIFLUSH);
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        return -1;
    }

    return 0;
}

uint16_t crc16(const uint8_t *buf, const uint32_t len)
{
    uint32_t i;
    uint16_t cksum;
    uint8_t s, t;
    uint32_t r;

    cksum = 0;
    for (i = 0; i < len; i++)
    {
        s = *buf++ ^ (uint8_t)(cksum >> 8);
        t = s ^ (s >> 4);
        r = (uint32_t)((cksum << 8) ^ t ^ (t << 5) ^ (t << 12));
        cksum = r & 0xFFFF;
    }
    return cksum;
}

uint32_t rtk_crc24q(const uint32_t lastCRC, const uint8_t *buff, const uint32_t len)
{
    static const uint32_t gs_tbl_CRC24Q[256] = {
        0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
        0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
        0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E,
        0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7,
        0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE,
        0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
        0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
        0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E,
        0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
        0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC,
        0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
        0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375,
        0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C,
        0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
        0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5,
        0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
        0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3,
        0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
        0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A,
        0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703,
        0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
        0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863,
        0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3,
        0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA,
        0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61,
        0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
        0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
        0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
        0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88,
        0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1,
        0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
        0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538};
    uint32_t crc = lastCRC;
    uint32_t i;

    for (i = 0; i < len; i++)
        crc = ((crc << 8) & 0xFFFFFF) ^ gs_tbl_CRC24Q[(crc >> 16) ^ buff[i]];
    return crc;
}

// 查表法计算CRC32
uint32_t CRC32_Cal(const uint32_t lastCRC, const uint8_t *buff, const uint32_t len)
{
    static const uint32_t TBL_CRC32[256] = {
        0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL,
        0x076dc419L, 0x706af48fL, 0xe963a535L, 0x9e6495a3L,
        0x0edb8832L, 0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L,
        0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L, 0x90bf1d91L,
        0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
        0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
        0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL,
        0x14015c4fL, 0x63066cd9L, 0xfa0f3d63L, 0x8d080df5L,
        0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L, 0xa2677172L,
        0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
        0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L,
        0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L,
        0x26d930acL, 0x51de003aL, 0xc8d75180L, 0xbfd06116L,
        0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L, 0xb8bda50fL,
        0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
        0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL,
        0x76dc4190L, 0x01db7106L, 0x98d220bcL, 0xefd5102aL,
        0x71b18589L, 0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L,
        0x7807c9a2L, 0x0f00f934L, 0x9609a88eL, 0xe10e9818L,
        0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
        0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL,
        0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L,
        0x65b0d9c6L, 0x12b7e950L, 0x8bbeb8eaL, 0xfcb9887cL,
        0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L, 0xfbd44c65L,
        0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
        0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL,
        0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L,
        0x44042d73L, 0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L,
        0x5005713cL, 0x270241aaL, 0xbe0b1010L, 0xc90c2086L,
        0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
        0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L,
        0x59b33d17L, 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL,
        0xedb88320L, 0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL,
        0xead54739L, 0x9dd277afL, 0x04db2615L, 0x73dc1683L,
        0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
        0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,
        0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL,
        0xf762575dL, 0x806567cbL, 0x196c3671L, 0x6e6b06e7L,
        0xfed41b76L, 0x89d32be0L, 0x10da7a5aL, 0x67dd4accL,
        0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
        0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L,
        0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL,
        0xd80d2bdaL, 0xaf0a1b4cL, 0x36034af6L, 0x41047a60L,
        0xdf60efc3L, 0xa867df55L, 0x316e8eefL, 0x4669be79L,
        0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
        0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL,
        0xc5ba3bbeL, 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L,
        0xc2d7ffa7L, 0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL,
        0x9b64c2b0L, 0xec63f226L, 0x756aa39cL, 0x026d930aL,
        0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
        0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L,
        0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L,
        0x86d3d2d4L, 0xf1d4e242L, 0x68ddb3f8L, 0x1fda836eL,
        0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L, 0x18b74777L,
        0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
        0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L,
        0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L,
        0xa7672661L, 0xd06016f7L, 0x4969474dL, 0x3e6e77dbL,
        0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L, 0x37d83bf0L,
        0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
        0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L,
        0xbad03605L, 0xcdd70693L, 0x54de5729L, 0x23d967bfL,
        0xb3667a2eL, 0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L,
        0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL, 0x2d02ef8dL};
    uint32_t crc = lastCRC;
    uint32_t i;

    for (i = 0u; i < len; i++)
    {
        crc = TBL_CRC32[(crc ^ buff[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc;
}

uint32_t ota_crc32(const uint32_t lastCRC, const uint8_t *buff, const uint32_t len)
{
    static const uint32_t gs_tbl_crc32[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c};
    uint32_t i;
    uint32_t crc = lastCRC;
    crc ^= 0xffffffff;
    for (i = 0; i < len; i++)
    {
        crc ^= buff[i];
        crc = (crc >> 4) ^ gs_tbl_crc32[crc & 0xF];
        crc = (crc >> 4) ^ gs_tbl_crc32[crc & 0xF];
    }
    crc ^= 0xffffffff;
    return crc;
}

// double转字符串, i_MaxDec最大小数位, 因sprintf %f失效
char *Double2String(const double i_data, char *o_str, const uint8_t i_MinInt, const uint8_t i_MaxDec)
{
    if (o_str != NULL)
    {
        int32_t ints = (int32_t)(i_data);
        uint32_t decs;
        char tempString[14];
        sprintf(tempString, "%%0%dd.%%0%dd", i_MinInt > 0 ? i_MinInt : 1, i_MaxDec > 0 ? i_MaxDec : 1);
        if (i_data > ints)
        {
            decs = (uint32_t)((i_data - ints) * pow(10, i_MaxDec));
        }
        else
        {
            decs = (uint32_t)((ints - i_data) * pow(10, i_MaxDec));
        }
        sprintf(o_str, tempString, ints, decs);
    }
    return o_str;
}

void timeadd(gtime_t t, const double sec, gtime_t *const o_data)
{
    double tt;
    t.sec += sec;
    tt = floor(t.sec);
    t.time += (int)tt;
    t.sec -= tt;
    *o_data = t;
}

void epoch2time(const double *ep, gtime_t *const o_data)
{
    const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
    gtime_t time = {0};
    int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon)
    {
        *o_data = time;
        return;
    }

    /*leap year if year%4 == 0 in 1901-2099*/

    days = (year - 1970) * 365 + ((year - 1969) >> 2) + doy[mon - 1] + day - 2 + ((year & 0x3) == 0 && mon >= 3 ? 1 : 0);
    sec = (int)floor(ep[5]);
    time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
    time.sec = ep[5] - sec;
    *o_data = time;
    return;
}

// 周内秒+周数 转 GPS日期时间
void gpst2time(const int week, const double sec, gtime_t *const o_data)
{
    static const double gpst0[6] = {1980, 1, 6, 0, 0, 0}; /* gps time reference */

    gtime_t t;
    epoch2time(gpst0, &t);

    if (sec < -1E9 || 1E9 < sec)
    {
        t.time += (time_t)(86400 * 7 * week);
        t.sec = 0;
    }
    else
    {
        t.time += (time_t)(86400 * 7 * week) + (int)sec;
        t.sec = sec - (int)sec;
    }
    *o_data = t;
}

// 周内秒+周数 转 BD日期时间
void bdst2time(const int week, const double sec, gtime_t *const o_data)
{
    static const double bdst0[6] = {2006, 1, 1, 0, 0, 0}; /* 北斗 time reference */

    gtime_t t;
    epoch2time(bdst0, &t);

    if (sec < -1E9 || 1E9 < sec)
    {
        t.time += (time_t)(86400 * 7 * week);
        t.sec = 0;
    }
    else
    {
        t.time += (time_t)(86400 * 7 * week) + (int)sec;
        t.sec = sec - (int)sec;
    }
    *o_data = t;
}

// + GPS/BD闰秒 (GPS时间和UTC时间差)
void gpst2utc(const gtime_t t, const int8_t leapSec, gtime_t *const o_data)
{
    timeadd(t, -leapSec, o_data);
}
void bdt2gpst(gtime_t t, gtime_t *const o_data)
{
    timeadd(t, 14.0, o_data);
}
void gpst2bdt(gtime_t t, gtime_t *const o_data)
{
    timeadd(t, -14.0, o_data);
}
// GPS日期时间 转 UTC日期时间
void time2epoch(gtime_t t, double *ep)
{
    const int mday[] = {/* # of days in a month */
                        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
                        31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int days, sec, mon, day;
    int tmpInt1, tmpInt2;
    /* leap year if year%4==0 in 1901-2099 */
    days = (int)(t.time / 86400);
    sec = (int)(t.time - (time_t)days * 86400);
    for (day = days % 1461, mon = 0; mon < 48; mon++)
    {
        if (day >= mday[mon])
            day -= mday[mon];
        else
            break;
    }
    ep[0] = 1970 + days / 1461 * 4 + mon / 12;
    ep[1] = mon % 12 + 1;
    ep[2] = day + 1;
#if 1 // 减少使用除法和求余
    tmpInt1 = sec / 60;
    ep[5] = sec - tmpInt1 * 60 + t.sec;
    tmpInt2 = tmpInt1 / 60;
    ep[4] = tmpInt1 - tmpInt2 * 60;
    ep[3] = tmpInt2;
#else
    ep[3] = sec / 3600;
    ep[4] = sec % 3600 / 60;
    ep[5] = sec % 60 + t.sec;
#endif
}

int8_t ConvMSBBitUInt1(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint8_t *const o_data)
{
    assert(i_nextBitOffset <= 8);
    assert(i_bitlen == 1);
    if (o_data != NULL)
    {
        *o_data = (uint8_t)(i_pbuf[0] >> ((uint8_t)8 - i_nextBitOffset) & (((uint8_t)1 << i_bitlen) - (uint8_t)1));
        return true;
    }
    return false;
}

int8_t ConvMSBBitUInt2_UInt9(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint16_t *const o_data)
{
    assert(i_nextBitOffset <= 16);
    assert(i_bitlen >= 2 && i_bitlen <= 9);
    if (o_data != NULL)
    {
        *o_data = (uint16_t)((((uint16_t)i_pbuf[0] << (uint16_t)8) +
                              (uint16_t)i_pbuf[1]) >>
                                 ((uint8_t)16 - i_nextBitOffset) &
                             (((uint16_t)1 << i_bitlen) - (uint16_t)1));
        if (*o_data != (((uint16_t)1 << i_bitlen) - (uint16_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvMSBBitUInt10_UInt17(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint32_t *const o_data)
{
    assert(i_nextBitOffset <= 24);
    assert(i_bitlen >= 10 && i_bitlen <= 17);
    if (o_data != NULL)
    {
        *o_data = (((uint32_t)i_pbuf[0] << 16) +
                   ((uint32_t)i_pbuf[1] << 8) +
                   (uint32_t)i_pbuf[2]) >>
                      ((uint8_t)24 - i_nextBitOffset) &
                  (((uint32_t)1 << i_bitlen) - (uint32_t)1);
        if (*o_data != (((uint32_t)1 << i_bitlen) - (uint32_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvMSBBitUInt18_UInt25(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint32_t *const o_data)
{
    assert(i_nextBitOffset <= 32);
    assert(i_bitlen >= 18 && i_bitlen <= 25);
    if (o_data != NULL)
    {
        *o_data = (((uint32_t)i_pbuf[0] << 24) +
                   ((uint32_t)i_pbuf[1] << 16) +
                   ((uint32_t)i_pbuf[2] << 8) +
                   (uint32_t)i_pbuf[3]) >>
                      ((uint8_t)32 - i_nextBitOffset) &
                  (((uint32_t)1 << i_bitlen) - (uint32_t)1);
        if (*o_data != (((uint32_t)1 << i_bitlen) - (uint32_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvMSBBitUInt26_UInt33(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint64_t *const o_data)
{
    assert(i_nextBitOffset <= 40);
    assert(i_bitlen >= 26 && i_bitlen <= 33);
    if (o_data != NULL)
    {
        *o_data = (((uint64_t)i_pbuf[0] << 32) +
                   ((uint64_t)i_pbuf[1] << 24) +
                   ((uint64_t)i_pbuf[2] << 16) +
                   ((uint64_t)i_pbuf[3] << 8) +
                   (uint64_t)i_pbuf[4]) >>
                      ((uint8_t)40 - i_nextBitOffset) &
                  (((uint64_t)1 << i_bitlen) - (uint64_t)1);
        if (*o_data != (((uint64_t)1 << i_bitlen) - (uint64_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvMSBBitUInt34_UInt41(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint64_t *const o_data)
{
    assert(i_nextBitOffset <= 48);
    assert(i_bitlen >= 34 && i_bitlen <= 41);
    if (o_data != NULL)
    {
        *o_data = (((uint64_t)i_pbuf[0] << 40) +
                   ((uint64_t)i_pbuf[1] << 32) +
                   ((uint64_t)i_pbuf[2] << 24) +
                   ((uint64_t)i_pbuf[3] << 16) +
                   ((uint64_t)i_pbuf[4] << 8) +
                   (uint64_t)i_pbuf[5]) >>
                      ((uint8_t)48 - i_nextBitOffset) &
                  (((uint64_t)1 << i_bitlen) - (uint64_t)1);
        if (*o_data != (((uint64_t)1 << i_bitlen) - (uint64_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvMSBBitUInt42_UInt49(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint64_t *const o_data)
{
    assert(i_nextBitOffset <= 56);
    assert(i_bitlen >= 42 && i_bitlen <= 49);
    if (o_data != NULL)
    {
        *o_data = (((uint64_t)i_pbuf[0] << 48) +
                   ((uint64_t)i_pbuf[1] << 40) +
                   ((uint64_t)i_pbuf[2] << 32) +
                   ((uint64_t)i_pbuf[3] << 24) +
                   ((uint64_t)i_pbuf[4] << 16) +
                   ((uint64_t)i_pbuf[5] << 8) +
                   (uint64_t)i_pbuf[6]) >>
                      ((uint8_t)56 - i_nextBitOffset) &
                  (((uint64_t)1 << i_bitlen) - (uint64_t)1);
        if (*o_data != (((uint64_t)1 << i_bitlen) - (uint64_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}
int8_t ConvMSBBitUInt50_UInt57(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, uint64_t *const o_data)
{
    assert(i_nextBitOffset <= 64);
    assert(i_bitlen >= 50 && i_bitlen <= 57);
    if (o_data != NULL)
    {
        *o_data = (((uint64_t)i_pbuf[0] << 56) +
                   ((uint64_t)i_pbuf[1] << 48) +
                   ((uint64_t)i_pbuf[2] << 40) +
                   ((uint64_t)i_pbuf[3] << 32) +
                   ((uint64_t)i_pbuf[4] << 24) +
                   ((uint64_t)i_pbuf[5] << 16) +
                   ((uint64_t)i_pbuf[6] << 8) +
                   (uint64_t)i_pbuf[7]) >>
                      ((uint8_t)64 - i_nextBitOffset) &
                  (((uint64_t)1 << i_bitlen) - (uint64_t)1);
        if (*o_data != (((uint64_t)1 << i_bitlen) - (uint64_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvMSBBitInt2_Int9(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int16_t *const o_data)
{
    assert(i_nextBitOffset <= 16);
    assert(i_bitlen >= 2 && i_bitlen <= 9);
    if (o_data != NULL)
    {
        uint16_t tempU16;
        if (ConvMSBBitUInt2_UInt9(i_pbuf, i_nextBitOffset, i_bitlen, &tempU16) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU16 != (((uint16_t)1 << signBitPos))) // 有效
            {
                if (tempU16 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int16_t)(tempU16 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int16_t)tempU16;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvMSBBitInt10_Int17(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int32_t *const o_data)
{
    if (o_data != NULL)
    {
        uint32_t tempU32;
        if (ConvMSBBitUInt10_UInt17(i_pbuf, i_nextBitOffset, i_bitlen, &tempU32) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU32 != (((uint32_t)1 << signBitPos))) // 有效
            {
                if (tempU32 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int32_t)(tempU32 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int32_t)tempU32;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvMSBBitInt18_Int25(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int32_t *const o_data)
{
    if (o_data != NULL)
    {
        uint32_t tempU32;
        if (ConvMSBBitUInt18_UInt25(i_pbuf, i_nextBitOffset, i_bitlen, &tempU32) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU32 != (((uint32_t)1 << signBitPos))) // 有效
            {
                if (tempU32 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int32_t)(tempU32 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int32_t)tempU32;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvMSBBitInt26_Int33(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data)
{
    if (o_data != NULL)
    {
        uint64_t tempU64;
        if (ConvMSBBitUInt26_UInt33(i_pbuf, i_nextBitOffset, i_bitlen, &tempU64) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU64 != (((uint64_t)1 << signBitPos))) // 有效
            {
                if (tempU64 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int64_t)(tempU64 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int64_t)tempU64;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvMSBBitInt34_Int41(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data)
{
    if (o_data != NULL)
    {
        uint64_t tempU64;
        if (ConvMSBBitUInt34_UInt41(i_pbuf, i_nextBitOffset, i_bitlen, &tempU64) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU64 != (((uint64_t)1 << signBitPos))) // 有效
            {
                if (tempU64 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int64_t)(tempU64 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int64_t)tempU64;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvMSBBitInt42_Int49(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data)
{
    if (o_data != NULL)
    {
        uint64_t tempU64;
        if (ConvMSBBitUInt42_UInt49(i_pbuf, i_nextBitOffset, i_bitlen, &tempU64) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU64 != (((uint64_t)1 << signBitPos))) // 有效
            {
                if (tempU64 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int64_t)(tempU64 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int64_t)tempU64;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvMSBBitInt50_Int57(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data)
{
    if (o_data != NULL)
    {
        uint64_t tempU64;
        if (ConvMSBBitUInt50_UInt57(i_pbuf, i_nextBitOffset, i_bitlen, &tempU64) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU64 != (((uint64_t)1 << signBitPos))) // 有效
            {
                if (tempU64 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int64_t)(tempU64 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int64_t)tempU64;
                }
                return 0;
            }
        }
    }
    return -1;
}

int8_t ConvLSBBitUInt1(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint8_t *const o_data)
{
    assert(i_bitOffset < 8);
    assert(i_bitlen == 1);
    return ConvMSBBitUInt1(i_pbuf, i_bitOffset, i_bitlen, o_data);
}

int8_t ConvLSBBitUInt2_UInt9(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint16_t *const o_data)
{
    assert(i_bitOffset < 8);
    assert(i_bitlen >= 2 && i_bitlen <= 9);
    if (o_data != NULL)
    {
        *o_data = (uint16_t)(((uint16_t)i_pbuf[0] +
                              ((uint16_t)i_pbuf[1] << 8)) >>
                                 i_bitOffset &
                             (((uint16_t)1 << i_bitlen) - (uint16_t)1));
        if (*o_data != (((uint16_t)1 << i_bitlen) - (uint16_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvLSBBitUInt10_UInt17(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint32_t *const o_data)
{
    assert(i_bitOffset < 8);
    assert(i_bitlen >= 10 && i_bitlen <= 17);
    if (o_data != NULL)
    {
        *o_data = ((uint32_t)i_pbuf[0] +
                   ((uint32_t)i_pbuf[1] << 8) +
                   ((uint32_t)i_pbuf[2] << 16)) >>
                      i_bitOffset &
                  (((uint32_t)1 << i_bitlen) - (uint32_t)1);
        if (*o_data != (((uint32_t)1 << i_bitlen) - (uint32_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvLSBBitUInt18_UInt25(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint32_t *const o_data)
{
    assert(i_bitOffset < 8);
    assert(i_bitlen >= 18 && i_bitlen <= 25);
    if (o_data != NULL)
    {
        *o_data = ((uint32_t)i_pbuf[0] +
                   ((uint32_t)i_pbuf[1] << 8) +
                   ((uint32_t)i_pbuf[2] << 16) +
                   ((uint32_t)i_pbuf[3] << 24)) >>
                      i_bitOffset &
                  (((uint32_t)1 << i_bitlen) - (uint32_t)1);
        if (*o_data != (((uint32_t)1 << i_bitlen) - (uint32_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvLSBBitUInt26_UInt33(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint64_t *const o_data)
{
    assert(i_bitOffset < 8);
    assert(i_bitlen >= 26 && i_bitlen <= 33);
    if (o_data != NULL)
    {
        *o_data = ((uint64_t)i_pbuf[0] +
                   ((uint64_t)i_pbuf[1] << 8) +
                   ((uint64_t)i_pbuf[2] << 16) +
                   ((uint64_t)i_pbuf[3] << 24) +
                   ((uint64_t)i_pbuf[4] << 32)) >>
                      i_bitOffset &
                  (((uint64_t)1 << i_bitlen) - (uint64_t)1);
        if (*o_data != (((uint64_t)1 << i_bitlen) - (uint64_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvLSBBitUInt34_UInt41(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint64_t *const o_data)
{
    assert(i_bitOffset < 8);
    assert(i_bitlen >= 34 && i_bitlen <= 41);
    if (o_data != NULL)
    {
        *o_data = ((uint64_t)i_pbuf[0] +
                   ((uint64_t)i_pbuf[1] << 8) +
                   ((uint64_t)i_pbuf[2] << 16) +
                   ((uint64_t)i_pbuf[3] << 24) +
                   ((uint64_t)i_pbuf[4] << 32) +
                   ((uint64_t)i_pbuf[5] << 40)) >>
                      i_bitOffset &
                  (((uint64_t)1 << i_bitlen) - (uint64_t)1);
        if (*o_data != (((uint64_t)1 << i_bitlen) - (uint64_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvLSBBitUInt42_UInt49(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint64_t *const o_data)
{
    assert(i_bitOffset < 8);
    assert(i_bitlen >= 42 && i_bitlen <= 49);
    if (o_data != NULL)
    {
        *o_data = ((uint64_t)i_pbuf[0] +
                   ((uint64_t)i_pbuf[1] << 8) +
                   ((uint64_t)i_pbuf[2] << 16) +
                   ((uint64_t)i_pbuf[3] << 24) +
                   ((uint64_t)i_pbuf[4] << 32) +
                   ((uint64_t)i_pbuf[5] << 40) +
                   ((uint64_t)i_pbuf[6] << 48)) >>
                      i_bitOffset &
                  (((uint64_t)1 << i_bitlen) - (uint64_t)1);
        if (*o_data != (((uint64_t)1 << i_bitlen) - (uint64_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvLSBBitUInt50_UInt57(const uint8_t *const i_pbuf, const uint8_t i_bitOffset, const uint8_t i_bitlen, uint64_t *const o_data)
{
    assert(i_bitOffset < 8);
    assert(i_bitlen >= 50 && i_bitlen <= 57);
    if (o_data != NULL)
    {
        *o_data = ((uint64_t)i_pbuf[0] +
                   ((uint64_t)i_pbuf[1] << 8) +
                   ((uint64_t)i_pbuf[2] << 16) +
                   ((uint64_t)i_pbuf[3] << 24) +
                   ((uint64_t)i_pbuf[4] << 32) +
                   ((uint64_t)i_pbuf[5] << 40) +
                   ((uint64_t)i_pbuf[6] << 48) +
                   ((uint64_t)i_pbuf[7] << 56)) >>
                      i_bitOffset &
                  (((uint64_t)1 << i_bitlen) - (uint64_t)1);
        if (*o_data != (((uint64_t)1 << i_bitlen) - (uint64_t)1))
        {
            return 0;
        }
        return 1;
    }
    return -1;
}

int8_t ConvLSBBitInt2_Int9(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int16_t *const o_data)
{
    assert(i_nextBitOffset <= 16);
    assert(i_bitlen >= 2 && i_bitlen <= 9);
    if (o_data != NULL)
    {
        uint16_t tempU16;
        if (ConvLSBBitUInt2_UInt9(i_pbuf, i_nextBitOffset, i_bitlen, &tempU16) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU16 != (((uint16_t)1 << signBitPos))) // 有效
            {
                if (tempU16 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int16_t)(tempU16 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int16_t)tempU16;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvLSBBitInt10_Int17(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int32_t *const o_data)
{
    if (o_data != NULL)
    {
        uint32_t tempU32;
        if (ConvLSBBitUInt10_UInt17(i_pbuf, i_nextBitOffset, i_bitlen, &tempU32) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU32 != (((uint32_t)1 << signBitPos))) // 有效
            {
                if (tempU32 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int32_t)(tempU32 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int32_t)tempU32;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvLSBBitInt18_Int25(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int32_t *const o_data)
{
    if (o_data != NULL)
    {
        uint32_t tempU32;
        if (ConvLSBBitUInt18_UInt25(i_pbuf, i_nextBitOffset, i_bitlen, &tempU32) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU32 != (((uint32_t)1 << signBitPos))) // 有效
            {
                if (tempU32 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int32_t)(tempU32 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int32_t)tempU32;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvLSBBitInt26_Int33(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data)
{
    if (o_data != NULL)
    {
        uint64_t tempU64;
        if (ConvLSBBitUInt26_UInt33(i_pbuf, i_nextBitOffset, i_bitlen, &tempU64) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU64 != (((uint64_t)1 << signBitPos))) // 有效
            {
                if (tempU64 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int64_t)(tempU64 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int64_t)tempU64;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvLSBBitInt34_Int41(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data)
{
    if (o_data != NULL)
    {
        uint64_t tempU64;
        if (ConvLSBBitUInt34_UInt41(i_pbuf, i_nextBitOffset, i_bitlen, &tempU64) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU64 != (((uint64_t)1 << signBitPos))) // 有效
            {
                if (tempU64 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int64_t)(tempU64 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int64_t)tempU64;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvLSBBitInt42_Int49(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data)
{
    if (o_data != NULL)
    {
        uint64_t tempU64;
        if (ConvLSBBitUInt42_UInt49(i_pbuf, i_nextBitOffset, i_bitlen, &tempU64) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU64 != (((uint64_t)1 << signBitPos))) // 有效
            {
                if (tempU64 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int64_t)(tempU64 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int64_t)tempU64;
                }
                return 0;
            }
        }
    }
    return -1;
}
int8_t ConvLSBBitInt50_Int57(const uint8_t *const i_pbuf, const uint8_t i_nextBitOffset, const uint8_t i_bitlen, int64_t *const o_data)
{
    if (o_data != NULL)
    {
        uint64_t tempU64;
        if (ConvLSBBitUInt50_UInt57(i_pbuf, i_nextBitOffset, i_bitlen, &tempU64) >= 0)
        {
            uint8_t signBitPos = i_bitlen - 1;
            if (tempU64 != (((uint64_t)1 << signBitPos))) // 有效
            {
                if (tempU64 >> signBitPos & 0x1)
                {
                    // 负数,反码补齐
                    *o_data = (int64_t)(tempU64 | (~0u << signBitPos));
                }
                else
                {
                    // 正数
                    *o_data = (int64_t)tempU64;
                }
                return 0;
            }
        }
    }
    return -1;
}