#pragma once

#include "global.h"

#include <netinet/in.h>  // struct sockadd_in, htons()
#include <arpa/inet.h>   // inet_addr()
#include <netinet/tcp.h> // struct tcp_info类定义
#include <sys/socket.h>
#include <netdb.h>

typedef int socket_t;
typedef struct
{                            /* tcp control type */
    int state = 0;           /* state (0:close,1:wait,2:connect) */
    char saddr[256] = {0};   /* address string */
    int port = 0;            /* port */
    struct sockaddr_in addr; /* address resolved */
    socket_t sock = 0;       /* socket descriptor */
    int tcon = 1000;         /* reconnect time (ms) (-1:never,0:now) */
    unsigned int tact = 0;   /* data active tick */
    unsigned int tdis = 0;   /* disconnect tick */
} tcp_t;

typedef struct
{                    /* tcp cilent type */
    tcp_t svr;       /* tcp server control */
    int toinact = 0; /* inactive timeout (ms) (0:no timeout) */
    int tirecon = 0; /* reconnect interval (ms) (0:no reconnect) */
} tcpcli_t;

#define VER_RTKLIB "1.0.0" /* library version */
#define NTRIP_AGENT "CSHG/" VER_RTKLIB
#define NTRIP_MAXRSP 1536                         /* max size of ntrip response */
#define NTRIP_MAXSTR 256                          /* max length of mountpoint string */
#define NTRIP_RSP_OK_CLI "ICY 200 OK\r\n"         /* ntrip response: client */
#define NTRIP_RSP_OK_SVR "OK\r\n"                 /* ntrip response: server */
#define NTRIP_RSP_SRCTBL "SOURCETABLE 200 OK\r\n" /* ntrip response: source table */
#define NTRIP_RSP_TBLEND "ENDSOURCETABLE"
#define NTRIP_RSP_HTTP "HTTP/"  /* ntrip response: http */
#define NTRIP_RSP_ERROR "ERROR" /* ntrip response: error */
#define MAXSTRPATH 1024         /* max length of stream path */
#define MAXSTRMSG 1024          /* max length of stream message */
typedef struct
{                                           /* ntrip control type */
    int state = 0;                          /* state (0:close,1:wait,2:connect) */
    int type = 0;                           /* type (0:server,1:client) */
    int nb = 0;                             /* response buffer size */
    char url[256] = {0};                    /* url for proxy */
    char mntpnt[256] = {0};                 /* mountpoint */
    char user[256] = {0};                   /* user */
    char passwd[256] = {0};                 /* password */
    char str[NTRIP_MAXSTR] = {0};           /* mountpoint string for server */
    unsigned char buff[NTRIP_MAXRSP] = {0}; /* response buffer */
    tcpcli_t *tcp;                          /* tcp client */
} ntrip_t;

extern int errsock(void);
extern int setsock(socket_t sock);
extern int recv_nb(socket_t sock, unsigned char *buff, int n);
extern int send_nb(socket_t sock, unsigned char *buff, int n);
extern int gentcp(tcp_t *tcp, int type);
extern void discontcp(tcp_t *tcp, int tcon);
extern int connect_nb(socket_t sock, struct sockaddr *addr, socklen_t len);
extern int consock(tcpcli_t *tcpcli);

extern tcpcli_t *opentcpcli(void);
extern void closetcpcli(tcpcli_t *tcpcli);
extern int waittcpcli(tcpcli_t *tcpcli);
extern int readtcpcli(tcpcli_t *tcpcli, unsigned char *buff, int n);
extern int writetcpcli(tcpcli_t *tcpcli, unsigned char *buff, int n);

extern int encbase64(char *str, const unsigned char *byte, int n);
extern int reqntrip_c(ntrip_t *ntrip);
extern int rspntrip_c(ntrip_t *ntrip);

extern int waitntrip(ntrip_t *ntrip);
extern ntrip_t *openntrip();
extern void closentrip(ntrip_t *ntrip);
extern int readntrip(ntrip_t *ntrip, unsigned char *buff, int n);
extern int writentrip(ntrip_t *ntrip, unsigned char *buff, int n);
