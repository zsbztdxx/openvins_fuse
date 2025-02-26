#include "ntrip.h"

static int toinact = 10000;   /* inactive timeout (ms) */
static int ticonnect = 10000; /* interval to re-connect (ms) */
static int tirate = 1000;     /* avraging time for data rate (ms) */
static int buffsize = 4096;   /* receive/send buffer size (bytes) */
int errsock(void)
{
    return errno;
}

int setsock(socket_t sock)
{
    int bs = buffsize, mode = 1;
#ifdef WIN32
    int tv = 0;
#else
    struct timeval tv = {0};
#endif
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv)) == -1 ||
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof(tv)) == -1)
    {
        LOG(WARNING) << "sockopt error: notimeo";
        close(sock);
        return 0;
    }
    if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (const char *)&bs, sizeof(bs)) == -1 ||
        setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (const char *)&bs, sizeof(bs)) == -1)
    {
        LOG(WARNING) << "sockopt error: bufsiz";
    }
    if (setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (const char *)&mode, sizeof(mode)) == -1)
    {
        LOG(WARNING) << "sockopt error: nodelay";
    }
    return 1;
}
int recv_nb(socket_t sock, unsigned char *buff, int n)
{
    struct timeval tv = {0};
    fd_set rs;

    FD_ZERO(&rs);
    FD_SET(sock, &rs);
    if (!select(sock + 1, &rs, NULL, NULL, &tv))
        return 0;
    return recv(sock, (char *)buff, n, 0);
}
int send_nb(socket_t sock, unsigned char *buff, int n)
{
    struct timeval tv = {0};
    fd_set ws;

    FD_ZERO(&ws);
    FD_SET(sock, &ws);
    if (!select(sock + 1, nullptr, &ws, nullptr, &tv))
        return 0;
    return send(sock, (char *)buff, n, 0);
}

int gentcp(tcp_t *tcp, int type)
{
    struct hostent *hp;
#ifdef SVR_REUSEADDR
    int opt = 1;
#endif
    /* generate socket */
    if ((tcp->sock = socket(AF_INET, SOCK_STREAM, 0)) == (socket_t)-1)
    {
        LOG(WARNING) << "socket error = " << errsock();
        tcp->state = -1;
        return 0;
    }
    if (!setsock(tcp->sock))
    {
        tcp->state = -1;
        return 0;
    }
    memset(&tcp->addr, 0, sizeof(tcp->addr));
    tcp->addr.sin_family = AF_INET;
    tcp->addr.sin_port = htons(tcp->port);

    if (!(hp = gethostbyname(tcp->saddr)))
    {
        LOG(WARNING) << "address error = " << tcp->saddr;
        close(tcp->sock);
        tcp->state = 0;
        tcp->tcon = ticonnect;
        tcp->tdis = tickget();
        return 0;
    }
    memcpy(&tcp->addr.sin_addr, hp->h_addr, hp->h_length);
    tcp->state = 1;
    tcp->tact = tickget();
    return 1;
}
void discontcp(tcp_t *tcp, int tcon)
{
    close(tcp->sock);
    tcp->state = 0;
    tcp->tcon = tcon;
    tcp->tdis = tickget();
}
int connect_nb(socket_t sock, struct sockaddr *addr, socklen_t len)
{
    struct timeval tv = {0};
    fd_set rs, ws;
    int err, flag;

    flag = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flag | O_NONBLOCK);
    if (connect(sock, addr, len) == -1)
    {
        err = errsock();
        if (err != EISCONN && err != EINPROGRESS && err != EALREADY)
            return -1;
        FD_ZERO(&rs);
        FD_SET(sock, &rs);
        ws = rs;
        if (select(sock + 1, &rs, &ws, nullptr, &tv) == 0)
            return 0;
    }
    return 1;
}
int consock(tcpcli_t *tcpcli)
{
    int stat, err;

    /* wait re-connect */
    if (tcpcli->svr.tcon < 0 || (tcpcli->svr.tcon > 0 &&
                                 (int)(tickget() - tcpcli->svr.tdis) < tcpcli->svr.tcon))
    {
        return 0;
    }
    /* non-block connect */
    if ((stat = connect_nb(tcpcli->svr.sock, (struct sockaddr *)&tcpcli->svr.addr,
                           sizeof(tcpcli->svr.addr))) == -1)
    {
        err = errsock();
        LOG(WARNING) << "connect error = " << err;
        close(tcpcli->svr.sock);
        tcpcli->svr.state = 0;
        return 0;
    }
    if (!stat)
    { /* not connect */
        return 0;
    }
    tcpcli->svr.state = 2;
    tcpcli->svr.tact = tickget();
    return 1;
}

/* open tcp client -----------------------------------------------------------*/
tcpcli_t *opentcpcli()
{
    tcpcli_t *tcpcli = nullptr;

    if (!(tcpcli = (tcpcli_t *)malloc(sizeof(tcpcli_t))))
        return nullptr;
    strcpy(tcpcli->svr.saddr, g_Config_param.ntrip_addr.c_str());
    tcpcli->svr.port = g_Config_param.ntrip_port;
    tcpcli->svr.tcon = 0;
    tcpcli->toinact = toinact;
    tcpcli->tirecon = ticonnect;
    return tcpcli;
}

/* close tcp client ----------------------------------------------------------*/
void closetcpcli(tcpcli_t *tcpcli)
{
    close(tcpcli->svr.sock);
    free(tcpcli);
}

int waittcpcli(tcpcli_t *tcpcli)
{
    if (tcpcli->svr.state < 0)
        return 0;

    if (tcpcli->svr.state == 0)
    { /* close */
        if (!gentcp(&tcpcli->svr, 1))
            return 0;
    }
    if (tcpcli->svr.state == 1)
    { /* wait */
        if (!consock(tcpcli))
            return 0;
    }
    if (tcpcli->svr.state == 2)
    { /* connect */
        if (tcpcli->toinact > 0 &&
            (int)(tickget() - tcpcli->svr.tact) > tcpcli->toinact)
        {
            LOG(WARNING) << "timeout";
            discontcp(&tcpcli->svr, tcpcli->tirecon);
            return 0;
        }
    }
    return 1;
}

int readtcpcli(tcpcli_t *tcpcli, unsigned char *buff, int n)
{
    int nr, err;
    if (!waittcpcli(tcpcli))
        return 0;
    if ((nr = recv_nb(tcpcli->svr.sock, buff, n)) == -1)
    {
        err = errsock();
        LOG(WARNING) << "recv error" << err;
        discontcp(&tcpcli->svr, tcpcli->tirecon);
        return 0;
    }
    if (nr > 0)
    {
        tcpcli->svr.tact = tickget();
    }
    return nr;
}
/* write tcp client ----------------------------------------------------------*/
int writetcpcli(tcpcli_t *tcpcli, unsigned char *buff, int n)
{
    int ns, err;
    if (!waittcpcli(tcpcli))
        return 0;
    if ((ns = send_nb(tcpcli->svr.sock, buff, n)) == -1)
    {
        err = errsock();
        LOG(WARNING) << "send error" << err;
        discontcp(&tcpcli->svr, tcpcli->tirecon);
        return 0;
    }
    if (ns > 0)
    {
        tcpcli->svr.tact = tickget();
    }
    return ns;
}

int encbase64(char *str, const unsigned char *byte, int n)
{
    const char table[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int i, j, k, b;

    for (i = j = 0; (i >> 3) < n;)
    {
        for (k = b = 0; k < 6; k++, i++)
        {
            b <<= 1;
            if ((i >> 3) < n)
                b |= (byte[(i >> 3)] >> (7 - (i & 0x7))) & 0x1;
        }
        str[j++] = table[b];
    }
    while (j & 0x3)
        str[j++] = '=';
    str[j] = '\0';
    return j;
}

/* send ntrip client request -------------------------------------------------*/
int reqntrip_c(ntrip_t *ntrip)
{
    char buff[1024] = {0};
    char user[512] = {0};
    char *p = buff;

    p += sprintf(p, "GET %s/%s HTTP/1.0\r\n", ntrip->url, ntrip->mntpnt);
    p += sprintf(p, "User-Agent: NTRIP %s\r\n", NTRIP_AGENT);

    if (!*ntrip->user)
    {
        p += sprintf(p, "Accept: */*\r\n");
        p += sprintf(p, "Connection: close\r\n");
    }
    else
    {
        p += sprintf(p, "Accept: */*\r\n");
        p += sprintf(p, "Connection: Keep-Alive\r\n");
        sprintf(user, "%s:%s", ntrip->user, ntrip->passwd);
        p += sprintf(p, "Authorization: Basic ");
        p += encbase64(p, (unsigned char *)user, strlen(user));
        p += sprintf(p, "\r\n");
    }
    p += sprintf(p, "\r\n");

    if (writetcpcli(ntrip->tcp, (unsigned char *)buff, p - buff) != p - buff)
        return 0;

    ntrip->state = 1;
    return 1;
}
int rspntrip_c(ntrip_t *ntrip)
{
    int i;
    char *p, *q;

    ntrip->buff[ntrip->nb] = '\0';

    if ((p = strstr((char *)ntrip->buff, NTRIP_RSP_OK_CLI)))
    { /* ok */
        q = (char *)ntrip->buff;
        p += strlen(NTRIP_RSP_OK_CLI);
        ntrip->nb -= p - q;
        for (i = 0; i < ntrip->nb; i++)
            *q++ = *p++;
        ntrip->state = 2;
        return 1;
    }
    if ((p = strstr((char *)ntrip->buff, NTRIP_RSP_SRCTBL)))
    { /* source table */
        if (!*ntrip->mntpnt)
        { /* source table request */
            ntrip->state = 2;
            return 1;
        }
        ntrip->nb = 0;
        ntrip->buff[0] = '\0';
        ntrip->state = 0;
        discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
    }
    else if ((p = strstr((char *)ntrip->buff, NTRIP_RSP_HTTP)))
    { /* http response */
        if ((q = strchr(p, '\r')))
            *q = '\0';
        else
            ntrip->buff[128] = '\0';
        ntrip->nb = 0;
        ntrip->buff[0] = '\0';
        ntrip->state = 0;
        discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
    }
    else if (ntrip->nb >= NTRIP_MAXRSP)
    { /* buffer overflow */
        ntrip->nb = 0;
        ntrip->buff[0] = '\0';
        ntrip->state = 0;
        discontcp(&ntrip->tcp->svr, ntrip->tcp->tirecon);
    }
    return 0;
}

int waitntrip(ntrip_t *ntrip)
{
    int n;
    char *p;

    if (ntrip->state < 0)
        return 0; /* error */

    if (ntrip->tcp->svr.state < 2)
        ntrip->state = 0; /* tcp disconnected */

    if (ntrip->state == 0)
    {
        if (!g_Config_param.ntrip_user.empty() &&
            !g_Config_param.ntrip_pwd.empty() &&
            !g_Config_param.ntrip_mnt.empty())
        {
            /* send request */
            if (!reqntrip_c(ntrip))
            {
                return 0;
            }
        }
        else
        {
            ntrip->state = 2;
        }
    }
    if (ntrip->state == 1)
    { /* read response */
        p = (char *)ntrip->buff + ntrip->nb;
        if ((n = readtcpcli(ntrip->tcp, (unsigned char *)p, NTRIP_MAXRSP - ntrip->nb - 1)) == 0)
        {
            return 0;
        }
        ntrip->nb += n;
        ntrip->buff[ntrip->nb] = '\0';

        /* wait response */
        return rspntrip_c(ntrip);
    }
    return 1;
}

/* open ntrip ----------------------------------------------------------------*/
ntrip_t *openntrip()
{
    ntrip_t *ntrip;
    int i;
    if (!(ntrip = (ntrip_t *)malloc(sizeof(ntrip_t))))
        return nullptr;

    ntrip->state = 0;
    ntrip->type = 1; /* 0:server,1:client */
    ntrip->nb = 0;
    ntrip->url[0] = '\0';
    ntrip->mntpnt[0] = ntrip->user[0] = ntrip->passwd[0] = ntrip->str[0] = '\0';
    memset(ntrip->buff, 0, sizeof(ntrip->buff));

    /* decode tcp/ntrip path */
    strcpy(ntrip->user, g_Config_param.ntrip_user.c_str());
    strcpy(ntrip->passwd, g_Config_param.ntrip_pwd.c_str());
    strcpy(ntrip->mntpnt, g_Config_param.ntrip_mnt.c_str());

    /* open tcp client stream */
    if (!(ntrip->tcp = opentcpcli()))
    {
        free(ntrip);
        return nullptr;
    }
    return ntrip;
}
/* close ntrip ---------------------------------------------------------------*/
void closentrip(ntrip_t *ntrip)
{
    closetcpcli(ntrip->tcp);
    free(ntrip);
}
/* read ntrip ----------------------------------------------------------------*/
int readntrip(ntrip_t *ntrip, unsigned char *buff, int n)
{
    int nb;
    if (!waitntrip(ntrip))
        return 0;
    if (ntrip->nb > 0)
    { /* read response buffer first */
        nb = ntrip->nb <= n ? ntrip->nb : n;
        memcpy(buff, ntrip->buff + ntrip->nb - nb, nb);
        ntrip->nb = 0;
        return nb;
    }
    return readtcpcli(ntrip->tcp, buff, n);
}

/* write ntrip ---------------------------------------------------------------*/
int writentrip(ntrip_t *ntrip, unsigned char *buff, int n)
{
    if (!waitntrip(ntrip))
        return 0;
    return writetcpcli(ntrip->tcp, buff, n);
}
