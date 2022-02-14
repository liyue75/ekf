#include <uxr/client/profile/transport/ip/udp/udp_transport_posix_nopoll.h>
#include "udp_transport_internal.h"

//#include <sys/types.h>
//#include <sys/socket.h>
//#include <netdb.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include "net/ipv6/addr.h"
#include "net/netif.h"

#include "net/sock/udp.h"

//#include "byteorder.h"

//static sock_udp_t sock;
//static sock_udp_ep_t remote;


bool uxr_init_udp_platform(
        uxrUDPPlatform* platform,
        uxrIpProtocol ip_protocol,
        const char* ip,
        const char* port)
{
    (void)ip_protocol;
//    (void)platform;
	bool rv = false;

	platform->fd = socket(AF_INET6, SOCK_DGRAM, 0);
	struct sockaddr_in6 sin;
	sin.sin6_family = AF_INET6;
	memset(&sin.sin6_addr, 0, sizeof(sin.sin6_addr));
	char *iface;
	iface = ipv6_addr_split_iface((char *)ip);
	if (iface) {
		netif_t *netif = netif_get_by_name(iface);
		if (netif) {
			int16_t id = netif_get_id((const netif_t *)netif);
			sin.sin6_scope_id = (uint32_t)id;
		} else {
			printf("unknown network interface \n");
		}
	}
	if (inet_pton(AF_INET6, ip, &sin.sin6_addr) != 1) {
		printf("Error: unable to parse destination address");
		return 0;
	}
	uint16_t portt;
	portt = atoi(port);
	sin.sin6_port = htons(portt);
	if(0 == connect(platform->fd, (struct sockaddr *)&sin, sizeof(sin))) {
	    rv = true;
    }

/*
    sock_udp_ep_t local = SOCK_IPV6_EP_ANY;
    local.port = 0;
    remote.family = AF_INET6;
    if (ipv6_addr_from_str((ipv6_addr_t *)&remote.addr.ipv6, ip) == NULL) {
        printf("error parsing IPv6 address\n");
        return -1;
    }
    remote.port = atoi(port);
    if (sock_udp_create(&sock, &local, &remote, 0) < 0) {
        printf("xrce-dds unable to open UDP socket on port %i\n", local.port);
        return -1;
    }
    rv = true;
*/
    return rv;
}

bool uxr_close_udp_platform(
        uxrUDPPlatform* platform)
{
    return (-1 == platform->fd) ? true : (0 == close(platform->fd));
//    (void )platform;
//    sock_udp_close(&sock);
//    return 1;
}

size_t uxr_write_udp_data_platform(
        uxrUDPPlatform* platform,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode)
{
    //(void) platform;
    size_t rv = 0;
    //printf("send buf = %d\n", (uint32_t)buf);
    ssize_t bytes_sent = /*sock_udp_send(&sock, (void *)buf, len, NULL);*/send(platform->fd, (void*)buf, len, 0);
    if (-1 != bytes_sent)
    {
        rv = (size_t)bytes_sent;
        *errcode = 0;
    }
    else
    {
        *errcode = 1;
    }
    return rv;
}

struct timeval {
	int32_t tv_sec;
	int32_t tv_usec;
};

size_t uxr_read_udp_data_platform(
        uxrUDPPlatform* platform,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode)
{

    size_t rv = 0;

    struct timeval tv;
    tv.tv_sec = timeout / 1000;
	tv.tv_usec = (timeout % 1000) * 1000;

    setsockopt(platform->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

//    (void) platform;
//    printf("timeout=%d\n",timeout);
    ssize_t bytes_received = /*sock_udp_recv(&sock, (void * )buf, len, timeout*1000, NULL);*/recv(platform->fd, (void*)buf, len, 0);
    if (-1 != bytes_received)
    {
        rv = (size_t)bytes_received;
        *errcode = 0;
    }
    else
    {
        *errcode = 1;
    }

    return rv;
}
