#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/tcp.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <errno.h>
#include "spectrum.h"


spectrum_eror_t openSocket(char* hostname, unsigned int port, int socketType, int packetLen)
{
    int sock;
    struct sockaddr_in host_addr;

    if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        return ERROR_OTHER;
    }

    memset(&host_addr, '0', sizeof(host_addr));

    host_addr.sin_family = AF_INET;
    host_addr.sin_port = htons(port);

    if(inet_pton(AF_INET, hostname, &host_addr.sin_addr)<=0) {
        close(sock);
        return ERROR_OTHER;
    }

    if( connect(sock, (struct sockaddr *)&host_addr, sizeof(host_addr)) < 0) {
        close(sock);
        switch(errno){
        case ECONNREFUSED:
        	return ERROR_CONNECT_REFUSED;
        	break;
        case ETIMEDOUT:
        	return ERROR_TIMEOUT;
        	break;
        default:
        	return ERROR_OTHER;
        }
    }

    int flag = 1;
    setsockopt(sock,            /* socket affected */
               IPPROTO_TCP,     /* set option at TCP level */
               TCP_NODELAY,     /* name of option */
               (char *) &flag,  /* the cast is historical cruft */
               sizeof(int));    /* length of option     value */



    char buffer[10];
    buffer[0]=0;
    buffer[1]=0;
    buffer[2]=0;
    buffer[3]=4;
    buffer[4]='S';
    buffer[5]=socketType;
    buffer[6]=packetLen>>8;
    buffer[7]=packetLen;
    send(sock, buffer, 8, 0);

    return sock;
}
