#include "spectrum.h"

#ifndef SOCKET_H_
#define SOCKET_H_

spectrum_eror_t openSocket(const char* hostname, unsigned int port, int socketType, int packetLen);


#endif /* SOCKET_H_ */
