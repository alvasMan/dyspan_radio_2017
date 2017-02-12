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
#include <pthread.h>
#include <signal.h>
#include <poll.h>
#include <math.h>
#include "spectrum.h"
#include "socket.h"




__attribute__((visibility("default"))) spectrum*  spectrum_init(char debug) {
	spectrum* newSpectrum = calloc(1, sizeof(spectrum));
	if(newSpectrum){
		newSpectrum->debug=debug;
		newSpectrum->isTransmitter=2;
		pthread_mutex_init(&newSpectrum->mutex, NULL);
	}
	return newSpectrum;
}

void __attribute__((visibility("default"))) spectrum_delete(spectrum* ctx){
	int i;
	if(ctx){
		if(ctx->socket_thread_created){
			/*
			 * Tell thread to quit
			 */
			ctx->socket_thread_quit=1;
			/* Sending a null signal does nothing, but it tests if it would be possible to send it
			 * Thus, it tests if the thread is alive.
			 */
			i=0;
			while ((i<100) && (pthread_kill(ctx->socket_thread, 0) == 0)) {
				usleep(50000);
				i++;
			}
			if(i==100){
				// The thread did not die on its own in 5 seconds, so we cancel it.
				pthread_cancel(ctx->socket_thread);
			}
			/* Pthread_join will wait for the thread to terminate and free the resources of the thread */
			pthread_join(ctx->socket_thread, NULL);
		}

		if(ctx->isTransmitter==0){
			close(ctx->fdCmd);
			close(ctx->fdRx);
		}
		if(ctx->isTransmitter==1){
			close(ctx->fdCmd);
			close(ctx->fdTx);
		}
		pthread_mutex_destroy(&ctx->mutex);
		free(ctx);
	}
}

spectrum_eror_t __attribute__((visibility("default"))) spectrum_getRadioNumber(spectrum* ctx){
	if(!ctx) return ERROR_INVALID;
	if(ctx->isTransmitter>=2) return ERROR_INVALID;
	return 1;
}

spectrum_eror_t __attribute__((visibility("default"))) spectrum_connect(spectrum* ctx, const char* hostname, uint16_t port, uint16_t requestedPacketLen, uint8_t isTransmitter){
	if(!ctx) return ERROR_INVALID;
	if(!hostname) return ERROR_INVALID;
	if(isTransmitter) isTransmitter=1;
	pthread_mutex_lock(&ctx->mutex);
	ctx->isTransmitter=isTransmitter;

	ctx->fdCmd = openSocket(hostname, port, 2, 0);
	if(ctx->fdCmd < 0){
		pthread_mutex_unlock(&ctx->mutex);
		return ctx->fdCmd;
	}
	if(isTransmitter){
		ctx->fdTx = openSocket(hostname, port, 0, requestedPacketLen);
		if(ctx->fdTx < 0) {
			close(ctx->fdCmd);
			pthread_mutex_unlock(&ctx->mutex);
			return ctx->fdTx;
		}
	}else{
		ctx->fdRx = openSocket(hostname, port, 1, 0);
		if(ctx->fdRx < 0) {
			pthread_mutex_unlock(&ctx->mutex);
			close(ctx->fdCmd);
			return ctx->fdRx;
		}
	}

	initBuffers(&ctx->cntProvidedThroughput[0]);
	initBuffers(&ctx->cntProvidedThroughput[1]);
	initBuffers(&ctx->cntDeliveredThroughput[0]);
	initBuffers(&ctx->cntDeliveredThroughput[1]);

	if(pthread_create( &ctx->socket_thread , NULL, (void*)spectrum_thread, ctx)){
		close(ctx->fdCmd);
		if(isTransmitter){
			close(ctx->fdTx);
		}else{
			close(ctx->fdRx);
		}
		pthread_mutex_unlock(&ctx->mutex);
		return ERROR_OTHER;
	}
	ctx->socket_thread_created=1;


	pthread_mutex_unlock(&ctx->mutex);
	return 0;
}

spectrum_eror_t __attribute__((visibility("default"))) spectrum_getPacket(spectrum* ctx, uint8_t* buffer, uint32_t bufferLength, int32_t timeoutMs){
	if(!ctx) return ERROR_INVALID;
	if(!buffer) return ERROR_INVALID;

	if(ctx->isTransmitter!=1) return ERROR_INVALID;
	pthread_mutex_lock(&ctx->mutex);

	uint8_t buf[1500];
	buf[0]=0;
	int len;
	while(buf[0]!='P'){
			recv(ctx->fdTx, buf, 4, 0);
			len=(buf[2]<<8)|buf[3];
			if(len>sizeof(buf)){
				pthread_mutex_unlock(&ctx->mutex);
				return ERROR_OTHER;
			}
			recv(ctx->fdTx, buf, len, 0);
	}
	len-=1;
	if(len>=bufferLength){
		pthread_mutex_unlock(&ctx->mutex);
		return ERROR_BUF;
	}
	memcpy(buffer,buf+1,len);
	buf[0]=0;
	buf[1]=0;
	buf[2]=0;
	buf[3]=1;
	buf[4]='F';
	send(ctx->fdTx, buf, 5, 0);
	pthread_mutex_unlock(&ctx->mutex);
	return len;
}

spectrum_eror_t __attribute__((visibility("default"))) spectrum_putPacket(spectrum* ctx, uint8_t* buffer, uint32_t bufferLength){
	if(!ctx) return ERROR_INVALID;
	if(!buffer) return ERROR_BUF;

	if(ctx->isTransmitter!=0) return ERROR_INVALID;

	if(bufferLength>1500) return ERROR_BUF;
	pthread_mutex_lock(&ctx->mutex);

    uint8_t buf[1505];
    bufferLength++;
    buf[0]=(bufferLength>>24)&0xFF;
    buf[1]=(bufferLength>>16)&0xFF;
    buf[2]=(bufferLength>>8)&0xFF;
    buf[3]=(bufferLength)&0xFF;
    bufferLength--;
    buf[4]='P';
    memcpy(buf+5, buffer, bufferLength);

    send(ctx->fdRx, buf, 5+bufferLength, 0);

    pthread_mutex_unlock(&ctx->mutex);
    return ERROR_OK;
}

spectrum_eror_t __attribute__((visibility("default"))) spectrum_waitForState(spectrum* ctx, uint32_t wantedState, int32_t timeoutMs){
	if(!ctx) return ERROR_INVALID;
	if(wantedState>3 || wantedState==0) return ERROR_INVALID;



	return 3;
}

void __attribute__((visibility("default"))) spectrum_errorToText(spectrum* ctx, spectrum_eror_t error, char* output, uint32_t len){
	if(error>0){
		snprintf(output, len, "%u", error);
		return;
	}
	switch(error){
	case ERROR_OK:
		snprintf(output, len, "OK");
		break;
	case ERROR_INVALID:
		snprintf(output, len, "Invalid");
		break;
	case ERROR_AUTH:
		snprintf(output, len, "Authentication error");
		break;
	case ERROR_TIMEOUT:
		snprintf(output, len, "Timeout");
		break;
	case ERROR_CONNECT_REFUSED:
		snprintf(output, len, "Connection refused");
		break;
	case ERROR_BUF:
		snprintf(output, len, "Buffer error");
		break;
	default:
		snprintf(output, len, "Unknown error");
	}
}

void __attribute__((visibility("default"))) spectrum_getStatusMessage(spectrum* ctx, spectrum_eror_t error, char* output, uint32_t len){
	if(len>=1){
		output[0]=0;
	}
}

double __attribute__((visibility("default"))) spectrum_getThroughput(spectrum* ctx, uint8_t radioNumber, int durationMs){
	if(radioNumber>=2) return 0;
	if(durationMs<0) durationMs=1024000-1;
	if(durationMs<10) durationMs=10;
	pthread_mutex_lock(&ctx->mutex);
	double result=getThroughtput(&ctx->cntDeliveredThroughput[radioNumber],durationMs/10);
	pthread_mutex_unlock(&ctx->mutex);
	return result;
}

double __attribute__((visibility("default"))) spectrum_getProvidedThroughput(spectrum* ctx, uint8_t radioNumber, int durationMs){
	if(radioNumber>=2) return 0;
	if(durationMs<0) durationMs=99999999;
	if(durationMs<10) durationMs=10;
	pthread_mutex_lock(&ctx->mutex);
	double result = getThroughtput(&ctx->cntProvidedThroughput[radioNumber],durationMs/10);
	pthread_mutex_unlock(&ctx->mutex);
	return result;
}
void spectrum_command(spectrum* ctx, uint8_t* cmd, int len) {
	uint32_t providedThroughput, deliveredThroughput;
	uint8_t radioId;
	if(len==0) return;
	switch(cmd[0]){
	case 'R':
		if(len<10) return;
		radioId=cmd[1];
		if(radioId<2){
			providedThroughput=(cmd[2]<<24)|(cmd[3]<<16)|(cmd[4]<<8)|(cmd[5]);
			deliveredThroughput=((cmd[6] & 0xFF)<<24)|(cmd[7]<<16)|(cmd[8]<<8)|(cmd[9]);
		//	printf("%u: Provided %u, Delivered %u\n",radioId, providedThroughput,deliveredThroughput);
			pthread_mutex_lock(&ctx->mutex);
			addData(&ctx->cntProvidedThroughput[radioId],providedThroughput);
			addData(&ctx->cntDeliveredThroughput[radioId],deliveredThroughput);
			pthread_mutex_unlock(&ctx->mutex);

		}
		break;
	}
}

void spectrum_thread(spectrum* ctx) {
	int i,len;
	char buf[512];
    struct pollfd fds[1];
    fds[0].fd = ctx->fdCmd;
    fds[0].events = POLLIN;

	while(!ctx->socket_thread_quit){
		int rv = poll(fds, 1, 100);
		if (rv < 0) {
			perror("poll");
			exit(1);
		} else if (rv == 0) {
				//Timeout
		} else {
			if (fds[0].revents & POLLIN) {
				i=read(ctx->fdCmd, buf, 4);
				if(i<=0){
						fprintf (stderr, "Fatal error reading from database. Cannot recover\n");
						exit(1);
				}
				len=(buf[2]<<8) | (buf[3]);
				if(len<512){
					i=read(ctx->fdCmd, buf, len);
					if(i<=0){
							fprintf (stderr, "Fatal error reading from database. Cannot recover\n");
							exit(1);
					}
					spectrum_command(ctx,(uint8_t*)buf,len);
				}
			}
		}
	}
}
