#include <stdio.h>
#include <pthread.h>

#ifndef SPECTRUM_H_
#define SPECTRUM_H_

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_ELEMENTS_BASE 1000
#define NUM_BUFFERS 10

struct packetCounter{
	long meanBuffers[NUM_BUFFERS];
	int* baseBuffer;
	int bufferIndex;
	int baseBufferSize;
};

typedef struct {
	//Flags
	char debug;
	uint8_t isTransmitter;

	//Sockets
	int fdTx;
	int fdRx;
	int fdCmd;

	//Counters
	struct packetCounter cntProvidedThroughput[2];
	struct packetCounter cntDeliveredThroughput[2];

	//Thread
	pthread_t socket_thread;
	uint8_t socket_thread_created;
	uint8_t socket_thread_quit;
	
	//Mutex
	pthread_mutex_t mutex;
} spectrum;

typedef enum {ERROR_OK=0, ERROR_INVALID=-1, ERROR_AUTH=-2,
				ERROR_TIMEOUT=-3, ERROR_CONNECT_REFUSED=-4,
				ERROR_OTHER=-5, ERROR_BUF=-6} spectrum_eror_t;

spectrum* spectrum_init(char debug);
void spectrum_delete(spectrum* ctx);
spectrum_eror_t spectrum_getRadioNumber(spectrum* ctx);
spectrum_eror_t spectrum_connect(spectrum* ctx, char* hostname, uint16_t port, uint16_t requestedPacketLen, uint8_t isTransmitter);
void spectrum_errorToText(spectrum* ctx, spectrum_eror_t error, char* output, uint32_t len);
spectrum_eror_t spectrum_getPacket(spectrum* ctx, uint8_t* buffer, uint32_t bufferLength, int32_t timeoutMs);
spectrum_eror_t spectrum_putPacket(spectrum* ctx, uint8_t* buffer, uint32_t bufferLength);
spectrum_eror_t spectrum_waitForState(spectrum* ctx, uint32_t wantedState, int32_t timeoutMs);
double spectrum_getThroughput(spectrum* ctx, uint8_t radioNumber, int durationMs);
double spectrum_getProvidedThroughput(spectrum* ctx, uint8_t radioNumber, int durationMs);
void spectrum_getStatusMessage(spectrum* ctx, spectrum_eror_t error, char* output, uint32_t len);

void spectrum_command(spectrum* ctx, uint8_t* cmd, int len);
void spectrum_thread(spectrum* ctx);




void initBuffers(struct packetCounter* packetCounter);
void addData(struct packetCounter* packetCounter,int size);
double getThroughtput(struct packetCounter* packetCounter,int fields);

#ifdef __cplusplus
}
#endif

#endif /* SPECTRUM_H_ */
