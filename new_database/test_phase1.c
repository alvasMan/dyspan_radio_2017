#include <packetLib.h>
#include <stdio.h>

int main(void) {
	char errorBuf[32];
	int myRadio;
	uint8_t packetBuffer[1500];
        uint64_t total_pu, total_pu_provided, total_su, total_su_provided;
        double ratio_pu, ratio_su;

	/*
	 * Create and connect the transmitter.
	 * Normally this would be on two different machines.
	 */
	spectrum* demoTx = spectrum_init(0);
	spectrum_eror_t retVal = spectrum_connect(demoTx, "127.0.0.1", 5002, 1500, 1);
	spectrum_errorToText(demoTx, retVal, errorBuf, sizeof(errorBuf));
	printf("TX connect: %s\n",errorBuf);
	if(retVal<0) return 1;

	/*
	 * Create the receiver
	 */
	spectrum* demoRx = spectrum_init(0);
	retVal = spectrum_connect(demoRx, "127.0.0.1", 5002, 0, 0);
	spectrum_errorToText(demoRx, retVal, errorBuf, sizeof(errorBuf));
	printf("RX connect: %s\n",errorBuf);
	if(retVal<0) return 1;

        spectrum* SURx = spectrum_init(0);
        retVal = spectrum_connect(SURx, "127.0.0.1", 5003, 0, 0);
	spectrum_errorToText(SURx, retVal, errorBuf, sizeof(errorBuf));
	printf("SU RX connect: %s\n",errorBuf);
	if(retVal<0) return 1;
	
	/*
	 * Get our radio number. Using RX and TX context gives same result. (It should be 1)
	 */
	myRadio = spectrum_getRadioNumber(demoRx);
	printf("Radio number: %u\n", myRadio);


	/* Wait for stage 1. This stage is entered when all radios (PU Tx/RX, SU Tx/Rx are connected).
	 * In 60 seconds the first real round will start. This is provided on request to provide a synchronisation 
         * hook so SU Tx and Rx can sync. You may transmit OTA. The PU is off.
         * You may wait for stage 2 directly if you don't need this */
	/*spectrum_waitForState(demoTx, 1, -1);*/
	spectrum_waitForState(demoRx, 1, -1);
	printf("Stage 1 has started.\n");
	
	/* Wait for stage 2. In this stage you should recognize the scenarios. It lasts 10 minutes. */
	/*spectrum_waitForState(demoTx, 2, -1);*/
	spectrum_waitForState(demoRx, 2, -1);
	printf("Stage 2 has started.\n");


	/* This examples shows how to use waitForState without blocking */
	while(spectrum_waitForState(demoRx, 3, 0) != 3){

		usleep(125000);
	}

	

	/*
	 * Wait for the start of stage 3 (here you get penalized for interference).
	 */
	/*spectrum_waitForState(demoTx, 3, -1);*/
	spectrum_waitForState(demoRx, 3, -1);
	printf("Stage 3 has started.\n");

	int i=0,j;
	while(1){
		i++;
		/*
		 * This gets a packet. Your radio should transmit it.
		 */
		/*retVal = spectrum_getPacket(demoTx, packetBuffer, sizeof(packetBuffer), -1);
		spectrum_errorToText(demoTx, retVal, errorBuf, sizeof(errorBuf));*/
		printf("Get Packet: %s:\n",errorBuf);
		if(retVal<0) continue;

		for(j=0;j<retVal;j++){
			printf("%02X ", packetBuffer[j]);
			if(j % 32 == 31){
				printf("\n");
			}
		}
		printf("\n");
		/*
		 * The transmission is not quite perfect, so some packets get damaged.
		 */
		if(i%16==15){
			packetBuffer[44]++;
		}
		/*
		 * We have "received" a packed and will now deliver it to the database.
		 */
		if(retVal>0){
			retVal = spectrum_putPacket(demoRx, packetBuffer, retVal);
			spectrum_errorToText(demoRx, retVal, errorBuf, sizeof(errorBuf));
			printf("Put Packet: %s\n",errorBuf);
		}
		/*
		 * Lets see how to PU is doing, you may call this with the Tx and Rx context
		 */
		/*fprintf(stderr,"\nPU: %.02f bps/%.02f bps\n", spectrum_getThroughput(demoTx, 0, 1000), spectrum_getProvidedThroughput(demoTx, 0, 1000));*/
		/*
		 * How are we doing?
		 */
		/*fprintf(stderr,"SU: %.02f bps/%.02f bps\n\n", spectrum_getThroughput(demoTx, myRadio, 1000), spectrum_getProvidedThroughput(demoTx, myRadio, 1000));*/

		usleep(75000);
	}

	/*
	 * Clean up
	 */
	/*spectrum_delete(demoTx);*/
	spectrum_delete(demoRx);

	return 0;
}
