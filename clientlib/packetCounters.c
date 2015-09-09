#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "spectrum.h"


#include <math.h>


void initBuffers(struct packetCounter* packetCounter){
	int i;
	int mult=1;
	for(i=0;i<NUM_BUFFERS;i++){
		mult*=2;
		packetCounter->meanBuffers[i]=0;
	}
	packetCounter->baseBufferSize=NUM_ELEMENTS_BASE*mult;
	packetCounter->baseBuffer=calloc(packetCounter->baseBufferSize, sizeof(int));
	packetCounter->bufferIndex=0;

}

void addData(struct packetCounter* packetCounter,int size){
	int i, mult=1;
	for(i=0;i<NUM_BUFFERS;i++){
		int addIndex, removeIndex;
		addIndex=packetCounter->bufferIndex-NUM_ELEMENTS_BASE*mult;
		removeIndex=packetCounter->bufferIndex-2*NUM_ELEMENTS_BASE*mult;
		if(addIndex<0) addIndex+=packetCounter->baseBufferSize;
		if(removeIndex<0) removeIndex+=packetCounter->baseBufferSize;
		packetCounter->meanBuffers[i]+=packetCounter->baseBuffer[addIndex];
		packetCounter->meanBuffers[i]-=packetCounter->baseBuffer[removeIndex];
		mult*=2;
	}
	packetCounter->baseBuffer[packetCounter->bufferIndex]=size;
	if(packetCounter->bufferIndex<packetCounter->baseBufferSize-1){
		packetCounter->bufferIndex++;
	}

}


double getThroughtput(struct packetCounter* packetCounter,int fields){
	int i, numReadFromBase, actualFields, actualFieldsNeeded, mult=1, fieldsUsed=0;;
	uint64_t sum=0;
	if(fields>packetCounter->bufferIndex){
		actualFields=packetCounter->bufferIndex;
	}else{
		actualFields=fields;
	}
	actualFieldsNeeded=actualFields;

	if(actualFields>NUM_ELEMENTS_BASE){
		numReadFromBase=NUM_ELEMENTS_BASE;
	}else{
		numReadFromBase=actualFields;
	}

	for(i=1;i<=numReadFromBase;i++){
		sum+=packetCounter->baseBuffer[packetCounter->bufferIndex-i];
	}
	actualFieldsNeeded-=numReadFromBase;
	fieldsUsed+=numReadFromBase;
	i=0;
	while(actualFieldsNeeded){
		int len=NUM_ELEMENTS_BASE*mult;
		sum+=packetCounter->meanBuffers[i];
		fieldsUsed+=len;
		if(actualFieldsNeeded>len){
			actualFieldsNeeded-=len;
		}else{
			actualFieldsNeeded=0;
		}
		i++;
		mult*=2;
	}
	if(fieldsUsed>packetCounter->bufferIndex){
		fieldsUsed=packetCounter->bufferIndex;
	}

	double result = (double)sum/(double)fieldsUsed*100*8;
	if(isnan(result)) result=0;
	return result;


}
