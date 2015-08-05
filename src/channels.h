#ifndef CHANNELS_H
#define CHANNELS_H


#include <iostream>


typedef enum {
    CH1 = 0,
    CH2,
    CH3,
    CH4
} ChannelNum;


typedef struct {
    std::string desc;  // description
    double f_center;   // the actual center frequency of that channel (as seen on the spectrum analyzer)
    double bandwidth;  // the actual bandwidth of that channel
    double rf_freq;    // the RF frequency that the USRP is tuned to (LO)
    double dsp_freq;   // offset from the LO to reach the center frequency
    double rate;       // the sampling rate used in the channel
} ChannelConfig;



#endif // CHANNELS_H

