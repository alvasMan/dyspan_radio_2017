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
    std::string desc;
    double f_center;
    double rf_freq;
    double dsp_freq;
    double bandwidth;
} ChannelConfig;



#endif // CHANNELS_H

