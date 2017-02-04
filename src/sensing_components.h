/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SensingModule.h
 * Author: connect
 *
 * Created on 31 January 2017, 14:35
 */
#include "ChannelPowerEstimator.h"
#include <uhd/usrp/multi_usrp.hpp>

#ifndef SENSINGMODULE_H
#define SENSINGMODULE_H

class SensingModule
{
public:
    SensingModule(ChannelPowerEstimator* estim) : pwr_estim(estim) {} // copy
    void setup_rx_chain(uhd::usrp::multi_usrp::sptr utx);       ///< Configure the rx_stream
    void run();                     ///< Run the USRP Rx and stores the values in pwr_estim
    void start();
    bool recv_fft_pwrs();
    
    ChannelPowerEstimator *pwr_estim;
    std::unique_ptr<PacketDetector> packet_detector;
    double current_timestamp;
    uhd::time_spec_t tspec;
    
private:
    uhd::usrp::multi_usrp::sptr usrp_tx;
    uhd::rx_streamer::sptr rx_stream;
    bool overflow_message = true;
    uhd::rx_metadata_t metadata;
};

namespace sensing_utils
{    
void launch_spectrogram_generator(uhd::usrp::multi_usrp::sptr& usrp_tx, ChannelPowerEstimator* estim);
//void launch_spectrogram_results_handler(uhd::usrp::multi_usrp::sptr& usrp_tx, ChannelPowerEstimator* estim);

void launch_spectrogram_to_file_thread(ChannelPowerEstimator* pwr_estim);
};

#endif /* SENSINGMODULE_H */

