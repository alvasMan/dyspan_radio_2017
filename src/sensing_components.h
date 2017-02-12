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
#include "../utils/json.hpp"
#include <map>

#ifndef SENSINGMODULE_H
#define SENSINGMODULE_H

class SensingModule
{
public:

    SensingModule(ChannelPowerEstimator* estim, bool crash_flag = false) : pwr_estim(estim), crash_on_overflow(crash_flag)
    {
    } // copy
    void setup_rx_chain(uhd::usrp::multi_usrp::sptr utx); ///< Configure the rx_stream
    void run(); ///< Run the USRP Rx and stores the values in pwr_estim
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
    bool crash_on_overflow;
};

class SituationalAwarenessApi;

class SensingHandler
{
public:
    int Nch = 4;
    std::unique_ptr<SensingModule> sensing_module;
    std::unique_ptr<ChannelPowerEstimator> pwr_estim;
    std::unique_ptr<SpectrogramGenerator> spectrogram_module;
    std::unique_ptr<TrainingJsonManager> json_learning_manager;
    std::unique_ptr<ChannelPacketRateTester> channel_rate_tester;
    SituationalAwarenessApi *pu_scenario_api;
};

class SpectrogramResizer
{
public:
    SpectrogramResizer(const BinMask& bmask, int n_out = 64) : bin_mask(bmask), Nout(n_out), out_mat_frac(Nout*bmask.n_sections(),-1)
    {
        setup();
    }
    void setup();
    void resize_line(vector<float>& out_vec, const vector<float>& in_vec);
    BinMask bin_mask;
    int Nout = 64;
private:
    vector<float> out_mat_frac;
};

namespace sensing_utils
{
SensingHandler make_sensing_handler(int Nch, std::string project_folder, std::string json_read_filename,
                                             std::string json_write_filename, SituationalAwarenessApi *pu_scenario_api, 
                                    bool has_sensing, bool has_deep_learning);

void launch_sensing_thread(uhd::usrp::multi_usrp::sptr& usrp_tx, SensingHandler* shandler);

void launch_learning_thread(uhd::usrp::multi_usrp::sptr& usrp_tx, SensingHandler* shandler);
//void launch_spectrogram_results_handler(uhd::usrp::multi_usrp::sptr& usrp_tx, ChannelPowerEstimator* estim);

void launch_spectrogram_to_file_thread(SensingHandler* shandler);

};

#endif /* SENSINGMODULE_H */

