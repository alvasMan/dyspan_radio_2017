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
#include "../utils/json.hpp"
#include "monitor_components.h"
#include "SU_parameters.h"
#include "matrix_utils.h"
#include <map>
#include <boost/optional.hpp>
#include <boost/asio.hpp>
#include "usrp_components.h"
#include "markov_chain_components.h"

using std::vector;
using std::string;

#ifndef SENSINGMODULE_H
#define SENSINGMODULE_H


//class SensingModule
//{
//public:
//
//    SensingModule(ChannelPowerEstimator* estim, bool crash_flag = false) : pwr_estim(estim), crash_on_overflow(crash_flag)
//    {
//    } // copy
//    void setup_rx_chain(uhd::usrp::multi_usrp::sptr utx); ///< Configure the rx_stream
//    void run(); ///< Run the USRP Rx and stores the values in pwr_estim
//    void start();
//    bool recv_fft_pwrs();
//
//    ChannelPowerEstimator *pwr_estim;
//    std::unique_ptr<PacketDetector> packet_detector;
//    double current_timestamp;
//    uhd::time_spec_t tspec;
//
//private:
//    uhd::usrp::multi_usrp::sptr usrp_tx;
//    uhd::rx_streamer::sptr rx_stream;
//    bool overflow_message = true;
//    uhd::rx_metadata_t metadata;
//    bool crash_on_overflow;
//};

class SituationalAwarenessApi;

class SensingThreadHandler
{
public:
    int Nch;
    ChannelPowerEstimator pwr_estim;
    USRPReader usrp_reader;
    PacketDetector packet_detector;
    SlidingChannelPacketRateMonitor rate_monitor;//TimedChannelPacketRateMonitor rate_monitor;//SlidingChannelPacketRateMonitor rate_monitor;
    ForgetfulChannelMonitor pwr_monitor;
    ChannelPacketRateTester channel_rate_tester;
    
    BinMask maskprops2;
    
    SensingThreadHandler() = default;
    void setup(SituationalAwarenessApi *pu_scenario_api, SU_tx_params* su_params, 
                vector<std::unique_ptr<buffer_utils::bounded_buffer<ChPowers>>>& bufs, 
                int _Nch = 4, int Nfft = 512);
    void run(uhd::usrp::multi_usrp::sptr& usrp_tx);

private:
    SituationalAwarenessApi* pu_api = nullptr;
    SU_tx_params* su_api = nullptr;
    vector<std::unique_ptr<buffer_utils::bounded_buffer<ChPowers>>> *spectrogram_buffers = nullptr;
};

class LearningThreadHandler
{
public:
    string project_folder = "";
    string json_read_filename = "";
    string json_write_filename = "";
    
    int Nch;
    BinMask maskprops2;
    
    // components
    USRPReader usrp_reader;
    ChannelPowerEstimator pwr_estim;
    PacketDetector packet_detector;
    ChannelPacketRateMonitor rate_monitor;
    boost::optional<TrainingJsonManager> json_learning_manager;
    
    void setup_filepaths(const string& folder_name, const string& json_rfile, const string& json_wfile);
    void setup(vector<std::unique_ptr<buffer_utils::bounded_buffer<ChPowers>>>& bufs, int _Nch = 4, int Nfft = 512);
    void run(uhd::usrp::multi_usrp::sptr& usrp_tx);
    
private:
    vector<std::unique_ptr<buffer_utils::bounded_buffer<ChPowers>>> *spectrogram_buffers;
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

class Spectrogram2FileThreadHandler
{
private:
    buffer_utils::bounded_buffer<ChPowers>* sensing_buffer = NULL;
    pair<int,int> CNNdims = {0,0};
    SpectrogramResizer sp_resizer;
    VectorMovingAverage mov_avg;
    Matrix<float> mat;
    size_t current_row = 0;
    size_t current_imgno = 0;
public:
    Spectrogram2FileThreadHandler(buffer_utils::bounded_buffer<ChPowers>* buf, 
               const BinMask& bmask, pair<int,int> CNN_dim = {64,64}, int step_size = 15);
    void run();
};

typedef boost::asio::ip::tcp::socket socket_type;
//typedef boost::asio::local::stream_protocol::socket socket_type;
typedef std::unique_ptr<socket_type> socket_ptr;

class Spectrogram2SocketThreadHandler
{
private:
    SituationalAwarenessApi *pu_api = NULL;
    buffer_utils::bounded_buffer<ChPowers>* sensing_buffer = NULL;
    pair<int,int> CNNdims = {0,0};
    //SpectrogramResizer sp_resizer;
    size_t moving_average_step = 15;
    Matrix<float> mat;
    size_t current_row = 0;
    size_t current_imgno = 0;
    
    DeepLearningModeCounter mode_counter;
    
    buffer_utils::bounded_buffer<std::pair<size_t, std::chrono::system_clock::time_point>> time_buffer{1000};
    
    boost::asio::io_service io_service;
    
    socket_ptr soc;
public:
    Spectrogram2SocketThreadHandler(Spectrogram2SocketThreadHandler&) = delete;
    Spectrogram2SocketThreadHandler(SituationalAwarenessApi* api, 
               buffer_utils::bounded_buffer<ChPowers>* buf, 
               const BinMask& bmask, pair<int,int> CNN_dim = {64,64}, int step_size = 15);
    void run_send();
    void run_recv();
};

namespace sensing_utils
{
//SensingHandler make_sensing_handler(int Nch, std::string project_folder, std::string json_read_filename,
//                                             std::string json_write_filename, SituationalAwarenessApi *pu_scenario_api, 
//                                    bool has_sensing, bool has_learning);
//
//void launch_sensing_thread(uhd::usrp::multi_usrp::sptr& usrp_tx, SensingHandler* shandler);
//
//void launch_learning_thread(uhd::usrp::multi_usrp::sptr& usrp_tx, SensingHandler* shandler);
////void launch_spectrogram_results_handler(uhd::usrp::multi_usrp::sptr& usrp_tx, ChannelPowerEstimator* estim);
//
//void launch_spectrogram_to_file_thread(SensingHandler* shandler);

};

#endif /* SENSINGMODULE_H */

