/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2015  Andre Puschmann, Francisco Paisana, Justin Tallon
 *
 * \section LICENSE
 *
 * This file is part of dyspanradio.
 *
 * dyspanradio is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * dyspanradio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#ifndef DYPANRADIO_H
#define DYPANRADIO_H

#include <boost/thread.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/convert.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/types/tune_request.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include "aligned_allocator.h"
#include "channels.h"

typedef std::vector<std::complex<float> > CplxFVec;
typedef std::vector<unsigned char> CharVec;
typedef std::vector<std::complex<float>, aligned_allocator<__m128, sizeof(__m128)> > ACplxFVec;

#define NUM_PADDING_NULL_SAMPLES 100
#define MAX_PAYLOAD_LEN 2500

typedef struct
{
    // generic radio parameters
    bool debug;
    bool has_sensing;
    std::string mode;
    std::string args;
    std::string txsubdev;
    std::string rxsubdev;
    size_t num_trx;
    size_t num_channels;
    double f_center;
    double channel_bandwidth;
    double channel_rate;
    double rx_gain_uhd;
    double tx_gain_uhd;
    double tx_gain_soft;
    std::string rx_antenna;
    bool tx_enabled;
    unsigned int stat_interval;

    // OFDM parameter
    unsigned int M;                     // number of subcarriers
    unsigned int cp_len;                // cyclic prefix length
    unsigned int taper_len;             // taper length
    unsigned char* p;                   // subcarrier allocation
    std::string fec0;
    std::string fec1;
    std::string crc;
    std::string mod;

    // challenge database parameters
    bool use_db;
    std::string db_ip;
    std::string db_user;
    std::string db_password;
} RadioParameter;


class DyspanRadio
{
public:
    DyspanRadio(const RadioParameter params) :
        radio_id_(-2),
        params_(params)
    {
        // validate input
        if (params.num_channels < 1) {
            throw std::runtime_error("error: must have at least one channel");
        } else if (params_.M < 8) {
            throw std::runtime_error("number of subcarriers must be at least 8");
        } else if (params_.cp_len < 1) {
            throw std::runtime_error("cyclic prefix length must be at least 1");
        } else if (params_.taper_len > params_.cp_len) {
            throw std::runtime_error("taper length cannot exceed cyclic prefix length");
        }

        // this is a special case for the 4x 5MHz receiver using two N210s
        if (params_.channel_bandwidth == 5e6 && params_.num_channels == 4 && params_.num_trx == 2) {
            // this initializes 4 channels such that the first two have the same rf_freq and the last two.
            // this makes sure that each N210 is tuned to two channels with the LO sitting between them
            std::cout << boost::str(boost::format("Configuring channels for 4x 5MHz using two N210s")) << std::endl;
            double offset = params_.channel_bandwidth / 2;
            double rf_freq = params_.f_center - params_.channel_bandwidth;
            channels_.push_back({"Channel0", params_.f_center + offset, params_.channel_bandwidth, rf_freq, +offset, params_.channel_rate});
            channels_.push_back({"Channel1", params_.f_center + offset, params_.channel_bandwidth, rf_freq, -offset, params_.channel_rate});

            rf_freq = params_.f_center + params_.channel_bandwidth;
            channels_.push_back({"Channel2", params_.f_center + offset, params_.channel_bandwidth, rf_freq, +offset, params_.channel_rate});
            channels_.push_back({"Channel3", params_.f_center + offset, params_.channel_bandwidth, rf_freq, -offset, params_.channel_rate});
        } else {
          if (params_.num_channels == 1) {
            std::cout << boost::str(boost::format("Configuring single channel")) << std::endl;
            double rf_freq = params_.f_center;
            channels_.push_back({"Channel0", params_.f_center, params_.channel_bandwidth, rf_freq, 0, params_.channel_rate});
          } else {
            std::cout << boost::str(boost::format("Configuring multiple channels")) << std::endl;
            // this initializes n channels such that the LO is set n*ch_bw apart from the given center freq
            double guard = 0.5e6;
            double lo_offset = params_.num_channels * params_.channel_bandwidth/2 + guard;
            double rf_freq = params_.f_center - lo_offset;
            for (int i = 0; i < params_.num_channels; i++) {
              std::string desc("Channel" + std::to_string(i));
              double dsp_offset = guard + params_.channel_bandwidth/2 + i*params_.channel_bandwidth;
              channels_.push_back({desc,
                                    params_.f_center,
                                    params_.channel_bandwidth,
                                    rf_freq,
                                    dsp_offset,
                                    params_.channel_rate});
            }
          }

        }
        // TODO: check that all channels have the same center frequency for faster tuning

        // Print configuration for each of the channels
        for (int i = 0; i < channels_.size(); i++) {
          std::cout << boost::str(boost::format("%s: f_c=%.5f GHz rf_freq=%.5f GHz dsp_freq=%.5f GHz bw=%.5f MHz rate=%.5f MHz") %
              (channels_[i].desc) %
              ((channels_[i].rf_freq + channels_[i].dsp_freq)/1e9) %
              (channels_[i].rf_freq/1e9) %
              (channels_[i].dsp_freq/1e6) %
              (channels_[i].bandwidth/1e6) %
              (channels_[i].rate/1e6)) << std::endl;
        }
    }

    virtual ~DyspanRadio() {}

    virtual void start() = 0;
    void stop()
    {
        // stop threads in reverse order
        boost::ptr_vector<boost::thread>::reverse_iterator it;
        for (it = threads_.rbegin(); it != threads_.rend(); ++it) {
            it->interrupt();
            it->join();
        }
    }

    void set_channel(uint32_t channel) {};

protected:
    // generic properties
    RadioParameter params_;
    std::vector< ChannelConfig > channels_;

    // needed for the challenge database
    int radio_id_;
    char error_buffer[32];          // buffer to hold error messages from challenge server

    boost::ptr_vector<boost::thread> threads_;
};

#endif // DYPANRADIO_H

