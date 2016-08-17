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
 
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <complex>
#include <vector>
#include <liquid/liquid.h>
#include "multichannelrx_pfb.h"

#define BST_DEBUG 0

static bool verbose = true;

// global callback function
namespace multichannelrxpfbdetail
{
int gCallback( unsigned char *  _header,
               int              _header_valid,
               unsigned char *  _payload,
               unsigned int     _payload_len,
               int              _payload_valid,
               framesyncstats_s _stats,
               void *           _userdata)
{
  static_cast<multichannelrx_pfb*>(_userdata)->callback(_header,
     _header_valid,
     _payload,
     _payload_len,
     _payload_valid,
     _stats,
     _userdata);
}
}

int multichannelrx_pfb::callback(unsigned char *  _header,
             int              _header_valid,
             unsigned char *  _payload,
             unsigned int     _payload_len,
             int              _payload_valid,
             framesyncstats_s _stats,
             void *           _userdata)
{
    if (verbose) {
        std::cout << boost::format("***** rssi=%7.2fdB evm=%7.2fdB, ") % _stats.rssi % _stats.evm;
        if (_header_valid) {
            uint32_t seq_no = (_header[0] << 24 | _header[1] << 16 | _header[2] << 8 | _header[3]);

            // do the stats ..
            if (last_seq_no_ == 0) {
                std::cout << boost::format("Setting first seqno: %6u") % seq_no << std::endl;
                last_seq_no_ = seq_no;
            } else {
                // count lost frames
                lost_frames_ += (seq_no - last_seq_no_ - 1);
                last_seq_no_ = seq_no;
            }
            total_frames_++;

            std::cout << boost::format("seqno: %6u (%6u lost), ") % seq_no % lost_frames_;
            if (_payload_valid) {
                std::cout << boost::format("payload size: %d") % _payload_len;
            } else {
                std::cout << boost::format("PAYLOAD INVALID");
            }
        } else {
            std::cout << boost::format("HEADER INVALID");
        }
        std::cout << std::endl;
    }
    return 0;
}




// default constructor
multichannelrx_pfb::multichannelrx_pfb(const RadioParameter params) :
    DyspanRadio(params),
    rx_to_mix_buffer_(THREAD_BUFFER_SIZE),
    mix_to_chan_buffer_(THREAD_BUFFER_SIZE),
    buffer_factory_(100, 100),
    total_frames_(0),
    last_seq_no_(0),
    lost_frames_(0)
{
    num_sampled_chans_ = params_.num_channels;
    if (params.num_channels == 4 && params.channel_bandwidth == 5e6) {
        std::cout << "Increasing number of channels to 5 to match full USRP sample rate." << std::endl;
        num_sampled_chans_++;
    }

    // create callbacks
    userdata  = (void **)             malloc(num_sampled_chans_ * sizeof(void *));
    callbacks = (framesync_callback*) malloc(num_sampled_chans_ * sizeof(framesync_callback));
    for (int i = 0; i < params.num_channels; i++) {
        callbacks[i] = multichannelrxpfbdetail::gCallback;
        userdata[i] = this;
    }

    // create frame synchronizers
    framesync = (ofdmflexframesync*)  malloc(num_sampled_chans_ * sizeof(ofdmflexframesync));
    for (int i = 0; i < params.num_channels; i++) {
        framesync[i] = ofdmflexframesync_create(params_.M, params_.cp_len, params_.taper_len, params_.p, callbacks[i], userdata[i]);
#if BST_DEBUG
        ofdmflexframesync_debug_enable(framesync[i]);
#endif
    }

    // design custom filterbank channelizer
    unsigned int m  = 2;        // prototype filter delay
    float As        = 60.0f;    // stop-band attenuation
    channelizer = firpfbch_crcf_create_kaiser(LIQUID_ANALYZER, num_sampled_chans_, m, As);

    // create NCO to center spectrum
    float offset = -0.5f*(float)(num_sampled_chans_-1) / (float)num_sampled_chans_ * M_PI;
    nco = nco_crcf_create(LIQUID_NCO);
    nco_crcf_set_frequency(nco, offset);

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % params_.args << std::endl;
    usrp_rx = uhd::usrp::multi_usrp::make(params_.args);
    std::cout << boost::format("Using Device: %s") % usrp_rx->get_pp_string() << std::endl;

    // channelizer input/output arrays
    max_spp_ = usrp_rx->get_device()->get_max_recv_samps_per_packet();

    // create neccesary buffer objects
    for (int i = 0; i < params_.num_channels; i++) {
        chan_to_sync_buffers_.push_back(new BlockingReaderWriterQueue<ItemPtr>(THREAD_BUFFER_SIZE));
    }
    // computer actual RF rate
    double rx_rf_rate = num_sampled_chans_ * params_.channel_bandwidth;

    // reset base station transmitter
    Reset();

    // configure receiver parameters
    usrp_rx->set_rx_rate(rx_rf_rate);
    usrp_rx->set_rx_freq(params_.f_center);
    usrp_rx->set_rx_gain(params_.rx_gain_uhd);
}

// destructor
multichannelrx_pfb::~multichannelrx_pfb()
{
    // destroy NCO
    nco_crcf_destroy(nco);

    // destroy channelizer
    firpfbch_crcf_destroy(channelizer);

    // destroy frame synchronizers
    unsigned int i;
    for (i=0; i<params_.num_channels; i++) {
#if BST_DEBUG
        char filename[64];
        sprintf(filename,"framesync_channel%u.m", i);
        ofdmflexframesync_debug_print(framesync[i], filename);
#endif
        ofdmflexframesync_destroy(framesync[i]);
    }
    free(framesync);
    free(userdata);
    free(callbacks);

    std::cout << "Total frames received: " << total_frames_ << std::endl;
    std::cout << "Lost frames: " << lost_frames_ << std::endl;
    if (total_frames_ > 0) {
        float fer = static_cast<float>(lost_frames_) / static_cast<float>(total_frames_);
        std::cout << boost::str(boost::format("Frame error rate: %.2f") % fer) << std::endl;
    }
}

void multichannelrx_pfb::start(void)
{
    // start threads
    threads_.push_back( new boost::thread( boost::bind( &multichannelrx_pfb::receive_thread, this ) ) );
    threads_.push_back( new boost::thread( boost::bind( &multichannelrx_pfb::mixdown_thread, this ) ) );
    threads_.push_back( new boost::thread( boost::bind( &multichannelrx_pfb::channelizer_thread, this ) ) );

    // start a synchronizer thread for each channel
    for (int i = 0; i < chan_to_sync_buffers_.size(); i++) {
        threads_.push_back( new boost::thread( boost::bind( &multichannelrx_pfb::synchronizer_thread, this, boost::ref(chan_to_sync_buffers_[i]), i) ) );
    }
}


// reset
void multichannelrx_pfb::Reset()
{
    // reset all objects
    unsigned int i;
    for (i=0; i<params_.num_channels; i++)
        ofdmflexframesync_reset(framesync[i]);

    firpfbch_crcf_reset(channelizer);

    // reset write index of channelizer buffer
    buffer_index = 0;
}


void multichannelrx_pfb::receive_thread(void)
{
    //create a receive streamer
    std::string wire_format("sc16");
    std::string cpu_format("fc32");
    uhd::stream_args_t stream_args(cpu_format, wire_format);
    uhd::rx_streamer::sptr rx_stream = usrp_rx->get_rx_stream(stream_args);

    //setup streaming
    std::cout << std::endl;
    std::cout << boost::format("Begin streaming now ..") << std::endl;
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = 0; // continuous
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t();
    usrp_rx->issue_stream_cmd(stream_cmd);

    //meta-data will be filled in by recv()
    uhd::rx_metadata_t metadata;
    bool overflow_message = true;

    try {
        while (true) {
            boost::this_thread::interruption_point();

            ItemPtr buffer = buffer_factory_.get_new();
            size_t num_rx_samps = rx_stream->recv(&buffer->data.front(), max_spp_, metadata, 3.0);
            buffer->len = num_rx_samps;

            //handle the error code
            if (metadata.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                std::cout << boost::format("Timeout while streaming") << std::endl;
                break;
            }
            if (metadata.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
                if (overflow_message){
                    overflow_message = false;
                    std::cerr << boost::format("Got an overflow indication, please reduce sample rate.");
                }
                continue;
            }
            if (metadata.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
                throw std::runtime_error(str(boost::format(
                                                 "Unexpected error code 0x%x"
                                                 ) % metadata.error_code));
            }

            // push on queue for next processing step
            rx_to_mix_buffer_.enqueue(buffer);
            //mix_to_chan_buffer_.pushBack(element);
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Receive thread interrupted." << std::endl;
        usrp_rx->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    }
}



void multichannelrx_pfb::mixdown_thread(void)
{
    try {
        while (true) {
            boost::this_thread::interruption_point();

            ItemPtr item;
            rx_to_mix_buffer_.wait_dequeue(item);

            // do the hard work here
            mix_down(&item->data.front(), item->len);
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Down-mixing thread interrupted." << std::endl;
    }
}



void multichannelrx_pfb::channelizer_thread(void)
{
    try {
        while (true) {
            boost::this_thread::interruption_point();

            ItemPtr buffer;
            mix_to_chan_buffer_.wait_dequeue(buffer);

            // do the hard work here
            channelize(&buffer->data.front(), buffer->len);


        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Channelizer thread interrupted." << std::endl;
    }
}


void multichannelrx_pfb::synchronizer_thread(BlockingReaderWriterQueue<ItemPtr> &queue, const int channel_index)
{
    try {
        while (true) {
            boost::this_thread::interruption_point();

            ItemPtr item;
            queue.wait_dequeue(item);

            // do the hard work here, run OFDM sychronizer ..
            sychronize(&item->data.front(), item->len, channel_index);
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Synchronizer thread for channel " << channel_index << " interrupted." << std::endl;
    }
}

void multichannelrx_pfb::mix_down(std::complex<float> * _x, unsigned int _num_samples)
{
    ItemPtr buffer = buffer_factory_.get_new();
    int counter = 0;

    // buffer_index will be the channel number
    for (int i = 0; i < _num_samples; i++) {
        // mix signal down and put resulting sample into channelizer input buffer
        nco_crcf_mix_down(nco, _x[i], &buffer->data[counter * num_sampled_chans_ + buffer_index]);
        nco_crcf_step(nco);

        buffer_index++;
        if (buffer_index == num_sampled_chans_) {
            // reset index
            buffer_index = 0;
            counter++;
        }
    }
    // update len field
    buffer->len = counter;

    mix_to_chan_buffer_.enqueue(buffer);
}


void multichannelrx_pfb::channelize(std::complex<float> * _y, unsigned int counter)
{
    ItemPtr buffer = buffer_factory_.get_new();
    buffer->len = counter; // set len to previous len

    // execute filterbank channelizer as analyzer ..
    for (int i = 0; i < counter; i++) {
        firpfbch_crcf_analyzer_execute(channelizer, &_y[i * num_sampled_chans_ + 0], &buffer->data[i * num_sampled_chans_]);
    }

    // add sync buffer to queue of all channels
    for (auto &i : chan_to_sync_buffers_) {
        i.enqueue(buffer);
    }
}

void multichannelrx_pfb::sychronize(std::complex<float> * _x, const int len, const int channel_index)
{
    for (int i = 0; i < len; i++) {
        ofdmflexframesync_execute(framesync[channel_index], &_x[i * num_sampled_chans_ + channel_index], 1);
    }
}
