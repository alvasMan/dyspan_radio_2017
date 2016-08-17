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

#ifndef MULTICHANNELRX_H
#define MULTICHANNELRX_H

// This class is based on liquid-usrp's multichannelrx class

#include "dyspanradio.h"
#include "Buffer.h"
#include "buffer_factory.h"
#include "readerwriterqueue.h"
#include <vector>
#include <map>
#include <liquid/liquid.h>

#define SKIP_MIXING 0

#define MULTITHREAD 1
#define THREAD_BUFFER_SIZE 20

using namespace moodycamel;

class multichannelrx_pfb : public DyspanRadio {
public:
    // default constructor
    multichannelrx_pfb(const RadioParameter params);

    // destructor
    ~multichannelrx_pfb();

    void start();
    void stop();

    // reset multi-channel receiver
    void Reset();

    int callback(unsigned char *  _header,
                 int              _header_valid,
                 unsigned char *  _payload,
                 unsigned int     _payload_len,
                 int              _payload_valid,
                 framesyncstats_s _stats,
                 void *           _userdata);

    // push samples into base station receiver


private:
    // ...
    void receive_thread();
    void mixdown_thread();
    void channelizer_thread();
    void synchronizer_thread(BlockingReaderWriterQueue<ItemPtr> &queue, const int channel_index);

    void mix_down(std::complex<float> * _x, unsigned int _num_samples);
    void channelize(std::complex<float> * _y, unsigned int counter);
    void sychronize(std::complex<float> * _x, const int len, const int channel_index);



    // finite impulse response polyphase filterbank channelizer
    firpfbch_crcf channelizer;      // channelizer size is 2*num_channels
    unsigned int buffer_index;      // input index

    size_t num_sampled_chans_;
    size_t max_spp_;

    // statistics
    uint32_t total_frames_;
    uint32_t lost_frames_;
    uint32_t last_seq_no_;

    BufferFactory<BufferItem> buffer_factory_;
    BlockingReaderWriterQueue<ItemPtr> rx_to_mix_buffer_;
    BlockingReaderWriterQueue<ItemPtr> mix_to_chan_buffer_;
    boost::ptr_vector<BlockingReaderWriterQueue<ItemPtr> > chan_to_sync_buffers_;

    // objects
    ofdmflexframesync * framesync;  // array of frame generator objects
    void ** userdata;               // array of userdata pointers
    framesync_callback * callbacks; // array of callback functions
    nco_crcf nco;                   // frequency-centering NCO

    uhd::usrp::multi_usrp::sptr usrp_rx;
};

#endif // MULTICHANNELRX_H

