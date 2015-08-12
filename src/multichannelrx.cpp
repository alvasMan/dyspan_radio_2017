
//
// multichannelrx.cc
//

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <complex>
#include <vector>
#include <liquid/liquid.h>


#include "multichannelrx.h"

#define BST_DEBUG 0

static bool verbose = true;
// global callback function
int callback(unsigned char *  _header,
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
            std::cout << boost::format("seqno: %6u, ") % seq_no;
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
multichannelrx::multichannelrx(const std::string args,
               const int num_channels,
               const double f_center,
               const double channel_bandwidth,
               const double channel_rate,
               const float rx_gain_uhd,
               unsigned int    M,            // OFDM: number of subcarriers
               unsigned int    cp_len,       // OFDM: cyclic prefix length
               unsigned int    taper_len,    // OFDM: taper prefix length
               unsigned char * p) :          // OFDM: subcarrier allocation
    DyspanRadio(num_channels, f_center, channel_bandwidth, channel_rate, M, cp_len, taper_len)
{
    // create callbacks
    userdata  = (void **)             malloc(num_channels * sizeof(void *));
    callbacks = (framesync_callback*) malloc(num_channels * sizeof(framesync_callback));
    for (int i = 0; i < num_channels_; i++) {
        userdata[i] = NULL;
        callbacks[i] = callback;
    }

    // create frame generators
    framesync = (ofdmflexframesync*)  malloc(num_channels * sizeof(ofdmflexframesync));
    for (int i = 0; i < num_channels_; i++) {
        framesync[i] = ofdmflexframesync_create(M_, cp_len_, taper_len_, p, callbacks[i], userdata[i]);
#if BST_DEBUG
        ofdmflexframesync_debug_enable(framesync[i]);
#endif
    }

    // design custom filterbank channelizer
    unsigned int m  = 7;        // prototype filter delay
    float As        = 60.0f;    // stop-band attenuation
    channelizer = firpfbch_crcf_create_kaiser(LIQUID_ANALYZER, 2*num_channels, m, As);

    // create NCO to center spectrum
    float offset = -0.5f*(float)(num_channels-1) / (float)num_channels * M_PI;
    nco = nco_crcf_create(LIQUID_VCO);
    nco_crcf_set_frequency(nco, offset);

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    usrp_rx = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp_rx->get_pp_string() << std::endl;


    // channelizer input/output arrays
    const size_t max_spp = usrp_rx->get_device()->get_max_recv_samps_per_packet();
    num_sampled_chans_ = 2 * num_channels_; // oversampling ratio of 2.0
    //y.resize(max_spp * num_sampled_chans);
    Y.resize(max_spp * num_sampled_chans_);

    // assign memory to storage
    const size_t buffer_block_size = max_spp * num_sampled_chans_ * sizeof(std::complex<float>);
    v.resize(MAX_BUFFER_BLOCKS * buffer_block_size);
    storage.add_block(&v.front(), v.size(), buffer_block_size);

    // reset base station transmitter
    Reset();

    // computer actual RF rate
    double rx_rf_rate = num_channels * channel_bandwidth;
    usrp_rx->set_rx_rate(2.0f * rx_rf_rate); // try to set rx rate (oversampled to compensate for CIC filter)

#if 0
    double usrp_rx_rate = usrp->get_rx_rate();
    double rx_resamp_rate = rx_rate / usrp_rx_rate; // compute arbitrary resampling rate (make up the difference in software)
#endif

    // set up remaining parameters
    usrp_rx->set_rx_freq(f_center_);
    usrp_rx->set_rx_gain(rx_gain_uhd);
}

// destructor
multichannelrx::~multichannelrx()
{
    // destroy NCO
    nco_crcf_destroy(nco);

    // destroy channelizer
    firpfbch_crcf_destroy(channelizer);

    // destroy frame synchronizers
    unsigned int i;
    for (i=0; i<num_channels_; i++) {
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
}

void multichannelrx::start(void)
{
    // start threads
    threads_.push_back( new boost::thread( boost::bind( &multichannelrx::receive_function, this ) ) );
    threads_.push_back( new boost::thread( boost::bind( &multichannelrx::channelizer_function, this ) ) );
}


// reset
void multichannelrx::Reset()
{
    // reset all objects
    unsigned int i;
    for (i=0; i<num_channels_; i++)
        ofdmflexframesync_reset(framesync[i]);

    firpfbch_crcf_reset(channelizer);

    // reset write index of channelizer buffer
    buffer_index = 0;
}


void multichannelrx::receive_function(void)
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

    max_spp_ = usrp_rx->get_device()->get_max_recv_samps_per_packet();
    CplxFVec buff(max_spp_);

    //meta-data will be filled in by recv()
    uhd::rx_metadata_t metadata;
    bool overflow_message = true;

    try {

        while (true) {
            boost::this_thread::interruption_point();

            size_t num_rx_samps = rx_stream->recv(&buff.front(), buff.size(), metadata, 3.0);

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
            mix_down(&buff[0], num_rx_samps);
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Receive thread interrupted." << std::endl;
        usrp_rx->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    }
}




void multichannelrx::channelizer_function(void)
{
    try {
        while (true) {
            boost::this_thread::interruption_point();

            BufferElement y;
            frame_buffer.popFront(y);
            channelize(&y.buffer[0], y.len);

#if 1
            {
                boost::lock_guard<boost::mutex> lock{mutex};
                storage.free_n(y.buffer, 1, max_spp_ * num_sampled_chans_ * sizeof(std::complex<float>));
            }
#endif

        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Channelizer thread interrupted." << std::endl;
    }
}





void multichannelrx::mix_down(std::complex<float> * _x, unsigned int _num_samples)
{

    std::complex<float> *y;
    {
        boost::lock_guard<boost::mutex> lock{mutex};
        y = static_cast<std::complex<float>*>(storage.malloc_n(1, max_spp_ * num_sampled_chans_ * sizeof(std::complex<float>)));
        assert(y != nullptr);
    }
    int counter = 0;

    // buffer_index will be the channel number
    for (int i = 0; i < _num_samples; i++) {
        // mix signal down and put resulting sample into
        // channelizer input buffer
        nco_crcf_mix_down(nco, _x[i], &y[counter * num_sampled_chans_ + buffer_index]);
        nco_crcf_step(nco);

        buffer_index++;
        if (buffer_index == num_sampled_chans_) {
            // reset index
            buffer_index = 0;
            counter++;
        }
    }
    //std::cout << "counter: " << counter << std::endl;
    //std::cout << "buffer_index: " << buffer_index << std::endl;
    //assert(counter == 90);
    BufferElement elem;
    elem.buffer = y;
    elem.len = counter;

    frame_buffer.pushBack(elem);

    //channelize(&y[0], counter);
}


void multichannelrx::channelize(std::complex<float> * _y, unsigned int counter)
{
    // execute filterbank channelizer as analyzer ..
    for (int i = 0; i < counter; i++) {
        firpfbch_crcf_analyzer_execute(channelizer, &_y[i * num_sampled_chans_ + 0], &Y[i * num_sampled_chans_]);
    }
    sychronize(counter);
}

void multichannelrx::sychronize(unsigned int counter)
{
    // run OFDM sychronizer ..
    for (int i = 0; i < counter; i++) {
        for (int k = 0; k < num_channels_; k++) {
            ofdmflexframesync_execute(framesync[k], &Y[i * num_sampled_chans_ + k], 1);
        }
    }
}
