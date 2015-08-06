
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

static bool verbose;
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
        // compute true carrier offset
        printf("***** rssi=%7.2fdB evm=%7.2fdB, ", _stats.rssi, _stats.evm);

        if (_header_valid) {
            unsigned int packet_id = (_header[0] << 8 | _header[1]);
        unsigned int channel = _header[2];
        printf("channel: %u ", channel);
            printf("rx packet id: %6u", packet_id);
            if (_payload_valid)
        {
          printf("\n");
        }
            else                printf(" PAYLOAD INVALID\n");
        } else {
            printf("HEADER INVALID\n");
        }
    } else {
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
    // validate input
    if (num_channels < 1) {
        fprintf(stderr,"error: multichannelrx::multichannelrx(), must have at least one channel\n");
        throw 0;
    } else if (M < 8) {
        fprintf(stderr,"error: multichannelrx::multichannelrx(), number of subcarriers must be at least 8\n");
        throw 0;
    } else if (cp_len < 1) {
        fprintf(stderr,"error: multichannelrx::multichannelrx(), cyclic prefix length must be at least 1\n");
        throw 0;
    } else if (taper_len > cp_len) {
        fprintf(stderr,"error: multichannelrx::multichannelrx(), taper length cannot exceed cyclic prefix length\n");
        throw 0;
    }
    unsigned int i;

    // create callbacks
    // create multi-channel receiver object
    callbacks = (framesync_callback*) malloc(num_channels * sizeof(framesync_callback));
    //void * userdata[num_channels_];
    //framesync_callback callbacks[num_channels_];
    for (int i=0; i<num_channels_; i++) {
        //userdata[i] = NULL;
        callbacks[i] = callback;
    }

    // create frame generators
    framesync = (ofdmflexframesync*)  malloc(num_channels * sizeof(ofdmflexframesync));
    userdata  = (void **)             malloc(num_channels * sizeof(void *));
    callbacks = (framesync_callback*) malloc(num_channels * sizeof(framesync_callback));
    for (i=0; i<num_channels; i++) {
        //userdata[i]  = _userdata[i];
        //callback[i]  = _callback[i];
        framesync[i] = ofdmflexframesync_create(M_, cp_len_, taper_len_, p, callbacks[i], NULL);
#if BST_DEBUG
        ofdmflexframesync_debug_enable(framesync[i]);
#endif
    }

    // design custom filterbank channelizer
    unsigned int m  = 7;        // prototype filter delay
    float As        = 60.0f;    // stop-band attenuation
    channelizer = firpfbch_crcf_create_kaiser(LIQUID_ANALYZER, 2*num_channels, m, As);

    // channelizer input/output arrays
    X = (std::complex<float>*) malloc( 2 * num_channels * sizeof(std::complex<float>) );
    x = (std::complex<float>*) malloc( 2 * num_channels * sizeof(std::complex<float>) );

    // create NCO to center spectrum
    float offset = -0.5f*(float)(num_channels-1) / (float)num_channels * M_PI;
    nco = nco_crcf_create(LIQUID_VCO);
    nco_crcf_set_frequency(nco, offset);

    // reset base station transmitter
    Reset();


    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    usrp_rx = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp_rx->get_pp_string() << std::endl;

    double rx_rf_rate = num_channels * channel_bandwidth;

    usrp_rx->set_rx_rate(rx_rf_rate);
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

    // free other buffers
    free(X);
    free(x);
}

void multichannelrx::start(void)
{
    // start threads
    threads_.push_back( new boost::thread( boost::bind( &multichannelrx::receive_function, this ) ) );
}


// reset
void multichannelrx::Reset()
{
    // reset all objects
    unsigned int i;
    for (i=0; i<num_channels_; i++)
        ofdmflexframesync_reset(framesync[i]);

    firpfbch_crcf_reset(channelizer);

    //nco_crcf_reset(nco);

    for (i=0; i<2*num_channels_; i++) {
        X[i] = 0.0f;
        x[i] = 0.0f;
    }

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

    //meta-data will be filled in by recv()
    uhd::rx_metadata_t metadata;
    bool overflow_message = true;

    try {

        while (true) {
            boost::this_thread::interruption_point();


            //allocate recv buffer and metatdata
            uhd::rx_metadata_t metadata;
            const size_t max_samps_per_packet = usrp_rx->get_device()->get_max_recv_samps_per_packet();
            CplxFVec buff(max_samps_per_packet);


            size_t num_rx_samps = rx_stream->recv(&buff.front(), buff.size(), metadata, 3.0);
            //size_t num_rx_samps = rx_stream->recv(e_detec.fftBins, e_detec.nBins, metadata, 5.0);

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


        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Receive thread interrupted." << std::endl;
    }
}


void multichannelrx::Execute(std::complex<float> * _x,
                                  unsigned int          _num_samples)
{
    unsigned int i;
    for (i=0; i<_num_samples; i++) {
#if 1
        // mix signal down and put resulting sample into
        // channelizer input buffer
        nco_crcf_mix_down(nco, _x[i], &x[buffer_index]);
        nco_crcf_step(nco);

        // update buffer index and...
        buffer_index++;
        if (buffer_index == 2*num_channels_) {
            // reset index
            buffer_index = 0;

            // run...
            RunChannelizer();
        }
#else
        buffer_index++;
        if ( (buffer_index % (2*num_channels))==0 )
            ofdmflexframesync_execute(framesync[0], &_x[i], 1);

#endif
    }
}

// TODO: make this multi-threaded (each synchronizer runs in its own thread)
void multichannelrx::RunChannelizer()
{
    // execute filterbank channelizer as analyzer
    firpfbch_crcf_analyzer_execute(channelizer, x, X);

    // push resulting samples through frame synchronizers one
    // sample at a time
    unsigned int i;
    for (i=0; i<num_channels_; i++)
        ofdmflexframesync_execute(framesync[i], &X[i], 1);
}


