#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <complex>
#include <vector>
#include <liquid/liquid.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include "multichannelrx.h"

#define BST_DEBUG 0

static bool verbose = true;

// global callback function
namespace multichannelrxdetail
{
int gCallback( unsigned char *  _header,
               int              _header_valid,
               unsigned char *  _payload,
               unsigned int     _payload_len,
               int              _payload_valid,
               framesyncstats_s _stats,
               void *           _userdata)
{
  static_cast<multichannelrx*>(_userdata)->callback(_header,
     _header_valid,
     _payload,
     _payload_len,
     _payload_valid,
     _stats,
     _userdata);
}
}

int multichannelrx::callback(unsigned char *  _header,
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
multichannelrx::multichannelrx(const std::string args,
               const std::string subdev,
               const int num_channels,
               const double f_center,
               const double channel_bandwidth,
               const double channel_rate,
               const float rx_gain_uhd,
               unsigned int    M,            // OFDM: number of subcarriers
               unsigned int    cp_len,       // OFDM: cyclic prefix length
               unsigned int    taper_len,    // OFDM: taper prefix length
               unsigned char * p,            // OFDM: subcarrier allocation
               bool debug) :
    DyspanRadio(num_channels, f_center, channel_bandwidth, channel_rate, M, cp_len, taper_len, debug),
    total_frames_(0),
    last_seq_no_(0),
    lost_frames_(0)
{
    // create callbacks
    userdata  = (void **)             malloc(num_sampled_chans_ * sizeof(void *));
    callbacks = (framesync_callback*) malloc(num_sampled_chans_ * sizeof(framesync_callback));
    for (int i = 0; i < num_channels_; i++) {
        callbacks[i] = multichannelrxdetail::gCallback;
        userdata[i] = this;
    }

    // create frame synchronizers
    framesync = (ofdmflexframesync*)  malloc(num_sampled_chans_ * sizeof(ofdmflexframesync));
    for (int i = 0; i < num_channels_; i++) {
        framesync[i] = ofdmflexframesync_create(M_, cp_len_, taper_len_, p, callbacks[i], userdata[i]);
#if BST_DEBUG
        ofdmflexframesync_debug_enable(framesync[i]);
#endif
    }

    Reset();

    std::string channel_list("0,1");

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    usrp_rx = uhd::usrp::multi_usrp::make(args);

    //always select the subdevice first, the channel mapping affects the other settings
    if (subdev != "")
        usrp_rx->set_rx_subdev_spec(subdev); //sets across all mboards

    std::cout << boost::format("Using Device: %s") % usrp_rx->get_pp_string() << std::endl;

    //set the rx sample rate (sets across all channels)
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (channel_bandwidth/1e6) << std::endl;
    usrp_rx->set_rx_rate(channel_bandwidth);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp_rx->get_rx_rate()/1e6) << std::endl << std::endl;

    // set the RX gain
    usrp_rx->set_rx_gain(rx_gain_uhd);

    // set LO to center frequency
    uhd::tune_result_t result = usrp_rx->set_rx_freq(f_center_);
    if (debug_) {
        std::cout << result.to_pp_string() << std::endl;
    }

    // configure DSP channels
    for (int i = 0; i < channels_.size(); i++) {
        // construct tuning request
        uhd::tune_request_t request;
        // don't touch RF part
        request.rf_freq_policy = uhd::tune_request_t::POLICY_NONE;
        request.rf_freq = 0;
        // only tune DSP frequency
        request.dsp_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
        request.dsp_freq = channels_.at(i).dsp_freq;
        uhd::tune_result_t result = usrp_rx->set_rx_freq(request, i);
        std::cout << result.to_pp_string() << std::endl;
    }

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    usrp_rx->set_time_now(uhd::time_spec_t(0.0));

    //detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        size_t chan = boost::lexical_cast<int>(channel_strings[ch]);
        if(chan >= usrp_rx->get_rx_num_channels()){
            throw std::runtime_error("Invalid channel(s) specified.");
        }
        else channel_nums.push_back(boost::lexical_cast<int>(channel_strings[ch]));
    }

    //create a receive streamer
    uhd::stream_args_t stream_args(CPU_FORMAT);
    stream_args.channels = channel_nums;
    rx_streamer_ = usrp_rx->get_rx_stream(stream_args);

    // create neccesary buffer objects
    for (int i = 0; i < num_channels_; i++) {
        sync_queue_.push_back(new Buffer<ItemPtr>(THREAD_BUFFER_SIZE));
    }
}

// destructor
multichannelrx::~multichannelrx()
{
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

    std::cout << "Total frames received: " << total_frames_ << std::endl;
    std::cout << "Lost frames: " << lost_frames_ << std::endl;
    if (total_frames_ > 0) {
        float fer = static_cast<float>(lost_frames_) / static_cast<float>(total_frames_);
        std::cout << boost::str(boost::format("Frame error rate: %.2f") % fer) << std::endl;
    }
}

void multichannelrx::start(void)
{
    // start threads
    threads_.push_back( new boost::thread( boost::bind( &multichannelrx::receive_thread, this ) ) );

    // start a synchronizer thread for each channel
    for (int i = 0; i < sync_queue_.size(); i++) {
        threads_.push_back( new boost::thread( boost::bind( &multichannelrx::synchronizer_thread, this, boost::ref(sync_queue_[i]), i) ) );
    }
}


// reset
void multichannelrx::Reset()
{
    // reset all objects
    unsigned int i;
    for (i=0; i<num_channels_; i++)
        ofdmflexframesync_reset(framesync[i]);
}


void multichannelrx::receive_thread()
{
    //uhd::set_thread_priority_safe();
    double seconds_in_future = 2.0;

    //setup streaming
    std::cout << boost::format("Using a sample rate %f Msps on %u channels")
                 % (usrp_rx->get_rx_rate()/1e6) % rx_streamer_->get_num_channels() << std::endl;
    std::cout << boost::format("Begin streaming in %f seconds") % seconds_in_future << std::endl;

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = 0;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
    rx_streamer_->issue_stream_cmd(stream_cmd); //tells all channels to stream

    //meta-data will be filled in by recv()
    uhd::rx_metadata_t md;
    bool overflow_message = true;

    //allocate buffers to receive with samples (one buffer per channel)
    const size_t samps_per_buff = rx_streamer_->get_max_num_samps();

    //the first call to recv() will block this many seconds before receiving
    double timeout = seconds_in_future + 0.1; //timeout (delay before receive + padding)

    try {
        while (true) {
            boost::this_thread::interruption_point();

            // allocate fresh buffers to receive (one buffer per channel)
            std::vector<ItemPtr> buffs;
            std::vector<std::complex<float> *> buff_ptrs;
            for (int i = 0; i < num_channels_; i++) {
                ItemPtr item = buffer_factory_.get_new();
                buffs.push_back(item);
                buff_ptrs.push_back(&item->data.front());
            }

            //receive a single packet
            size_t num_rx_samps = rx_streamer_->recv(
                buff_ptrs, samps_per_buff, md, timeout
            );

            // update number of received samples
            for (int i = 0; i < buffs.size(); i++) {
                buffs[i]->len = num_rx_samps;
            }

            // use a small timeout for subsequent packets
            timeout = 0.1;

            // handle the error code
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                std::cout << boost::format("Timeout while streaming") << std::endl;
                continue;
            }
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
                if (overflow_message){
                    overflow_message = false;
                    std::cerr << boost::format("Got an overflow indication, please reduce sample rate.");
                }
                continue;
            }
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
                throw std::runtime_error(str(boost::format(
                                                 "Unexpected error code 0x%x"
                                                 ) % md.error_code));
            }

            if (debug_) std::cout << boost::format(
                "Received packet: %u samples, %u full secs, %f frac secs"
            ) % num_rx_samps % md.time_spec.get_full_secs() % md.time_spec.get_frac_secs() << std::endl;


            // push buffer for each channel on the synchronizer thread's queue
            assert(sync_queue_.size() == buffs.size());
            for (int i = 0; i < buffs.size(); i++) {
                sync_queue_.at(i).pushBack(buffs[i]);
            }
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Receive thread interrupted." << std::endl;
        usrp_rx->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    }
}

void multichannelrx::synchronizer_thread(Buffer<ItemPtr> &queue, const int channel_index)
{
    try {
        while (true) {
            boost::this_thread::interruption_point();

            ItemPtr item;
            queue.popFront(item);

            // do the hard work here, run OFDM sychronizer ..
            sychronize(&item->data.front(), item->len, channel_index);
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Synchronizer thread for channel " << channel_index << " interrupted." << std::endl;
    }
}

inline void multichannelrx::sychronize(std::complex<float> * _x, const int len, const int channel_index)
{
    ofdmflexframesync_execute(framesync[channel_index], _x, len);
}
