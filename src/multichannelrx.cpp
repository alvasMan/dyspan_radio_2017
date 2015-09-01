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
  static_cast<multichannelrx*>(static_cast<CustomUserdata*>(_userdata)->callback)->callback(_header,
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
    CustomUserdata *data = static_cast<CustomUserdata*>(_userdata);

    if (verbose) {
        std::cout << boost::format("***** CH %d rssi=%7.2fdB evm=%7.2fdB, ") % data->channel % _stats.rssi % _stats.evm;
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
            rx_frames_++;

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
    rx_frames_(0),
    last_seq_no_(0),
    lost_frames_(0)
{
    // create callbacks
    userdata  = (void **)             malloc(num_channels_ * sizeof(void *));
    callbacks = (framesync_callback*) malloc(num_channels_ * sizeof(framesync_callback));

    for (int i = 0; i < num_channels_; i++) {
        // allocate memory for custom data as well
        CustomUserdata* data = (CustomUserdata*) malloc(sizeof(CustomUserdata));
        data->callback = this;
        data->channel = i;
        userdata[i] = data;
        callbacks[i] = multichannelrxdetail::gCallback;
    }

    // create frame synchronizers
    framesync = (ofdmflexframesync*)  malloc(num_channels_ * sizeof(ofdmflexframesync));
    for (int i = 0; i < num_channels_; i++) {
        framesync[i] = ofdmflexframesync_create(M_, cp_len_, taper_len_, p, callbacks[i], userdata[i]);
#if BST_DEBUG
        ofdmflexframesync_debug_enable(framesync[i]);
#endif
    }

    Reset();

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    usrp_rx = uhd::usrp::multi_usrp::make(args);

    // always select the subdevice first, the channel mapping affects the other settings
    if (not subdev.empty()) {
        usrp_rx->set_rx_subdev_spec(subdev); //sets across all mboards
    }

    std::cout << boost::format("Using Device: %s") % usrp_rx->get_pp_string() << std::endl;

    // sanity checks
    assert(num_channels_ == channels_.size());
    if (num_channels_ > usrp_rx->get_rx_num_channels()) {
        throw std::runtime_error("Invalid channel(s) specified.");
    }

    // first perform all configurations that apply across all channels
    // set the rx sample rate
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (channel_bandwidth/1e6) << std::endl;
    usrp_rx->set_rx_rate(channel_bandwidth);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp_rx->get_rx_rate()/1e6) << std::endl << std::endl;

    // check for motherboards and set sync mode to mimo if 2 USRPs are found
    if (usrp_rx->get_num_mboards() == 2) {
        std::cout << boost::format("Configure receiver in MIMO mode...") << std::endl;

        //make mboard 1 a slave over the MIMO Cable
        usrp_rx->set_clock_source("mimo", 1);
        usrp_rx->set_time_source("mimo", 1);

        //set time on the master (mboard 0)
        usrp_rx->set_time_now(uhd::time_spec_t(0.0), 0);

        //sleep a bit while the slave locks its time to the master
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));

        // set the RX gain for on first and third channel (for first and second USRP)
        usrp_rx->set_rx_gain(rx_gain_uhd, 0);
        usrp_rx->set_rx_gain(rx_gain_uhd, 2);
    } else {
        usrp_rx->set_time_now(uhd::time_spec_t(0.0));
        usrp_rx->set_rx_gain(rx_gain_uhd);
    }

    // set antenna port for each channel
    // TODO: this needs to be changed for B210 or other daughterboard, make it a parameter?
    const std::string antenna("J1");
    for (int i = 0; i < channels_.size(); i++) {
        usrp_rx->set_rx_antenna(antenna, i);
        std::cout << boost::format("Using RX antenna %s on channel %d") % usrp_rx->get_rx_antenna(i) % i << std::endl;
    }

    // configure DSP channels
    std::vector<size_t> channel_nums;
    for (int i = 0; i < channels_.size(); i++) {
        // construct tuning request
        uhd::tune_request_t request;
        // don't touch RF part for every second channel (we have two DSP channels in each FPGA)
        request.rf_freq_policy = (i % 2 == 0) ? uhd::tune_request_t::POLICY_MANUAL : uhd::tune_request_t::POLICY_NONE;
        request.rf_freq = channels_.at(i).rf_freq;
        // only tune DSP frequency
        request.dsp_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
        request.dsp_freq = channels_.at(i).dsp_freq;
        uhd::tune_result_t result = usrp_rx->set_rx_freq(request, i);
        std::cout << result.to_pp_string() << std::endl;

        // add channel to channel map
        channel_nums.push_back(boost::lexical_cast<int>(i));

        // create neccesary buffer objects
        sync_queue_.push_back(new Buffer<ItemPtr>(THREAD_BUFFER_SIZE));
    }

    // finally, create a receive streamer
    uhd::stream_args_t stream_args(CPU_FORMAT);
    stream_args.channels = channel_nums;
    rx_streamer_ = usrp_rx->get_rx_stream(stream_args);
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
    for (int i = 0; i < num_channels_; i++)
        free(userdata[i]);
    free(userdata);
    free(callbacks);

    uint32_t total_frames = rx_frames_ + lost_frames_;
    std::cout << "Received frames: " << rx_frames_ << std::endl;
    std::cout << "Lost frames: " << lost_frames_ << std::endl;
    if (total_frames > 0) {
        float fer = static_cast<float>(lost_frames_) / static_cast<float>(total_frames);
        fer *= 100;
        std::cout << boost::str(boost::format("Frame error rate: %.2f%%") % fer) << std::endl;
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

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
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
                break;
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
