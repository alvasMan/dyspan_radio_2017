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

    evm_(_stats.evm);

    if (params_.debug)
        std::cout << boost::format("***** CH %d rssi=%7.2fdB evm=%7.2fdB, ") % data->channel % _stats.rssi % _stats.evm;
    if (_header_valid) {
        uint32_t seq_no = (_header[0] << 24 | _header[1] << 16 | _header[2] << 8 | _header[3]);
        if (params_.debug)
            std::cout << boost::format("seqno: %6u (%6u lost), ") % seq_no % lost_frames_;

        if (_payload_valid) {
            if (params_.debug)
                std::cout << boost::format("payload size: %d") % _payload_len << std::endl;

            // put frame on internal buffer
            if (params_.use_db) {
                boost::shared_ptr<CharVec> frame( new CharVec(_payload_len) );
                memcpy(&frame->front(), _payload, _payload_len);
                rx_buffer_.pushBack(frame);
            }

            // do the stats ..
            boost::lock_guard<boost::mutex> lock(mutex_);
            if (last_seq_no_ == 0) {
                std::cout << boost::format("Setting first seqno: %6u") % seq_no << std::endl;
                last_seq_no_ = seq_no;
            } else {
                // count lost frames
                lost_frames_ += (seq_no - last_seq_no_ - 1);
                last_seq_no_ = seq_no;
            }
            rx_frames_++;
        } else {
            if (params_.debug)
                std::cout << boost::format("PAYLOAD INVALID") << std::endl;;
        }
    } else {
        if (params_.debug)
            std::cout << boost::format("HEADER INVALID") << std::endl;;
    }
    return 0;
}




// default constructor
multichannelrx::multichannelrx(const RadioParameter params) :
    DyspanRadio(params),
    rx_frames_(0),
    last_seq_no_(0),
    lost_frames_(0),
    rx_(NULL),
    rx_buffer_(500)
{
    // create callbacks
    userdata  = (void **)             malloc(params_.num_channels * sizeof(void *));
    callbacks = (framesync_callback*) malloc(params_.num_channels * sizeof(framesync_callback));

    for (int i = 0; i < params_.num_channels; i++) {
        // allocate memory for custom data as well
        CustomUserdata* data = (CustomUserdata*) malloc(sizeof(CustomUserdata));
        data->callback = this;
        data->channel = i;
        userdata[i] = data;
        callbacks[i] = multichannelrxdetail::gCallback;
    }

    // create frame synchronizers
    framesync = (ofdmflexframesync*)  malloc(params_.num_channels * sizeof(ofdmflexframesync));
    for (int i = 0; i < params_.num_channels; i++) {
        framesync[i] = ofdmflexframesync_create(params_.M, params_.cp_len, params_.taper_len, params_.p, callbacks[i], userdata[i]);
#if BST_DEBUG
        ofdmflexframesync_debug_enable(framesync[i]);
#endif
    }

    Reset();

    if (params_.use_db) {
        // create and connect to challenge database
        rx_ = spectrum_init(0);
        spectrum_eror_t ret = spectrum_connect(rx_, (char*)params_.db_ip.c_str(), 5003, 0, 0);
        spectrum_errorToText(rx_, ret, error_buffer, sizeof(error_buffer));
        std::cout << boost::format("RX connect: %s") % error_buffer << std::endl;
        if (ret < 0) {
            throw std::runtime_error("Couldn't connect to challenge database");
        }

        // get radio number
        radio_id_ = spectrum_getRadioNumber(rx_);
        std::cout << boost::format("RX radio number: %d") % radio_id_ << std::endl;

        // wait for the start of stage 3 (here you get penalized for interference).
        // The testing database starts in this state so this will instantly return.
        if(params_.new_db==false)
            spectrum_waitForState(rx_, 3, -1);
        else
            spectrum_waitForState(rx_,params_.phase_num,-1);
        std::cout << boost::format("Stage 3 has started.") << std::endl;
    }

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % params_.args << std::endl;
    usrp_rx = uhd::usrp::multi_usrp::make(params_.args);

    // always select the subdevice first, the channel mapping affects the other settings
    if (not params_.rxsubdev.empty()) {
        usrp_rx->set_rx_subdev_spec(params_.rxsubdev); //sets across all mboards
    }

    std::cout << boost::format("Using Device: %s") % usrp_rx->get_pp_string() << std::endl;

    // sanity checks
    assert(params_.num_channels == channels_.size());

    std::cout << boost::format("This setup supports %d channels.") % usrp_rx->get_rx_num_channels() << std::endl;
    if (params_.num_channels > usrp_rx->get_rx_num_channels()) {
        throw std::runtime_error("Invalid channel(s) specified.");
    }

    // first perform all configurations that apply across all channels
    // set the rx sample rate
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (params_.channel_bandwidth/1e6) << std::endl;
    usrp_rx->set_rx_rate(params_.channel_bandwidth);
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
        usrp_rx->set_rx_gain(params_.rx_gain_uhd, 0);
        usrp_rx->set_rx_gain(params_.rx_gain_uhd, 2);
    } else {
        usrp_rx->set_time_now(uhd::time_spec_t(0.0));
        usrp_rx->set_rx_gain(params_.rx_gain_uhd);
    }

    // set antenna port for each channel
    for (int i = 0; i < channels_.size(); i++) {
        usrp_rx->set_rx_antenna(params_.rx_antenna, i);
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
    for (int i = 0; i < params_.num_channels; i++) {
#if BST_DEBUG
        char filename[64];
        sprintf(filename,"framesync_channel%u.m", i);
        ofdmflexframesync_debug_print(framesync[i], filename);
#endif
        ofdmflexframesync_destroy(framesync[i]);
    }
    free(framesync);
    for (int i = 0; i < params_.num_channels; i++)
        free(userdata[i]);
    free(userdata);
    free(callbacks);

    // disconnect and destroy handle to challenge DB
    if (rx_)
        spectrum_delete(rx_);
}

void multichannelrx::start(void)
{
    // start threads
    threads_.push_back( new boost::thread( boost::bind( &multichannelrx::receive_thread, this ) ) );
    threads_.push_back( new boost::thread( boost::bind( &multichannelrx::frame_delivery_thread, this ) ) );

    // start a synchronizer thread for each channel
    for (int i = 0; i < sync_queue_.size(); i++) {
        threads_.push_back( new boost::thread( boost::bind( &multichannelrx::synchronizer_thread, this, boost::ref(sync_queue_[i]), i) ) );
    }

    threads_.push_back( new boost::thread( boost::bind( &multichannelrx::statistic_thread, this ) ) );
}


// reset
void multichannelrx::Reset()
{
    // reset all objects
    for (int i = 0; i < params_.num_channels; i++)
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
            for (int i = 0; i < params_.num_channels; i++) {
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

#if 0
            if (debug_) std::cout << boost::format(
                "Received packet: %u samples, %u full secs, %f frac secs"
            ) % num_rx_samps % md.time_spec.get_full_secs() % md.time_spec.get_frac_secs() << std::endl;
#endif

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


void multichannelrx::statistic_thread(void)
{
    try {
        while (true) {
            boost::this_thread::interruption_point();
            {
                boost::lock_guard<boost::mutex> lock(mutex_);

                uint32_t total_frames = rx_frames_ + lost_frames_;
                float fer = 0.0;
                if (total_frames > 0) {
                    fer = static_cast<float>(lost_frames_) / static_cast<float>(total_frames);
                    fer *= 100;
                }
                std::cout << boost::str(boost::format("RX: %6d Lost: %6d FER: %.2f%% EVM: %.2f%%") % rx_frames_ % lost_frames_ % fer % mean(evm_)) << std::endl;

                rx_frames_ = 0;
                lost_frames_ = 0;
                evm_ = acc_mean();
            }

            if (params_.use_db) {
                std::cout << boost::format("PU: %.02f bps / %.02f bps") %
                             spectrum_getThroughput(rx_, 0, 10) %
                             spectrum_getProvidedThroughput(rx_, 0, 10) << std::endl;

                double throughput = spectrum_getThroughput(rx_, radio_id_, 10) / 1e6;
                double provided = spectrum_getProvidedThroughput(rx_, radio_id_, 10) / 1e6;
                double ratio = 100 * throughput / provided;
                std::cout << boost::format("SU: %.02f Mbps / %.02f Mbps (%.02f%%)") %
                             throughput %
                             provided %
                             ratio << std::endl;
            }

            boost::this_thread::sleep(boost::posix_time::seconds(params_.stat_interval));
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Statistic thread for channel interrupted." << std::endl;
    }
}


void multichannelrx::frame_delivery_thread(void)
{
    try {
        while (true) {
            boost::this_thread::interruption_point();

            boost::shared_ptr<CharVec> frame;
            rx_buffer_.popFront(frame);

            //std::cout << boost::format("rx_buffer_: %d/%d") % rx_buffer_.size() % rx_buffer_.capacity() << std::endl;

            // pass received frame to challenge DB
            assert(frame->size() <= 1500); // maximum size of the client libarary
            spectrum_eror_t ret = spectrum_putPacket(rx_, &frame->front(), frame->size());
            spectrum_errorToText(rx_, ret, error_buffer, sizeof(error_buffer));
            if (ret < 0) {
                std::cout << boost::format("RX putPacket: %s") % error_buffer << std::endl;
                throw std::runtime_error("Couldn't connect to challenge database");
            }
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Frame delivery thread for channel interrupted." << std::endl;
    }
}

inline void multichannelrx::sychronize(std::complex<float> * _x, const int len, const int channel_index)
{
    ofdmflexframesync_execute(framesync[channel_index], _x, len);
}
