#ifndef OFDMRECEIVER_H
#define OFDMRECEIVER_H

#include "dyspanradio.h"
#include "Buffer.h"
#include "buffer_factory.h"
#include "readerwriterqueue.h"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <vector>
#include <map>
#include <liquid/liquid.h>
#include "spectrum.h"

#define SKIP_MIXING 0

#define MULTITHREAD 1
#define THREAD_BUFFER_SIZE 20
#define STAT_INTERVAL 2

#define CPU_FORMAT "fc32"

typedef struct {
    void* callback;
    int channel;
} CustomUserdata;

using namespace moodycamel;
using namespace  boost::accumulators;

typedef accumulator_set<float, stats<tag::mean> > acc_mean;

class multichannelrx : public DyspanRadio {
public:
    // default constructor
    multichannelrx(const RadioParameter params);

    // destructor
    ~multichannelrx();

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

private:
    // ...
    void receive_thread();
    void statistic_thread();
    void frame_delivery_thread();
    void synchronizer_thread(Buffer<ItemPtr> &queue, const int channel_index);
    void sychronize(std::complex<float> * _x, const int len, const int channel_index);

    // statistics
    uint32_t rx_frames_;
    uint32_t lost_frames_;
    uint32_t last_seq_no_;
    acc_mean evm_;
    boost::mutex mutex_;

    BufferFactory<BufferItem> buffer_factory_;
    boost::ptr_vector<Buffer<ItemPtr> > sync_queue_;
    Buffer<boost::shared_ptr<CharVec> > rx_buffer_;

    // objects
    ofdmflexframesync * framesync;  // array of frame generator objects
    void ** userdata;               // array of userdata pointers
    framesync_callback * callbacks; // array of callback functions
    spectrum* rx_;                  // handle to the challenge database

    uhd::usrp::multi_usrp::sptr usrp_rx;
    uhd::rx_streamer::sptr rx_streamer_;
};

#endif // OFDMRECEIVER_H
