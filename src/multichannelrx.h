#ifndef OFDMRECEIVER_H
#define OFDMRECEIVER_H

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

#define CPU_FORMAT "fc32"

typedef struct {
    void* callback;
    int channel;
} CustomUserdata;

using namespace moodycamel;

class multichannelrx : public DyspanRadio {
public:
    // default constructor
    multichannelrx(const std::string args,
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
                   bool debug);

    // destructor
    ~multichannelrx();

    void start();
    void stop();

    // reset multi-channel receiver
    void Reset();

    // accessor methods
    unsigned int GetNumChannels() { return num_channels_; }

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
    void synchronizer_thread(Buffer<ItemPtr> &queue, const int channel_index);
    void sychronize(std::complex<float> * _x, const int len, const int channel_index);

    // statistics
    uint32_t rx_frames_;
    uint32_t lost_frames_;
    uint32_t last_seq_no_;

    BufferFactory<BufferItem> buffer_factory_;
    boost::ptr_vector<Buffer<ItemPtr> > sync_queue_;

    // objects
    ofdmflexframesync * framesync;  // array of frame generator objects
    void ** userdata;               // array of userdata pointers
    framesync_callback * callbacks; // array of callback functions

    uhd::usrp::multi_usrp::sptr usrp_rx;
    uhd::rx_streamer::sptr rx_streamer_;
};

#endif // OFDMRECEIVER_H
