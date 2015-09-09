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
#include "spectrum.h"

typedef std::vector<std::complex<float> > CplxFVec;
typedef std::vector<std::complex<float>, aligned_allocator<__m128, sizeof(__m128)> > ACplxFVec;

#define NUM_PADDING_NULL_SAMPLES 100
#define MAX_PAYLOAD_LEN 2500
#define CHALLENGE_DB_IP "127.0.0.1"

class DyspanRadio
{
public:
    DyspanRadio(const int num_channels,
                const size_t num_trx,
                const double f_center,
                const double channel_bandwidth,
                const double channel_rate,
                unsigned int M,
                unsigned int cp_len,
                unsigned int taper_len,
                bool debug,
                bool use_challenge_db) :
        radio_id_(-2),
        num_channels_(num_channels),
        num_trx_(num_trx),
        f_center_(f_center),
        channel_bandwidth_(channel_bandwidth),
        channel_rate_(channel_rate),
        M_(M),
        cp_len_(cp_len),
        taper_len_(taper_len),
        debug_(debug),
        use_challenge_db_(use_challenge_db)
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

        // this is a special case for the 4x 5MHz receiver using two N210s
        if (channel_bandwidth == 5e6 && num_channels_ == 4 && num_trx_ == 2) {
            // this initializes 4 channels such that the first two have the same rf_freq and the last two.
            // this makes sure that each N210 is tuned to two channels with the LO sitting between them
            std::cout << boost::str(boost::format("Configuring channels for 4x 5MHz using two N210s")) << std::endl;
            double offset = channel_bandwidth / 2;
            double rf_freq = f_center - channel_bandwidth;
            channels_.push_back({"Channel0", f_center_ + offset, channel_bandwidth_, rf_freq, +offset, channel_rate});
            channels_.push_back({"Channel1", f_center_ + offset, channel_bandwidth_, rf_freq, -offset, channel_rate});

            rf_freq = f_center + channel_bandwidth;
            channels_.push_back({"Channel2", f_center_ + offset, channel_bandwidth_, rf_freq, +offset, channel_rate});
            channels_.push_back({"Channel3", f_center_ + offset, channel_bandwidth_, rf_freq, -offset, channel_rate});
        } else {
            // initialize channels, add two in each iteration
            for (int i = 0; i < num_channels_; i += 2) {
                double offset = i / 2 * channel_bandwidth_ + channel_bandwidth_ / 2;
                // add left neighbor
                channels_.push_back({"Channel" + std::to_string(i), f_center_ + offset, channel_bandwidth_, f_center_, +offset, channel_rate});
                // add right neighbor
                channels_.push_back({"Channel" + std::to_string(i + 1), f_center + offset, channel_bandwidth, f_center, -offset, channel_rate});
            }
        }
        // TODO: check that all channels have the same center frequency for faster tuning
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

protected:
    // generic properties
    double f_center_;
    int num_channels_;
    size_t num_trx_;                   // number of transceivers (RF frontends)
    double channel_bandwidth_;
    double channel_rate_;
    std::vector< ChannelConfig > channels_;
    bool debug_;
    bool use_challenge_db_;
    int radio_id_;
    char error_buffer[32];          // buffer to hold error messages from challenge server

    // OFDM properties
    unsigned int M_;                 // number of subcarriers
    unsigned int cp_len_;            // cyclic prefix length
    unsigned int taper_len_;         // taper length

    boost::ptr_vector<boost::thread> threads_;
};

#endif // DYPANRADIO_H

