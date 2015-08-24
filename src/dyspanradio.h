#ifndef DYPANRADIO_H
#define DYPANRADIO_H

#include <boost/thread.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include "aligned_allocator.h"
#include "channels.h"

typedef std::vector<std::complex<float> > CplxFVec;
typedef std::vector<std::complex<float>, aligned_allocator<__m128, sizeof(__m128)> > ACplxFVec;

class DyspanRadio
{
public:
    DyspanRadio(const int num_channels,
                const double f_center,
                const double channel_bandwidth,
                const double channel_rate,
                unsigned int M,
                unsigned int cp_len,
                unsigned int taper_len,
                bool debug) :
        num_channels_(num_channels),
        f_center_(f_center),
        channel_bandwidth_(channel_bandwidth),
        channel_rate_(channel_rate),
        M_(M),
        cp_len_(cp_len),
        taper_len_(taper_len),
        debug_(debug)
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

        // initialize channels, add two in each iteration
        assert(num_channels_ % 2 == 0);
        for (int i = 0; i < num_channels_; i += 2) {
            double offset = i / 2 * channel_bandwidth_ + channel_bandwidth_ / 2;
            // add left neighbor
            channels_.push_back({"Channel" + std::to_string(i), f_center_ + offset, channel_bandwidth_, f_center_, +offset, channel_rate});
            // add right neighbor
            channels_.push_back({"Channel" + std::to_string(i + 1), f_center + offset, channel_bandwidth, f_center, -offset, channel_rate});
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
    double channel_bandwidth_;
    double channel_rate_;
    std::vector< ChannelConfig > channels_;
    bool debug_;

    // OFDM properties
    unsigned int M_;                 // number of subcarriers
    unsigned int cp_len_;            // cyclic prefix length
    unsigned int taper_len_;         // taper length

    boost::ptr_vector<boost::thread> threads_;

};

#endif // DYPANRADIO_H

