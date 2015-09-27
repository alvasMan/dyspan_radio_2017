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
typedef std::vector<unsigned char> CharVec;
typedef std::vector<std::complex<float>, aligned_allocator<__m128, sizeof(__m128)> > ACplxFVec;

#define NUM_PADDING_NULL_SAMPLES 100
#define MAX_PAYLOAD_LEN 2500

typedef struct
{
    // generic radio parameters
    bool debug;
    std::string mode;
    std::string args;
    std::string txsubdev;
    std::string rxsubdev;
    size_t num_trx;
    size_t num_channels;
    double f_center;
    double channel_bandwidth;
    double channel_rate;
    double rx_gain_uhd;
    double tx_gain_uhd;
    double tx_gain_soft;
    std::string rx_antenna;
    bool has_learning;

    // OFDM parameter
    unsigned int M;                     // number of subcarriers
    unsigned int cp_len;                // cyclic prefix length
    unsigned int taper_len;             // taper length
    unsigned char* p;                   // subcarrier allocation
    std::string fec0;
    std::string fec1;
    std::string crc;
    std::string mod;

    // challenge database parameters
    bool use_db;
    std::string db_ip;
    std::string db_user;
    std::string db_password;
} RadioParameter;


class DyspanRadio
{
public:
    DyspanRadio(const RadioParameter params) :
        radio_id_(-2),
        params_(params)
    {
        // validate input
        if (params.num_channels < 1) {
            throw std::runtime_error("error: must have at least one channel");
        } else if (params_.M < 8) {
            throw std::runtime_error("number of subcarriers must be at least 8");
        } else if (params_.cp_len < 1) {
            throw std::runtime_error("cyclic prefix length must be at least 1");
        } else if (params_.taper_len > params_.cp_len) {
            throw std::runtime_error("taper length cannot exceed cyclic prefix length");
        }

        // this is a special case for the 4x 5MHz receiver using two N210s
        if (params_.channel_bandwidth == 5e6 && params_.num_channels == 4 && params_.num_trx == 2) {
            // this initializes 4 channels such that the first two have the same rf_freq and the last two.
            // this makes sure that each N210 is tuned to two channels with the LO sitting between them
            std::cout << boost::str(boost::format("Configuring channels for 4x 5MHz using two N210s")) << std::endl;
            double offset = params_.channel_bandwidth / 2;
            double rf_freq = params_.f_center - params_.channel_bandwidth;
            channels_.push_back({"Channel0", params_.f_center + offset, params_.channel_bandwidth, rf_freq, +offset, params_.channel_rate});
            channels_.push_back({"Channel1", params_.f_center + offset, params_.channel_bandwidth, rf_freq, -offset, params_.channel_rate});

            rf_freq = params_.f_center + params_.channel_bandwidth;
            channels_.push_back({"Channel2", params_.f_center + offset, params_.channel_bandwidth, rf_freq, +offset, params_.channel_rate});
            channels_.push_back({"Channel3", params_.f_center + offset, params_.channel_bandwidth, rf_freq, -offset, params_.channel_rate});
        } else {
            // initialize channels, add two in each iteration
            for (int i = 0; i < params_.num_channels; i += 2) {
                double offset = i / 2 * params_.channel_bandwidth + params_.channel_bandwidth / 2;
                // add left neighbor
                channels_.push_back({"Channel" + std::to_string(i), params_.f_center + offset, params_.channel_bandwidth, params_.f_center, +offset, params_.channel_rate});
                // add right neighbor
                channels_.push_back({"Channel" + std::to_string(i + 1), params_.f_center + offset, params_.channel_bandwidth, params_.f_center, -offset, params_.channel_rate});
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
    RadioParameter params_;
    std::vector< ChannelConfig > channels_;

    // needed for the challenge database
    int radio_id_;
    char error_buffer[32];          // buffer to hold error messages from challenge server

    boost::ptr_vector<boost::thread> threads_;
};

#endif // DYPANRADIO_H

