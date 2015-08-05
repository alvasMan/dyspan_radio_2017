#ifndef OFDMTRANSCEIVER_H
#define OFDMTRANSCEIVER_H

// This class is based on liquid-usrp's ofdmtxrx class.

#include <complex>
#include <boost/thread.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/ptr_container/ptr_deque.hpp>
#include <liquid/liquid.h>
#include <uhd/usrp/multi_usrp.hpp>
#include "Buffer.h"
#include "channels.h"
#include "EnergyDetector.h"

typedef std::vector<std::complex<float> > CplxFVec;

class OfdmTransceiver
{
public:
    // default constructor   
    OfdmTransceiver(const std::string args,
                    const int num_channels,
                    const double f_center,
                    const double channel_bandwidth,
                    const double channel_rate,
                    const float tx_gain_soft,
                    const float tx_gain_uhd,
                    const float rx_gain_uhd);

    // destructor
    ~OfdmTransceiver();

    //
    // transmitter methods
    //
    void set_tx_freq(float _tx_freq);
    void set_tx_rate(float _tx_rate);
    void set_tx_gain_soft(float gain);
    void set_tx_gain_uhd(float gain);
    void set_tx_antenna(char * _tx_antenna);

    void run();
    void stop();
    void reset_tx();

    // update payload data on a particular channel
    void transmit_packet();

    //
    // receiver methods
    //
    void set_rx_freq(float _rx_freq);
    void set_rx_rate(float _rx_rate);
    void set_rx_gain_uhd(float _rx_gain_uhd);
    void set_rx_antenna(char * _rx_antenna);
    void reset_rx();
    void start_rx();
    void stop_rx();

    //
    // additional methods
    //
    void debug_enable();
    void debug_disable();

    // specify rx worker method as friend function so that it may
    // gain acess to private members of the class
    friend void * ofdmtxrx_rx_worker(void * _arg);

private:
    // generic properties
    int num_channels_;
    double channel_bandwidth_;
    double channel_rate_;
    std::vector< ChannelConfig > channels_;
    boost::ptr_vector<boost::thread> threads_;
    bool debug_;             // is debugging enabled?

    // OFDM properties
    unsigned int M;                 // number of subcarriers
    unsigned int cp_len;            // cyclic prefix length
    unsigned int taper_len;         // taper length
    ofdmflexframegenprops_s fgprops;// frame generator properties

    // transmitter objects
    ofdmflexframegen fg;            // frame generator object
    std::complex<float> * fgbuffer; // frame generator output buffer [size: M + cp_len x 1]
    //boost::scoped_ptr<CplxFVec> fgbuffer; // TODO: Convert to smart ptr

    unsigned int fgbuffer_len;      // length of frame generator buffer
    float tx_gain;                  // soft transmit gain (linear)
    uint32_t seq_no_;

    //boost::ptr_deque<CplxFVec> frame_buffer;
    Buffer<boost::shared_ptr<CplxFVec> > frame_buffer;

    // receiver objects
    EnergyDetector e_detec;
    uhd::time_spec_t timestamp_;

    // member functions
    void transmit_function();
    void random_transmit_function(); // this is just a test function which randomly transmits on every available channel
    void modulation_function();
    void receive_function();
    void reconfigure_usrp(const int num);

    // RF objects and properties
    uhd::usrp::multi_usrp::sptr usrp_tx;
    uhd::usrp::multi_usrp::sptr usrp_rx;
    uhd::tx_metadata_t          metadata_tx;
};

#endif // OFDMTRANSCEIVER_H
