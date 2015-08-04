#ifndef OFDMTRANSCEIVER_H
#define OFDMTRANSCEIVER_H

// This class is based on liquid-usrp's ofdmtxrx class.

#include <complex>
#include <boost/thread.hpp>
#include <liquid/liquid.h>
#include <uhd/usrp/multi_usrp.hpp>
#include "Buffer.h"
#include "channels.h"

typedef std::vector<std::complex<float> > CplxFVec;

class OfdmTransceiver
{
public:
    // default constructor
    OfdmTransceiver(const std::string args,
                    const double freq,
                    const double rate,
                    const float tx_gain_soft,
                    const float tx_gain_uhd);

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
    // set timespec for timeout
    //  _ts         :   pointer to timespec structure
    //  _timeout    :   time before timeout
    void set_timespec(struct timespec * _ts,
                      float             _timeout);

    bool verbose;

    // OFDM properties
    unsigned int M;                 // number of subcarriers
    unsigned int cp_len;            // cyclic prefix length
    unsigned int taper_len;         // taper length
    ofdmflexframegenprops_s fgprops;// frame generator properties

    // transmitter objects
    ofdmflexframegen fg;            // frame generator object
    std::complex<float> * fgbuffer; // frame generator output buffer [size: M + cp_len x 1]
    unsigned int fgbuffer_len;      // length of frame generator buffer
    float tx_gain;                  // soft transmit gain (linear)
#if 0
    pthread_t tx_process;           // transmit thread
    pthread_mutex_t tx_mutex;       // transmit mutex
#endif
    Buffer<CplxFVec> frame_buffer;


    // receiver objects
    //ofdmflexframesync fs;           // frame synchronizer object

    std::vector< ChannelConfig > channels_;


    boost::scoped_ptr< boost::thread > tx_thread_, modulation_thread_;

    void transmit_function();
    void modulation_function();
    void reconfigure_usrp(const int num);
    //pthread_t rx_process;           // receive thread
    //pthread_mutex_t rx_mutex;       // receive mutex
    //pthread_cond_t  rx_cond;        // receive condition
    bool rx_running;                // is receiver running? (physical receiver)
    bool rx_thread_running;         // is receiver thread running?
    bool debug_;             // is debugging enabled?

    // RF objects and properties
    uhd::usrp::multi_usrp::sptr usrp_tx;
    uhd::usrp::multi_usrp::sptr usrp_rx;
    uhd::tx_metadata_t          metadata_tx;
};

#endif // OFDMTRANSCEIVER_H
