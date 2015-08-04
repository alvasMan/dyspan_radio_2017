
#include "ofdmtransceiver.h"
#include <boost/assign.hpp>
#include <uhd/types/tune_request.hpp>

// default constructor
//  _M              :   OFDM: number of subcarriers
//  _cp_len         :   OFDM: cyclic prefix length
//  _taper_len      :   OFDM: taper prefix length
//  _p              :   OFDM: subcarrier allocation
//  _callback       :   frame synchronizer callback function
//  _userdata       :   user-defined data structure
OfdmTransceiver::OfdmTransceiver(const std::string args, const double freq, const double rate, const float tx_gain_soft, const float tx_gain_uhd) :
    M(256),
    cp_len(16),
    taper_len(4),
    debug_(true)
{
    // create frame generator
    unsigned char * p = NULL;   // subcarrier allocation (default)
    ofdmflexframegenprops_init_default(&fgprops);
    fgprops.check           = LIQUID_CRC_32;
    fgprops.fec0            = LIQUID_FEC_NONE;
    fgprops.fec1            = LIQUID_FEC_HAMMING128;
    fgprops.mod_scheme      = LIQUID_MODEM_QPSK;
    fg = ofdmflexframegen_create(M, cp_len, taper_len, p, &fgprops);

    // allocate memory for frame generator output (single OFDM symbol)
    fgbuffer_len = M + cp_len;
    fgbuffer = (std::complex<float>*) malloc(fgbuffer_len * sizeof(std::complex<float>));

    // create frame synchronizer
    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    usrp_tx = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp_tx->get_pp_string() << std::endl;
    //usrp_rx = uhd::usrp::multi_usrp::make(dev_addr);

    // initialize channels
    // std::string description, double f_center, double rf_freq, double dsp_freq, double bandwidth
    //channels_.push_back({"Channel1", 2.445e9, 2e6, 0.0});
    channels_.push_back({"Channel2", 2.445e9, 2.4475e9, 5e6, 2e6});
    channels_.push_back({"Channel3", 2.450e9, 2.4475e9, -5e6, 2e6});

    double rf_freq = 2.4475e9;

    // initialize default tx values
    set_tx_freq(rf_freq);
    set_tx_rate(rate);
    set_tx_gain_soft(tx_gain_soft);
    set_tx_gain_uhd(tx_gain_uhd);

    // TODO: check that all channels have the same center frequency for faster tuning
}


OfdmTransceiver::~OfdmTransceiver()
{

}


//
// transmitter methods
//
void OfdmTransceiver::run(void)
{
    //std::signal(SIGINT, &OfdmTransceiver::signal_handler);
    // start threads
    modulation_thread_.reset( new boost::thread( boost::bind( &OfdmTransceiver::modulation_function, this ) ) );
    tx_thread_.reset( new boost::thread( boost::bind( &OfdmTransceiver::transmit_function, this ) ) );
}


void OfdmTransceiver::stop()
{
    // stopping threads
    modulation_thread_->interrupt();
    tx_thread_->interrupt();
}



// This function creates new frames and pushes them on a shared buffer
void OfdmTransceiver::modulation_function(void)
{
    try {
        while (true)
        {
            boost::this_thread::interruption_point();

            // data arrays
            int payload_len = 1000;
            unsigned char header[8];
            unsigned char payload[payload_len];

            unsigned int pid;
            unsigned int i;
            int num_frames = 10;
            for (pid=0; pid<num_frames; pid++) {
                if (debug_)
                    printf("tx packet id: %6u\n", pid);

                // write header (first two bytes packet ID, remaining are random)
                header[0] = (pid >> 8) & 0xff;
                header[1] = (pid     ) & 0xff;
                for (i=2; i<8; i++)
                    header[i] = rand() & 0xff;

                // initialize payload
                for (i=0; i<payload_len; i++)
                    payload[i] = rand() & 0xff;


                // fector buffer to send data to device
                CplxFVec usrp_buffer(fgbuffer_len);
                ofdmflexframegen_setprops(fg, &fgprops);

                // assemble frame
                ofdmflexframegen_assemble(fg, header, payload, payload_len);

                // generate a single OFDM frame
                bool last_symbol = false;
                unsigned int i;
                while (not last_symbol) {

                    // generate symbol
                    last_symbol = ofdmflexframegen_writesymbol(fg, fgbuffer);

                    // copy symbol and apply gain
                    for (i=0; i<fgbuffer_len; i++)
                        usrp_buffer[i] = fgbuffer[i] * tx_gain;
                } // while loop

                frame_buffer.pushBack(usrp_buffer);
            } // packet loop
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Modulation thread interrupted." << std::endl;
    }
}


void OfdmTransceiver::transmit_function(void)
{
    try {

        while (true) {
            boost::this_thread::interruption_point();

            // check if channel needs to be reconfigured
            // send 50 packets on second channel once every 100 packets
            static int counter = 1;
            if (counter++ % 100 == 0) {
                reconfigure_usrp(1);
                for (int i = 0; i < 50; i++)
                    transmit_packet();
                reconfigure_usrp(0);
            } else {
                // transmit frame
                transmit_packet();
            }
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Transmit thread interrupted." << std::endl;
    }
}


void OfdmTransceiver::reconfigure_usrp(const int num)
{
    // get random channel
    //num = rand() % channels_.size();
    //double rf_lo = usrp_tx->get_tx_freq();

    // construct tuning request
    uhd::tune_request_t request;
    // don't touch RF part
    request.rf_freq_policy = uhd::tune_request_t::POLICY_NONE;
    request.rf_freq = 0;
    // only tune DSP frequency
    request.dsp_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
    request.dsp_freq = channels_.at(num).dsp_freq;
    uhd::tune_result_t result = usrp_tx->set_tx_freq(request);

    if (debug_) {
        std::cout << result.to_pp_string() << std::endl;
    }
}

void OfdmTransceiver::transmit_packet()
{
    // set up the metadata flags
    metadata_tx.start_of_burst = false; // never SOB when continuous
    metadata_tx.end_of_burst   = false; //
    metadata_tx.has_time_spec  = false; // set to false to send immediately

    CplxFVec buffer;
    frame_buffer.popFront(buffer);
    std::cout << boost::format("Size is: %d") % buffer.size() << std::endl;

    // send samples to the device
    usrp_tx->get_device()->send(
        &buffer.front(), buffer.size(),
        metadata_tx,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF
    );

    // send a few extra samples to the device
    // NOTE: this seems necessary to preserve last OFDM symbol in
    //       frame from corruption
    usrp_tx->get_device()->send(
        &buffer.front(), buffer.size(),
        metadata_tx,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF
    );

    // send a mini EOB packet
    metadata_tx.start_of_burst = false;
    metadata_tx.end_of_burst   = true;

    usrp_tx->get_device()->send("", 0, metadata_tx,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF);
}


// set transmitter frequency
void OfdmTransceiver::set_tx_freq(float freq)
{
    std::cout << boost::format("Setting TX Center Frequency: %f") % freq << std::endl;
    uhd::tune_request_t request(freq, 0.0);
    uhd::tune_result_t result = usrp_tx->set_tx_freq(request);
    if (debug_) {
        std::cout << result.to_pp_string() << std::endl;
    }
}

// set transmitter sample rate
void OfdmTransceiver::set_tx_rate(float rate)
{
    //set transmit parameter
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp_tx->set_tx_rate(rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp_tx->get_tx_rate()/1e6) << std::endl << std::endl;


}

// set transmitter software gain
void OfdmTransceiver::set_tx_gain_soft(float gain)
{
    tx_gain = powf(10.0f, gain/20.0f);
}

// set transmitter hardware (UHD) gain
void OfdmTransceiver::set_tx_gain_uhd(float gain)
{
    std::cout << boost::format("Setting TX Gain %f") % gain << std::endl;
    usrp_tx->set_tx_gain(gain);
}

// set transmitter antenna
void OfdmTransceiver::set_tx_antenna(char * _tx_antenna)
{
    usrp_rx->set_tx_antenna(_tx_antenna);
}

// reset transmitter objects and buffers
void OfdmTransceiver::reset_tx()
{
    ofdmflexframegen_reset(fg);
}




//
// receiver methods
//

// set receiver frequency
void OfdmTransceiver::set_rx_freq(float _rx_freq)
{
    usrp_rx->set_rx_freq(_rx_freq);
}

// set receiver sample rate
void OfdmTransceiver::set_rx_rate(float _rx_rate)
{
    usrp_rx->set_rx_rate(_rx_rate);
}

// set receiver hardware (UHD) gain
void OfdmTransceiver::set_rx_gain_uhd(float _rx_gain_uhd)
{
    usrp_rx->set_rx_gain(_rx_gain_uhd);
}

// set receiver antenna
void OfdmTransceiver::set_rx_antenna(char * _rx_antenna)
{
    usrp_rx->set_rx_antenna(_rx_antenna);
}

// reset receiver objects and buffers
void OfdmTransceiver::reset_rx()
{
    //ofdmflexframesync_reset(fs);
}


