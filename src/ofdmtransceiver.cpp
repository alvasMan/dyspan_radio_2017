
#include "ofdmtransceiver.h"
#include <boost/assign.hpp>
#include <uhd/types/tune_request.hpp>
#include <iostream>
#include <string>

#define DEBUG_MODE

#ifdef DEBUG_MODE
#include <ctime>
#endif
// default constructor
//  _M              :   OFDM: number of subcarriers
//  _cp_len         :   OFDM: cyclic prefix length
//  _taper_len      :   OFDM: taper prefix length
//  _p              :   OFDM: subcarrier allocation
//  _callback       :   frame synchronizer callback function
//  _userdata       :   user-defined data structure
OfdmTransceiver::OfdmTransceiver(const std::string args, const int num_channels, const double f_center, const double channel_bandwidth, const double channel_rate, const float tx_gain_soft, const float tx_gain_uhd, const float rx_gain_uhd) :
    M(256),
    cp_len(16),
    taper_len(4),
    num_channels_(num_channels),
    channel_rate_(channel_rate),
    channel_bandwidth_(channel_bandwidth),
    seq_no_(0),
    debug_(false)
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
    usrp_tx = uhd::usrp::multi_usrp::make(std::string("master_clock_rate=20e6"));
    usrp_tx->set_tx_subdev_spec(std::string("A:B")); 
    std::cout << boost::format("Using Device: %s") % usrp_tx->get_pp_string() << std::endl;
    usrp_rx = uhd::usrp::multi_usrp::make(std::string("master_clock_rate=20e6"));
    usrp_rx->set_rx_subdev_spec(std::string("A:B"));
    // initialize channels, add two in each iteration
    assert(num_channels % 2 == 0);
    for (int i = 0; i < num_channels; i += 2) {
        double offset = i / 2 * channel_bandwidth + channel_bandwidth / 2;
        // add left neighbor
        channels_.push_back({"Channel" + std::to_string(i), f_center + offset, channel_bandwidth, f_center, +offset, channel_rate});
        // add right neighbor
        channels_.push_back({"Channel" + std::to_string(i + 1), f_center + offset, channel_bandwidth, f_center, -offset, channel_rate});
    }
    // TODO: check that all channels have the same center frequency for faster tuning

    // initialize default tx values
    set_tx_freq(f_center);
    set_tx_rate(channel_rate);
    set_tx_gain_soft(tx_gain_soft);
    set_tx_gain_uhd(tx_gain_uhd);

    double rx_rf_rate = num_channels * channel_bandwidth;
    set_rx_freq(f_center);
    set_rx_rate(rx_rf_rate);
    set_rx_gain_uhd(rx_gain_uhd);
    //set_rx_antenna("J1");

    // setting up the energy detector (number of averages,window step size,fftsize)
    e_detec.set_parameters(60, num_channels, 1024);// Andre: these are the parameters of the sensing (number of averages,window step size,fftsize)
}


OfdmTransceiver::~OfdmTransceiver()
{
    delete fgbuffer;
}


//
// transmitter methods
//
void OfdmTransceiver::run(void)
{
    // start threads
    threads_.push_back( new boost::thread( boost::bind( &OfdmTransceiver::modulation_function, this ) ) );
    threads_.push_back( new boost::thread( boost::bind( &OfdmTransceiver::receive_function, this ) ) );

    // either start random transmit function or normal one ..
  threads_.push_back( new boost::thread( boost::bind( &OfdmTransceiver::random_transmit_function, this ) ) );
  //  threads_.push_back( new boost::thread( boost::bind( &OfdmTransceiver::transmit_function, this ) ) );
}


void OfdmTransceiver::stop()
{
    boost::ptr_vector<boost::thread>::iterator it;
    for (it = threads_.begin(); it != threads_.end(); ++it) {
        it->interrupt();
        it->join();
    }
}



// This function creates new frames and pushes them on a shared buffer
void OfdmTransceiver::modulation_function(void)
{
    try {
        int payload_len = 1000;
        unsigned char header[8];
        unsigned char payload[payload_len];

        while (true)
        {
            boost::this_thread::interruption_point();

            if (debug_)
                printf("tx packet id: %6u\n", seq_no_);

            // write header (first four bytes sequence number, remaining are random)
            header[0] = (seq_no_ >> 24) & 0xff;
            header[1] = (seq_no_ >> 16) & 0xff;
            header[2] = (seq_no_ >> 8 ) & 0xff;
            header[3] = (seq_no_      ) & 0xff;
            for (int i = 4; i < 8; i++)
                header[i] = rand() & 0xff;

            // initialize payload
            for (int i = 0; i < payload_len; i++)
                payload[i] = rand() & 0xff;

            // create fresh buffer for this frame
            boost::shared_ptr<CplxFVec> usrp_buffer( new CplxFVec(fgbuffer_len) );

            ofdmflexframegen_setprops(fg, &fgprops);

            // assemble frame
            ofdmflexframegen_assemble(fg, header, payload, payload_len);

            // generate a single OFDM frame
            bool last_symbol = false;
            while (not last_symbol) {
                // generate symbol
                last_symbol = ofdmflexframegen_writesymbol(fg, fgbuffer);

                // copy symbol and apply gain
                for (int i = 0; i < fgbuffer_len; i++)
                    (*usrp_buffer.get())[i] = fgbuffer[i] * tx_gain;
            }

            frame_buffer.pushBack(usrp_buffer);
            seq_no_++;
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
                boost::this_thread::sleep(boost::posix_time::seconds(1));
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


void OfdmTransceiver::random_transmit_function(void)
{
    try {
        
        bool yes = true;
        int a = 0;
        while (true) {
            boost::this_thread::interruption_point();
            
            // get random channel
            //int num = rand() % channels_.size();
           // reconfigure_usrp(num);
            for (int i = 0; i < 2; i++)
                transmit_packet();
               // if(yes)
                //{
                    //set_tx_freq(2.5e9);
                    //boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                 //   yes = false;
                    a++;
                    if(a == 4)
                        a=0;
                    if(yes){
                        reconfigure_usrp(1);
                        yes = false;
                    }
                //}
            //boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Random transmit thread interrupted." << std::endl;
    }
}


void OfdmTransceiver::reconfigure_usrp(const int num)
{
    // construct tuning request
    uhd::tune_request_t request;
    // don't touch RF part
    request.rf_freq_policy = uhd::tune_request_t::POLICY_NONE;
    request.rf_freq = 0;
    // only tune DSP frequency
    request.dsp_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
    request.dsp_freq = channels_.at(num).dsp_freq;
    request.args = uhd::device_addr_t("mode_n=integer");
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

    boost::shared_ptr<CplxFVec> buffer;
    frame_buffer.popFront(buffer);// new CplxFVec(fgbuffer_len) );
    //std::cout << boost::format("Size is: %d") % buffer.size() << std::endl;
    float mag = 0;
   // for(int i= 0; i < buffer->size();i++)
    //{
    //    mag += buffer[i];
   // }
        //mag = mag/buffer->size();
        //std::cout << "mag of IQ samples : " << mag << std::endl; 
    // send samples to the device
    usrp_tx->get_device()->send(
        &buffer->front(), buffer->size(),
        metadata_tx,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF
    );

    // send a few extra samples to the device
    // NOTE: this seems necessary to preserve last OFDM symbol in
    //       frame from corruption
    usrp_tx->get_device()->send(
        &buffer->front(), buffer->size(),
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


void OfdmTransceiver::receive_function(void)
{
    
    #ifdef DEBUG_MODE
    time_t tnow, tlast = time(0);
#endif

    //create a receive streamer
    std::string wire_format("sc16");
    std::string cpu_format("fc32");
    uhd::stream_args_t stream_args(cpu_format, wire_format);
    uhd::rx_streamer::sptr rx_stream = usrp_tx->get_rx_stream(stream_args);

    //setup streaming
    std::cout << std::endl;
    std::cout << boost::format("Begin streaming now ..") << std::endl;
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = 0; // continuous
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t();
    usrp_tx->issue_stream_cmd(stream_cmd);

    //meta-data will be filled in by recv()
    uhd::rx_metadata_t metadata;
    bool overflow_message = true;

    try {

        while (true) {
            boost::this_thread::interruption_point();

            do {
                std::vector<std::complex<float> > rxBuff;
                rxBuff.resize(100);
                //size_t num_rx_samps1 = rx_stream->recv(&rxBuff.front(), rxBuff.size(), metadata, 3.0);
                size_t num_rx_samps = rx_stream->recv(e_detec.fftBins, e_detec.nBins, metadata, 5.0);

                //handle the error code
                if (metadata.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                    std::cout << boost::format("Timeout while streaming") << std::endl;
                    break;
                }
                if (metadata.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
                    if (overflow_message){
                        overflow_message = false;
                        std::cerr << boost::format("Got an overflow indication, please reduce sample rate.");
                    }
                    continue;
                }
                if (metadata.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
                    throw std::runtime_error(str(boost::format(
                                                     "Unexpected error code 0x%x"
                                                     ) % metadata.error_code));
                }

                timestamp_ = metadata.time_spec;

                // Always call process because we write on fftBins nBins samples from uhd (see above)
                e_detec.process(timestamp_.get_real_secs());
           } while(not e_detec.result_exists());

            double tstamp;
            std::vector<float> ch_power;
            e_detec.pop_result(tstamp, ch_power);// ch_power is your sensing results, free channels will appear as a 0
            int numfree = 0;

            // print power levels for each channel
#ifdef DEBUG_MODE
            tnow = time(0);
            // print power levels for each channel
            if (difftime(tnow,tlast) > 0.01) {
                std::cout << "Energy: " << print_vector_dB(ch_power);
                std::cout << "p(Detection): " << e_detec.noise_filter->print_ch_pdetec();
                std::cout << "noise floor: ";
                for (int i = 0; i < ch_power.size(); i++) {
                    float dB_value = 10*log10(e_detec.noise_filter->ch_noise_floor(i)), detec_rate = e_detec.noise_filter->ch_detec_rate(i);
                    std::cout << boost::format("%d: %1.4f dB\t") % i % dB_value;
                }
                std::cout << std::endl;
                tlast = tnow;
            }
#endif


#if 0
            //std::cout << " Channels are :  " << "1: "<< ch_power[0] << " 2 :"<<ch_power[1] << " 3 :"<< ch_power[2] <<" 4 :"<< ch_power[3] << std::endl;
            for(int i =0; i<4; i++)// Andre: I have added this simple way of parsing the sensing as a placeholder, the learning stuff will go here in reality
            {
                if(ch_power[i] == 0)
                {
                    numfree++;
                    next_channel = i;
                }
            }
            if(numfree > 2)
            {
                for(int i =0;i<4;i++)
                {
                    if(ch_power[i]>0)
                    {
                        pu_channel = i;
                        my_channel = i;
                    }
                }
            }
#endif

        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Receive thread interrupted." << std::endl;
    }
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
    std::cout << "Actual TX gain: " << usrp_tx->get_tx_gain() << std::endl;
}

// set transmitter antenna
void OfdmTransceiver::set_tx_antenna(char * _tx_antenna)
{
    usrp_tx->set_tx_antenna(_tx_antenna);
    std::cout << "Using TX Antenna: " << usrp_tx->get_tx_antenna() << std::endl;
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

    // check LO Lock
    std::vector<std::string> sensor_names;
    sensor_names = usrp_rx->get_rx_sensor_names(0);
    if(std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end())
    {
        uhd::sensor_value_t lo_locked = usrp_rx->get_rx_sensor("lo_locked",0);
        std::cout << "Checking RX: " << lo_locked.to_pp_string() <<  "..." << std::endl;
        if(!lo_locked.to_bool())
            std::cout << "Failed to lock LO" << std::endl;
    }
}

// set receiver sample rate
void OfdmTransceiver::set_rx_rate(float _rx_rate)
{
    usrp_rx->set_rx_rate(_rx_rate);
    std::cout << "Actual RX Rate: " << (usrp_rx->get_rx_rate()/1e6) << " Msps" << std::endl;
}

// set receiver hardware (UHD) gain
void OfdmTransceiver::set_rx_gain_uhd(float _rx_gain_uhd)
{
    usrp_rx->set_rx_gain(_rx_gain_uhd);
    std::cout << "Actual RX gain: " << usrp_rx->get_rx_gain() << " dB" << std::endl;
}

// set receiver antenna
void OfdmTransceiver::set_rx_antenna(char * _rx_antenna)
{
    usrp_rx->set_rx_antenna(_rx_antenna);
    std::cout << "Using RX Antenna: " << usrp_rx->get_rx_antenna() << std::endl;
}

// reset receiver objects and buffers
void OfdmTransceiver::reset_rx()
{
    //ofdmflexframesync_reset(fs);
}


