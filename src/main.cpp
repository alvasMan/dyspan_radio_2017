#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <complex>
#include <csignal>
#include <uhd/types/tune_request.hpp>
#include <boost/filesystem.hpp>

#include "ofdmtransceiver.h"

using namespace boost;
namespace po = boost::program_options;
using namespace std;

static bool verbose = false;
static bool stop_signal_called = false;

//static OfdmTransceiver trx;

template<typename samp_type> void calc(std::vector<samp_type> &buff, typename samp_type::value_type &res)
{
#if 0
    // print first samples
    for (size_t i = 0; i < 10; i++) {
        std::cout << boost::format("%d: (%12.2d + i * %12.2d)") % i % buff[i].real() % buff[i].imag() << std::endl;
    }
#endif

    // compute RSSI using standard math library
    typename samp_type::value_type rssi_hat = 0;
    
    // sum up the squared magnitude of the samples
    for (size_t i = 0; i < buff.size(); i++) {
        rssi_hat += pow(abs(buff[i]), 2); //rssi + pow(abs(buff[i]), 2);pow(norm(buff[i]), 2);
    }
    
    if (sizeof(typename samp_type::value_type) == 2) {
        // convert to float if in sc16 mode
        res = 10 * log10f((boost::uint32_t)rssi_hat << 16);
        if(verbose) std::cout << boost::format("RSSI short: %d") % res;
    } else {
        res = 10 * log10f(rssi_hat);
        if(verbose) std::cout << boost::format("RSSI float: %12.8f") % res;
    }
}

#if 0
void transmit_thread(
    uhd::usrp::multi_usrp::sptr usrp,
    float what = 1.0)
{


    // transit a burst of packets
    txcvr.start_tx();
    printf("transmitting burst...\n");
    while ( timer_toc(timer_tx) < tx_burst_time) {
        // get next available channel (blocking)
        unsigned int c = txcvr.get_available_channel();
        assert( c < num_channels);

        // assemble packet
        unsigned int this_packet_len = rand() % payload_len;
        assemble_packet(pid, header, payload, this_packet_len);

        // transmit frame on channel 'c'
        printf("transmitting packet %6u (%6u bytes) on channel %6u\n", pid, this_packet_len, c);
        //int rc =
        txcvr.transmit_packet(c, header, payload, this_packet_len, ms, fec0, fec1);

        // update packet counter on channel 'c'
        pid++;
    }

}
#endif



static void signal_handler(int signum)
{
    std::cout << "Terminating .." << std::endl;
    stop_signal_called = true;
}


int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, type, threshold, wirefmt;
    double seconds_in_future;
    size_t total_num_samps;
    size_t samps_per_packet;
    size_t spb;
    size_t num_total_packets, packets_per_second;
    size_t tx_buffer_size;
    double rate, freq, rx_gain, tx_gain_soft, tx_gain_uhd;
    float ampl;
    
    
    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("secs", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to receive")
        ("type", po::value<std::string>(&type)->default_value("short"), "sample type: double, float, or short")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(10000), "total number of samples to receive")
        ("spb", po::value<size_t>(&spb)->default_value(512), "samples per recv buffer")
        ("spp", po::value<size_t>(&samps_per_packet)->default_value(20000), "number of samples per transmit packet")
        ("npackets", po::value<size_t>(&num_total_packets)->default_value(100), "How many packets to transmit in total")
        ("pps", po::value<size_t>(&packets_per_second)->default_value(1), "How many packets to transmit per second")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.9)), "amplitude of each sample")
        ("rate", po::value<double>(&rate)->default_value(2000e3), "rate of incoming samples")
        ("freq",po::value<double>(&freq)->default_value(2450e6),"Sets Center Frequency")
        ("rxgain",po::value<double>(&rx_gain)->default_value(5),"Sets receive gain")
        ("txgain_soft",po::value<double>(&tx_gain_soft)->default_value(-12),"Sets software transmit gain")
        ("txgain_uhd",po::value<double>(&tx_gain_uhd)->default_value(10),"Sets UHD transmit gain")
        ("txbufsize",po::value<size_t>(&tx_buffer_size)->default_value(10),"How many frames in Tx buffer")
        ("threshold", po::value<std::string>(&threshold)->default_value("-75"), "RSSI threshold in dBm")
        ("wirefmt", po::value<std::string>(&wirefmt)->default_value("sc16"), "wire format (sc8 or sc16)")
        ("dilv", "specify to disable inner-loop verbose")
    ;
    
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("5G Spectrum Sharing Challenge %s") % desc << std::endl;
        return ~0;
    }

    verbose = vm.count("dilv") == 0;

    std::signal(SIGINT, signal_handler);
    std::cout << "Press Ctrl + C to stop radio ..." << std::endl;
    
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    OfdmTransceiver trx(args, freq, rate, tx_gain_soft, tx_gain_uhd);
    trx.run();

    // sleep until end ..
    while (not stop_signal_called) {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
    }
    trx.stop();
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return 0;
}
