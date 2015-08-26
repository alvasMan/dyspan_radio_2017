
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <complex>
#include <csignal>

#include <boost/filesystem.hpp>
#include "ofdmtransceiver.h"
#include "multichannelrx.h"
#include "multichannelrx_pfb.h"

using namespace boost;
namespace po = boost::program_options;
using namespace std;

static bool stop_signal_called = false;

static void signal_handler(int signum)
{
    std::cout << "Terminating .." << std::endl;
    stop_signal_called = true;
}


int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string mode;
    bool debug;
    std::string args, type, threshold, wirefmt;
    double seconds_in_future;
    size_t total_num_samps;
    size_t samps_per_packet;
    size_t spb;
    size_t num_total_packets, packets_per_second;
    size_t tx_buffer_size;
    size_t num_channels;
    double channel_bandwidth, channel_rate, freq, rx_gain, tx_gain_soft, tx_gain_uhd;
    float ampl;


    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("mode", po::value<std::string>(&mode)->default_value("tx"), "Mode selection (tx/rx/rx_pfb)")
        ("secs", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to receive")
        ("type", po::value<std::string>(&type)->default_value("short"), "sample type: double, float, or short")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(10000), "total number of samples to receive")
        ("spb", po::value<size_t>(&spb)->default_value(512), "samples per recv buffer")
        ("spp", po::value<size_t>(&samps_per_packet)->default_value(20000), "number of samples per transmit packet")
        ("npackets", po::value<size_t>(&num_total_packets)->default_value(100), "How many packets to transmit in total")
        ("pps", po::value<size_t>(&packets_per_second)->default_value(1), "How many packets to transmit per second")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.9)), "amplitude of each sample")
        ("num_channels", po::value<size_t>(&num_channels)->default_value(2), "Number of channels (must be multiple of two)")
        ("channel_bandwidth", po::value<double>(&channel_bandwidth)->default_value(5000e3), "Bandwidth of each individual channel")
        ("channel_rate", po::value<double>(&channel_rate)->default_value(2000e3), "Transmission rate in each individual channel")
        ("freq",po::value<double>(&freq)->default_value(2.4475e9),"Sets center frequency")
        ("rxgain",po::value<double>(&rx_gain)->default_value(15),"Sets UHD receive gain")
        ("txgain_soft",po::value<double>(&tx_gain_soft)->default_value(-12),"Sets software transmit gain")
        ("txgain_uhd",po::value<double>(&tx_gain_uhd)->default_value(10),"Sets UHD transmit gain")
        ("txbufsize",po::value<size_t>(&tx_buffer_size)->default_value(10),"How many frames in Tx buffer")
        ("threshold", po::value<std::string>(&threshold)->default_value("-75"), "RSSI threshold in dBm")
        ("wirefmt", po::value<std::string>(&wirefmt)->default_value("sc16"), "wire format (sc8 or sc16)")
        ("debug", po::value<bool>(&debug)->default_value(false), "Whether to print debug messages")
    ;
    
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("5G Spectrum Sharing Challenge %s") % desc << std::endl;
        return ~0;
    }

    std::signal(SIGINT, signal_handler);

    // TODO: make them a parameter
    int M = 48;
    int cp_len = 6;
    int taper_len = 4;
    unsigned char * p = NULL;   // default subcarrier allocation

    // create radio
    boost::shared_ptr<DyspanRadio> radio;
    if (mode == "tx")
        radio.reset(new OfdmTransceiver(args, num_channels, freq, channel_bandwidth, channel_rate, tx_gain_soft, tx_gain_uhd, rx_gain, debug));
    else if (mode == "rx")
        radio.reset(new multichannelrx(args, num_channels, freq, channel_bandwidth, channel_rate, rx_gain, M, cp_len, taper_len, p, debug));
    else if (mode == "rx_pfb")
        radio.reset(new multichannelrx_pfb(args, num_channels, freq, channel_bandwidth, channel_rate, rx_gain, M, cp_len, taper_len, p, debug));
    else
        throw std::runtime_error("Invalid mode specified.");

    // delay start a bit ..
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    std::cout << "Press Ctrl + C to stop radio ..." << std::endl;
    radio->start();

    // sleep until end ..
    while (not stop_signal_called) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    radio->stop();
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return 0;
}
