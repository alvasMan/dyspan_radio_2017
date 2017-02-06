/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2015  Andre Puschmann, Francisco Paisana, Justin Tallon
 *
 * \section LICENSE
 *
 * This file is part of dyspanradio.
 *
 * dyspanradio is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * dyspanradio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

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
static int key = 0;
static int last_key = 0;

static void signal_handler(int signum)
{
    std::cout << "Terminating .." << std::endl;
    stop_signal_called = true;
}

int read_key()
{
  char input[128];

  fd_set set;
  FD_ZERO(&set);
  FD_SET(0, &set);

  struct timeval to;
  to.tv_sec = 0;
  to.tv_usec = 0;

  int n = select(1, &set, NULL, NULL, &to);
  if (n == 1) {
    // stdin ready
    if (fgets(input, sizeof(input), stdin)) {
      last_key = key;
      key = atoi(input);
    }
    return 0;
  } else if (n < 0) {
    // error
    perror("select");
    return -1;
  } else {
    return 0;
  }
}


int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    RadioParameter params;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&params.args)->default_value("master_clock_rate=20e6"), "single uhd device address args")
        ("txsubdev", po::value<std::string>(&params.txsubdev)->default_value("A:B"), "TX Subdev specification")
        ("rxsubdev", po::value<std::string>(&params.rxsubdev)->default_value("A:A"), "RX Subdev specification")
        ("mode", po::value<std::string>(&params.mode)->default_value("tx"), "Mode selection (tx/rx/rx_pfb)")
        ("numtrx", po::value<size_t>(&params.num_trx)->default_value(1), "Number of transceivers")

        ("num_channels", po::value<size_t>(&params.num_channels)->default_value(4), "Number of channels (must be multiple of two)")
        ("channel_bandwidth", po::value<double>(&params.channel_bandwidth)->default_value(2500e3), "Bandwidth of each individual channel")
        ("channel_rate", po::value<double>(&params.channel_rate)->default_value(2500e3), "Transmission rate in each individual channel")
        ("freq",po::value<double>(&params.f_center)->default_value(2490000000),"Sets center frequency")
        ("rxgain",po::value<double>(&params.rx_gain_uhd)->default_value(15),"Sets UHD receive gain")
        ("txgain_soft",po::value<double>(&params.tx_gain_soft)->default_value(-12),"Sets software transmit gain")
        ("txgain_uhd",po::value<double>(&params.tx_gain_uhd)->default_value(10),"Sets UHD transmit gain")
        ("rxantenna", po::value<std::string>(&params.rx_antenna)->default_value("TX/RX"), "RX antenna to use")
        ("stat_interval",po::value<unsigned int>(&params.stat_interval)->default_value(2),"Statistic interval")

        ("M",po::value<unsigned int>(&params.M)->default_value(48),"Number of subcarriers")
        ("cp_len",po::value<unsigned int>(&params.cp_len)->default_value(6),"Cyclic prefix length")
        ("taper_len",po::value<unsigned int>(&params.taper_len)->default_value(4),"Taper length")
        ("fec0",po::value<std::string>(&params.fec0)->default_value("none"),"FEC for header")
        ("fec1",po::value<std::string>(&params.fec1)->default_value("h128"),"FEC for payload")
        ("crc",po::value<std::string>(&params.crc)->default_value("crc32"),"CRC")
        ("mod",po::value<std::string>(&params.mod)->default_value("qpsk"),"Modulation scheme")

        ("challenge", po::value<bool>(&params.use_db)->default_value(false), "Whether to connect and use challenge database")
        ("db_ip", po::value<std::string>(&params.db_ip)->default_value("127.0.0.1"), "Database IP")
        ("db_user", po::value<std::string>(&params.db_user)->default_value("?"), "Database user")
        ("db_pass", po::value<std::string>(&params.db_password)->default_value("?"), "Database password")

        ("tx_enabled", po::value<bool>(&params.tx_enabled)->default_value(true), "learning on or off")
        ("debug", po::value<bool>(&params.debug)->default_value(false), "Whether to print debug messages")
        ("sensing", po::value<bool>(&params.has_sensing)->default_value(true), "Whether to start sensing thread")
        ("sensing_to_file", po::value<bool>(&params.sensing_to_file)->default_value(false), "Whether to store spectrogram in binary file")
        ("project_folder", po::value<std::string>(&params.project_folder)->default_value("../../"), "Needed to find config files")
        ("read_learning_file", po::value<std::string>(&params.read_learning_file)->default_value("learned_data.json"), "Which file to configure context algorithms")
        ("write_learning_file", po::value<std::string>(&params.write_learning_file)->default_value("written_data.json"), "Which file to write output of context algorithms")
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

    // make sure to set remaining radio parameters
    params.p = NULL;

    // install signal handler
    std::signal(SIGINT, signal_handler);

    // create radio
    boost::shared_ptr<DyspanRadio> radio;
    if (params.mode == "tx")
        radio.reset(new OfdmTransceiver(params));
    else if (params.mode == "rx")
        radio.reset(new multichannelrx(params));
    else if (params.mode == "rx_pfb")
        radio.reset(new multichannelrx_pfb(params));
    else
        throw std::runtime_error("Invalid mode specified.");

    // delay start a bit ..
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    std::cout << "Press Ctrl + C to stop radio ..." << std::endl;
    radio->start();

    // sleep until end ..
    while (not stop_signal_called) {
      read_key();
      if (key != last_key) {
        boost::shared_ptr<OfdmTransceiver> tx = boost::dynamic_pointer_cast<OfdmTransceiver>(radio);
        if (tx != NULL)
          tx->set_channel(key);
      }
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    radio->stop();
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return 0;
}
