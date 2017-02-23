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

#include "ofdmtransceiver.h"
#include "modulation_search_api.h"
#include <boost/assign.hpp>

#include <iostream>
#include <string>
#include "general_utils.hpp"
#include "channel_hopper.hpp"

#define DEBUG_MODE

#ifdef DEBUG_MODE
#include <ctime>
#endif

using std::cout;
using std::endl;

OfdmTransceiver::OfdmTransceiver(const RadioParameter params) :
    DyspanRadio(params),
    seq_no_(0),
    payload_len_(1500)
{
    assert(payload_len_ <= MAX_PAYLOAD_LEN);

    // create frame generator
    ofdmflexframegenprops_init_default(&fgprops);
    fgprops.check           = liquid_getopt_str2crc(params_.crc.c_str());
    fgprops.fec0            = liquid_getopt_str2fec(params_.fec0.c_str());
    fgprops.fec1            = liquid_getopt_str2fec(params_.fec1.c_str());
    fgprops.mod_scheme      = liquid_getopt_str2mod(params_.mod.c_str());
    fg = ofdmflexframegen_create(params_.M, params_.cp_len, params_.taper_len, params_.p, &fgprops);

    // allocate memory for frame generator output (single OFDM symbol)
    fgbuffer_len = params_.M + params_.cp_len;
    fgbuffer = (std::complex<float>*) malloc(fgbuffer_len * sizeof(std::complex<float>));

    //create a usrp device
    cout << endl;
    cout << boost::format("Creating the usrp device with: %s...") % params_.args << endl;
    usrp_tx = uhd::usrp::multi_usrp::make(params_.args);
    usrp_tx->set_tx_subdev_spec(params_.txsubdev);
    cout << boost::format("Using Device: %s") % usrp_tx->get_pp_string() << endl;
    usrp_rx = uhd::usrp::multi_usrp::make(params_.args);
    usrp_rx->set_rx_subdev_spec(params_.rxsubdev);

    // cout << boost::format("number of channels ::: %s") % usrp_rx->get_rx_num_channels() << endl;

    // initialize default tx values
    double rx_rf_rate = params_.num_channels * params_.channel_bandwidth;
    set_tx_freq(params_.f_center);
    set_tx_rate(params_.channel_rate);
    set_tx_gain_soft(params_.tx_gain_soft);
    current_gain = params_.tx_gain_uhd;
    set_tx_gain_uhd(current_gain);

    // tune to first channel with setting the LO
    assert(channels_.size() > 0);
    if(params_.tx_enabled)
        su_params_api.reset(new SU_tx_params());
    reconfigure_usrp(0, false);//true);

    set_rx_freq(params_.f_center);
    set_rx_rate(rx_rf_rate);
    set_rx_gain_uhd(params_.rx_gain_uhd);
    
    set_rx_antenna(params_.rx_antenna);

    // Add PU parameters and scenarios
    pu_data = context_utils::make_rf_environment();    // this is gonna read files
    pu_scenario_api.reset(new SituationalAwarenessApi(*pu_data));
    
    // Add SU config. stuff
    channel_hopper.reset(new SimpleChannelHopper(*pu_scenario_api));
    //power_controller.reset(new PowerSearcher(5,-1));
    
    // check if no weird configuration
    assert(params_.has_sensing || (!params_.sensing_to_file && !params_.has_deep_learning && !params_.has_learning));
    
    if(params_.sensing_to_file)
        ch_pwrs_buffers.emplace_back(new bounded_buffer<ChPowers>(1000));
    if(params_.has_deep_learning)
        ch_pwrs_buffers.emplace_back(new bounded_buffer<ChPowers>(1000));
    
    BinMask *bin_mask_ptr = NULL;
    if(params_.has_sensing && params_.has_learning)
    {
        learning_chain.reset(new LearningThreadHandler());
        learning_chain->setup_filepaths(params_.project_folder, params_.read_learning_file, params_.write_learning_file);
        learning_chain->setup(ch_pwrs_buffers, 4, 512);
        bin_mask_ptr = &learning_chain->pwr_estim.bin_mask;
    }
    else if(params_.has_sensing)
    {
        sensing_chain.reset(new SensingThreadHandler());
        sensing_chain->setup(pu_scenario_api.get(), su_params_api.get(), ch_pwrs_buffers, 4, 512);
        bin_mask_ptr = &sensing_chain->pwr_estim.bin_mask;
    }
    
    auto it = ch_pwrs_buffers.begin();
    if(params_.sensing_to_file)
    {
        spectrogram2file_chain.reset(new Spectrogram2FileThreadHandler(it->get(), *bin_mask_ptr));
        ++it;
    }
    if(params_.has_deep_learning)
    {
        deep_learning_chain.reset(new Spectrogram2SocketThreadHandler(pu_scenario_api.get(), it->get(), *bin_mask_ptr));
        ++it;
    }
    assert(it==ch_pwrs_buffers.end());

    if (params_.use_db)
    {
        // create and connect to challenge database
        tx_ = spectrum_init(0);
        spectrum_eror_t ret = spectrum_connect(tx_, (char*)params_.db_ip.c_str(), 5002, payload_len_, 1);
        spectrum_errorToText(tx_, ret, error_buffer, sizeof(error_buffer));
        cout << boost::format("TX connect: %s") % error_buffer << endl;
        if (ret < 0) {
            throw std::runtime_error("Couldn't connect to challenge database");
        }

        // get radio number
        int radio = spectrum_getRadioNumber(tx_);
        cout << boost::format("TX radio number: %d") % radio << endl;

        // wait for the start of stage 3 (here you get penalized for interference).
        // The testing database starts in this state so this will instantly return.
        spectrum_waitForState(tx_, 3, -1);
        cout << boost::format("Stage 3 has started.") << endl;

        // ::TODO:: Instantiate dbApi and create a thread for launch_database_thread function

    }

    //create a transmit streamer
    uhd::stream_args_t stream_args("fc32"); //complex floats
    uhd::tx_streamer::sptr tmp = usrp_tx->get_tx_stream(stream_args);
    tx_streamer = tmp;
}


OfdmTransceiver::~OfdmTransceiver()
{
    cout << "Transmitted " << seq_no_ << " frames." << endl;
    if (tx_)
        spectrum_delete(tx_);
    delete fgbuffer;
}


//
// transmitter methods
//
void OfdmTransceiver::start(void)
{
    // start transmission threads, only if tx is enabled
    // NOTE: non tx enabled mode is still useful for us to extract IQ samples
    if(params_.tx_enabled)
    {
        // either start random transmit function or normal one ..
        threads_.push_back( new boost::thread( boost::bind( &OfdmTransceiver::modulation_function, this ) ) );
        threads_.push_back( new boost::thread( boost::bind( &OfdmTransceiver::random_transmit_function, this ) ) );
        //threads_.push_back( new boost::thread( boost::bind( &OfdmTransceiver::transmit_function, this ) ) );
                
        //threads_.push_back(new boost::thread(boost::bind(&OfdmTransceiver::launch_change_places, this)));
    }

    // Launch thread that handles database throughput queries
    if(params_.use_db)
    {
        int radio_id = spectrum_getRadioNumber(tx_);
        threads_.push_back(new boost::thread(boost::bind(launch_database_thread, tx_, radio_id, 10000)));
    }
    else
    {
        threads_.push_back(new boost::thread(launch_mock_database_thread));
    }


    // start sensing thread
    if (params_.has_sensing)
    {
        cout << "Starting sensing threads..." << endl;
        // the two threads communicate through the e_detec buffer
        if(params_.has_learning)
        {
            threads_.push_back(new boost::thread(boost::bind(&LearningThreadHandler::run, learning_chain.get(), usrp_tx)));
        }
        else
        {
            threads_.push_back(new boost::thread(boost::bind(&SensingThreadHandler::run, &(*sensing_chain), usrp_tx)));
            if(params_.has_deep_learning)
            {
                threads_.push_back(new boost::thread(boost::bind(&Spectrogram2SocketThreadHandler::run_recv, deep_learning_chain.get())));
                threads_.push_back(new boost::thread(boost::bind(&Spectrogram2SocketThreadHandler::run_send, deep_learning_chain.get())));
            }
        }
        
        if(spectrogram2file_chain)
            threads_.push_back(new boost::thread(boost::bind(&Spectrogram2FileThreadHandler::run,spectrogram2file_chain.get())));
        //threads_.push_back(new boost::thread(context_utils::launch_mock_scenario_update_thread, pu_scenario_api.get()));
    }
    else
    {
        // launch a scenario updater
        threads_.push_back(new boost::thread(context_utils::launch_mock_scenario_update_thread, pu_scenario_api.get()));
    }
}

//void OfdmTransceiver::set_channel()
//{
//    
//}


// This function creates new frames and pushes them on a shared buffer
void OfdmTransceiver::modulation_function(void)
{
    ModulationSearchApi &mod_api = ModulationSearchApi::getInstance(); //There is probably a better place to initialize this, it will be here for now.
    boost::this_thread::sleep(boost::posix_time::seconds(2));
    try {
        unsigned char header[8];
        unsigned char payload[MAX_PAYLOAD_LEN];
        memset(payload, 0, MAX_PAYLOAD_LEN);

        while (true) {
            boost::this_thread::interruption_point();

            // Check if params_.change_mod_period ms passed.
            if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_mod_change).count()) > params_.change_mod_period)
            {
                modulation_scheme mod_scheme = ModulationSearchApi::getInstance().changeOfdmMod();
                fgprops.mod_scheme = mod_scheme;
                last_mod_change = std::chrono::system_clock::now();
                #ifdef DEBUG_MODE
                	std::cout << __FUNCTION__ << ": " << "Changing Modulation to " << modulation_types[mod_scheme].name << std::endl;
                #endif
            }

            if(power_controller)
            {
                int new_gain = power_controller->CCompute(current_gain);

                if(new_gain != current_gain)
                {
                    cout << "Changing Powers! " << new_gain << endl;
                    set_tx_gain_uhd(new_gain);
                    current_gain = new_gain;
                }
            }
            
            channel_hopper->work();
            if(channel_hopper->current_channel != current_channel)
            {
                current_channel = channel_hopper->current_channel;
                cout << "Changed to channel: " << current_channel << endl;
                reconfigure_usrp(current_channel);
            }   
            // write header (first four bytes sequence number, remaining are random)
            // TODO: also use remaining 4 bytes for payload
            header[0] = (seq_no_ >> 24) & 0xff;
            header[1] = (seq_no_ >> 16) & 0xff;
            header[2] = (seq_no_ >> 8 ) & 0xff;
            header[3] = (seq_no_      ) & 0xff;
            for (int i = 4; i < 8; i++)
                header[i] = rand() & 0xff;

            int actual_payload_len = payload_len_;
            if (params_.use_db)
            {
                // get packet from database
                // FIXME: payload_len must not be smaller than the size requested during init
                spectrum_eror_t ret = spectrum_getPacket(tx_, payload, payload_len_, -1);
                spectrum_errorToText(tx_, ret, error_buffer, sizeof(error_buffer));
                if (ret < 0) {
                    cout << boost::format("Error: %s") % error_buffer << endl;
                    throw std::runtime_error("Couldn't connect to challenge database");
                }
                // getPacket returns actual length
                actual_payload_len = ret;
            } else {
                // fill with dummy payload
                for (int i = 0; i < actual_payload_len; i++)
                    payload[i] = rand() & 0xff;
            }

            ofdmflexframegen_setprops(fg, &fgprops);

            // assemble frame
            ofdmflexframegen_assemble(fg, header, payload, actual_payload_len);
            if (params_.debug)
                ofdmflexframegen_print(fg);

            size_t num_symbols = ofdmflexframegen_getframelen(fg);
            const size_t frame_size = num_symbols * fgbuffer_len;

            // create fresh buffer for this frame
            boost::shared_ptr<CplxFVec> usrp_buffer( new CplxFVec(frame_size) );
            unsigned int bytes_written = 0;
            while (num_symbols--) {
                //ofdmflexframegen_writesymbol(fg, fgbuffer);
                ofdmflexframegen_write(fg, fgbuffer, fgbuffer_len);
                // copy symbol and apply gain
                for (int i = 0; i < fgbuffer_len; i++)
                    (*usrp_buffer.get())[bytes_written + i] = fgbuffer[i] * tx_gain;
                bytes_written += fgbuffer_len;
            }
            assert(bytes_written == frame_size);
            frame_buffer.pushBack(usrp_buffer);

            if (params_.debug)
                cout << boost::format("TX frame %6u (%d Bytes)") % seq_no_ % actual_payload_len << endl;

            seq_no_++;
        }
    }
    catch(boost::thread_interrupted)
    {
        cout << "Modulation thread interrupted." << endl;
    }
}

/*
void OfdmTransceiver::change_ofdm_mod()
{

    float current_score = DatabaseApi::getInstance().current_score();

	static modulation_scheme mod_scheme = static_cast<modulation_scheme>(fgprops.mod_scheme);
	switch (mod_scheme)
    {

		case LIQUID_MODEM_BPSK:
			mod_scheme = modulation_scheme::LIQUID_MODEM_QPSK;
			break;
		case LIQUID_MODEM_QPSK:
			mod_scheme = modulation_scheme::LIQUID_MODEM_QAM4;
			break;
		case LIQUID_MODEM_QAM4:
			mod_scheme = modulation_scheme::LIQUID_MODEM_QAM8;
			break;
		case LIQUID_MODEM_QAM8:
			mod_scheme = modulation_scheme::LIQUID_MODEM_QAM16;
			break;
		case LIQUID_MODEM_QAM16:
			mod_scheme = modulation_scheme::LIQUID_MODEM_QAM32;
			break;
		case LIQUID_MODEM_QAM32:
			mod_scheme = modulation_scheme::LIQUID_MODEM_QAM64;
			break;
		case LIQUID_MODEM_QAM64:
			mod_scheme = modulation_scheme::LIQUID_MODEM_QAM128;
			break;
		case LIQUID_MODEM_QAM128:
			mod_scheme = modulation_scheme::LIQUID_MODEM_QAM256;
			break;
		case LIQUID_MODEM_QAM256:
			mod_scheme = LIQUID_MODEM_BPSK;
			break;

		default:
			throw std::runtime_error("Unknown modulation scheme requested");
    }

	fgprops.mod_scheme = mod_scheme;

#ifdef DEBUG_MODE
	std::cout << __FUNCTION__ << ": " << "Changing Modulation to " << modulation_types[mod_scheme].name << std::endl;
#endif
}
*/
void OfdmTransceiver::transmit_function(void)
{
  try {
    boost::this_thread::interruption_point();
    while (true) {
      transmit_packet();
    }
  }
  catch(boost::thread_interrupted)
  {
    cout << "Transmit thread interrupted." << endl;
  }
}


void OfdmTransceiver::random_transmit_function(void)
{
    try {
      while (true) {
        boost::this_thread::interruption_point();

        // get random channel
        if (next_channel == 9) 
        {
          int num = rand() % channels_.size();
          reconfigure_usrp(num, false);
        }

        transmit_packet();
        // boost::this_thread::sleep(boost::posix_time::milliseconds(500));
      }
    }
    catch(boost::thread_interrupted)
    {
        cout << "Random transmit thread interrupted." << endl;
    }
}

// change USRP frequency
void OfdmTransceiver::reconfigure_usrp(const int num, bool tune_lo)
{
    static vector<int> channel_map = {3, 1, 0, 2};
    // construct tuning request
    current_channel = num;
    int internal_num = num;

    if (channels_.size() > 1)
        internal_num = channel_map[num];

    if(su_params_api)   // sets the SU-TX channel so the sensing sees it
        su_params_api->set_channel(num);
    
    uhd::tune_request_t request;
    // don't touch RF part

    if (tune_lo)
    {
        request.rf_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
        request.rf_freq = channels_.at(internal_num).rf_freq;
    }
    else
    {
        request.rf_freq_policy = uhd::tune_request_t::POLICY_NONE;
        request.rf_freq = 0;
    }

    // only tune DSP frequency
    request.dsp_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
    request.dsp_freq = channels_.at(internal_num).dsp_freq;
    request.args = uhd::device_addr_t("mode_n=integer");
    uhd::tune_result_t result = usrp_tx->set_tx_freq(request);

    if (params_.debug)
    {
        cout << result.to_pp_string() << endl;
    }
}

// COMMENT: To many payload invalid with this "new" transmit_packet()
 void OfdmTransceiver::transmit_packet()
 {
   boost::shared_ptr<CplxFVec> buffer;
    frame_buffer.popFront(buffer);

    //setup metadata for the first packet
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = false;
    md.has_time_spec = false;

    //the first call to send() will block this many seconds before sending:
    double timeout = 1.5; //std::max(0 seconds_in_future) + 0.1; //timeout (delay before transmit + padding)

    const size_t spb = tx_streamer->get_max_num_samps();
    size_t total_num_samps = buffer->size();
    size_t num_acc_samps = 0; //number of accumulated samples
    while (num_acc_samps < total_num_samps)
    {
        size_t samps_to_send = total_num_samps - num_acc_samps;
        if (samps_to_send > spb)
        {
            samps_to_send = spb;
        }
        else
        {
            md.end_of_burst = true;
        }

        //send a single packet
        size_t num_tx_samps = tx_streamer->send(
                                                &buffer->front() + num_acc_samps, samps_to_send, md, timeout
                                                );

        //do not use time spec for subsequent packets
        md.has_time_spec = false;
        md.start_of_burst = false;

        if (num_tx_samps < samps_to_send)
        {
            std::cerr << "Send timeout..." << endl;
        }
        //cout << boost::format("Sent packet: %u samples of %u") % num_tx_samps % total_num_samps << endl;

        num_acc_samps += num_tx_samps;
    }
 }


//void OfdmTransceiver::transmit_packet()
//{
//    // set up the metadata flags
//    metadata_tx.start_of_burst = false; // never SOB when continuous
//    metadata_tx.end_of_burst   = false; //
//    metadata_tx.has_time_spec  = false; // set to false to send immediately
//
//    boost::shared_ptr<CplxFVec> buffer;
//    frame_buffer.popFront(buffer);
//
//    // send samples to the device
//    usrp_tx->get_device()->send(
//        &buffer->front(), buffer->size(),
//        metadata_tx,
//        uhd::io_type_t::COMPLEX_FLOAT32,
//        uhd::device::SEND_MODE_FULL_BUFF
//    );
//
//    // send a few extra samples and EOB to the device
//    // NOTE: this seems necessary to preserve last OFDM symbol in
//    //       frame from corruption
//    metadata_tx.start_of_burst = false;
//    metadata_tx.end_of_burst   = true;
//    CplxFVec dummy(NUM_PADDING_NULL_SAMPLES);
//    usrp_tx->get_device()->send(
//        &dummy.front(), dummy.size(),
//        metadata_tx,
//        uhd::io_type_t::COMPLEX_FLOAT32,
//        uhd::device::SEND_MODE_FULL_BUFF
//    );
//}
 
void OfdmTransceiver::launch_change_places()
{
    for(;;)
    {
        boost::this_thread::interruption_point();

        // get data from socket. may block until result arrives
        //buffer_utils::rdataset<ChPowers> dset;
        //e_detec.pop_result(dset);
        std::vector<float> ch_powers;

        // call the CHAAANGE PLACES
        process_sensing(ch_powers);
    }
}

void OfdmTransceiver::process_sensing(std::vector<float> ChPowers)
{
    //e_detec.noise_filter->filter(ChPowers);

    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-last_ch_tstamp;
    if(elapsed_seconds.count()>2)
    {
        cout << "time elapsed " << elapsed_seconds.count() << endl;
        cout << "CHAAAAAAAAAAANGE PLACES!" << endl;
        cout << "Changing Powers! " << current_gain << endl;

        // channel map will be defined by learning code
        constexpr int channel_map[] = {2, 0, 3, 1};

        last_ch_tstamp = std::chrono::system_clock::now();
        
        short ch = channel_map[current_channel]%channels_.size();
        reconfigure_usrp(ch);
        //reconfigure_usrp(current_channel);

        cout << "Current Challenge Score: " << DatabaseApi::getInstance().current_score() << endl;
        cout << "Current Challenge Scenario: " << pu_scenario_api->PU_scenario_idx() << endl;
    }
}

// set transmitter frequency
void OfdmTransceiver::set_tx_freq(float freq)
{
    cout << boost::format("Setting TX Center Frequency: %f") % freq << endl;
    uhd::tune_request_t request(freq);
    uhd::tune_result_t result = usrp_tx->set_tx_freq(request);

    if (params_.debug) {
        cout << result.to_pp_string() << endl;
    }
    cout << "Actual TX Frequency: " << (usrp_tx->get_tx_freq()/1e6) << " MHz" << endl;
}

// set transmitter sample rate
void OfdmTransceiver::set_tx_rate(float rate)
{
    //set transmit parameter
    cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << endl;
    usrp_tx->set_tx_rate(rate);
    cout << boost::format("Actual TX Rate: %f Msps...") % (usrp_tx->get_tx_rate()/1e6) << endl << endl;
}

// set transmitter software gain
void OfdmTransceiver::set_tx_gain_soft(float gain)
{
    tx_gain = powf(10.0f, gain/20.0f);
}

// set transmitter hardware (UHD) gain
void OfdmTransceiver::set_tx_gain_uhd(float gain)
{
    cout << boost::format("Setting TX Gain %f") % gain << endl;
    usrp_tx->set_tx_gain(gain);
    cout << "Actual TX gain: " << usrp_tx->get_tx_gain() << endl;
}

// set transmitter antenna
void OfdmTransceiver::set_tx_antenna(char * _tx_antenna)
{
    usrp_tx->set_tx_antenna(_tx_antenna);
    cout << "Using TX Antenna: " << usrp_tx->get_tx_antenna() << endl;
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
    cout << "Actual RX Frequency: " << (usrp_rx->get_rx_freq()/1e6) << " MHz" << endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(500));

    // check LO Lock
    std::vector<std::string> sensor_names;
    sensor_names = usrp_rx->get_rx_sensor_names(0);
    if(std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end())
    {
        uhd::sensor_value_t lo_locked = usrp_rx->get_rx_sensor("lo_locked",0);
        cout << "Checking RX: " << lo_locked.to_pp_string() <<  "..." << endl;
        if(!lo_locked.to_bool())
            cout << "Failed to lock LO" << endl;
    }
}

// set receiver sample rate
void OfdmTransceiver::set_rx_rate(float _rx_rate)
{
    usrp_rx->set_rx_rate(_rx_rate);
    cout << "Actual RX Rate: " << (usrp_rx->get_rx_rate()/1e6) << " Msps" << endl;
}

// set receiver hardware (UHD) gain
void OfdmTransceiver::set_rx_gain_uhd(float _rx_gain_uhd)
{
    usrp_rx->set_rx_gain(_rx_gain_uhd);
    cout << "Actual RX gain: " << usrp_rx->get_rx_gain() << " dB" << endl;
}

// set receiver antenna
void OfdmTransceiver::set_rx_antenna(const std::string& _rx_antenna)
{
    usrp_rx->set_rx_antenna(_rx_antenna.c_str());
    cout << "Using RX Antenna: " << usrp_rx->get_rx_antenna() << endl;
}

// reset receiver objects and buffers
void OfdmTransceiver::reset_rx()
{
    //ofdmflexframesync_reset(fs);
}

void OfdmTransceiver::set_channel(uint32_t num)
{
  // channel 9 means random hopping after each packet
  if (num == 9) {
    cout << "Switching Tx mode to random hopping." << endl;
    next_channel = num;
  } else if (num < channels_.size()) {
    cout << "Tuning to channel " << num << endl;
    reconfigure_usrp(num);
  } else {
    cout << "Unknown channel " << num << endl;
  }
}
