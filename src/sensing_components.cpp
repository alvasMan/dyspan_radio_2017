/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "general_utils.hpp"
#include "sensing_components.h"
#include <fstream>
#include <cmath>
#include <chrono>
#include <ctime>
#include <memory>

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::chrono::system_clock;
using namespace nlohmann;

void SensingModule::start()
{
    //setup streaming
    std::cout << std::endl;
    std::cout << boost::format("Begin streaming now ..") << std::endl;
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = 0; // continuous
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t();
    usrp_tx->issue_stream_cmd(stream_cmd);
}

bool SensingModule::recv_fft_pwrs()
{
    size_t num_rx_samps = rx_stream->recv(&(*pwr_estim)[0], pwr_estim->fft_size(), metadata, 5.0);

    //handle the error code
    if (metadata.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
    {
        std::cout << boost::format("Timeout while streaming") << std::endl;
        return false;
    }
    else if (metadata.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)
    {
        if (overflow_message)
        {
            overflow_message = false;
            std::cerr << boost::format("Got an overflow indication, please reduce sample rate.");
        }
        return true;
    }
    else if (metadata.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE)
    {
        throw std::runtime_error(str(boost::format(
                                                   "Unexpected error code 0x%x"
                                                   ) % metadata.error_code));
    }

    tspec = metadata.time_spec;
    current_timestamp = tspec.get_real_secs();

    // Always call process because we write on fftBins nBins samples from uhd (see above)
    pwr_estim->process(current_timestamp);
    
    return true;
}

void SensingModule::setup_rx_chain(uhd::usrp::multi_usrp::sptr utx)
{
    usrp_tx = utx;
    //create a receive streamer
    std::string wire_format("sc16");
    std::string cpu_format("fc32");
    uhd::stream_args_t stream_args(cpu_format, wire_format);
    rx_stream = usrp_tx->get_rx_stream(stream_args);
}

void resize_ch_pwr_outputsdB(std::vector<float>& out, const vector<float>& in)
{
    float interp_factor = out.size()/(float)in.size();
    
    //cout << "in size: " << in.size() << endl;
    
    for(int i = 0; i < out.size(); ++i)
    {
        float i_val = i/interp_factor;
        float i_floor = std::floor(i_val);
        float i_ceil = std::ceil(i_val);
        
        if(i_ceil!=in.size())
        {
            float alpha = (i_val-i_floor);
            out[i] = 10*log10((1-alpha)*in[i_floor] + alpha*in[i_ceil]);
        }
        else
            out[i] = in[i_floor];
    }
}


namespace sensing_utils
{
SensingHandler make_sensing_handler(int Nch, std::string project_folder, std::string json_read_filename,
                                             std::string json_write_filename, SituationalAwarenessApi *pu_scenario_api, 
                                    bool has_sensing, bool has_deep_learning)
{
    SensingHandler shandler;
    
    shandler.Nch = Nch;
    
    if(has_sensing)
    {
        int moving_average_size = 1;
        int Nfft = 512;
        shandler.pwr_estim.reset(new ChannelPowerEstimator());
        shandler.pwr_estim->set_parameters(moving_average_size, Nfft, shandler.Nch);
        //shandler.pwr_estim->set_parameters(16, 512, 4, 0.4, 0.4);//(150, num_channels, 512, 0.4);// Andre: these are the parameters of the sensing (number of averages,window step size,fftsize)
        
        // SETUP JSON LEARNER/READER
        std::string learning_folder = project_folder+"learning_files/";
        shandler.json_learning_manager.reset(new TrainingJsonManager(learning_folder + json_read_filename, learning_folder + json_write_filename));
        shandler.json_learning_manager->read(); // reads the config file
        
        // SETUP USRP Reader to PowerChannelEstimator input
        shandler.sensing_module.reset(new SensingModule(shandler.pwr_estim.get()));
        
        if(has_deep_learning)
            shandler.spectrogram_module.reset(new SpectrogramGenerator(shandler.Nch, moving_average_size));
        
        shandler.channel_rate_tester.reset(new ChannelPacketRateTester(pu_scenario_api));
    }
    
    shandler.pu_scenario_api = pu_scenario_api;
    
    return std::move(shandler);
}

void launch_sensing_thread(uhd::usrp::multi_usrp::sptr& usrp_tx, SensingHandler* shandler)
{
    auto t1 = system_clock::now();
    scenario_number_type old_scenario_number = -1;
    time_format last_tstamp = -1000000;
    
    shandler->sensing_module->setup_rx_chain(usrp_tx);
    auto packet_detector = PacketDetector(shandler->Nch, 15, 2);
    std::unique_ptr<ChannelPacketRateMonitor> par_monitor(new ChannelPacketRateMonitor(shandler->Nch, 0.1));
    
    // start streaming
    shandler->sensing_module->start();
    
    // loop
    try 
    {
        while (true)
        {
            boost::this_thread::interruption_point();
// receive data from USRP and place it in the buffer
            if(shandler->sensing_module->recv_fft_pwrs()==false)
                break;
            
            // Discover packets through a moving average
            packet_detector.work(shandler->pwr_estim->current_tstamp, shandler->pwr_estim->output_ch_pwrs);
            
            par_monitor->work(packet_detector.detected_pulses);
            
            // Check the list of possible scenarios
            if(!packet_detector.detected_pulses.empty() || (shandler->pwr_estim->current_tstamp-last_tstamp)>2)
            {
                auto possible_scenario_numbers = shandler->channel_rate_tester->possible_scenario_idxs(par_monitor.get());
            
                // If update, update the API
                if(old_scenario_number != possible_scenario_numbers[0])
                {
                    cout << "DEBUG: New scenario " << possible_scenario_numbers[0] << endl;
                    old_scenario_number = possible_scenario_numbers[0];
                    shandler->pu_scenario_api->set_PU_scenario(old_scenario_number);
                }
                last_tstamp = shandler->pwr_estim->current_tstamp;
            }
            
            // clear the just detected packets
            packet_detector.detected_pulses.clear();
            
            // Print to screen
            auto t2 = system_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() > 2)
            {
                cout << "STATUS: Packet Arrival Periods per Channel: " << monitor_utils::print_packet_period(*par_monitor) << endl;
                t1 = t2;
            }
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Sensing thread interrupted." << std::endl;
    }
}

void launch_learning_thread(uhd::usrp::multi_usrp::sptr& usrp_tx, SensingHandler* shandler)
{
    auto t1 = system_clock::now();
    
    shandler->sensing_module->setup_rx_chain(usrp_tx);
    auto packet_detector = PacketDetector(shandler->Nch, 15, 2);
    auto ch_monitor = ForgetfulChannelMonitor(shandler->Nch, 0.1);
    auto par_monitor = ChannelPacketRateMonitor(shandler->Nch, 0.1);
    
    // start streaming
    shandler->sensing_module->start();
    
    // loop
    try 
    {
        while (true)
        {
            boost::this_thread::interruption_point();

            // receive data from USRP and place it in the buffer
            if(shandler->sensing_module->recv_fft_pwrs()==false)
                break;
            
            // Discover packets through a moving average
            packet_detector.work(shandler->pwr_estim->current_tstamp, shandler->pwr_estim->output_ch_pwrs);
            
            // Perform channel occupancy measurements
            vector<float> ch_snr(shandler->Nch);
            for(int i = 0; i < ch_snr.size(); ++i)
                ch_snr[i] = shandler->pwr_estim->output_ch_pwrs[i] / packet_detector.params[i].noise_floor;
            ch_monitor.work(ch_snr);
            
            par_monitor.work(packet_detector.detected_pulses);
            
            // Analyse difference in TOAs
            // TODO: Store TDOAs and reference TOAs
            
            // Move data to spectrogram
            if(shandler->spectrogram_module)
                shandler->spectrogram_module->work(shandler->pwr_estim->current_tstamp, shandler->pwr_estim->output_ch_pwrs);
            
            packet_detector.detected_pulses.clear();
            
            auto t2 = system_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() > 2)
            {
//                std::vector<float> pars(par_monitor.Nch);
//                for(int i = 0; i < par_monitor.Nch; ++i)
//                    pars[i] = par_monitor.packet_arrival_rate(i);
//                cout << "DEBUG: Packet Arrival Rates: " << print_range(pars) << endl;
                cout << "STATUS: Packet Arrival Rates per Channel: " << monitor_utils::print_packet_rate(par_monitor) << endl;
                t1 = t2;
            }
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Sensing thread interrupted." << std::endl;
    }
    
    json j = {{par_monitor.json_key(),par_monitor.to_json()}};
    shandler->json_learning_manager->write(j);
}

// This thread just receives the samples coming from the SensingModule and saves them to a file
void launch_spectrogram_to_file_thread(SensingHandler* shandler)
{
    std::string filename = "/home/connect/repo/generated_files/temp.bin";
    std::vector<int> NNdims = {64,64};
    
    std::ofstream of;
    of.open(filename, std::ios::out | std::ios::binary);
    
    if(of.is_open()==false)
    {
        std::cout << "Spectrogram2FileThread: Unable to open file in " << filename << std::endl;
        return; // TODO: send a signal
    }
    // loop
    try 
    {
        std::vector<float> resized_pwr_outputsdB(NNdims[1]);
        while(true)
        {
            boost::this_thread::interruption_point();
            
            //blocks waiting
            buffer_utils::rdataset<ChPowers>  ch_powers = shandler->spectrogram_module->results.get_rdataset();
            
            if(ch_powers.empty())
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            
            assert(ch_powers().second.size()>0);
            
            //cout << "DEBUG: reading from e_detec timestamp: " << ch_powers().first << " buffer: " << print_range(ch_powers().second) << endl;
            
            // resize to NN dimensions
            resize_ch_pwr_outputsdB(resized_pwr_outputsdB, ch_powers().second);
            
            // i can't max to 1 here
            
            // write to file
            of.write((char*)&resized_pwr_outputsdB[0], resized_pwr_outputsdB.size()*sizeof(float));
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "STATUS: Spectrogram to file thread successfully interrupted." << std::endl;
    }
    
    of.close();
}


};