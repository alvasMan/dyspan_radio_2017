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
#include "matplotlibcpp.h"

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
        if(crash_on_overflow==true)
            throw std::runtime_error("Samples are corrupted now");
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

void SpectrogramResizer::setup()
{
    float mask_interp_factor = bin_mask.size() / Nout;
    
    int nBins = bin_mask.size();
    int n_cols = bin_mask.n_sections();
    vector<int> out_mat_count(Nout*n_cols,0);
    for(int i = 0; i < bin_mask.size(); ++i)
    {
        int shift_idx = (i+nBins/2)%nBins;
        if(bin_mask[i]<0)
            continue;
        int out_idx = std::floor(shift_idx/mask_interp_factor);
        out_mat_count[out_idx*n_cols + bin_mask[i]]++;
    }
    
    out_mat_frac = std::vector<float>(Nout*n_cols,0);
    for(int i = 0; i < Nout; ++i)
    {
        int tot = std::accumulate(&out_mat_count[i*n_cols],&out_mat_count[(i+1)*n_cols],0);
        if(tot>0)
            for(int j = 0; j < n_cols; ++j)
                out_mat_frac[i*n_cols+j] = out_mat_count[i*n_cols+j] / (float)tot;
    }
    for(int i = 0; i < Nout; ++i)
    {
        bool all_zeros = std::find_if(&out_mat_frac[i*n_cols], &out_mat_frac[(i+1)*n_cols], [](float f){return f>0;})==&out_mat_frac[(i+1)*n_cols];
        if(all_zeros)
        {
            int n = i;
            while(n<Nout && out_mat_frac[n*n_cols]==-1)
                n++;
            if(i>0 && n!=Nout)
            {
                for(int j = 0; j < Nout; ++j)
                {
                    out_mat_frac[i*n_cols+j] = (1-1/(2.0*n))*out_mat_frac[(i-1)*n_cols+j] + (1/(2.0*n))*out_mat_frac[n*n_cols+j];
                }
            }
        }
    }
}

void SpectrogramResizer::resize_line(vector<float>& out_vec, const vector<float>& in_vec)
{
    assert(out_vec.size()==Nout);
    assert(in_vec.size()==bin_mask.n_sections());
    
    int n_cols = bin_mask.n_sections();
    for(int i = 0; i < Nout; ++i)
    {
        out_vec[i] = 0;
        for(int j = 0; j < n_cols; ++j)
        {
            out_vec[i] += in_vec[j] * out_mat_frac[i*n_cols+j];
        }
    }
}

namespace sensing_utils
{
SensingHandler make_sensing_handler(int Nch, std::string project_folder, std::string json_read_filename,
                                             std::string json_write_filename, SituationalAwarenessApi *pu_scenario_api, 
                                    bool has_sensing, bool has_learning)
{
    SensingHandler shandler;
    
    shandler.Nch = Nch;
    
    if(has_sensing)
    {
        int moving_average_size = 1;
        int Nfft = 512;
        shandler.pwr_estim.reset(new ChannelPowerEstimator());
        //shandler.pwr_estim->set_parameters(moving_average_size, Nfft, shandler.Nch);
        //auto maskprops = sensing_utils::generate_bin_mask(shandler.Nch, Nfft, 0.8);
        auto maskprops = sensing_utils::generate_bin_mask_and_reference(shandler.Nch, Nfft, 0.8, 0.12);
        assert(maskprops.bin_mask.size()==Nfft);
        shandler.pwr_estim->set_parameters(moving_average_size, maskprops);
        //shandler.pwr_estim->set_parameters(16, 512, 4, 0.4, 0.4);//(150, num_channels, 512, 0.4);// Andre: these are the parameters of the sensing (number of averages,window step size,fftsize)
        
        // SETUP JSON LEARNER/READER
        std::string learning_folder = project_folder+"learning_files/";
        shandler.json_learning_manager.reset(new TrainingJsonManager(learning_folder + json_read_filename, learning_folder + json_write_filename));
        shandler.json_learning_manager->read(); // reads the config file
        
        // SETUP USRP Reader to PowerChannelEstimator input
        if(has_learning==false)
            shandler.sensing_module.reset(new SensingModule(shandler.pwr_estim.get()));
        else
            shandler.sensing_module.reset(new SensingModule(shandler.pwr_estim.get(),true));
        
        if(has_learning)
            shandler.spectrogram_module.reset(new SpectrogramGenerator(maskprops, moving_average_size));
        
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
    auto packet_detector = PacketDetector(shandler->Nch, 15, 10);
    std::unique_ptr<ChannelPacketRateMonitor> par_monitor(new ChannelPacketRateMonitor(shandler->Nch, 0.1));
    auto ch_monitor = ForgetfulChannelMonitor(shandler->Nch, 0.01);
    vector<int> ch_counter(shandler->Nch,0);
    
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
            
            vector<float> ch_pwrs = sensing_utils::relative_channel_powers(shandler->pwr_estim->bin_mask, shandler->pwr_estim->output_ch_pwrs);
            assert(ch_pwrs.size()==shandler->Nch);
            
            // Discover packets through a moving average
            packet_detector.work(shandler->pwr_estim->current_tstamp, ch_pwrs);
            
            par_monitor->work(packet_detector.detected_pulses);
            
            // Perform channel occupancy measurements
//            vector<float> ch_snr(shandler->Nch);
//            for(int i = 0; i < ch_snr.size(); ++i)
//                ch_snr[i] = shandler->pwr_estim->output_ch_pwrs[i] / packet_detector.params[i].noise_floor;
            ch_monitor.work(ch_pwrs);//shandler->pwr_estim->output_ch_pwrs);//ch_snr);
            for(auto &e : packet_detector.detected_pulses)
                ch_counter[std::get<1>(e)]++;
            
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
                cout << "STATUS: Packet Power per Channel: " << print_range(ch_monitor.channel_energy, [](float f){return 10*log10(f);}) << endl;
                vector<float> noise_pwr;
                for(auto& e : packet_detector.params)
                    noise_pwr.push_back(e.noise_floor);
                cout << "STATUS: Noise Floor per Channel: " << print_range(noise_pwr, [](float f){return 10*log10(f);}) << endl;
                cout << "STATUS: Number of detected per channel: " << print_range(ch_counter) << endl;
//                vector<float> d(512);
//                for(int i = 0; i < 512; ++i)
//                    d[i] = 10*log10(abs(shandler->pwr_estim->fftBins[(i+256)%512]));
//                matplotlibcpp::plot(d);
//                matplotlibcpp::show();
                //cout << "\n FFT input: " << print_container_dB(&shandler->pwr_estim->fftBins[0], &shandler->pwr_estim->fftBins[512]) << endl;
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
    
    scenario_number_type old_scenario_number = -1;
    shandler->sensing_module->setup_rx_chain(usrp_tx);
    auto packet_detector = PacketDetector(shandler->Nch, 15, 10);
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
            
            vector<float> ch_pwrs = sensing_utils::relative_channel_powers(shandler->pwr_estim->bin_mask, shandler->pwr_estim->output_ch_pwrs);
            
            
            // Discover packets through a moving average
            packet_detector.work(shandler->pwr_estim->current_tstamp, ch_pwrs);
            
            ch_monitor.work(ch_pwrs);
            par_monitor.work(packet_detector.detected_pulses);
            
            // Move data to spectrogram
            if(shandler->spectrogram_module)
                shandler->spectrogram_module->push_line(shandler->pwr_estim->current_tstamp, shandler->pwr_estim->output_ch_pwrs);
            
            // clear the just detected packets
            packet_detector.detected_pulses.clear();
            
            // Print to screen
            auto t2 = system_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() > 2)
            {
                cout << "STATUS: Packet Arrival Periods per Channel: " << monitor_utils::print_packet_period(par_monitor) << endl;
                cout << "STATUS: Packet Power per Channel: " << print_range(ch_monitor.channel_energy, [](float f){return 10*log10(f);}) << endl;
                cout << "STATUS: Buffer at " << shandler->spectrogram_module->results.estimated_size()*100.0f/shandler->spectrogram_module->results.capacity() << "%" << endl;
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
    SpectrogramResizer sp_resizer(shandler->spectrogram_module->bin_mask, NNdims[1]);
    
    std::ofstream of;
    of.open(filename, std::ios::out | std::ios::binary);
    
    long n_ffts_read = 0;
    long skip_n = std::ceil(1.0 / (shandler->pwr_estim->nBins/10.0e6));
    long max_written = std::ceil(120.0 / (shandler->pwr_estim->nBins/10.0e6)) + skip_n;
    
    cout << "Writing Spectrogram to file." << endl;
    cout << "Going to write a total of " << max_written << " fft lines" << endl;
    cout << "Going to skip the first " << skip_n << " fft lines" << endl;
    
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
            buffer_utils::rdataset<ChPowers>  ch_powers = shandler->spectrogram_module->pop_line();
            
            // convert to dB ----> Only after the averaging
//            for(int i = 0; i < ch_powers().second.size(); ++i)
//                if(ch_powers().second[i]!=0)
//                    ch_powers().second[i] = 10*log10(ch_powers().second[i]);
//                else
//                    ch_powers().second[i] = -240;
            
            
//            if(ch_powers.empty())
//                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            
            assert(ch_powers().second.size()>0);
            
            //cout << "DEBUG: reading from e_detec timestamp: " << ch_powers().first << " buffer: " << print_range(ch_powers().second) << endl;
            
            // resize to NN dimensions
            sp_resizer.resize_line(resized_pwr_outputsdB, ch_powers().second);
            //resize_ch_pwr_outputsdB(resized_pwr_outputsdB, ch_powers().second);
            
            // i can't max to 1 here
            
            n_ffts_read++;
            if(n_ffts_read>=skip_n)
            {    
                // write to file
                of.write((char*)&resized_pwr_outputsdB[0], resized_pwr_outputsdB.size()*sizeof(float));
                if(n_ffts_read>=max_written)
                {
                    cout << "STATUS: Spectrogram successfully written to file." << endl;
                    break;
                }
            }
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "STATUS: Spectrogram to file thread successfully interrupted." << std::endl;
    }
    
    of.close();
    cout << "STATUS: Closing thread dedicated to writing the spectrogram to a file." << endl;
}


};