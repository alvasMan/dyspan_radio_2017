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
#include "SU_parameters.h"

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::chrono::system_clock;
using namespace nlohmann;
using boost::asio::ip::tcp;

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
    auto packet_detector = PacketDetector(shandler->Nch, 9, 12);
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
            cout << "THIS IS DEPRECATED" << endl;
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
            if(false && (!packet_detector.detected_pulses.empty() || (shandler->pwr_estim->current_tstamp-last_tstamp)>2))
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
                cout << "STATUS: Scenario " << old_scenario_number << endl;
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

//void launch_deep_learning_client(SensingHandler* shandler)
//{
//    std::vector<int> NNdims = {64,64};
//    SpectrogramResizer sp_resizer(shandler->spectrogram_module->bin_mask, NNdims[1]);
//    
//    // loop
//    try 
//    {
//        std::vector<float> resized_pwr_outputsdB(NNdims[1]);
//        while(true)
//        {
//            boost::this_thread::interruption_point();
//            
//            //blocks waiting
//            buffer_utils::rdataset<ChPowers>  ch_powers = shandler->spectrogram_module->pop_line();
//            
//            // convert to dB ----> Only after the averaging
////            for(int i = 0; i < ch_powers().second.size(); ++i)
////                if(ch_powers().second[i]!=0)
////                    ch_powers().second[i] = 10*log10(ch_powers().second[i]);
////                else
////                    ch_powers().second[i] = -240;
//            
//            
////            if(ch_powers.empty())
////                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
//            
//            assert(ch_powers().second.size()>0);
//            
//            //cout << "DEBUG: reading from e_detec timestamp: " << ch_powers().first << " buffer: " << print_range(ch_powers().second) << endl;
//            
//            // resize to NN dimensions
//            sp_resizer.resize_line(resized_pwr_outputsdB, ch_powers().second);
//            //resize_ch_pwr_outputsdB(resized_pwr_outputsdB, ch_powers().second);
//            
//            // i can't max to 1 here
//
//            n_ffts_read++;
//            if (n_ffts_read >= skip_n)
//            {
//                // write to file
//                of.write((char*) &resized_pwr_outputsdB[0], resized_pwr_outputsdB.size() * sizeof (float));
//                if (n_ffts_read >= max_written)
//                {
//                    cout << "STATUS: Spectrogram successfully written to file." << endl;
//                    break;
//                }
//            }
//        }
//    }
//    catch(boost::thread_interrupted)
//    {
//        std::cout << "STATUS: Spectrogram to file thread successfully interrupted." << std::endl;
//    }
//}

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

void LearningThreadHandler::setup_filepaths(const string& folder_name, const string& json_rfile, const string& json_wfile)
{
    project_folder = folder_name;
    json_read_filename = json_rfile;
    json_write_filename = json_wfile;
}

void LearningThreadHandler::setup(vector<std::unique_ptr<buffer_utils::bounded_buffer<ChPowers>>>& bufs, int _Nch, int Nfft)
{
    Nch = _Nch;
    
    auto maskprops = sensing_utils::generate_bin_mask_and_reference(Nch, Nfft, 0.8, 0.12);
    
    pwr_estim.set_parameters(1, maskprops);
    packet_detector = PacketDetector(Nch, 9, 12);
    rate_monitor = ChannelPacketRateMonitor(Nch, 0.1);
    
    if(json_read_filename != "" && json_write_filename != "")
    {
        string learning_folder = project_folder+"learning_files/";
        json_learning_manager.emplace(learning_folder + json_read_filename, learning_folder + json_write_filename);
        json_learning_manager->read(); // reads the config file
    }
        
    spectrogram_buffers = &bufs;
}

void LearningThreadHandler::run(uhd::usrp::multi_usrp::sptr& usrp_tx)
{      
    auto t1 = system_clock::now();
    vector<int> ch_counter(Nch,0);
    
    usrp_reader.setup(usrp_tx, true);
    usrp_reader.start();
    
    try
    {
        while (true)
        {
            boost::this_thread::interruption_point();
            
            // receive data from USRP and place it in the buffer
            if(usrp_reader.recv_block(&pwr_estim[0], pwr_estim.nBins, pwr_estim.current_tstamp)==false)
            {
                cout << "ERROR: Could not read samples from USRP" << endl;
                break;
            }
            
            pwr_estim.process(pwr_estim.current_tstamp);
            
            vector<float> ch_pwrs = sensing_utils::relative_channel_powers(pwr_estim.bin_mask, pwr_estim.output_ch_pwrs);
            
            // Discover packets through a moving average
            packet_detector.work(pwr_estim.current_tstamp, ch_pwrs);
            
            rate_monitor.work(packet_detector.detected_pulses);
            for(auto &e : packet_detector.detected_pulses)
                ch_counter[std::get<1>(e)]++;
            
            // move data to file or caffe
            for(auto&e : *spectrogram_buffers)
            {
                auto wdst = e->get_wdataset();
                wdst().first = pwr_estim.current_tstamp;
                wdst().second = pwr_estim.output_ch_pwrs;
            }
            
            // clear the just detected packets
            packet_detector.detected_pulses.clear();
            
            // Print to screen
            auto t2 = system_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() > 2)
            {
                cout << "STATUS: Packet Arrival Periods per Channel: " << monitor_utils::print_packet_period(rate_monitor) << endl;
                //cout << "STATUS: Packet Power per Channel: " << print_range(pwr_monitor.channel_energy, [](float f){return 10*log10(f);}) << endl;
                vector<float> noise_pwr;
                for(auto& e : packet_detector.params)
                    noise_pwr.push_back(e.noise_floor);
                cout << "STATUS: Noise Floor per Channel: " << print_range(noise_pwr, [](float f){return 10*log10(f);}) << endl;
                cout << "STATUS: Number of detected per channel: " << print_range(ch_counter) << endl;
                t1 = t2;
            }
        }
    }
    catch(boost::thread_interrupted)
    {
        cout << "STATUS: Learning Thread was interrupted." << endl;
    }
    
    if(json_learning_manager) // write to json file
    {
        json j = {{rate_monitor.json_key(),rate_monitor.to_json()}};
        json_learning_manager->write(j);
    }
}

void SensingThreadHandler::setup(SituationalAwarenessApi *pu_scenario_api, SU_tx_params* su_params, 
                                 vector<std::unique_ptr<buffer_utils::bounded_buffer<ChPowers>>>& bufs, int _Nch, int Nfft)
{
    Nch = _Nch;
    pu_api = pu_scenario_api;
    
    // Set ChannelPowerEstimator
    //auto maskprops = sensing_utils::generate_bin_mask(shandler.Nch, Nfft, 0.8);
    auto maskprops = sensing_utils::generate_bin_mask_and_reference(Nch, Nfft, 0.8, 0.12);
    pwr_estim.set_parameters(1, maskprops);
    assert(maskprops.bin_mask.size()==Nfft);    
    
    // Set USRP to FFT writer
    // not yet
    
    // Setup Packet Detector
    packet_detector = PacketDetector(Nch, 9, 12);
    
    // Setup Channel Packet Arrival Rate Estimator
    rate_monitor = ChannelPacketRateMonitor(Nch, 0.1);
    
    // Setup Forgetful Channel Energy Monitor
    pwr_monitor = ForgetfulChannelMonitor(Nch, 0.0001);
    
    // This will guess the scenario based on packet arrival rate
    channel_rate_tester = ChannelPacketRateTester(pu_scenario_api);
    
    su_api = su_params;
    spectrogram_buffers = &bufs;
}

void SensingThreadHandler::run(uhd::usrp::multi_usrp::sptr& usrp_tx)
{
    auto t1 = system_clock::now();
    
    short current_channel = -1;
    scenario_number_type old_scenario_number = -1;
    time_format last_tstamp = -1000000;
    vector<int> ch_counter(Nch,0);
    
    usrp_reader.setup(usrp_tx);
    usrp_reader.start();
    
    try
    {
        while (true)
        {
            boost::this_thread::interruption_point();
            
            // receive data from USRP and place it in the buffer
            if(usrp_reader.recv_block(&pwr_estim.fftBins[0], pwr_estim.nBins, pwr_estim.current_tstamp)==false)
            {
                cout << "ERROR: Could not read samples from USRP" << endl;
                break;
            }
            
            pwr_estim.process(pwr_estim.current_tstamp);
            
            vector<float> ch_snrs = sensing_utils::relative_channel_powers(pwr_estim.bin_mask, pwr_estim.output_ch_pwrs);
            assert(ch_snrs.size()==Nch);
            
            if(su_api)
            {   // set the PU channel to -1 just to ignore it
                current_channel = su_api->channel();       
                ch_snrs[current_channel] = -1;
            }
            
            // Discover packets through a moving average
            packet_detector.work(pwr_estim.current_tstamp, ch_snrs);
            
            rate_monitor.work(packet_detector.detected_pulses);
            
            pwr_monitor.work(ch_snrs);//shandler->pwr_estim->output_ch_pwrs);//ch_snr);
            for(auto &e : packet_detector.detected_pulses)
                ch_counter[std::get<1>(e)]++;
            
            // move data to file or caffe
            for(auto&e : *spectrogram_buffers)
            {
                auto wdst = e->get_wdataset();
                wdst().first = pwr_estim.current_tstamp;
                wdst().second = pwr_estim.output_ch_pwrs;
            }
            
            // Check the list of possible scenarios
            if((!packet_detector.detected_pulses.empty() || (pwr_estim.current_tstamp-last_tstamp)>2))
            {
                auto possible_scenario_numbers = channel_rate_tester.possible_scenario_idxs(&rate_monitor, current_channel);
            
                // If update, update the API
                if(old_scenario_number != possible_scenario_numbers[0])
                {
                    cout << "DEBUG: New scenario " << possible_scenario_numbers[0] << endl;
                    old_scenario_number = possible_scenario_numbers[0];
                    pu_api->set_PU_scenario(old_scenario_number);
                }
                last_tstamp = pwr_estim.current_tstamp;
            }
            
            
            // clear the just detected packets
            packet_detector.detected_pulses.clear();
            
                        // Print to screen
            auto t2 = system_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() > 2)
            {
                cout << "\nSTATUS: Sensing Statistics"
                     << "\n> Packet Arrival Periods per Channel: \t" << monitor_utils::print_packet_period(rate_monitor)
                     << "\n> Packet Arrival Variances per Channel: " << monitor_utils::print_packet_delay_variance(rate_monitor)
                     << "\n> Packet Power per Channel: \t\t" << print_range(pwr_monitor.channel_energy, ",\t", [](float f){return 10*log10(f);}) << endl;
                vector<float> noise_pwr;
                for(auto& e : packet_detector.params)
                    noise_pwr.push_back(e.noise_floor);
                cout << "> Noise Floor per Channel: \t\t" << print_range(noise_pwr, ",\t", [](float f){return 10*log10(f);}) << endl;
                cout << "> Number of detected per channel: \t" << print_range(ch_counter) << endl;
                //cout << "STATUS: Scenario " << old_scenario_number << endl;
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
        cout << "STATUS: Sensing Thread was interrupted." << endl;
    }
}

Spectrogram2FileThreadHandler::Spectrogram2FileThreadHandler(buffer_utils::bounded_buffer<ChPowers>* buf, 
                                                             const BinMask& bmask, pair<int,int> CNN_dim, int step_size):
sp_resizer(bmask, CNN_dim.second)
{
    sensing_buffer = buf;
    CNNdims = CNN_dim;
    mat = Matrix<float>(CNN_dim.first, CNN_dim.second);
    mov_avg = VectorMovingAverage(mat.cols(), step_size, step_size);
}

void Spectrogram2FileThreadHandler::run()
{
    std::string filename = "/home/connect/repo/generated_files/temp.bin";
    
    std::ofstream of;
    of.open(filename, std::ios::out | std::ios::binary);
    
    long n_ffts_read = 0;
    long skip_n = std::ceil(1.0 / (512/10.0e6));
    long max_written = std::ceil(120.0 / (512/10.0e6)) + skip_n;
    
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
        std::vector<float> pwr_outputs(CNNdims.second);
        while(true)
        {
            boost::this_thread::interruption_point();
            
            //blocks waiting
            buffer_utils::rdataset<ChPowers>  section_powers = sensing_buffer->get_rdataset();
            
            // resize to NN dimensions
            sp_resizer.resize_line(pwr_outputs, section_powers().second);
            
            // i can't max to 1 here
            
            n_ffts_read++;
            if(n_ffts_read>=skip_n)
            {    
                // write to file
                of.write((char*)&pwr_outputs[0], pwr_outputs.size()*sizeof(float));
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

Spectrogram2SocketThreadHandler::Spectrogram2SocketThreadHandler(SituationalAwarenessApi* api, 
                                                                 buffer_utils::bounded_buffer<ChPowers>* buf, 
                                                                 const BinMask& bmask, pair<int,int> CNN_dim, int step_size) :
sp_resizer(bmask, CNN_dim.second)
{
    pu_api = api;
    sensing_buffer = buf;
    CNNdims = CNN_dim;
    mat = Matrix<float>(CNN_dim.first, CNN_dim.second);
    //sp_resizer = SpectrogramResizer(bmask, mat.cols());
    moving_average_step = step_size;
    
    // setup connection to caffe
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(tcp::v4(), "127.0.0.1", "12345");
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    soc.reset(new boost::asio::ip::tcp::socket(io_service));
    boost::asio::connect(*soc, endpoint_iterator);
}

void Spectrogram2SocketThreadHandler::run_send()
{
    vector<float> pwr_outputs(mat.cols());
    vector<uint8_t> buffer;
    buffer.reserve(mat.cols()*mat.rows()+sizeof(current_imgno));
    float current_max = -std::numeric_limits<float>::max();
    float current_min = std::numeric_limits<float>::max();
    assert(current_max<0);
    vector<float> sum_powers(64,0);
    size_t cur_idx = 0;

    // loop
    try 
    {
        while(true)
        {
            boost::this_thread::interruption_point();
            
            //blocks waiting
            buffer_utils::rdataset<ChPowers>  section_powers = sensing_buffer->get_rdataset();
            
            // resize to NN dimensions
            sp_resizer.resize_line(pwr_outputs, section_powers().second);
            
            // Averaging
            for(int i = 0; i < mat.cols(); ++i)
                sum_powers[i] += pwr_outputs[i];
            
            // if not output available restart loop
            if(++cur_idx < moving_average_step)
                continue;
            cur_idx = 0;

            // convert to dB
            for(int i = 0; i < mat.cols(); ++i)
                mat.at(current_row,i) = 10*log10(sum_powers[i]);
            sum_powers.clear();
            sum_powers.resize(64,0);
            
            if(++current_row == mat.rows())
            {
                // copy img no
                buffer.resize(sizeof(current_imgno));
                memcpy(&buffer[0], (void*)&current_imgno, sizeof(current_imgno));
                
                // compute max/min
                auto it = max_element(mat.begin(), mat.end());
                current_max = *it;
                it = min_element(mat.begin(), mat.end());
                current_min = *it;
                
                // normalize
                float range = current_max-current_min;
                //assert(mat.total_size()==64*64);
                for(auto &val : mat)
                    buffer.push_back((uint8_t)round((val-current_min)*255/range));

                {
                    auto wdst = time_buffer.get_wdataset();
                    wdst().first = current_imgno;
                    wdst().second = std::chrono::system_clock::now(); // timestamp
                } // release
                
                // send to caffe
                //cout << "DEBUG: number of bytes sent to socket: " <<  buffer.size() << endl;
                boost::asio::write(*soc, boost::asio::buffer(&buffer[0], buffer.size()));
                
                // reset
                current_max = -std::numeric_limits<float>::max();
                current_min = std::numeric_limits<float>::max();
                current_imgno++;
                current_row = 0;
                buffer.clear();
            }
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "STATUS: Deep learning transmit thread successfully interrupted." << std::endl;
    }
}

pair<size_t,vector<float>> recv_predictions(tcp::socket& soc)
{
    size_t siz_el = sizeof(float);//sizeof(short)+;
    boost::system::error_code ec;
    size_t img_no;
    pair<size_t, vector<float>> ret;
    short Nclasses;
    
    boost::asio::read(soc, boost::asio::buffer(&ret.first,sizeof(img_no)), ec);
    if(ec)
        throw boost::system::system_error(ec);
    boost::asio::read(soc, boost::asio::buffer(&Nclasses,sizeof(Nclasses)), ec);
    if(ec)
        throw boost::system::system_error(ec);
    
    ret.second.resize(Nclasses);
    size_t toread = Nclasses*siz_el;
    vector<uint8_t> vec(toread);
    boost::asio::read(soc, boost::asio::buffer(&vec[0], toread), ec);
    if(ec)
        throw boost::system::system_error(ec);
    for (int i = 0; i < Nclasses; ++i)
    {
        //pair<short,float> p;
        //memcpy(&p.first, &vec[i*siz_el], sizeof(short));
        memcpy(&ret.second[i], &vec[i*siz_el], sizeof(float));
        //ret.second.push_back(p);
    }
    return ret;
}

void Spectrogram2SocketThreadHandler::run_recv()
{
    std::chrono::system_clock::time_point  t2, tprev, t1;
    size_t imgno;
    scenario_number_type old_scenario_number = -1;
    vector<long> count_scenarios(10,0);
    
    // loop
    tprev = std::chrono::system_clock::now();
    try 
    {
        while(true)
        {
            boost::this_thread::interruption_point();
            
            auto rdst = time_buffer.get_rdataset();
            imgno = rdst().first;
            t1 = rdst().second;
            
            auto pred = recv_predictions(*soc);
            t2 = std::chrono::system_clock::now();
            assert(rdst().first == pred.first);
            
            auto it = max_element(pred.second.begin(), pred.second.end());
            scenario_number_type scen = std::distance(pred.second.begin(), it);
            count_scenarios[scen]++;
            
            if(old_scenario_number != scen || std::chrono::duration_cast<std::chrono::seconds>(t2 - tprev).count() > 2)
            {
                cout << "DEBUG: Scenario " << scen << endl;
                cout << "DEBUG: Counts: " << print_range(count_scenarios) << endl;
                old_scenario_number = scen;
                pu_api->set_PU_scenario(scen);
                tprev = std::chrono::system_clock::now();
            }
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "STATUS: Deep learning receive thread successfully interrupted." << std::endl;
    }
}