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
#include "markov_chain_components.h"

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::chrono::system_clock;
using namespace nlohmann;
using boost::asio::ip::tcp;


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

    //auto maskprops = sensing_utils::generate_bin_mask_and_reference(Nch, Nfft, 0.8, 0.12);
    auto maskprops = sensing_utils::generate_bin_mask_no_guard(64, Nfft);
    maskprops2 = sensing_utils::generate_bin_mask_and_reference(Nch, 64, 0.75, 0.125, false, false);
    cout << "Mask1: " << print_range(maskprops) << endl;
    cout << "Mask2: " << print_range(maskprops2) << endl;

    pwr_estim.set_parameters(1, maskprops);
    packet_detector = PacketDetector(Nch, 9, 12);
    rate_monitor = ChannelPacketRateMonitor(Nch);

    if(json_read_filename != "" && json_write_filename != "")
    {
        string learning_folder = project_folder+"learning_files/";
        json_learning_manager.reset(new TrainingJsonManager(learning_folder + json_read_filename, learning_folder + json_write_filename));
        json_learning_manager->read(); // reads the config file
    }

    spectrogram_buffers = &bufs;
}

void LearningThreadHandler::run(uhd::usrp::multi_usrp::sptr& usrp_tx)
{
    auto t1 = system_clock::now();
    vector<int> ch_counter(Nch,0);
    vector<float> second_pwrs(maskprops2.n_sections(),0);

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

            //vector<float> ch_pwrs = sensing_utils::relative_channel_powers(pwr_estim.bin_mask, pwr_estim.output_ch_pwrs);
            sensing_utils::apply_bin_mask(&second_pwrs[0], &pwr_estim.output_ch_pwrs[0], maskprops2);
            vector<float> ch_pwrs = sensing_utils::relative_channel_powers(maskprops2, second_pwrs);

            // Discover packets through a moving average
            packet_detector.work(pwr_estim.current_tstamp, ch_pwrs);

            rate_monitor.work(packet_detector.detected_pulses, pwr_estim.current_tstamp);
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

//    auto maskprops = sensing_utils::generate_bin_mask_and_reference(Nch, Nfft, 0.8, 0.12);
//    pwr_estim.set_parameters(1, maskprops);
//    assert(maskprops.bin_mask.size()==Nfft);

    auto maskprops = sensing_utils::generate_bin_mask_no_guard(64, Nfft);
    maskprops2 = sensing_utils::generate_bin_mask_and_reference(Nch, 64, 0.75, 0.125, false, false);
    cout << "Mask1: " << print_range(maskprops) << endl;
    cout << "Mask2: " << print_range(maskprops2) << endl;

    pwr_estim.set_parameters(1, maskprops);

    // Set USRP to FFT writer
    // not yet

    // Setup Packet Detector
    packet_detector = PacketDetector(Nch, 9, 8);

    // Setup Channel Packet Arrival Rate Estimator
    float avg_pkt_interval_ms = 7.5;
    float time_window_ms = 200;//250;//500;
    rate_monitor = TimedChannelPacketRateMonitor(Nch,time_window_ms);//SlidingChannelPacketRateMonitor(Nch,round(time_window_ms/avg_pkt_interval_ms));
    //rate_monitor = SlidingChannelPacketRateMonitor(Nch,round(time_window_ms/avg_pkt_interval_ms));
    
    // Setup Forgetful Channel Energy Monitor
    pwr_monitor = ForgetfulChannelMonitor(Nch, 0.0001);

    // This will guess the scenario based on packet arrival rate
    channel_rate_tester = ChannelPacketRateTester(pu_scenario_api);

    su_api = su_params;
    spectrogram_buffers = &bufs;
}

// safter margin for configuration to take place
constexpr float TIME_THRES = 0.015;

class SelfInterferenceCanceller
{
public:
    std::deque<std::pair<time_format,short>> channels_queue;
    
    void work(short new_ch, time_format tstamp, vector<float>& ch_pwrs)
    {
        while(!channels_queue.empty() and (tstamp-channels_queue.front().first) > TIME_THRES)
            channels_queue.pop_front();
        
        if(new_ch >= 0)
        {
            if(channels_queue.empty() || new_ch != channels_queue.back().second)
                channels_queue.push_back(std::make_pair(tstamp, new_ch));
            for(auto& e : channels_queue)
                ch_pwrs[e.second] = -1;
        }
    }
};

void SensingThreadHandler::run(uhd::usrp::multi_usrp::sptr& usrp_tx)
{
    auto t1 = system_clock::now();
    
    short new_channel = -1;
    short old_channel = 0;
    bool in_transition = false;
    time_format last_hop_tstamp = -1;
    SelfInterferenceCanceller canceller;
    
    scenario_number_type old_scenario_number = -1;
    scenario_number_type old_expanded_scenario_number = -1;
    time_format last_tstamp = -1000000;
    vector<int> ch_counter(Nch,0);
    vector<float> second_pwrs(maskprops2.n_sections(),0);
    vector<long> scenario_counter(11,0);

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
            
            // removes that leakage
            float min_val = *min_element(pwr_estim.output_ch_pwrs.begin(), pwr_estim.output_ch_pwrs.end());
            pwr_estim.output_ch_pwrs[0] = min_val;
            pwr_estim.output_ch_pwrs.back() = min_val;
            
            //vector<float> ch_snrs = sensing_utils::relative_channel_powers(pwr_estim.bin_mask, pwr_estim.output_ch_pwrs);
            
            sensing_utils::apply_bin_mask(&second_pwrs[0], &pwr_estim.output_ch_pwrs[0], maskprops2);
            vector<float> ch_snrs = sensing_utils::relative_channel_powers(maskprops2, second_pwrs);
            
            if(su_api)
            {
                su_api->set_clock(pwr_estim.current_tstamp);
                // set the PU channel to -1 just to ignore it
                new_channel = su_api->channel();
//                if(new_channel >= 0)
//                {
//                    if(new_channel != old_channel)
//                    {
//                        if(in_transition == false)
//                        {
//                            in_transition = true;
//                            last_hop_tstamp = pwr_estim.current_tstamp;
//                            ch_snrs[old_channel] = -1;
//                        }
//                        else if(pwr_estim.current_tstamp-last_hop_tstamp < TIME_THRES)
//                        {
//                            ch_snrs[old_channel] = -1;
//                        }
//                        else
//                        {
//                            old_channel = new_channel;
//                            in_transition = false;
//                        }
//                    }
//                    ch_snrs[new_channel] = -1;
//                }
                canceller.work(new_channel, pwr_estim.current_tstamp, ch_snrs);
            }
            
            // Discover packets through a moving average
            packet_detector.work(pwr_estim.current_tstamp, ch_snrs);
            
//            cout << "Current forbidden channels: " << new_channel << "," << old_channel << endl;
//            cout << "Detected pulses: " << print_range(packet_detector.detected_pulses, [](std::tuple<double,int,float> p)
//            {
//                return std::get<1>(p);
//            });
            
            rate_monitor.work(packet_detector.detected_pulses, pwr_estim.current_tstamp);
            
            pwr_monitor.work(ch_snrs);//shandler->pwr_estim->output_ch_pwrs);//ch_snr);
            for(auto &e : packet_detector.detected_pulses)
                ch_counter[std::get<1>(e)]++;
            
            push_detected_packets(packet_detector.detected_pulses, pu_api);
            
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
                auto expanded_scenario_pairs = channel_rate_tester.possible_expanded_scenarios(&rate_monitor,new_channel);
                auto possible_scenario_numbers = channel_rate_tester.possible_scenario_idxs(expanded_scenario_pairs);
                //cout << "scenario idxs: " << possible_scenario_numbers[0] << ", " << expanded_scenario_pairs[0].first << endl;
                scenario_counter[possible_scenario_numbers[0]]++;
                // If update, update the API
                if(old_expanded_scenario_number != expanded_scenario_pairs[0].first)
                {
                    //cout << "DEBUG: oldSchool: Scenario " << possible_scenario_numbers[0] << endl;
                    old_scenario_number = possible_scenario_numbers[0];
                    pu_api->set_PU_scenario(old_scenario_number);
                    old_expanded_scenario_number = expanded_scenario_pairs[0].first;
                    pu_api->set_PU_expanded_scenario(old_expanded_scenario_number);
                    cout << "occupied channels: " << print_range(pu_api->expanded_scenarios.scenarios_expanded_list[old_expanded_scenario_number].ch_occupied_mask) 
                         << ", exp. scenario: " << old_expanded_scenario_number << ", scenario: " << possible_scenario_numbers[0] << endl;
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
                auto max_it = std::max_element(scenario_counter.begin(),scenario_counter.end());
                long sum = std::accumulate(scenario_counter.begin(),scenario_counter.end(),0);
                vector<float> noise_pwr;
                for(auto& e : packet_detector.params)
                    noise_pwr.push_back(e.noise_floor);
                cout << "> Noise Floor per Channel: \t\t" << print_range(noise_pwr, ",\t", [](float f){return 10*log10(f);}) << endl;
                cout << "> Total number of detected per channel: \t" << print_range(ch_counter) << endl;
                cout << "> Transient number of detected per channel: \t" << print_range(rate_monitor.time_and_intervals,[](const boost::circular_buffer<pair<time_format,time_format>>& a){return a.size();}) << endl;
                cout << "> OldSchool most visited scenario: " << distance(scenario_counter.begin(),max_it) << ", rate: " << *max_it/(double)sum << endl;
                //cout << "STATUS: Scenario " << old_scenario_number << endl;
//                vector<float> d(512);
//                for(int i = 0; i < 512; ++i)
//                    d[i] = 10*log10(abs(pwr_estim.fftBins[(i+256)%512]));
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
    long max_written = std::ceil(60.0 / (512/10.0e6)) + skip_n;//std::ceil(120.0 / (512/10.0e6)) + skip_n;
    
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
            //sp_resizer.resize_line(pwr_outputs, section_powers().second);

            // i can't max to 1 here

            n_ffts_read++;
            if(n_ffts_read>=skip_n)
            {
                // cut the corner
//                section_powers().second[0] = *min_element(section_powers().second.begin(), section_powers().second.end());

                // write to file
                of.write((char*)&section_powers().second[0], section_powers().second.size()*sizeof(float));
                //of.write((char*)&pwr_outputs[0], pwr_outputs.size()*sizeof(float));
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
                                                                 const BinMask& bmask, pair<int,int> CNN_dim, int step_size)
//: sp_resizer(bmask, CNN_dim.second)
{
    pu_api = api;
    sensing_buffer = buf;
    CNNdims = CNN_dim;
    mat = Matrix<float>(CNN_dim.first, CNN_dim.second);
    //sp_resizer = SpectrogramResizer(bmask, mat.cols());
    moving_average_step = step_size;

    float spectrogram_duration_ms = 50;
    float time_window_ms = 250;//500;
    mode_counter = markov_utils::make_deeplearning_mode_counter(time_window_ms, spectrogram_duration_ms, 10);//pu_api->environment_data->scenario_list.size());

//    // setup connection to caffe
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(tcp::v4(), "127.0.0.1", "12345");
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    soc.reset(new socket_type(io_service));
    boost::asio::connect(*soc, endpoint_iterator);
    assert(soc.get()!=nullptr);

//    const boost::asio::local::stream_protocol::endpoint ep("/tmp/caffe");
//    socket_ptr soc(new socket_type(io_service));
//    soc->connect(ep);

    cout << "STATUS: Connection with Caffe was established" << endl;
}

void Spectrogram2SocketThreadHandler::run_send()
{
    assert(soc.get()!=nullptr);
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
            //sp_resizer.resize_line(pwr_outputs, section_powers().second);
            
            float min_val = *min_element(section_powers().second.begin(), section_powers().second.end());
            section_powers().second[0] = min_val;
            section_powers().second.back() = min_val;
            
            // Averaging
            for(int i = 0; i < mat.cols(); ++i)
                sum_powers[i] += section_powers().second[i];

            // if not output available restart loop
            if(++cur_idx < moving_average_step)
                continue;
            cur_idx = 0;

            // convert to dB
            for(int i = 0; i < mat.cols(); ++i)
                mat.at(current_row,i) = 10*log10(sum_powers[i]);
            
//            for(int j = 0; j < mat.rows(); ++j)
//                if(mat.at(current_row,j) < mat.at(current_row,0))
//                    mat.at(current_row,0) = mat.at(current_row,j);
//            mat.at(current_row,mat.rows()-1) = mat.at(current_row,0);
            
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

pair<size_t,vector<float>> recv_predictions(socket_type& soc)
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
    assert(soc.get()!=nullptr);
    std::chrono::system_clock::time_point  t2, tprev, t1, tprint;
    size_t imgno;
    scenario_number_type old_scenario_number = -1;
    vector<long> count_scenarios(10,0);
    vector<long> count_mode_scenarios(10,0);

    // loop
    tprint = std::chrono::system_clock::now();
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

            mode_counter.push(scen);
            scenario_number_type scen_avg = mode_counter.current_state();
            count_mode_scenarios[scen_avg]++;

            tprint = std::chrono::system_clock::now();
            if(old_scenario_number != scen_avg || std::chrono::duration_cast<std::chrono::seconds>(tprint - tprev).count() > 2)
            {
                cout << "\nDEBUG: DeepLearning: Scenario " << scen_avg << ". Statistics:"
                     << "\n> Single spectrogram+caffe counts: \t" << print_range(count_scenarios)
                     << "\n> Mode spectrogram+caffe counts: \t" << print_range(count_mode_scenarios) << endl;
                auto max_it = std::max_element(count_mode_scenarios.begin(),count_mode_scenarios.end());
                long sum = std::accumulate(count_mode_scenarios.begin(),count_mode_scenarios.end(),0);
                cout << "> DeepLearning most visited scenario: " << distance(count_mode_scenarios.begin(),max_it) << ", rate: " << *max_it/(double)sum << endl;
                cout << "> Last probability vector: " << print_range(pred.second) << endl;
                old_scenario_number = scen_avg;
                pu_api->set_PU_scenario(scen_avg);
                tprev = tprint;
            }
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "STATUS: Deep learning receive thread successfully interrupted." << std::endl;
    }
}
