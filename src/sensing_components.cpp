/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "general_utils.hpp"
#include "sensing_components.h"
#include <fstream>
#include <cmath>

using std::cout;
using std::endl;
using std::vector;

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

void launch_spectrogram_generator(uhd::usrp::multi_usrp::sptr& usrp_tx, ChannelPowerEstimator* pwr_estim)
{
    SensingModule s(pwr_estim);
    s.setup_rx_chain(usrp_tx);
    s.packet_detector.reset(new PacketDetector(pwr_estim->Nch, 15, 2));
    
    // start streaming
    s.start();
    
    // loop
    try 
    {
        while (true)
        {
            boost::this_thread::interruption_point();

            // receive data from USRP and place it in the buffer
            if(s.recv_fft_pwrs()==false)
                break;
            
            // Discover packets through a moving average
            s.packet_detector->work(pwr_estim->current_tstamp, pwr_estim->output_ch_pwrs);
            
            // Analyse difference in TOAs
            // TODO: Store TDOAs and reference TOAs
        }
    }
    catch(boost::thread_interrupted)
    {
        std::cout << "Sensing thread interrupted." << std::endl;
    }
}

// This thread just receives the samples coming from the SensingModule and saves them to a file
void launch_spectrogram_to_file_thread(ChannelPowerEstimator* pwr_estim)
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
            buffer_utils::rdataset<ChPowers>  ch_powers = pwr_estim->pop_result();
            
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