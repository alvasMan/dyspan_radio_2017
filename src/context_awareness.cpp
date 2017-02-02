/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "context_awareness.h"
#include <iostream>
#include <fstream>
#include <string>
#include <iterator>     // std::back_inserter
#include <algorithm>    // std::copy
#include <tuple>
#include <stdexcept>
#include <boost/tokenizer.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

using namespace std;

ScenarioDescriptor tokenize_to_scenario(const string& line)
{
    ScenarioDescriptor s;
    
    boost::tokenizer<boost::escaped_list_separator<char> > tok(line);
    
    auto it = tok.begin();
    // read channel width
    s.channel_width = atoi(it->c_str());
    // read num of channels
    s.n_visited_channels = atoi((++it)->c_str());
    // read delay
    s.packet_delay = atoi((++it)->c_str());
    // poisson distribution?
    s.poisson_flag = *(++it)=="poisson";
    
    return s;
}

tuple<int8_t, float, vector<float> > read_PU_params_from_file(const string& filename)
{
    int8_t num_channels = -1;
    float bw = -1;
    vector<float> delays;

    std::ifstream fptr;
    fptr.open(filename);
    
    if (fptr.is_open()==false)
        throw std::exception();//return make_tuple(num_channels,bw,delays);
    
    std::string line;
    while(std::getline(fptr, line))
    {
        if(line[0]=='#')    // comments
            continue;
        boost::tokenizer<boost::escaped_list_separator<char> > tok(line);
        
        auto it = tok.begin();
        if(*it=="delays")
        {
            for(++it; it != tok.end(); ++it)
                delays.push_back(atof(it->c_str()));
            sort(delays.begin(), delays.end());
        }
        else if(*it=="num_channels")
        {
            num_channels = atoi((++it)->c_str());
        } else if(*it=="channel_bandwidth")
        {
            bw = atof((++it)->c_str());
        }
    }
    fptr.close();
    
    if(num_channels == -1 || bw == -1 || delays.size() == 0)
        throw std::exception();
    
    return make_tuple(num_channels,bw,delays);
}

vector<ScenarioDescriptor> read_scenarios_from_file(const std::string& filename)
{
    std::vector<ScenarioDescriptor> scenario_list;
    std::ifstream fptr;
    fptr.open(filename);
    
    if (fptr.is_open()==false)
        throw std::exception();//return {};
    
    std::string line;
    while(std::getline(fptr, line))
    {
        if(line[0]=='#')    // comments
            continue;
        scenario_list.push_back(tokenize_to_scenario(line));
    }
    fptr.close();
    
    return scenario_list;
}

string search_project_folder()
{
    boost::filesystem::path full_path( boost::filesystem::current_path() );
    //cout << "Current path is : " << full_path << endl;
    
    // check if in src folder
    string str = full_path.string();
    if(str.find("src"))
    {
        full_path /= "../../";
        //cout << "Current path is : " << full_path << endl;
        return full_path.string();
    }
    
    // check if scenarios
    // TODO:
    
    throw std::exception();
}

namespace context_utils
{
unique_ptr<RFEnvironmentData> make_rf_environment()
{
    // Read from the scenario files in folder scenario/
    // Note: I am ashamed of how I programmed this one
    string project_folder;
    try
    {
        project_folder = search_project_folder();
    }
    catch(std::exception())
    {
        cout << "Could not find scenarios folder" << endl;
        throw std::exception();
    }
    string params_file = project_folder + "scenarios/PU_params.txt";
    string scenarios_file = project_folder + "scenarios/scenario_list.txt";

    // Create the environment structure
    std::unique_ptr<RFEnvironmentData> env;

    try
    {
        int8_t num_ch;
        float bw;
        vector<float> delays;
        std::tie(num_ch, bw, delays) = read_PU_params_from_file(params_file);
        auto scenarios = read_scenarios_from_file(scenarios_file);
        env.reset(new RFEnvironmentData(num_ch, bw, delays, scenarios));
    }
    catch (std::exception &e)
    {
        cout << "ERROR: There was an issue while parsing the PU files" << endl;
        throw std::exception();
    }

    return std::move(env);
}

vector<int8_t> find_free_channels(SituationalAwarenessApi& api)
{
    vector<int8_t> channels = api.stats.occupied_channels();
    return context_utils::find_free_channels(channels, api.get_environment()->num_channels);
}

void launch_mock_scenario_update_thread(SituationalAwarenessApi* scenario_api)
{
    int scenario_idx = 0;
    while(true)
    {
        scenario_idx = (scenario_idx + 1) % scenario_api->get_environment()->scenario_list.size();
        cout << "DEBUG: Gonna change the scenario to " << scenario_idx << endl;
        scenario_api->set_PU_scenario(scenario_idx);
        
        boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
    }
}
};