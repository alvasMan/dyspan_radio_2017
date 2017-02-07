/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "json_utils.h"
#include <cassert>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <fstream>

using namespace std;
using namespace nlohmann;

string return_current_time_and_date()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    return ss.str();
}


void TrainingJsonManager::read()
{
    std::ifstream fptr(r_file);
    if(fptr.is_open()==false)
        throw std::runtime_error("Could not open read json file " + r_file);
    if(fptr.peek() == std::ifstream::traits_type::eof()) // if it is empty
    {
        j_read["scenario_data"] = json::array();
        j_read["new_data"] = json::array();
    }
    else
    {
        fptr >> j_read;
    }
    is_read_flag = true;
    merge_new_data();
}

json merge_two_json_entries(json& jscenario, json& jnew, vector<int> channel_occupancy)
{
    json j;
    
    for(json::iterator it = jnew.begin(); it != jnew.end(); ++it)
    {
        std::unique_ptr<JsonScenarioMonitor> monitor_ptr = monitor_utils::make_scenario_monitor(it.key());
        if(jscenario.find(monitor_ptr->json_key())!=jscenario.end())
            monitor_ptr->from_json(jscenario[monitor_ptr->json_key()]);
        monitor_ptr->merge_json(jnew[monitor_ptr->json_key()], channel_occupancy);
        
        j[monitor_ptr->json_key()] = monitor_ptr->to_json();
    }
    
    return j;
}

void TrainingJsonManager::merge_new_data()
{
    assert(is_read_flag==true);
    j_write["scenario_data"] = j_read["scenario_data"];
    j_write["new_data"] = json::array();
    
    for(auto& new_el : j_read["new_data"])
    //for(auto it = j_read["new_data"].begin(); it != j_read["new_data"].end(); ++it)
    {
        bool scenario_defined = new_el["scenario"].is_number();
        vector<int> channel_occupancy = new_el["channel_occupancy"].get<vector<int>>();
        bool channels_defined = find_if(channel_occupancy.begin(), channel_occupancy.end(),[](int d){return d<0;})==channel_occupancy.end();
        if(scenario_defined==true && channels_defined==true)
        {// it is valid and ready to merge
            int scenario_idx = new_el["scenario"].get<int>();
            auto it2 = find_if(j_write["scenario_data"].begin(), j_write["scenario_data"].end(), 
                 [&](decltype(*j_write["scenario_data"].begin())& a)
                 {
                    return a["scenario"].get<int>()==scenario_idx;
                 });
            if(it2 != j_write["scenario_data"].end()) // these is an entry already
            {
                (*it2)["monitor_data"] = merge_two_json_entries((*it2)["monitor_data"], new_el["monitor_data"], channel_occupancy);
            }
            else // create a new one
            {
                json jempty;
                json jnew_entry = {{"scenario",scenario_idx},
                    {"monitor_data",merge_two_json_entries(jempty, new_el["monitor_data"], channel_occupancy)}
                };
                j_write["scenario_data"].push_back(jnew_entry);
            }
        }
        else
            j_write["new_data"].push_back(new_el);
    }
}

// JSON FILE
// "new_data": Where I am going to insert the freshly captured data from the environment
//
// "scenario_data": After manually altering the scenario number and occupied channels, the "new_data" will move to "scenario_data"
//                  and used to configure the algorithms
//
//
void TrainingJsonManager::write(json& j)
{
    std::vector<int> v({-1,-1,-1,-1});
    json j_new = {
        {"current_time", return_current_time_and_date()},
        {"scenario", "unknown"},
        {"channel_occupancy", v},
        {"monitor_data", j}
    };
    
    // read a JSON file
    std::ofstream ofptr(w_file);
    if(ofptr.is_open()==false)
    {
        throw std::runtime_error("ERROR1: SavingLearningToFile: Unable to open file");
    }
    
    j_write["new_data"].push_back(j_new);
    
    ofptr << std::setw(4) << j_write;
    ofptr.close();
    cout << "STATUS: Successfully wrote learned data to json file " << w_file << endl;
}

// TODO: Move data in json to monitor testers