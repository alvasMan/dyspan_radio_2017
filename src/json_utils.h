/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   json_utils.h
 * Author: connect
 *
 * Created on 06 February 2017, 18:42
 */
#include "../utils/json.hpp"

#ifndef JSON_UTILS_H
#define JSON_UTILS_H


class JsonScenarioMonitor
{
public:
    virtual std::string json_key() = 0;
    virtual nlohmann::json to_json() = 0;
    virtual void from_json(nlohmann::json& j, std::vector<int> ch_occupancy = {}) = 0;
    virtual void merge_json(nlohmann::json& j2, std::vector<int> ch_occupancy = {}) = 0;
    virtual ~JsonScenarioMonitor() {}
};

class TrainingJsonManager
{
public:
    TrainingJsonManager() = default;
    TrainingJsonManager(const std::string& config_json, const std::string& new_json) : 
        w_file(new_json), r_file(config_json) {}
    std::string w_file; // TODO: DO THIS IN THE PROPER WAY
    std::string r_file;
    
    nlohmann::json j_read;
    nlohmann::json j_write;
    
    bool is_read_flag = false;
    
    void read();
    void merge_new_data();
    void write(nlohmann::json& j);
};


namespace monitor_utils
{
std::unique_ptr<JsonScenarioMonitor> make_scenario_monitor(const std::string& type);
};


#endif /* JSON_UTILS_H */

