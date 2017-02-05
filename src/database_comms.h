/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   database.h
 * Author: connect
 *
 * Created on 02 February 2017, 11:06
 */
#include <chrono>
#include <mutex>
#include <memory>
#include <boost/circular_buffer.hpp>
#include "spectrum.h"

#ifndef DATABASE_H
#define DATABASE_H


using std::chrono::duration;
//using std::chrono::high_resolution_clock;
using std::chrono::system_clock;

class DbReply
{
public:
    DbReply() : throughput(-1), tstamp(system_clock::now()) {}
    DbReply(float val) : throughput(val), tstamp(system_clock::now()) {}
    DbReply(float val, system_clock::time_point t) : throughput(val), tstamp(t) {}
    
    float throughput;
    system_clock::time_point tstamp;
};

class DatabaseApi
{
public:
    DatabaseApi(int cap = 10);

    // setters
    void push_Tsu_real(const DbReply& r)
    {
        std::lock_guard<std::mutex> lk(mut);
        Tsu_real_values.push_back(r);
    }
    void push_Tpu_real(const DbReply& r)
    {
        std::lock_guard<std::mutex> lk(mut);
        Tpu_real_values.push_back(r);
    }
    void push_Tsu(const DbReply& r)
    {
        std::lock_guard<std::mutex> lk(mut);
        Tsu_values.push_back(r);
    }
    void push_Tpu(const DbReply& r)
    {
        std::lock_guard<std::mutex> lk(mut);
        Tpu_values.push_back(r);
    }
    
    // getters
    float Tsu_real()
    {
        std::lock_guard<std::mutex> lk(mut);
        return Tsu_real_values.back().throughput;
    }
    float Tpu_real()
    {
        std::lock_guard<std::mutex> lk(mut);
        return Tpu_real_values.back().throughput;
    }
    float Tsu()
    {
        std::lock_guard<std::mutex> lk(mut);
        return Tsu_values.back().throughput;
    }
    float Tpu()
    {
        std::lock_guard<std::mutex> lk(mut);
        return Tpu_values.back().throughput;
    }
    float current_score()
    {
        std::lock_guard<std::mutex> lk(mut);
        if(Tpu_values.back().throughput == 0)
            return 0;
        else
            return std::exp(-10.0f*(1-Tpu_real_values.back().throughput/Tpu_values.back().throughput)) * Tsu_real_values.back().throughput;
    }
    
private:
    boost::circular_buffer<DbReply> Tsu_real_values; // i need a circular buffer to compute derivatives later
    boost::circular_buffer<DbReply> Tpu_real_values;
    boost::circular_buffer<DbReply> Tsu_values;
    boost::circular_buffer<DbReply> Tpu_values;
    
    std::mutex mut;
};

void launch_mock_database_thread(DatabaseApi* db_api);
void launch_database_thread(DatabaseApi* db_api, spectrum* spec, int radio_number);

#endif /* DATABASE_H */

