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


// This is a Singleton class
class DatabaseApi
{
public:
    static DatabaseApi & getInstance()
    {
	static DatabaseApi instance(10);

	return instance;
    }

    // Remove copy CTOR and assigment operator. This is a singleton after all.
    DatabaseApi(DatabaseApi const &) = delete;
    void operator=(DatabaseApi const &) = delete;

    // setters
    void push_Tsu_provided(const DbReply& r)
    {
        std::lock_guard<std::mutex> lk(mut);
        _Tsu_provided.push_back(r);
    }
    void push_Tpu_provided(const DbReply& r)
    {
        std::lock_guard<std::mutex> lk(mut);
        _Tpu_provided.push_back(r);
    }
    void push_Tsu(const DbReply& r)
    {
        std::lock_guard<std::mutex> lk(mut);
        _Tsu.push_back(r);
    }
    void push_Tpu(const DbReply& r)
    {
        std::lock_guard<std::mutex> lk(mut);
        _Tpu.push_back(r);
    }
    
    // getters
    float Tsu_provided()
    {
        std::lock_guard<std::mutex> lk(mut);

        return _Tsu_provided.empty()? -1 : _Tsu_provided.back().throughput;
    }

    float Tpu_provided()
    {
        std::lock_guard<std::mutex> lk(mut);
        return _Tpu_provided.empty()? -1 : _Tpu_provided.back().throughput;
    }

    float Tsu()
    {
        std::lock_guard<std::mutex> lk(mut);
        return _Tsu.empty()? -1 : _Tsu.back().throughput;
    }

    float Tpu()
    {
        std::lock_guard<std::mutex> lk(mut);
        return _Tpu.empty()? -1 : _Tpu.back().throughput;
    }

    float current_score()
    {
        std::lock_guard<std::mutex> lk(mut);
        if(_Tpu.back().throughput == 0)
            return 0;
        else
            return std::exp(-10.0f*(1-_Tpu.back().throughput/_Tpu_provided.back().throughput)) * _Tsu.back().throughput;
    }
    
private:
    // i need a circular buffer to compute derivatives later
    boost::circular_buffer<DbReply> _Tsu_provided;
    boost::circular_buffer<DbReply> _Tpu_provided;
    boost::circular_buffer<DbReply> _Tsu;
    boost::circular_buffer<DbReply> _Tpu;
    
    std::mutex mut;

    // Singleton classics: private CTOR and pointer to instance
    DatabaseApi(int cap = 10);
};

void launch_mock_database_thread();
void launch_database_thread(spectrum* spec, int radio_number, unsigned int sleep_time, bool debug_print);

#endif /* DATABASE_H */

