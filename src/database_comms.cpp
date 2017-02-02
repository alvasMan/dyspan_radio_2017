/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "database_comms.h"
#include <boost/thread.hpp>

using std::cout;
using std::endl;

// NOTE: this is just for testing
void launch_mock_database_thread(DatabaseApi* db_api)
{
    float tsu = 0;
    float tpu = 0;
    while(true)
    {
//        std::cout << "DEBUG: Gonna update database score" << endl;
        db_api->push_Tsu(DbReply(tsu));
        db_api->push_Tsu_real(DbReply(tsu*0.9));
        db_api->push_Tpu(DbReply(tpu));
        db_api->push_Tpu_real(DbReply(tpu*0.9));
        
//        std::cout << "DEBUG: Updated database score" << endl;
        
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
        tsu+=2;
        tpu++;
    }
}

// TODO: Write it
void launch_database_thread(DatabaseApi* db_api, spectrum* spec)
{
    float tsu = 0;
    float tpu = 0;
    while(true)
    {
        // TODO: Query Database
        
        // TODO: Update throughputs
        db_api->push_Tsu(DbReply(tsu));
        db_api->push_Tsu_real(DbReply(tsu*0.9));
        db_api->push_Tpu(DbReply(tpu));
        db_api->push_Tpu_real(DbReply(tpu*0.9));
        
        // Is sleep needed?
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
    }
}