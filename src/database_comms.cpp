/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "database_comms.h"
#include <boost/thread.hpp>

using std::cout;
using std::endl;

DatabaseApi::DatabaseApi(int cap): Tsu_real_values(cap),
	Tpu_real_values(cap),
	Tsu_values(cap),
	Tpu_values(cap)
{
}

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
void launch_database_thread(DatabaseApi* db_api, spectrum* spec, int radio_number)
{
    float tsu = 0;
    float tsu_provided = 0;

    float tpu = 0;
    float tpu_provided = 0;

    while(true)
    {
        // TODO: Query Database
        tpu = spectrum_getThroughput(spec, 0, -1);
        tpu_provided = spectrum_getProvidedThroughput(spec, 0, -1);

        tsu = spectrum_getThroughput(spec, radio_number, -1);
        tsu_provided = spectrum_getProvidedThroughput(spec, radio_number, -1);

        // TODO: Update throughputs
        db_api->push_Tsu(DbReply(tsu));
        db_api->push_Tsu_real(DbReply(tsu*0.9));

        db_api->push_Tpu(DbReply(tpu));
        db_api->push_Tpu_real(DbReply(tpu*0.9));
        
        // Is sleep needed?
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
    }
}
