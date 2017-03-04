/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "database_comms.h"
#include <boost/thread.hpp>
#include <chrono>
#include <boost/format.hpp>

using std::cout;
using std::endl;


// private CTOR
DatabaseApi::DatabaseApi(int cap):
        _Tsu_provided(cap),
	_Tpu_provided(cap),
	_Tsu(cap),
	_Tpu(cap)
{
}

// NOTE: this is just for testing
void launch_mock_database_thread()
{
    DatabaseApi &db_api = DatabaseApi::getInstance();

    float tsu = 0;
    float tpu = 0;

    try
    {
        while(true)
        {
            boost::this_thread::interruption_point();

    //        std::cout << "DEBUG: Gonna update database score" << endl;
            db_api.push_Tsu(DbReply(tsu));
            db_api.push_Tsu_provided(DbReply(tsu*1.1));
            db_api.push_Tpu(DbReply(tpu));
            db_api.push_Tpu_provided(DbReply(tpu*1.1));

    //        std::cout << "DEBUG: Updated database score" << endl;

            boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
            tsu+=2;
            tpu++;
        }
    }
    catch(boost::thread_interrupted)
    {
        cout << "STATUS: Database thread successfully interrupted." << endl;
    }
}

// TODO: Write it
void launch_database_thread(spectrum* spec, int radio_number, unsigned int sleep_time, bool debug_print)
{
    float print_interval_msec = 2000;
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point t2 = std::chrono::system_clock::now();
    DatabaseApi &db_api = DatabaseApi::getInstance();

    float tsu = 0;
    float tsu_provided = 0;

    float tpu = 0;
    float tpu_provided = 0;
    uint16_t average_time = sleep_time;

    try {
       while(true)
       {
          boost::this_thread::interruption_point();

          // TODO: Query Database
          tpu = spectrum_getThroughput(spec, 0, average_time);
          tpu_provided = spectrum_getProvidedThroughput(spec, 0, average_time);
          tsu = spectrum_getThroughput(spec, radio_number, average_time);
          tsu_provided = spectrum_getProvidedThroughput(spec, radio_number, average_time);


/*
#ifdef DEBUG
          std::cout << "Tsu: "      << tsu          << std::endl;
          std::cout << "Tsu_real: " << tsu_provided << std::endl;
          std::cout << "Tpu: "      << tpu          << std::endl;
          std::cout << "Tpu_real: " << tpu_provided << std::endl;
#endif
*/
          // TODO: Update throughputs
          db_api.push_Tsu(DbReply(tsu));
          db_api.push_Tsu_provided(DbReply(tsu_provided));

          db_api.push_Tpu(DbReply(tpu));
          db_api.push_Tpu_provided(DbReply(tpu_provided));

          // Is sleep needed?
          boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_time));

          t2 = std::chrono::system_clock::now();
          if(std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() > print_interval_msec)
          {
              //Get overall score (usefull for debug)
            float tpu_ever = spectrum_getThroughput(spec, 0, 120);
            float tpu_provided_ever = spectrum_getProvidedThroughput(spec, 0, 120);
            float tsu_ever = spectrum_getThroughput(spec, radio_number, 120);
            float score_ever = std::exp(-10.0f*(1-tpu_ever/tpu_provided_ever)) * tpu_ever;

            if(debug_print)
            {
                std::cout << "Statistics:" << std::endl;

                std::cout << "> Tsu [n,d,n/d]: " << boost::format("%|8t|[%4d, %|22t|%4d, %|34t|%8.2f]") % tsu % tsu_provided % (tsu/(float)tsu_provided) << std::endl;
                std::cout << "> Tpu [n,d,n/d]: " << boost::format("%|8t|[%4d, %|22t|%4d, %|34t|%8.2f]") % tpu % tpu_provided % (tpu/(float)tpu_provided) << std::endl;

                //std::cout << "> Tpu [n,d,n/d]: " << boost::format("%|8t|[%1%, %|22t|%2%, %|34t|%3%]") % tpu % tpu_provided % (tpu/(float)tpu_provided) << std::endl;
                std::cout << "> Score: \t\t\t\t" << db_api.current_score() << std::endl;
                std::cout << "> Score since last two minutes: \t\t\t\t" << score_ever << std::endl << std::endl;
            }
            t1=t2;
          }
       }
    }
    catch(boost::thread_interrupted)
    {
       cout << "STATUS: Database thread successfully interrupted." << endl;
    }
}
