/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2017  Andre Puschmann, Francisco Paisana, Justin Tallon
 *
 * \section LICENSE
 *
 * This file is part of dyspanradio.
 *
 * dyspanradio is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * dyspanradio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include "modulation_search_api.h"
#include "database_comms.h"
#include <boost/thread.hpp>

using std::cout;
using std::endl;

ModulationSearchApi::ModulationSearchApi()
: m_first_lookup(true),
  m_stop_searching(false),
  m_gain_changed(false),
  m_mod_samples(6),
  m_sample_counter(0),
  m_this_mod_tsu_v(m_mod_samples),
  m_previous_mod_tsu_v(m_mod_samples),
  m_this_mod_tsu_offered_v(m_mod_samples),
  m_previous_mod_tsu_offered_v(m_mod_samples),
  m_modulation_list{
    //LIQUID_MODEM_QAM4,
    LIQUID_MODEM_QAM8,
    LIQUID_MODEM_QAM16,
    LIQUID_MODEM_QAM32,
    LIQUID_MODEM_QAM64,
    LIQUID_MODEM_QAM128,
    LIQUID_MODEM_QAM256}
{
    initializeSearch();
}

void
ModulationSearchApi::setGainChanged(bool gain_changed)
{
    cout<<"Gain chaned to " << gain_changed<<endl<<endl;
    m_gain_changed = gain_changed;
}

void
ModulationSearchApi::initializeSearch()
{
    m_current_modulation = m_modulation_list.begin();
    std::fill(m_this_mod_tsu_v.begin(),m_this_mod_tsu_v.end(),0);
    std::fill(m_previous_mod_tsu_v.begin(),m_previous_mod_tsu_v.end(),0);
    std::fill(m_this_mod_tsu_offered_v.begin(),m_this_mod_tsu_offered_v.end(),0);
    std::fill(m_previous_mod_tsu_offered_v.begin(),m_previous_mod_tsu_offered_v.end(),0);

    m_first_lookup = true;
    m_gain_changed = false;
    m_sample_counter=0;
    m_stop_searching=false;
    best_cal_stats = calibration_stats();
    return;
}

void
ModulationSearchApi::linearModSearch()
{
    //This function should only be called when all the samples have been collected

    ++m_current_modulation;

    float sum_of_tsu_this= accumulate(m_this_mod_tsu_v.begin(), m_this_mod_tsu_v.end(), 0.0);
    float sum_of_tsu_previous=accumulate(m_previous_mod_tsu_v.begin(), m_previous_mod_tsu_v.end(), 0.0);
    float sum_of_tsu_offered_this=accumulate(m_this_mod_tsu_offered_v.begin(), m_this_mod_tsu_offered_v.end(), 0.0);
    float sum_of_tsu_offered_previous=accumulate(m_previous_mod_tsu_offered_v.begin(), m_previous_mod_tsu_offered_v.end(), 0.0);

    float ratio_this_mod = sum_of_tsu_this/sum_of_tsu_offered_this;
    
    if(best_cal_stats.tsu < sum_of_tsu_this)
        best_cal_stats = calibration_stats(*(m_current_modulation-1),ratio_this_mod,sum_of_tsu_this);

    if( m_stop_searching == false )
    {
//        cout << "Comparing:" <<endl;
//        cout << "\t Sum TSU previous:" <<sum_of_tsu_previous<<endl;
//        cout << "\t Sum TSU this:" <<sum_of_tsu_this<<endl;
//        cout << "\t Ratio TSU this:" <<ratio_this_mod<<endl;

        // && ratio_this_mod > 0.6
        if ( (sum_of_tsu_this <= sum_of_tsu_previous)|| sum_of_tsu_this == 0) //End of constelation list, or previous mod was better
        {
            m_current_modulation-=2;
            m_stop_searching = true;
        }
        else
        {
            //++next_modulation;
            if (m_current_modulation == m_modulation_list.end())
            {
                --m_current_modulation;
                m_stop_searching = true;
            }
        }
    }
    //}
    m_previous_mod_tsu_v.assign(m_this_mod_tsu_v.begin(),m_this_mod_tsu_v.end());
    m_previous_mod_tsu_offered_v.assign(m_this_mod_tsu_offered_v.begin(),m_this_mod_tsu_offered_v.end());
    return;
}

std::tuple<bool,modulation_scheme,modulation_scheme,calibration_stats,calibration_stats>
ModulationSearchApi::changeOfdmMod()
{
//    std::cout << "Entering: " <<__FUNCTION__ << std::endl;
    calibration_stats ret_stats;
    float su_throughput = DatabaseApi::getInstance().Tsu(); //The SU throughput is averaged at 10ms at the moment.
    float su_provided = DatabaseApi::getInstance().Tsu_provided();
    if(su_provided == 0)
        return std::make_tuple(false,*m_current_modulation,*m_current_modulation,ret_stats,best_cal_stats); //No SU trhoughput offered, can't do anything with this.

    //bool gain_changed = HasGainChanged();
    std::vector<modulation_scheme>::const_iterator previous_modulation = m_current_modulation;

    if(m_gain_changed) //The gain has changed, we need to restart our search.
    {
        std::cout<<"Start New Mod Search"<<std::endl<<std::endl;
        initializeSearch();
    }
    else //The gain has not changed, carry on searching
    {
        if(m_sample_counter < m_mod_samples)
        {
            //collecting samples, don't do anything yet
//            cout << "\tCollecting: "<< endl;
//            cout << "\t"<<su_throughput<<" "<<su_provided<<" "<<m_sample_counter<<endl;
            m_this_mod_tsu_v[m_sample_counter] = su_throughput;
            m_this_mod_tsu_offered_v[m_sample_counter] = su_provided;
            //m_previous_tsu_provided_v[m_sample_counter] = su_throughput_provided;
            m_sample_counter++;
        }
        else
        {
            //Look at previous throughput and search for next modulation
//            cout << "\tLooking: "<<endl;
            float tsu = accumulate(m_this_mod_tsu_v.begin(),m_this_mod_tsu_v.end(),0.0);
            float tsu_offered = accumulate(m_this_mod_tsu_offered_v.begin(),m_this_mod_tsu_offered_v.end(),0.0);
            float tsu_ratio = tsu/tsu_offered;
            ret_stats = calibration_stats(*m_current_modulation,tsu,tsu_ratio);
            linearModSearch(); //this will increment the m_current_modulation when necessary
            m_sample_counter=0;
        }
    }

	return std::make_tuple(m_stop_searching, *m_current_modulation, *previous_modulation,ret_stats,best_cal_stats);
}
