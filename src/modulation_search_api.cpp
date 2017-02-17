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
: m_stop_searching(false),
  m_gain_changed(true),
  m_previous_tsu(0),
  m_modulation_list{
    LIQUID_MODEM_QAM4,
    LIQUID_MODEM_QAM8,
    LIQUID_MODEM_QAM16,
    LIQUID_MODEM_QAM32,
    LIQUID_MODEM_QAM64,
    LIQUID_MODEM_QAM128,
    LIQUID_MODEM_QAM256}
{
    m_current_modulation = m_modulation_list.begin();
}

modulation_scheme
ModulationSearchApi::changeOfdmMod()
{

    float su_throughput = DatabaseApi::getInstance().Tsu(); //The SU throughput is averaged at 10ms at the moment.
    float su_provided = DatabaseApi::getInstance().Tsu_provided();
    //bool gain_changed = HasGainChanged();
    std::vector<modulation_scheme>::const_iterator next_modulation;

    if(m_gain_changed) //The gain has changed, we need to restart our search.
    {
        next_modulation = m_modulation_list.begin();
        m_gain_changed = false;
    }
    else //The gain has not changed, carry on searching
    {
        next_modulation = m_current_modulation;
        if( m_stop_searching == false )
        {

            if (su_throughput <= m_previous_tsu) //End of constelation list, or previous mod was better
            {
                --next_modulation;
                m_stop_searching = true;
            }
            else
            {
                ++next_modulation;
                if (next_modulation == m_modulation_list.end())
                {
                    --next_modulation;
                    m_stop_searching = true;
                }
            }
        }
    }
    ///*
    std::cout << __FUNCTION__ << std::endl;
    std::cout << "\t" << "Previous Modulation: " << modulation_types[*m_current_modulation].name << std::endl;
    std::cout << "\t" << "Next Modulation: " << modulation_types[*next_modulation].name << std::endl;
    std::cout << "\t" << "Previous SU Throughput: " << m_previous_tsu << std::endl;
    std::cout << "\t" << "Database SU Throughput: " << su_throughput << std::endl;
    std::cout << "\t" << "Provided SU Throughput: " << su_provided << std::endl;
    //*/
    m_previous_tsu = su_throughput;
    m_current_modulation = next_modulation;
	return *next_modulation;
}
