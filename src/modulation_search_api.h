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

#include <mutex>
#include <vector>
#include <liquid/liquid.h>

#ifndef MODULATION_SEARCH_API_H
#define MODULATION_SEARCH_API_H


// This is a Singleton class that will keep some basic info about the current tx status relevant for the adptative modulation.
// This includes tx gain information, current modulation scheme,
// This class will also provide algorithms to search for the best modulation for the current gain
class ModulationSearchApi
{
public:
    static ModulationSearchApi & getInstance()
    {
      static ModulationSearchApi instance;
      return instance;
    }

    //Linear search
    modulation_scheme changeOfdmMod();
    void changeOfdmModLinear();

    // Remove copy CTOR and assigment operator. This is a singleton after all.
    ModulationSearchApi(ModulationSearchApi const &) = delete;
    void operator=(ModulationSearchApi const &) = delete;

    // setters
    /*void push_Tsu_provided(const DbReply& r)
    {
        std::lock_guard<std::mutex> lk(mut);
        _Tsu_provided.push_back(r);
    }*/

private:
    std::mutex m_mut;
    bool m_stop_searching;
    bool m_gain_changed;
    float m_previous_tsu;
    std::vector<modulation_scheme>::const_iterator m_current_modulation;
    int m_current_usrp_gain;
    std::vector<modulation_scheme> m_modulation_list;

    // Singleton classics: private CTOR and pointer to instance
    ModulationSearchApi();
};
#endif /* DATABASE_H */
