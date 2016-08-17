/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2015  Andre Puschmann, Francisco Paisana, Justin Tallon
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

#ifndef CHANNELS_H
#define CHANNELS_H


#include <iostream>


typedef enum {
    CH1 = 0,
    CH2,
    CH3,
    CH4
} ChannelNum;


typedef struct {
    std::string desc;  // description
    double f_center;   // the actual center frequency of that channel (as seen on the spectrum analyzer)
    double bandwidth;  // the actual bandwidth of that channel
    double rf_freq;    // the RF frequency that the USRP is tuned to (LO)
    double dsp_freq;   // offset from the LO to reach the center frequency
    double rate;       // the sampling rate used in the channel
} ChannelConfig;



#endif // CHANNELS_H

