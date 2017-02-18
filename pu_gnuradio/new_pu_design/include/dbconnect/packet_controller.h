/* -*- c++ -*- */
/* 
 * Copyright 2017 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifndef INCLUDED_DBCONNECT_PACKET_CONTROLLER_H
#define INCLUDED_DBCONNECT_PACKET_CONTROLLER_H

#include <dbconnect/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace dbconnect {

    /*!
     * \brief <+description of block+>
     * \ingroup dbconnect
     *
     */
    //class DBCONNECT_API packet_controller : virtual public gr::sync_block
    class DBCONNECT_API packet_controller : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<packet_controller> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of dbconnect::packet_controller.
       *
       * To avoid accidental use of raw pointers, dbconnect::packet_controller's
       * constructor is in a private implementation
       * class. dbconnect::packet_controller::make is the public interface for
       * creating new instances.
       */
      static sptr make(float samp_rate, const std::vector<int> swtime, int delay_1, int delay_2, int tconst, int mean1, int mean2, int mean3, int seed, int gain_period, const std::vector<int> &gain_vals, const std::vector<int> &scen_list, bool rand_scen);
	  virtual void get_packet(pmt::pmt_t msg) = 0;
	  virtual void request_cmd(int pktcnt) = 0; 
	  virtual int  get_delay(int dist_type) = 0;
	  virtual void update_channels(int dist_type) = 0;
	  virtual int  min_delay(void) = 0;
    };

  } // namespace dbconnect
} // namespace gr

#endif /* INCLUDED_DBCONNECT_PACKET_CONTROLLER_H */

