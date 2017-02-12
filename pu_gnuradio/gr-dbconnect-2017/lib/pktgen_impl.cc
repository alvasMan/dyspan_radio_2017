/* -*- c++ -*- */
/* 
 * Copyright 2015 <+YOU OR YOUR COMPANY+>.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/block_detail.h>
#include <string.h>
#include <random>
#include <vector>
#include "pktgen_impl.h"
#define dout d_debug && std::cout

extern "C"{
#include "packetLib.h"
}

namespace gr {
  namespace dbconnect {


    /*
     * The private constructor
     */

	pktgen_impl::pktgen_impl(float interval, int pktsize, bool quit, bool debug, bool rand_scen, const std::string &host, int port, int tmin, int tmax, int tconst, int mean1, int mean2 , int mean3, int swtime, float psc, int seed, int gain_min, int gain_max, int gain_period, int gain_incr, int fixed_scenario, int first_channel, int second_channel) :
		block("pktgen",
				gr::io_signature::make(0, 0, 0),
				gr::io_signature::make(0, 0, 0)),
		//d_msg(msg),
		//d_nmsg_total(num_msg),
		d_nmsg_total(-1),
		d_nmsg_left(-1),
		//d_inter_per0(0),
		//d_inter_per1(0),
		//d_inter_per2(0),
		//d_inter_per3(0),
		//d_dist_per(0),
		d_tmin(tmin),
		d_tmax(tmax),
		d_tconst(tconst),
		d_swtime(swtime),
		d_mean1(mean1),
		d_mean2(mean2),
		d_mean3(mean3),
		d_psc(psc),
		d_seed(seed),
		d_gain_max(gain_max),
		d_gain_min(gain_min),
		d_gain_period(gain_period),
		d_gain_incr(gain_incr),
		d_interval(interval),
		d_debug(debug),
		d_rand_scen(rand_scen),
		//d_port(port),
		//d_host(host),
		d_pktsize(pktsize),
		d_finished(false),
		d_quit(quit),
                d_fixed_scenario(fixed_scenario),
                d_first_channel(first_channel),
                d_second_channel(second_channel) {

	message_port_register_out(pmt::mp("out0"));
	message_port_register_out(pmt::mp("out1"));
	message_port_register_out(pmt::mp("out2"));
	message_port_register_out(pmt::mp("out3"));
	message_port_register_out(pmt::mp("cmd"));

	d_thread = new boost::thread(boost::bind(&pktgen_impl::run, this, this));
	tx_connect(host,port);
}

    /*
     * Our virtual destructor.
     */

	pktgen_impl::~pktgen_impl() {
	gr::thread::scoped_lock(d_mutex);

	d_finished = true;
	d_thread->interrupt();
	d_thread->join();
	delete d_thread;
	}

void pktgen_impl:: tx_connect(const std::string &host, int port)
	{
		char * d_host = new char[host.size() + 1];
		std::copy(host.begin(), host.end(), d_host);
		d_host[host.size()] = '\0';

		dout << "pktsize:" << d_pktsize << std::endl;
		demoTx = spectrum_init(0);
		//spectrum_eror_t retVal = spectrum_connect(demoTx, "127.0.0.1", d_port, (uint16_t) d_pktsize, 1);
		spectrum_eror_t retVal = spectrum_connect(demoTx, d_host, port, (uint16_t) d_pktsize, 1);
		spectrum_errorToText(demoTx, retVal, d_errorBuf, sizeof(d_errorBuf));
		dout << "TX connect:" << d_errorBuf << std::endl;
		d_myRadio = spectrum_getRadioNumber(demoTx);
		dout << "Radio number:" << d_myRadio << std::endl;
		delete[] d_host;

	}

std::vector<int> scenario_generator(int scenario)
{
	std::vector<int> scenario_list;
	if(scenario==-1)
	{
		scenario_list.resize(10);
		for(int i = 0; i < 10; ++i)
		{
			scenario_list[i] = i;
		}
		std::random_shuffle(&scenario_list[0], &scenario_list[10]);
	}
	else
	{
		scenario_list.resize(1);
		scenario_list[0] = scenario;
	}
	return scenario_list;
}

void
pktgen_impl::run(pktgen_impl *instance) {

	try {

	// flow graph startup delay
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	//spectrum_waitForState(demoTx, 3, -1);
	dout << "Stage 3 started" << std::endl;
	std::default_random_engine d_gen1(d_seed);
	std::default_random_engine d_gen2(d_seed);
	std::default_random_engine d_gen3(d_seed);
	std::default_random_engine d_gen4(d_seed);
	std::default_random_engine d_gen5(d_seed);
	std::uniform_int_distribution<int> d_dist1(0,3);
	std::poisson_distribution<int> d_dist2(d_mean1);
	std::poisson_distribution<int> d_dist3(d_mean2);
	std::poisson_distribution<int> d_dist4(d_mean3);
	std::uniform_int_distribution<int> d_dist5(0,1);
	//std::geometric_distribution<int> d_dist4(d_psc);
	int d_inter_per0=0;
	int d_inter_per1=0;
	int d_inter_per2=0;
	int d_inter_per3=0;
	int d_inter_gain=0;
	int d_dist_per=1000000; // i have to change this from 0 to a high value bc the organizers are idiots
	int chan=0;
	int idx=0;
	/*
	  dist_type decides the scenario type
	    All specified delays are minimum delays between packets. There could be slight variations because of 
		pdu_fillpath implementation.

		0. Single random channel  -- deterministic inter packet delay1
		1. Single random channel  -- deterministic inter packet delay2 
		2. 2 Channel hopping         -- interpacket delay = delay1, Hopping pattern based on seed 
		3. 4 Channel hopping         -- interpacket delay = delay2, Hopping pattern based on seed 
		4. 2 Channel occupied        -- deterministic inter packet delay (delay1)
		5. 4 Channel occupied        -- deterministic inter packet delay (delay1)
		6. back to back on all channels (tconst)
		7. poisson 1  (inter packet delay configured from upper layers)
		8. poisson 2    "
		9. poisson 3   "
	   */
	int dist_type =0;
	int scncnt=0;
	srand(d_seed);
	std::vector<int> scenearr = scenario_generator(d_fixed_scenario);
	bool first_change = true;
	bool flag_change_scenario = true;
	int delay_1 = d_tmin;
	int delay_2 = d_tmax;
	bool sel_chan0 = false;  
	bool sel_chan1 = false;  
	bool sel_chan2 = false;  
	bool sel_chan3 = false;  
	int d_gain=d_gain_min;
	int hopping_2_chan[2]={d_first_channel,d_second_channel};
	int occ_2_chan[2]={d_first_channel,d_second_channel};

	if(d_fixed_scenario<0)
	{
		//select hopping channels
		hopping_2_chan[0] = d_dist1(d_gen1); 
		int schan = d_dist1(d_gen1);
		while( schan == hopping_2_chan[0])
		{
			schan = d_dist1(d_gen1); 
		}
		hopping_2_chan[1] = schan; 
		//select occupied channels
		occ_2_chan[0] = d_dist1(d_gen1); 
		schan = d_dist1(d_gen1);
		while( schan == occ_2_chan[0])
		{
			schan = d_dist1(d_gen1); 
		}
		occ_2_chan[1] = schan; 
	}

	//set to first scenario from the array
	dist_type = scenearr[0];

	switch (dist_type) {
		case 0:
		case 1:
		case 3:
			  sel_chan0 = false;  
			  sel_chan1 = false;  
			  sel_chan2 = false;  
			  sel_chan3 = false;  
			  chan = d_dist1(d_gen1);
			  switch(chan){
				  case 0:
			  		  sel_chan0 = true;  
					  break;
				  case 1:
			  		  sel_chan1 = true;  
				  	  break;
				  case 2:
			  		  sel_chan2 = true;  
				  	  break;
				  case 3:
			  		  sel_chan3 = true;  
				  	  break;
			  }
			  break;
		case 2:
			  idx = d_dist5(d_gen5);
			  sel_chan0 = false;  
			  sel_chan1 = false;  
			  sel_chan2 = false;  
			  sel_chan3 = false;  
			  switch(hopping_2_chan[idx]){
				  case 0:
			  		  sel_chan0 = true;  
					  break;
				  case 1:
			  		  sel_chan1 = true;  
				  	  break;
				  case 2:
			  		  sel_chan2 = true;  
				  	  break;
				  case 3:
			  		  sel_chan3 = true;  
				  	  break;
			  }
			  break;
		case 4:
			  sel_chan0 = false;  
			  sel_chan1 = false;  
			  sel_chan2 = false;  
			  sel_chan3 = false;  
			  for(int j=0; j<2; j++)
			  {
			  	switch(occ_2_chan[j]){
			  	    case 0:
			  			  sel_chan0 = true;  
			  	  	  break;
			  	    case 1:
			  			  sel_chan1 = true;  
			  	    	  break;
			  	    case 2:
			  			  sel_chan2 = true;  
			  	    	  break;
			  	    case 3:
			  			  sel_chan3 = true;  
			  	    	  break;
			  	}
			  }
			  break;
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
			  sel_chan0 = true;  
			  sel_chan1 = true;  
			  sel_chan2 = true;  
			  sel_chan3 = true;  
			  break;
		default:
			  break;
	}

	while(1) {
		float delay;
		{
			gr::thread::scoped_lock(d_mutex);
			if(d_finished || !d_nmsg_left) {
				d_finished = true;
				if(d_quit) {
					boost::this_thread::sleep(boost::posix_time::milliseconds(500));
					post(pmt::mp("system"), pmt::cons(pmt::mp("done"), pmt::from_long(1)));
				}
				break;
			}

			/* get packet and send it as pmt 
			*/
			spectrum_eror_t retVal; 
			pmt::pmt_t pdu;

			/*
			int j;
			for(j=0;j<d_pktsize;j++){
			dout << std::hex << int(d_packetBuffer[j]) ;
			dout << ",";
			if(j % 32 == 31){
				dout << std::endl;
			}
			}
			dout << std::endl;
			*/
			if(d_inter_per0 <=0)
			{
				dout << "Inside 0" << std::endl;
				if (sel_chan0)
				{
					retVal = spectrum_getPacket(demoTx, d_packetBuffer, sizeof(d_packetBuffer), -1);
					spectrum_errorToText(demoTx, retVal, d_errorBuf, sizeof(d_errorBuf));
					pdu = pmt::make_blob(d_packetBuffer, d_pktsize);
					dout << "Sending pkt 0" << std::endl;
					message_port_pub( pmt::mp("out0"), pmt::cons( pmt::PMT_NIL, pdu ));

					switch ( dist_type) {
						case 0:
							  d_inter_per0 = delay_1;
							  break;
						case 1:
							  d_inter_per0 = delay_2;
							  break;
						case 2:
							  d_inter_per0 = delay_1;
			  				  idx = d_dist5(d_gen5);
			  				  sel_chan0 = false;  
			  				  sel_chan1 = false;  
			  				  sel_chan2 = false;  
			  				  sel_chan3 = false;  
			  				  switch(hopping_2_chan[idx]){
			  				      case 0:
			  				  		  sel_chan0 = true;  
			  				    	  break;
			  				      case 1:
			  				  		  sel_chan1 = true;  
			  				      	  break;
			  				      case 2:
			  				  		  sel_chan2 = true;  
			  				      	  break;
			  				      case 3:
			  				  		  sel_chan3 = true;  
			  				      	  break;
			  				  }
							  break;
						case 3:
							  d_inter_per0 = delay_2;
			  				  sel_chan0 = false;  
			  				  sel_chan1 = false;  
			  				  sel_chan2 = false;  
			  				  sel_chan3 = false;  
			  				  chan = d_dist1(d_gen1);
			  				  switch(chan){
			  				      case 0:
			  				  		  sel_chan0 = true;  
			  				    	  break;
			  				      case 1:
			  				  		  sel_chan1 = true;  
			  				      	  break;
			  				      case 2:
			  				  		  sel_chan2 = true;  
			  				      	  break;
			  				      case 3:
			  				  		  sel_chan3 = true;  
			  				      	  break;
			  				  }
							  break;
						case 4:
							  d_inter_per0 = delay_1;
							  break;
						case 5:
							  d_inter_per0 = delay_1;
							  break;
						case 6:
							  d_inter_per0 = d_tconst;
							  break;
						case 7:
							  d_inter_per0 = d_dist2(d_gen2);
							  break;
						case 8:
							  d_inter_per0 = d_dist3(d_gen3);
							  break;
						case 9:
						      d_inter_per0 = d_dist4(d_gen4);
							  break;
						default:
							  d_inter_per0 = d_tconst;
							  break;
					}
				}
				else
				{
						d_inter_per0 = d_tconst;
				}

			}
		
			if(d_inter_per1 <=0)
			{
				dout << "Inside 1" << std::endl;
				if (sel_chan1)
				{
					retVal = spectrum_getPacket(demoTx, d_packetBuffer, sizeof(d_packetBuffer), -1);
					spectrum_errorToText(demoTx, retVal, d_errorBuf, sizeof(d_errorBuf));
					pdu = pmt::make_blob(d_packetBuffer, d_pktsize);
					dout << "Sending pkt 1" << std::endl;
					message_port_pub( pmt::mp("out1"), pmt::cons( pmt::PMT_NIL, pdu ));
					
					switch ( dist_type) {
						case 0:
							  d_inter_per1 = delay_1;
							  break;
						case 1:
							  d_inter_per1 = delay_2;
							  break;
						case 2:
							  d_inter_per1 = delay_1;
			  				  idx = d_dist5(d_gen5);
			  				  sel_chan0 = false;  
			  				  sel_chan1 = false;  
			  				  sel_chan2 = false;  
			  				  sel_chan3 = false;  
			  				  switch(hopping_2_chan[idx]){
			  				      case 0:
			  				  		  sel_chan0 = true;  
			  				    	  break;
			  				      case 1:
			  				  		  sel_chan1 = true;  
			  				      	  break;
			  				      case 2:
			  				  		  sel_chan2 = true;  
			  				      	  break;
			  				      case 3:
			  				  		  sel_chan3 = true;  
			  				      	  break;
			  				  }
							  break;
						case 3:
							  d_inter_per1 = delay_2;
			  				  sel_chan0 = false;  
			  				  sel_chan1 = false;  
			  				  sel_chan2 = false;  
			  				  sel_chan3 = false;  
			  				  chan = d_dist1(d_gen1);
			  				  switch(chan){
			  				      case 0:
			  				  		  sel_chan0 = true;  
			  				    	  break;
			  				      case 1:
			  				  		  sel_chan1 = true;  
			  				      	  break;
			  				      case 2:
			  				  		  sel_chan2 = true;  
			  				      	  break;
			  				      case 3:
			  				  		  sel_chan3 = true;  
			  				      	  break;
			  				  }
							  break;
						case 4:
							  d_inter_per1 = delay_1;
							  break;
						case 5:
							  d_inter_per1 = delay_1;
							  break;
						case 6:
							  d_inter_per1 = d_tconst;
							  break;
						case 7:
							  d_inter_per1 = d_dist2(d_gen2);
							  break;
						case 8:
							  d_inter_per1 = d_dist3(d_gen3);
							  break;
						case 9:
						      d_inter_per1 = d_dist4(d_gen4);
							  break;
						default:
							  d_inter_per1 = d_tconst;
							  break;
					}
				}
				else
				{
						d_inter_per1 = d_tconst;
				}
			}

			if(d_inter_per2 <=0)
			{
				dout << "Inside 2" << std::endl;
				if (sel_chan2)
				{
					retVal = spectrum_getPacket(demoTx, d_packetBuffer, sizeof(d_packetBuffer), -1);
					spectrum_errorToText(demoTx, retVal, d_errorBuf, sizeof(d_errorBuf));
					pdu = pmt::make_blob(d_packetBuffer, d_pktsize);
					dout << "Sending pkt 2" << std::endl;
					message_port_pub( pmt::mp("out2"), pmt::cons( pmt::PMT_NIL, pdu ));
					
					switch ( dist_type) {
						case 0:
							  d_inter_per2 = delay_1;
							  break;
						case 1:
							  d_inter_per2 = delay_2;
							  break;
						case 2:
							  d_inter_per2 = delay_1;
			  				  idx = d_dist5(d_gen5);
			  				  sel_chan0 = false;  
			  				  sel_chan1 = false;  
			  				  sel_chan2 = false;  
			  				  sel_chan3 = false;  
			  				  switch(hopping_2_chan[idx]){
			  				      case 0:
			  				  		  sel_chan0 = true;  
			  				    	  break;
			  				      case 1:
			  				  		  sel_chan1 = true;  
			  				      	  break;
			  				      case 2:
			  				  		  sel_chan2 = true;  
			  				      	  break;
			  				      case 3:
			  				  		  sel_chan3 = true;  
			  				      	  break;
			  				  }
							  break;
						case 3:
							  d_inter_per2 = delay_2;
			  				  sel_chan0 = false;  
			  				  sel_chan1 = false;  
			  				  sel_chan2 = false;  
			  				  sel_chan3 = false;  
			  				  chan = d_dist1(d_gen1);
			  				  switch(chan){
			  				      case 0:
			  				  		  sel_chan0 = true;  
			  				    	  break;
			  				      case 1:
			  				  		  sel_chan1 = true;  
			  				      	  break;
			  				      case 2:
			  				  		  sel_chan2 = true;  
			  				      	  break;
			  				      case 3:
			  				  		  sel_chan3 = true;  
			  				      	  break;
			  				  }
							  break;
						case 4:
							  d_inter_per2 = delay_1;
							  break;
						case 5:
							  d_inter_per2 = delay_1;
							  break;
						case 6:
							  d_inter_per2 = d_tconst;
							  break;
						case 7:
							  d_inter_per2 = d_dist2(d_gen2);
							  break;
						case 8:
							  d_inter_per2 = d_dist3(d_gen3);
							  break;
						case 9:
						      d_inter_per2 = d_dist4(d_gen4);
							  break;
						default:
							  d_inter_per2 = d_tconst;
							  break;
					}
				}
				else
				{
						d_inter_per2 = d_tconst;
				}
			}

			if(d_inter_per3 <=0)
			{
				dout << "Inside 3" << std::endl;
				if (sel_chan3)
				{
					retVal = spectrum_getPacket(demoTx, d_packetBuffer, sizeof(d_packetBuffer), -1);
					spectrum_errorToText(demoTx, retVal, d_errorBuf, sizeof(d_errorBuf));
					pdu = pmt::make_blob(d_packetBuffer, d_pktsize);
					dout << "Sending pkt 3" << std::endl;
					message_port_pub( pmt::mp("out3"), pmt::cons( pmt::PMT_NIL, pdu ));
					switch ( dist_type) {
						case 0:
							  d_inter_per3 = delay_1;
							  break;
						case 1:
							  d_inter_per3 = delay_2;
							  break;
						case 2:
							  d_inter_per3 = delay_1;
			  				  idx = d_dist5(d_gen5);
			  				  sel_chan0 = false;  
			  				  sel_chan1 = false;  
			  				  sel_chan2 = false;  
			  				  sel_chan3 = false;  
			  				  switch(hopping_2_chan[idx]){
			  				      case 0:
			  				  		  sel_chan0 = true;  
			  				    	  break;
			  				      case 1:
			  				  		  sel_chan1 = true;  
			  				      	  break;
			  				      case 2:
			  				  		  sel_chan2 = true;  
			  				      	  break;
			  				      case 3:
			  				  		  sel_chan3 = true;  
			  				      	  break;
			  				  }
							  break;
						case 3:
							  d_inter_per3 = delay_2;
			  				  sel_chan0 = false;  
			  				  sel_chan1 = false;  
			  				  sel_chan2 = false;  
			  				  sel_chan3 = false;  
			  				  chan = d_dist1(d_gen1);
			  				  switch(chan){
			  				      case 0:
			  				  		  sel_chan0 = true;  
			  				    	  break;
			  				      case 1:
			  				  		  sel_chan1 = true;  
			  				      	  break;
			  				      case 2:
			  				  		  sel_chan2 = true;  
			  				      	  break;
			  				      case 3:
			  				  		  sel_chan3 = true;  
			  				      	  break;
			  				  }
							  break;
						case 4:
							  d_inter_per3 = delay_1;
							  break;
						case 5:
							  d_inter_per3 = delay_1;
							  break;
						case 6:
							  d_inter_per3 = d_tconst;
							  break;
						case 7:
							  d_inter_per3 = d_dist2(d_gen2);
							  break;
						case 8:
							  d_inter_per3 = d_dist3(d_gen3);
							  break;
						case 9:
						      d_inter_per3 = d_dist4(d_gen4);
							  break;
						default:
							  d_inter_per3 = d_tconst;
							  break;
					}
				}
				else
				{
						d_inter_per3 = d_tconst;
				}
			}
			d_inter_per0--;
			d_inter_per1--;
			d_inter_per2--;
			d_inter_per3--;
			d_dist_per++;
			d_inter_gain++;

			if(d_nmsg_left > 0) {
				d_nmsg_left--;
			}
			
			delay = d_interval;

			if(d_dist_per>=d_swtime && (d_fixed_scenario>=0 || first_change))
			{
				first_change = false;
				scncnt++;
				d_dist_per =0;
				//dist_type++;
				dist_type = scenearr[scncnt%scenearr.size()];
				// Select channel based on scenario
				switch (dist_type) {
					case 0:
					case 1:
					case 3:
						  sel_chan0 = false;  
						  sel_chan1 = false;  
						  sel_chan2 = false;  
						  sel_chan3 = false;  
						  chan = (d_fixed_scenario < 0) ? d_dist1(d_gen1) : d_first_channel;
						  switch(chan){
							  case 0:
						  		  sel_chan0 = true;  
								  break;
							  case 1:
						  		  sel_chan1 = true;  
							  	  break;
							  case 2:
						  		  sel_chan2 = true;  
							  	  break;
							  case 3:
						  		  sel_chan3 = true;  
							  	  break;
						  }
						  break;
					case 2:
						  sel_chan0 = false;  
						  sel_chan1 = false;  
						  sel_chan2 = false;  
						  sel_chan3 = false;  
						  idx = d_dist5(d_gen5);
						  switch(hopping_2_chan[idx]){
							  case 0:
						  		  sel_chan0 = true;  
								  break;
							  case 1:
						  		  sel_chan1 = true;  
							  	  break;
							  case 2:
						  		  sel_chan2 = true;  
							  	  break;
							  case 3:
						  		  sel_chan3 = true;  
							  	  break;
						  }
						  break;
					case 4:
						  sel_chan0 = false;  
						  sel_chan1 = false;  
						  sel_chan2 = false;  
						  sel_chan3 = false;  
						  for(int j=0; j<2; j++)
						  {
						  	switch(occ_2_chan[j]){
						  	    case 0:
						  			  sel_chan0 = true;  
						  	  	  break;
						  	    case 1:
						  			  sel_chan1 = true;  
						  	    	  break;
						  	    case 2:
						  			  sel_chan2 = true;  
						  	    	  break;
						  	    case 3:
						  			  sel_chan3 = true;  
						  	    	  break;
						  	}
						  }
						  break;
					case 5:
					case 6:
					case 7:
					case 8:
					case 9:
						  sel_chan0 = true;  
						  sel_chan1 = true;  
						  sel_chan2 = true;  
						  sel_chan3 = true;  
						  break;
					default:
						  break;
				}
				if (d_rand_scen && scncnt == 5)
				{
					srand(d_seed);
					std::random_shuffle(&scenearr[0], &scenearr[scenearr.size()]);
				}
				scncnt = scncnt%scenearr.size();
			}
			
			if(d_inter_gain>=d_gain_period)
			{
				dout << "DEBUG: " << d_first_channel << std::endl;
				d_inter_gain=0;
				d_gain = d_gain + d_gain_incr;
				if (d_gain > d_gain_max)
				{
					d_gain = d_gain_min;
				}
				message_port_pub(pmt::mp("cmd"), pmt::cons(pmt::mp("gain"), pmt::mp(d_gain)));
				
			}


		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(delay));
	} 

	} catch(boost::thread_interrupted) {
		gr::thread::scoped_lock(d_mutex);
		dout << "PMS: thread interrupted" << std::endl;
		d_finished = true;
		if(d_quit) {
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			post(pmt::mp("system"), pmt::cons(pmt::mp("done"), pmt::from_long(1)));
		}
	}
}

void
pktgen_impl::set_nmsg(int nmsg) {
	gr::thread::scoped_lock(d_mutex);
	d_nmsg_total = nmsg;
}

int
pktgen_impl::get_nmsg() {
	gr::thread::scoped_lock(d_mutex);
	return d_nmsg_total;
}

void
pktgen_impl::set_delay(float delay) {
	gr::thread::scoped_lock(d_mutex);
	d_interval = delay;
}

float
pktgen_impl::get_delay() {
	gr::thread::scoped_lock(d_mutex);
	return d_interval;
}


void
pktgen_impl::start_tx() {
	gr::thread::scoped_lock(d_mutex);

	if(is_running()) return;

	d_nmsg_left = d_nmsg_total;
	d_finished = false;
	d_thread->join();
	delete d_thread;

	d_thread = new boost::thread(boost::bind(&pktgen_impl::run, this, this));
}

void
pktgen_impl::stop_tx() {
	d_thread->interrupt();
}

bool
pktgen_impl::is_running() {
	return !d_finished;
}

    pktgen::sptr
    pktgen::make(float interval, int pktsize, bool quit, bool debug, bool rand_scen, const std::string &host, int port, int tmin, int tmax, int tconst, int mean1, int mean2, int mean3, int swtime, float psc, int seed,int gain_min, int gain_max, int gain_period, int gain_incr, int fixed_scenario, int first_channel, int second_channel)
    {
      return gnuradio::get_initial_sptr
        (new pktgen_impl(interval, pktsize, quit, debug, rand_scen, host, port, tmin, tmax, tconst, mean1, mean2, mean3, swtime, psc, seed, gain_min, gain_max, gain_period, gain_incr, fixed_scenario, first_channel, second_channel));
    }

  } /* namespace dbconnect */
} /* namespace gr */

