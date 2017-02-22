/* -*- c++ -*- */

#define DBCONNECT_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "dbconnect_swig_doc.i"

%{
#include "dbconnect/pktgen.h"
#include "dbconnect/pktrecv.h"
#include "dbconnect/pdu_fillpath_cpp.h"
#include "dbconnect/packet_controller.h"
#include "dbconnect/cmd_pktgen.h"
%}
%include "dbconnect/pktgen.h"
GR_SWIG_BLOCK_MAGIC2(dbconnect, pktgen);
%include "dbconnect/pktrecv.h"
GR_SWIG_BLOCK_MAGIC2(dbconnect, pktrecv);
%include "dbconnect/pdu_fillpath_cpp.h"
GR_SWIG_BLOCK_MAGIC2(dbconnect, pdu_fillpath_cpp);

%include "dbconnect/packet_controller.h"
GR_SWIG_BLOCK_MAGIC2(dbconnect, packet_controller);
%include "dbconnect/cmd_pktgen.h"
GR_SWIG_BLOCK_MAGIC2(dbconnect, cmd_pktgen);
